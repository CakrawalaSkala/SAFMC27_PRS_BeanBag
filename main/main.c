#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "easy_gpio.h"
#include "lora.h"

#if CONFIG_FREERTOS_HZ < 1000
#error "Atur CONFIG_FREERTOS_HZ=1000 agar polling dan timer millisecond konsisten"
#endif

static const char *TAG = "PRS_PANTRY";

typedef enum {
    NODE_MAIN   = 1,
    NODE_PANTRY = 2,
    NODE_ICE    = 3,
    NODE_OVEN   = 4,
    NODE_DRONE  = 5
} node_id_t;

static const uint8_t s_node_id = NODE_PANTRY;

#define PIN_LORA_SCK     4
#define PIN_LORA_MISO    5
#define PIN_LORA_MOSI    6
#define PIN_LORA_NSS     7
#define PIN_LORA_RST     8
#define PIN_LORA_DIO0    2

#define PIN_SR_LATCH     0
#define PIN_SR_CLK       3
#define PIN_SR_DATA      10
#define PIN_EN_MOTOR     1

#define PIN_SERVO        20
#define PIN_SW_DOCK      21
#define PIN_BTN_MANUAL   9

#define DOCK_SIMULATION_JUMPER 1
#define PRS_SIMULASI_MEKANISME 1
#define PRS_ENABLE_ACTUATORS   0
#define PRS_CALIBRATION_OK     0
#define PRS_ALLOW_STOCK_RESET  0
#define PRS_PHYSICAL_ENABLED   (PRS_ENABLE_ACTUATORS && !PRS_SIMULASI_MEKANISME)
#define DOCK_ACTIVE_LEVEL      LOW

#define ENABLE_MANUAL_BUTTON  1

#define TOTAL_SLOT       12
#define DOCK_SIDES       6
#define PKT_LEN          3
_Static_assert(PKT_LEN == 3, "Paket LoRa harus 3 byte");
_Static_assert(TOTAL_SLOT == DOCK_SIDES * 2, "Setiap sisi punya 2 jalur stok");

#define LORA_FREQ_HZ     433000000
#define LORA_TX_POWER_MW 9
#define TASK_POLL_MS     10
#define STATUS_LOG_MS    5000
#define DOCK_DEBOUNCE_MS 30
#define BUTTON_DEBOUNCE_MS 30
#define BUTTON_HOLD_MS   3000
#define SERVO_PWM_HZ     50
#define SERVO_PWM_BITS   13
#define SERVO_US_GREEN   1200  
#define SERVO_US_RED     1533  
#define SERVO_DELAY_MS   400
#define PULL_MS          1500
#define DRONE_FRESH_MS   500

#if PRS_ENABLE_ACTUATORS && (PRS_SIMULASI_MEKANISME || !PRS_CALIBRATION_OK)
#error "Aktuator Pantry memerlukan mode fisik dan kalibrasi mekanisme"
#endif

#if PRS_ENABLE_ACTUATORS && !defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
#error "GPIO20/21 PRS membutuhkan konsol USB Serial JTAG dan UART0 nonaktif"
#endif

typedef enum {
    CMD_NONE     = 0x0,
    CMD_MAIN1    = 0x1,  
    CMD_MAIN2    = 0x2,  
    CMD_OPT1     = 0x3,  
    CMD_OPT2     = 0x4,  
    CMD_SW_DRONE = 0x5,  
    CMD_SW_CAM   = 0x6   
} command_t;

typedef enum {
    LANE_GREEN = 0, 
    LANE_RED   = 1, 
    LANE_NONE  = 2
} lane_t;

typedef enum {
    PULL_IDLE = 0,
    PULL_SERVO_SETTLING,
    PULL_MOTOR_ON
} pull_stage_t;

typedef struct {
    pull_stage_t stage;
    lane_t       lane;
    int          stock_slot;   
    int          mag_side;     
    uint32_t     stage_since_ms;
} pull_state_t;

static lora_t *s_radio;
static QueueHandle_t s_cmd_q;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static bool s_stock_used[TOTAL_SLOT];
static bool s_mag_side_used[DOCK_SIDES];
static bool s_is_docked;
static uint16_t s_motor_mask;
static lane_t s_current_lane = LANE_NONE;
static pull_state_t s_pull = { .stage = PULL_IDLE, .lane = LANE_NONE,
                               .stock_slot = -1, .mag_side = -1 };

static uint8_t s_latched_cmd = CMD_NONE;
static uint32_t s_rx_main;
static uint32_t s_tx_count;
static uint32_t s_tx_failed;
static uint8_t s_event_pending;
static bool s_dock_synced;
static uint8_t s_drone_slots[DOCK_SIDES];
static uint8_t s_remote_drone_id;
static uint32_t s_sync_count;
static uint32_t s_last_drone_ms;

static uint32_t servo_us_to_duty(uint16_t us)
{
    
    return (uint32_t)(((uint64_t)us * ((1UL << SERVO_PWM_BITS) - 1) + 10000) / 20000);
}

static esp_err_t set_lane(lane_t lane)
{
    if (lane == s_current_lane) return ESP_OK;
    if (lane != LANE_GREEN && lane != LANE_RED) return ESP_ERR_INVALID_ARG;
    const uint16_t us = (lane == LANE_GREEN) ? SERVO_US_GREEN : SERVO_US_RED;
    esp_err_t err = !PRS_PHYSICAL_ENABLED ? ESP_OK : analogWrite(PIN_SERVO, servo_us_to_duty(us));
    if (err == ESP_OK) {
        s_current_lane = lane;
        ESP_LOGI(TAG, "%s jalur %s", !PRS_PHYSICAL_ENABLED ? "Simulasi servo" : "Servo diarahkan ke", lane == LANE_GREEN ? "HIJAU" : "MERAH");
    } else {
        ESP_LOGE(TAG, "Gagal mengatur servo jalur: %s", esp_err_to_name(err));
    }
    return err;
}

static void sr_write(uint16_t value)
{
    digitalWrite(PIN_SR_LATCH, LOW);
    shiftOut(PIN_SR_DATA, PIN_SR_CLK, MSBFIRST, (uint8_t)(value >> 8));
    shiftOut(PIN_SR_DATA, PIN_SR_CLK, MSBFIRST, (uint8_t)(value & 0xFF));
    digitalWrite(PIN_SR_LATCH, HIGH);
}

static void motor_set(int index, bool on)
{
    if (index < 0 || index >= TOTAL_SLOT) return;
    if (on) s_motor_mask |= (uint16_t)(1U << index);
    else    s_motor_mask &= (uint16_t)~(1U << index);
    if (PRS_PHYSICAL_ENABLED) {
        sr_write(s_motor_mask);
        digitalWrite(PIN_EN_MOTOR, s_motor_mask != 0 ? HIGH : LOW);
    }
}

static void motors_off(void)
{
    digitalWrite(PIN_EN_MOTOR, LOW);
    s_motor_mask = 0;
    sr_write(0);
}

static void reset_pull_state(void)
{
    s_pull.stage = PULL_IDLE;
    s_pull.lane = LANE_NONE;
    s_pull.stock_slot = -1;
    s_pull.mag_side = -1;
}

static void abort_pull(void)
{
    if (s_pull.stage == PULL_IDLE) return;
    motors_off();
    reset_pull_state();
    ESP_LOGW(TAG, "Pengambilan dihentikan karena drone meninggalkan dock");
}

static void send_status(void)
{
    uint8_t red = 0, green = 0, landed = 0, event = 0;
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < DOCK_SIDES; i++) {
        if (!s_stock_used[2 * i])     green++;
        if (!s_stock_used[2 * i + 1]) red++;
    }
    landed = s_is_docked ? 1 : 0;
    event = s_event_pending;
    portEXIT_CRITICAL(&s_mux);

    const uint8_t pkt[PKT_LEN] = {
        s_node_id,
        (uint8_t)((red << 4) | green),
        (uint8_t)((landed << 4) | event)
    };
    esp_err_t err = lora_send(s_radio, pkt, PKT_LEN, 0);
    if (err == ESP_OK) {
        s_tx_count++;
    } else {
        s_tx_failed++;
        ESP_LOGE(TAG, "Gagal kirim status Pantry: %s", esp_err_to_name(err));
    }
}

static void handle_main_command(uint8_t cmd)
{
    bool new_event = false;
    if (cmd == CMD_NONE) {
        s_latched_cmd = CMD_NONE;
    } else if (cmd != s_latched_cmd) {
        s_latched_cmd = cmd;
        new_event = true;
    }

    if (!new_event) return;
    if (cmd != CMD_MAIN1 && cmd != CMD_MAIN2) return;

    uint8_t item = cmd;
    if (xQueueSend(s_cmd_q, &item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Antrean pengambilan penuh; CMD 0x%02X diabaikan", cmd);
    }
}

static void on_rx(lora_t *radio, const lora_packet_t *p, void *ctx)
{
    (void)radio;
    (void)ctx;
    if (!p->crc_ok || p->len != PKT_LEN) return;

    if (p->data[0] == NODE_MAIN) {
        if ((p->data[1] & 0x0Fu) != 0 || p->data[2] != 0) return;
        s_rx_main++;
        handle_main_command((uint8_t)(p->data[1] >> 4));
        send_status();
        return;
    }
    if (p->data[0] != NODE_DRONE) return;
    uint8_t drone_id = (uint8_t)(p->data[1] >> 4);
    if (drone_id != 1) return;
    uint16_t slots = (uint16_t)(((p->data[1] & 0x0Fu) << 8) | p->data[2]);
    uint8_t remote[DOCK_SIDES];
    for (int i = 0; i < DOCK_SIDES; ++i) {
        remote[i] = (uint8_t)((slots >> (10 - i * 2)) & 0x03u);
        if (remote[i] > 2) return;
    }

    uint8_t ack = 0;
    portENTER_CRITICAL(&s_mux);
    s_remote_drone_id = drone_id;
    s_last_drone_ms = millis();
    for (int i = 0; i < DOCK_SIDES; ++i) s_drone_slots[i] = remote[i];
    s_dock_synced = s_is_docked;
    if (s_event_pending != 0) {
        uint8_t event = s_event_pending;
        int index = (event - 1) % DOCK_SIDES;
        uint8_t color = event <= DOCK_SIDES ? 2 : 1;
        if (remote[index] == color) {
            ack = event;
            s_event_pending = 0;
            s_sync_count++;
        }
    }
    if (s_is_docked && s_pull.stage == PULL_IDLE && s_event_pending == 0) {
        for (int i = 0; i < DOCK_SIDES; ++i) {
            s_mag_side_used[i] = remote[i] != 0;
        }
    }
    portEXIT_CRITICAL(&s_mux);
    if (ack) {
        ESP_LOGI(TAG, "Sinkronisasi berhasil: Drone mencatat slot %u, jenis %s",
                 (unsigned)((ack - 1) % DOCK_SIDES + 1), ack <= DOCK_SIDES ? "HIJAU" : "MERAH");
    }
}

static void start_pull(lane_t lane, uint32_t now)
{
    if (!s_is_docked) {
        ESP_LOGW(TAG, "Picking ditolak: jumper docking belum terpasang");
        return;
    }
    if (!s_dock_synced || s_last_drone_ms == 0 ||
        (uint32_t)(now - s_last_drone_ms) > DRONE_FRESH_MS) {
        ESP_LOGW(TAG, "Picking ditolak: status slot Drone belum sinkron atau kedaluwarsa");
        return;
    }
    if (s_event_pending != 0) {
        ESP_LOGW(TAG, "Picking ditolak: menunggu konfirmasi slot sebelumnya dari Drone");
        return;
    }
    if (s_pull.stage != PULL_IDLE) {
        ESP_LOGW(TAG, "Picking ditolak: proses sebelumnya masih berlangsung");
        return;
    }

    int stock_slot = -1, side = -1;
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < DOCK_SIDES; i++) {
        if (!s_mag_side_used[i] && s_drone_slots[i] == 0) {
            side = i;
            break;
        }
    }
    if (side >= 0) {
        if (!PRS_PHYSICAL_ENABLED) {
            for (int i = 0; i < DOCK_SIDES; ++i) {
                int idx = i * 2 + lane;
                if (!s_stock_used[idx]) {
                    stock_slot = idx;
                    break;
                }
            }
        } else {
            int idx = side * 2 + lane;
            if (!s_stock_used[idx]) stock_slot = idx;
        }
    }
    portEXIT_CRITICAL(&s_mux);
    if (side < 0 || stock_slot < 0) {
        ESP_LOGW(TAG, "Picking %s ditolak: magazine penuh atau stok tidak tersedia",
                 lane == LANE_GREEN ? "HIJAU" : "MERAH");
        return;
    }
    bool servo_moves = s_current_lane != lane;
    if (set_lane(lane) != ESP_OK) return;
    s_pull.lane = lane;
    s_pull.stock_slot = stock_slot;
    s_pull.mag_side = side;
    s_pull.stage_since_ms = now;
    s_pull.stage = servo_moves ? PULL_SERVO_SETTLING : PULL_MOTOR_ON;
    ESP_LOGI(TAG, "%s PICK ONE %s: stok PRS %u menuju slot Drone %u",
             !PRS_PHYSICAL_ENABLED ? "SIMULASI" : "MULAI",
             lane == LANE_GREEN ? "HIJAU" : "MERAH",
             (unsigned)(stock_slot + 1), (unsigned)(side + 1));
    if (!servo_moves) motor_set(stock_slot, true);
}

static void service_pull(uint32_t now)
{
    if (PRS_PHYSICAL_ENABLED && s_pull.stage != PULL_IDLE &&
        (s_last_drone_ms == 0 || (uint32_t)(now - s_last_drone_ms) > DRONE_FRESH_MS)) {
        abort_pull();
        ESP_LOGW(TAG, "Motor dihentikan: telemetri Drone sudah kedaluwarsa");
        return;
    }
    if (s_pull.stage == PULL_SERVO_SETTLING &&
        (uint32_t)(now - s_pull.stage_since_ms) >= SERVO_DELAY_MS) {
        motor_set(s_pull.stock_slot, true);
        s_pull.stage = PULL_MOTOR_ON;
        s_pull.stage_since_ms = now;
    }
    if (s_pull.stage == PULL_MOTOR_ON &&
        (uint32_t)(now - s_pull.stage_since_ms) >= PULL_MS) {
        int slot = s_pull.stock_slot;
        int side = s_pull.mag_side;
        motor_set(slot, false);
        portENTER_CRITICAL(&s_mux);
        s_stock_used[slot] = true;
        s_mag_side_used[side] = true;
        s_event_pending = (uint8_t)(side + 1 + (s_pull.lane == LANE_RED ? DOCK_SIDES : 0));
        portEXIT_CRITICAL(&s_mux);
        ESP_LOGI(TAG, "Selesai ambil: stok %d -> sisi magazine %d (belum diverifikasi sensor)",
                 slot + 1, side + 1);
        reset_pull_state();
    }
}

static bool dock_input_active(void)
{
    return digitalRead(PIN_SW_DOCK) == DOCK_ACTIVE_LEVEL;
}

static void service_dock(uint32_t now)
{
    static bool last_raw;
    static bool initialized;
    static uint32_t raw_since;

    bool raw = dock_input_active();
    if (!initialized) {
        initialized = true;
        last_raw = raw;
        raw_since = now;
    }
    if (raw != last_raw) {
        last_raw = raw;
        raw_since = now;
    }
    if ((uint32_t)(now - raw_since) < DOCK_DEBOUNCE_MS) return;
    if (raw == s_is_docked) return;

    portENTER_CRITICAL(&s_mux);
    s_is_docked = raw;
    if (raw) {

        memset(s_mag_side_used, 0, sizeof(s_mag_side_used));
        s_dock_synced = false;
    }
    portEXIT_CRITICAL(&s_mux);
    if (raw) {
#if DOCK_SIMULATION_JUMPER
        ESP_LOGI(TAG, "SIMULASI DOCK AKTIF: jumper GPIO%d ke GND terpasang", PIN_SW_DOCK);
#else
        ESP_LOGI(TAG, "Limit switch Pantry aktif: drone docking");
#endif
    } else {
        abort_pull();
        xQueueReset(s_cmd_q);
#if DOCK_SIMULATION_JUMPER
        ESP_LOGI(TAG, "SIMULASI DOCK NONAKTIF: jumper GPIO%d ke GND dilepas", PIN_SW_DOCK);
#else
        ESP_LOGI(TAG, "Limit switch Pantry lepas: dock kosong");
#endif
    }
}

#if ENABLE_MANUAL_BUTTON
static void service_manual_button(uint32_t now)
{
    static bool raw_last;
    static bool stable_down;
    static bool initialized;
    static bool long_done;
    static uint32_t raw_since;
    static uint32_t press_since;

    bool raw_down = digitalRead(PIN_BTN_MANUAL) == LOW;
    if (!initialized) {
        initialized = true;
        raw_last = raw_down;
        raw_since = now;
    }
    if (raw_down != raw_last) {
        raw_last = raw_down;
        raw_since = now;
    }
    if ((uint32_t)(now - raw_since) < BUTTON_DEBOUNCE_MS) return;
    if (raw_down != stable_down) {
        stable_down = raw_down;
        if (raw_down) {
            press_since = now;
            long_done = false;
        } else if (!long_done && s_pull.stage == PULL_IDLE) {
            lane_t next = s_current_lane == LANE_GREEN ? LANE_RED : LANE_GREEN;
            (void)set_lane(next);
        }
    }
    if (stable_down && !long_done &&
        (uint32_t)(now - press_since) >= BUTTON_HOLD_MS) {
        long_done = true;
        if (s_pull.stage == PULL_IDLE && PRS_ALLOW_STOCK_RESET) {
            portENTER_CRITICAL(&s_mux);
            memset(s_stock_used, 0, sizeof(s_stock_used));
            memset(s_mag_side_used, 0, sizeof(s_mag_side_used));
            s_event_pending = 0;
            s_dock_synced = false;
            portEXIT_CRITICAL(&s_mux);
            ESP_LOGW(TAG, "RESET MANUAL: stok dan magazine logis dihapus; periksa ulang stok fisik");
        } else {
            ESP_LOGW(TAG, "RESET MANUAL ditolak: reset stok dinonaktifkan atau motor masih aktif");
        }
    }
}
#endif

static void mechanism_task(void *arg)
{
    (void)arg;
    uint32_t last_report = millis();
    for (;;) {
        uint32_t now = millis();
        service_dock(now);
#if ENABLE_MANUAL_BUTTON
        service_manual_button(now);
#endif
        if (!s_is_docked && s_pull.stage != PULL_IDLE) abort_pull();
        uint8_t cmd;
        while (xQueueReceive(s_cmd_q, &cmd, 0) == pdTRUE) {
            if (cmd == CMD_MAIN1) start_pull(LANE_GREEN, now);
            else if (cmd == CMD_MAIN2) start_pull(LANE_RED, now);
        }
        service_pull(now);
        if ((uint32_t)(now - last_report) >= STATUS_LOG_MS) {
            last_report = now;
            uint8_t red = 0, green = 0;
            char slots[DOCK_SIDES + 1];
            uint8_t pending, active_id;
            uint32_t synced;
            portENTER_CRITICAL(&s_mux);
            for (int i = 0; i < DOCK_SIDES; ++i) {
                if (!s_stock_used[i * 2]) green++;
                if (!s_stock_used[i * 2 + 1]) red++;
                slots[i] = s_drone_slots[i] == 1 ? 'R' : s_drone_slots[i] == 2 ? 'G' : '-';
            }
            slots[DOCK_SIDES] = '\0';
            pending = s_event_pending;
            active_id = s_remote_drone_id;
            synced = s_sync_count;
            portEXIT_CRITICAL(&s_mux);
            ESP_LOGI(TAG, "Diagnostik docking: GPIO%d=%d (%s), status=%s", PIN_SW_DOCK,
                     digitalRead(PIN_SW_DOCK),
                     digitalRead(PIN_SW_DOCK) == DOCK_ACTIVE_LEVEL ? "TERHUBUNG GND" : "TERLEPAS",
                     s_is_docked ? "MENDARAT" : "TIDAK MENDARAT");
            ESP_LOGI(TAG, "Status: stok R:%u G:%u, slot Drone=%s, dock=%s, event=%u, sinkron=%lu, RX=%lu TX=%lu gagal=%lu ID=%u",
                     (unsigned)red, (unsigned)green, slots,
                     s_is_docked ? "YA" : "TIDAK", (unsigned)pending,
                     (unsigned long)synced, (unsigned long)s_rx_main,
                     (unsigned long)s_tx_count, (unsigned long)s_tx_failed,
                     (unsigned)active_id);
        }
        vTaskDelay(pdMS_TO_TICKS(TASK_POLL_MS));
    }
}

static esp_err_t init_pins(void)
{
    ESP_RETURN_ON_ERROR(pinMode(PIN_SW_DOCK, INPUT_PULLUP), TAG, "Gagal inisialisasi input dock");
#if DOCK_SIMULATION_JUMPER
    ESP_LOGW(TAG, "MODE SIMULASI DOCK: jumper GPIO%d ke GND = mendarat, lepas = tidak mendarat", PIN_SW_DOCK);
    ESP_LOGW(TAG, "Simulasi tidak membuktikan drone hadir; uji aktuator dalam kondisi aman");
#else
    ESP_LOGI(TAG, "MODE LIMIT SWITCH: GPIO%d aktif LOW, menggunakan pull-up internal", PIN_SW_DOCK);
#endif
#if ENABLE_MANUAL_BUTTON
    ESP_RETURN_ON_ERROR(pinMode(PIN_BTN_MANUAL, INPUT_PULLUP), TAG, "Gagal inisialisasi tombol manual");
#endif
    ESP_RETURN_ON_ERROR(pinMode(PIN_SR_DATA, OUTPUT), TAG, "Gagal inisialisasi data register geser");
    ESP_RETURN_ON_ERROR(pinMode(PIN_SR_CLK, OUTPUT), TAG, "Gagal inisialisasi clock register geser");
    ESP_RETURN_ON_ERROR(pinMode(PIN_SR_LATCH, OUTPUT), TAG, "Gagal inisialisasi latch register geser");
    ESP_RETURN_ON_ERROR(pinMode(PIN_EN_MOTOR, OUTPUT), TAG, "Gagal inisialisasi enable motor");
    digitalWrite(PIN_EN_MOTOR, LOW);
    digitalWrite(PIN_SR_CLK, LOW);
    digitalWrite(PIN_SR_DATA, LOW);
    motors_off();

    if (PRS_PHYSICAL_ENABLED) {
        ESP_RETURN_ON_ERROR(analogWriteFrequency(SERVO_PWM_HZ), TAG, "Gagal mengatur frekuensi PWM servo");
        ESP_RETURN_ON_ERROR(analogWriteResolution(SERVO_PWM_BITS), TAG, "Gagal mengatur resolusi PWM servo");
    }
    return set_lane(LANE_GREEN);
}

static esp_err_t radio_init(void)
{
    lora_config_t cfg = LORA_CONFIG_DEFAULT(PIN_LORA_MOSI, PIN_LORA_MISO, PIN_LORA_SCK,
                                            PIN_LORA_NSS, PIN_LORA_RST, PIN_LORA_DIO0);
    cfg.frequency_hz = LORA_FREQ_HZ;
    cfg.sf = LORA_SF6;
    cfg.bw = LORA_BW_500K;
    cfg.cr = LORA_CR_4_5;
    cfg.preamble_len = 6;
    cfg.crc = true;
    cfg.header_mode = LORA_HEADER_IMPLICIT;
    cfg.implicit_payload_len = PKT_LEN;
    cfg.ldro = LORA_LDRO_OFF;
    cfg.tx_power_mw = LORA_TX_POWER_MW;
    
    return lora_begin(&cfg, &s_radio);
}

void app_main(void)
{
    
    ESP_ERROR_CHECK(init_pins());
    s_cmd_q = xQueueCreate(4, sizeof(uint8_t));
    configASSERT(s_cmd_q);

    ESP_ERROR_CHECK(radio_init());
    ESP_ERROR_CHECK(lora_start_receive(s_radio, on_rx, NULL));

    BaseType_t ok = xTaskCreate(mechanism_task, "pantry_mechanism", 4096, NULL, 5, NULL);
    configASSERT(ok == pdPASS);
    ESP_LOGI(TAG, "PRS Pantry Node %u siap: ring 1->2->5->1; paket %u byte",
             (unsigned)s_node_id, (unsigned)PKT_LEN);
    ESP_LOGW(TAG, "Mode trial: mekanisme %s, landing melalui jumper GPIO21",
             !PRS_PHYSICAL_ENABLED ? "VIRTUAL TANPA MOTOR/SERVO" : "FISIK");
}