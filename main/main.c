#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "lora.h"
#include "easy_gpio.h"
#include "button.h"

static const char *TAG = "prs_bb";

// pinout esp32-c3 supermini
#define PIN_LORA_SCK    4
#define PIN_LORA_MISO   5
#define PIN_LORA_MOSI   6
#define PIN_LORA_NSS    7
#define PIN_LORA_RST    8
#define PIN_LORA_DIO0   2

#define PIN_SR_LATCH    0
#define PIN_SR_CLK      3
#define PIN_SR_DATA     10
#define PIN_EN_MOTOR    1

#define PIN_SERVO       20
#define PIN_SW_DOCK     21  // aktif LOW
#define PIN_BTN_MANUAL  9   // tap: tes servo, hold: reset

#define TOTAL_SLOT      12
#define DOCK_SIDES      6
#define PULL_MS         1500
#define MOTOR_PWM_MAX   8191

// kalibrasi pulsa servo (beda 30 derajat antar jalur)
#define SERVO_US_GREEN  1200
#define SERVO_US_RED    1533
#define SERVO_DELAY_MS  400

#define NODE_ID         "PRS_BB"
#define PREV_ID         "MAIN"

enum {
    LANE_GREEN = 0, // jalur genap (0, 2, 4, 6, 8, 10)
    LANE_RED   = 1, // jalur ganjil (1, 3, 5, 7, 9, 11)
    LANE_NONE  = 2
};

static lora_t *lora = NULL;
static uint16_t sr_mask = 0;

static bool slot_used[TOTAL_SLOT];
static bool side_used[DOCK_SIDES];

static volatile bool is_docked = false;
static volatile bool is_pulling = false;
static volatile int cur_slot = -1;
static volatile int cur_side = -1;
static uint32_t t_pull_start = 0;

static uint8_t cur_lane = LANE_NONE;
static int last_slot = 0;
static int last_side = 0;
static uint8_t last_lane = LANE_NONE;

static void sr_write(uint16_t val) {
    digitalWrite(PIN_SR_LATCH, LOW);
    shiftOut(PIN_SR_DATA, PIN_SR_CLK, MSBFIRST, (uint8_t)(val >> 8));
    shiftOut(PIN_SR_DATA, PIN_SR_CLK, MSBFIRST, (uint8_t)(val & 0xFF));
    digitalWrite(PIN_SR_LATCH, HIGH);
}

static void motor_set(int idx, bool on) {
    if (idx < 0 || idx >= TOTAL_SLOT) return;

    if (on) {
        sr_mask |= (1U << idx);
        sr_write(sr_mask);
        analogWrite(PIN_EN_MOTOR, MOTOR_PWM_MAX);
    } else {
        sr_mask &= ~(1U << idx);
        sr_write(sr_mask);
        if (sr_mask == 0) {
            analogWrite(PIN_EN_MOTOR, 0);
        }
    }
}

static void motors_off(void) {
    analogWrite(PIN_EN_MOTOR, 0);
    sr_mask = 0;
    sr_write(0);
    is_pulling = false;
    cur_slot = -1;
    cur_side = -1;
}

static void servo_write_us(uint32_t us) {
    analogWrite(PIN_SERVO, (us * 8191) / 20000);
}

static void set_lane(uint8_t lane) {
    if (cur_lane == lane) return;

    ESP_LOGI(TAG, "servo -> %s", lane == LANE_GREEN ? "hijau" : "merah");
    servo_write_us(lane == LANE_GREEN ? SERVO_US_GREEN : SERVO_US_RED);
    delay(SERVO_DELAY_MS);
    cur_lane = lane;
}

static void tx_status(void) {
    int rem_g = 0, rem_r = 0, mag_cnt = 0;

    for (int i = 0; i < DOCK_SIDES; i++) {
        if (!slot_used[i * 2]) rem_g++;
        if (!slot_used[i * 2 + 1]) rem_r++;
        if (side_used[i]) mag_cnt++;
    }

    char c = (last_lane == LANE_GREEN) ? 'G' : (last_lane == LANE_RED) ? 'R' : '-';
    char buf[96];
    snprintf(buf, sizeof(buf), "%s|DOCK:%d|TRK:%c|SLOT:%d|SIDE:%d|MAG:%d/%d|REM:G%d_R%d",
             NODE_ID, is_docked ? 1 : 0, c, last_slot, last_side,
             mag_cnt, DOCK_SIDES, rem_g, rem_r);

    lora_send_string(lora, buf);
    ESP_LOGI(TAG, "tx: %s", buf);
}

static void btn_tap_cb(gpio_num_t pin, uint8_t count, void *arg) {
    set_lane(cur_lane == LANE_GREEN ? LANE_RED : LANE_GREEN);
}

static void btn_hold_cb(gpio_num_t pin, uint32_t ms, void *arg) {
    if (is_pulling) motors_off();
    memset(slot_used, 0, sizeof(slot_used));
    memset(side_used, 0, sizeof(side_used));
    last_slot = 0;
    last_side = 0;
    last_lane = LANE_NONE;
    ESP_LOGI(TAG, "reset slot & mag");
}

static void start_pull(uint8_t lane) {
    if (is_pulling) {
        ESP_LOGW(TAG, "motor sibuk");
        return;
    }

    set_lane(lane);

    // tiap sisi dock (0..5) sejajar dengan slot (side*2 + lane)
    int target_side = -1;
    int target_slot = -1;
    for (int s = 0; s < DOCK_SIDES; s++) {
        int slot = (s * 2) + lane;
        if (!side_used[s] && !slot_used[slot]) {
            target_side = s;
            target_slot = slot;
            break;
        }
    }

    if (target_slot < 0) {
        ESP_LOGW(TAG, "stok %s habis / mag penuh", lane == LANE_GREEN ? "hijau" : "merah");
        return;
    }

    cur_side = target_side;
    cur_slot = target_slot;
    t_pull_start = millis();
    is_pulling = true;

    ESP_LOGI(TAG, "tarik slot %d (%d deg) -> side %d",
             target_slot + 1, target_slot * 30, target_side + 1);
    motor_set(target_slot, true);
}

static void motor_task(void *arg) {
    while (1) {
        if (!is_docked) {
            if (is_pulling) {
                ESP_LOGW(TAG, "drone lepas, stop");
                motors_off();
            }
            delay(20);
            continue;
        }

        if (is_pulling && cur_slot >= 0) {
            if ((millis() - t_pull_start) >= PULL_MS) {
                int s = cur_slot;
                int side = cur_side;
                motor_set(s, false);

                slot_used[s] = true;
                side_used[side] = true;
                last_slot = s + 1;
                last_side = side + 1;
                last_lane = s % 2;

                is_pulling = false;
                cur_slot = -1;
                cur_side = -1;
                ESP_LOGI(TAG, "done slot %d -> side %d", s + 1, side + 1);
            }
        }
        delay(10);
    }
}

static void lora_task(void *arg) {
    lora_packet_t pkt;
    bool prev_dock = false;

    while (1) {
        is_docked = (digitalRead(PIN_SW_DOCK) == LOW);

        if (is_docked != prev_dock) {
            prev_dock = is_docked;
            if (is_docked) {
                memset(side_used, 0, sizeof(side_used));
                ESP_LOGI(TAG, "docked");
            } else {
                motors_off();
                ESP_LOGI(TAG, "airborne");
            }
        }

        if (lora_receive(lora, &pkt, 50) == ESP_OK && pkt.len > 0) {
            char buf[64];
            snprintf(buf, sizeof(buf), "%.*s", pkt.len, pkt.data);

            if (strncmp(buf, PREV_ID, strlen(PREV_ID)) == 0) {
                tx_status();
            }

            if (is_docked) {
                if (strstr(buf, "CMD_PULL_GREEN") || strcmp(buf, "2") == 0) {
                    start_pull(LANE_GREEN);
                } else if (strstr(buf, "CMD_PULL_RED") || strcmp(buf, "7") == 0) {
                    start_pull(LANE_RED);
                } else if (strstr(buf, "CMD_RESET_MAG")) {
                    memset(side_used, 0, sizeof(side_used));
                }
            }
        }
        delay(10);
    }
}

static void init_pins(void) {
    pinMode(PIN_SW_DOCK, INPUT_PULLUP);

    pinMode(PIN_SR_DATA, OUTPUT);
    pinMode(PIN_SR_CLK, OUTPUT);
    pinMode(PIN_SR_LATCH, OUTPUT);
    digitalWrite(PIN_SR_LATCH, LOW);

    analogWriteFrequency(50);
    analogWriteResolution(13);
    analogWrite(PIN_EN_MOTOR, 0);

    motors_off();
    memset(slot_used, 0, sizeof(slot_used));
    memset(side_used, 0, sizeof(side_used));

    set_lane(LANE_GREEN);

    button_on_tap((gpio_num_t)PIN_BTN_MANUAL, 1, btn_tap_cb, NULL, false);
    button_on_hold((gpio_num_t)PIN_BTN_MANUAL, 3000, btn_hold_cb, NULL, false);
}

void app_main(void) {
    init_pins();

    lora_config_t cfg = LORA_CONFIG_DEFAULT(
        PIN_LORA_MOSI, PIN_LORA_MISO, PIN_LORA_SCK,
        PIN_LORA_NSS, PIN_LORA_RST, PIN_LORA_DIO0
    );
    cfg.frequency_hz = 433000000;
    cfg.sf = LORA_SF7;
    cfg.bw = LORA_BW_125K;
    cfg.cr = LORA_CR_4_5;
    cfg.sync_word = 0x16;
    cfg.tx_power_dbm = 14;

    if (lora_begin(&cfg, &lora) != ESP_OK) {
        ESP_LOGE(TAG, "lora init err");
        return;
    }

    xTaskCreate(motor_task, "motor_task", 3072, NULL, 5, NULL);
    xTaskCreate(lora_task, "lora_task", 4096, NULL, 5, NULL);
}