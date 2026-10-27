#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "lora.h"

static const char *TAG = "PRS_BEANBAG";

/* ========================================================================= */
/*  KONFIGURASI HARDWARE & PIN                                               */
/* ========================================================================= */

#define LORA_PIN_MOSI       6
#define LORA_PIN_MISO       5
#define LORA_PIN_SCK        4
#define LORA_PIN_NSS        7
#define LORA_PIN_RST        10
#define LORA_PIN_DIO0       3

#define I2C_MASTER_SDA_IO   1
#define I2C_MASTER_SCL_IO   2
#define I2C_MASTER_FREQ_HZ  400000

#define MCP23017_ADDR_MOTORS    0x20  // MCP #1: Output 12 Motor DC
#define MCP23017_ADDR_SWITCHES  0x21  // MCP #2: Input 12 Limit Switch Video

#define MCP_IODIRA   0x00
#define MCP_IODIRB   0x01
#define MCP_GPPUA    0x0C
#define MCP_GPPUB    0x0D
#define MCP_GPIOA    0x12
#define MCP_GPIOB    0x13
#define MCP_OLATA    0x14
#define MCP_OLATB    0x15

#define PIN_DRONE_LIMIT_SW   GPIO_NUM_0   // Limit Switch Docking Drone (Active LOW)
#define PIN_SERVO_PADDOCK    GPIO_NUM_3   // Servo Rotator Dock (PWM LEDC)
#define TOTAL_BEANBAGS       12

typedef enum {
    COLOR_GREEN = 0, // Gula (Pedal Kanan)
    COLOR_RED   = 1  // Tepung (Pedal Kiri)
} beanbag_color_t;

typedef struct {
    bool is_drone_docked;
    bool slot_empty[TOTAL_BEANBAGS];
    beanbag_color_t slot_color[TOTAL_BEANBAGS]; // Array warna tiap slot (Hijau/Merah)
    int current_active_slot;                    // Slot yang sedang ditarik motor
    bool is_motor_pulling;
} prs_state_t;

static i2c_master_bus_handle_t i2c_bus_handle;
static i2c_master_dev_handle_t mcp_motors_handle;
static i2c_master_dev_handle_t mcp_switches_handle;

static prs_state_t g_prs;
static lora_t *g_radio = NULL;

/* ========================================================================= */
/*  DRIVER MCP23017 (I2C)                                                     */
/* ========================================================================= */

static esp_err_t mcp_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_transmit(dev, write_buf, sizeof(write_buf), -1);
}

static esp_err_t mcp_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data) {
    return i2c_master_transmit_receive(dev, &reg, 1, data, 1, -1);
}

static void mcp23017_init_all(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));

    i2c_device_config_t dev_cfg_m = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MCP23017_ADDR_MOTORS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg_m, &mcp_motors_handle));

    i2c_device_config_t dev_cfg_s = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MCP23017_ADDR_SWITCHES,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg_s, &mcp_switches_handle));

    // MCP #1 Motors (Output, Matikan Semua)
    mcp_write_reg(mcp_motors_handle, MCP_IODIRA, 0x00);
    mcp_write_reg(mcp_motors_handle, MCP_IODIRB, 0x00);
    mcp_write_reg(mcp_motors_handle, MCP_OLATA,  0x00);
    mcp_write_reg(mcp_motors_handle, MCP_OLATB,  0x00);

    // MCP #2 Switches Video (Input + Internal Pull-Up)
    mcp_write_reg(mcp_switches_handle, MCP_IODIRA, 0xFF);
    mcp_write_reg(mcp_switches_handle, MCP_IODIRB, 0xFF);
    mcp_write_reg(mcp_switches_handle, MCP_GPPUA,  0xFF);
    mcp_write_reg(mcp_switches_handle, MCP_GPPUB,  0xFF);

    ESP_LOGI(TAG, "2x MCP23017 Berhasil Diinisialisasi!");
}

static void set_mcp_motor_state(uint8_t slot, bool turn_on) {
    static uint16_t current_mask = 0x0000;
    if (slot >= TOTAL_BEANBAGS) return;

    if (turn_on) {
        current_mask |= (1 << slot);
    } else {
        current_mask &= ~(1 << slot);
    }

    mcp_write_reg(mcp_motors_handle, MCP_OLATA, (uint8_t)(current_mask & 0xFF));
    mcp_write_reg(mcp_motors_handle, MCP_OLATB, (uint8_t)((current_mask >> 8) & 0xFF));
}

static void stop_all_mcp_motors(void) {
    mcp_write_reg(mcp_motors_handle, MCP_OLATA, 0x00);
    mcp_write_reg(mcp_motors_handle, MCP_OLATB, 0x00);
    g_prs.is_motor_pulling = false;
    g_prs.current_active_slot = -1;
}

static bool read_mcp_video_switch(uint8_t slot) {
    if (slot >= TOTAL_BEANBAGS) return false;

    uint8_t port_a = 0xFF, port_b = 0xFF;
    mcp_read_reg(mcp_switches_handle, MCP_GPIOA, &port_a);
    mcp_read_reg(mcp_switches_handle, MCP_GPIOB, &port_b);

    uint16_t all_switches = (port_b << 8) | port_a;
    return ((all_switches & (1 << slot)) == 0); // Active LOW
}

/* ========================================================================= */
/*  SERVO PADDOCK & HARDWARE INIT                                            */
/* ========================================================================= */

static void paddock_servo_init(void) {
    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num         = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .gpio_num       = PIN_SERVO_PADDOCK,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&channel_conf);
}

static void paddock_servo_set_us(uint32_t pulse_us) {
    uint32_t duty = (pulse_us * 8191) / 20000;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void prs_hardware_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_DRONE_LIMIT_SW),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    paddock_servo_init();
    paddock_servo_set_us(1500); // Neutral

    mcp23017_init_all();
    memset(&g_prs, 0, sizeof(prs_state_t));

    // Inisialisasi susunan warna bergantian (Hijau = Genap, Merah = Ganjil)
    // Cek fisik switch saat boot: kalau switch tertekan, tandai slot langsung kosong
    for (int i = 0; i < TOTAL_BEANBAGS; i++) {
        g_prs.slot_color[i] = (i % 2 == 0) ? COLOR_GREEN : COLOR_RED;
        g_prs.slot_empty[i] = read_mcp_video_switch(i);
    }
    g_prs.current_active_slot = -1;
}

static inline bool check_drone_docked(void) {
    return (gpio_get_level(PIN_DRONE_LIMIT_SW) == 0);
}

/* ========================================================================= */
/*  LOGIKA PENARIKAN BERDASARKAN WARNA (PEDAL KANAN vs KIRI)                 */
/* ========================================================================= */

static void align_dock_for_slot(int slot_index) {
    ESP_LOGI(TAG, "[DOCK] Menyesuaikan posisi dock ke Slot #%d...", slot_index + 1);
}

static void execute_rotate_paddock(void) {
    ESP_LOGI(TAG, "[ACTION] Dual Tap: Memutar Servo Paddock...");
    paddock_servo_set_us(2000);
    vTaskDelay(pdMS_TO_TICKS(1200));
    paddock_servo_set_us(1500);
    ESP_LOGI(TAG, "[ACTION] Selesai Putar Paddock.");
}

static void start_pulling_material_by_color(beanbag_color_t target_color) {
    if (g_prs.is_motor_pulling) {
        ESP_LOGW(TAG, "Masih ada proses penarikan berlangsung!");
        return;
    }

    int target_slot = -1;

    for (int i = 0; i < TOTAL_BEANBAGS; i++) {
        if (g_prs.slot_color[i] == target_color && !g_prs.slot_empty[i]) {
            target_slot = i;
            break;
        }
    }

    if (target_slot == -1) {
        ESP_LOGW(TAG, "Bahan warna %s sudah habis!", (target_color == COLOR_GREEN) ? "HIJAU (Gula)" : "MERAH (Tepung)");
        return;
    }

    align_dock_for_slot(target_slot);

    g_prs.current_active_slot = target_slot;
    g_prs.is_motor_pulling = true;
    
    ESP_LOGI(TAG, "[ACTION] Pedal %s -> Menarik Bahan Slot #%d (%s) ke tengah...",
             (target_color == COLOR_GREEN) ? "KANAN" : "KIRI",
             target_slot + 1,
             (target_color == COLOR_GREEN) ? "HIJAU" : "MERAH");

    set_mcp_motor_state(target_slot, true);
}

/* ========================================================================= */
/*  FREERTOS TASKS                                                           */
/* ========================================================================= */

static void prs_motor_monitor_task(void *pvParameters) {
    while (1) {
        if (!g_prs.is_drone_docked) {
            if (g_prs.is_motor_pulling) {
                ESP_LOGW(TAG, "[SAFETY] Drone terbang! Mematikan motor.");
                stop_all_mcp_motors();
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (g_prs.is_motor_pulling && g_prs.current_active_slot != -1) {
            uint8_t slot = g_prs.current_active_slot;

            if (read_mcp_video_switch(slot)) {
                set_mcp_motor_state(slot, false);
                g_prs.is_motor_pulling = false;
                g_prs.slot_empty[slot] = true;

                ESP_LOGI(TAG, "[SUCCESS] Slot #%d (%s) menekan switch video! Motor STOP.", 
                         slot + 1, (g_prs.slot_color[slot] == COLOR_GREEN) ? "HIJAU" : "MERAH");
                
                g_prs.current_active_slot = -1;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void lora_prs_task(void *pvParameters) {
    lora_packet_t pkt;
    bool last_dock_status = false;

    while (1) {
        g_prs.is_drone_docked = check_drone_docked();

        if (g_prs.is_drone_docked != last_dock_status) {
            last_dock_status = g_prs.is_drone_docked;
            if (g_prs.is_drone_docked) {
                ESP_LOGI(TAG, ">>> DRONE DOCKED (PRS Active) <<<");
                lora_send_string(g_radio, "PRS_BB:ACTIVE");
            } else {
                ESP_LOGI(TAG, ">>> DRONE AIRBORNE <<<");
                stop_all_mcp_motors();
                lora_send_string(g_radio, "PRS_BB:IDLE");
            }
        }

        esp_err_t err = lora_receive(g_radio, &pkt, 50);
        if (err == ESP_OK && pkt.len > 0) {
            char cmd_str[32];
            snprintf(cmd_str, sizeof(cmd_str), "%.*s", pkt.len, pkt.data);

            if (g_prs.is_drone_docked) {
                if (strcmp(cmd_str, "CMD_ROTATE") == 0 || strcmp(cmd_str, "3") == 0) {
                    execute_rotate_paddock();
                }
                else if (strcmp(cmd_str, "CMD_PULL_GREEN") == 0 || strcmp(cmd_str, "2") == 0) {
                    start_pulling_material_by_color(COLOR_GREEN);
                }
                else if (strcmp(cmd_str, "CMD_PULL_RED") == 0 || strcmp(cmd_str, "7") == 0) {
                    start_pulling_material_by_color(COLOR_RED);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ========================================================================= */
/*  MAIN ENTRY POINT                                                         */
/* ========================================================================= */

void app_main(void) {
    ESP_LOGI(TAG, "=== STARTING PRS BEANBAG GEN-5 ===");

    prs_hardware_init();

    lora_config_t lora_cfg = LORA_CONFIG_DEFAULT(
        LORA_PIN_MOSI, LORA_PIN_MISO, LORA_PIN_SCK,
        LORA_PIN_NSS, LORA_PIN_RST, LORA_PIN_DIO0
    );
    lora_cfg.frequency_hz = 433000000;
    lora_cfg.sf           = LORA_SF7;
    lora_cfg.bw           = LORA_BW_125K;
    lora_cfg.cr           = LORA_CR_4_5;
    lora_cfg.sync_word    = 0x16;
    lora_cfg.tx_power_dbm = 14;

    esp_err_t err = lora_begin(&lora_cfg, &g_radio);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gagal Inisialisasi LoRa! Error: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "LoRa SX1278 Berhasil Dimulai!");

    xTaskCreate(prs_motor_monitor_task, "prs_motor_task", 3072, NULL, 5, NULL);
    xTaskCreate(lora_prs_task,          "lora_prs_task",   4096, NULL, 5, NULL);
}