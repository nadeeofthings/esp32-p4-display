#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_ldo_regulator.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "bsp_board_extra.h"
#include "lvgl.h"

static const char *TAG = "ME231_DISPLAY";

// Native high-resolution fonts
extern const lv_font_t font_bold_320;
extern const lv_font_t font_splash_140;

// RS485 & UART Hardware Configurations (ESP32-P4 onboard RS485 transceiver)
#define RS485_UART_PORT         (UART_NUM_1)
#define RS485_TXD_PIN           (27)
#define RS485_RXD_PIN           (26)
#define RS485_BAUD_RATE         (19200)
#define RS485_BUF_SIZE          (512)

// Modbus RTU Parameters (Meatrol ME231)
#define MODBUS_SLAVE_ID         (1)
#define MODBUS_START_ADDR       (1006)
#define MODBUS_REG_COUNT        (22)
#define MODBUS_POLL_INTERVAL_MS (100) // 10Hz (every 100ms)

// Global LVGL UI elements and last known good values (default 0.0V and 0.0A)
static lv_obj_t *voltage_label = NULL;
static lv_obj_t *current_label = NULL;

static float g_last_voltage = 0.0f;
static float g_last_current = 0.0f;

// Enable LDO VO4 (3.3V) for RS485 transceiver on Waveshare ESP32-P4 board
static esp_err_t bsp_enable_ldo_vo4(void)
{
    static esp_ldo_channel_handle_t vo4_chan = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = 4,
        .voltage_mv = 3300,
    };
    esp_err_t err = esp_ldo_acquire_channel(&ldo_cfg, &vo4_chan);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "LDO VO4 enabled at 3300mV for RS485 transceiver");
    } else {
        ESP_LOGE(TAG, "Failed to enable LDO VO4: %s", esp_err_to_name(err));
    }
    return err;
}

// Compute Modbus RTU CRC-16 (Polynomial 0xA001)
static uint16_t modbus_crc16(const uint8_t *buffer, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)buffer[i];
        for (int j = 8; j != 0; j--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Initialize UART for Modbus RTU RS485 (19200 Baud, 8N1)
static void init_rs485_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = RS485_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(RS485_UART_PORT, RS485_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RS485_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART_PORT, RS485_TXD_PIN, RS485_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "RS485 UART initialized (19200 Baud, 8N1, TX Pin %d, RX Pin %d)", RS485_TXD_PIN, RS485_RXD_PIN);
}

// Task: Polls Meatrol ME231 via Modbus RTU at 10Hz (every 100ms)
static void modbus_reader_task(void *arg)
{
    uint8_t req[8];
    req[0] = MODBUS_SLAVE_ID;                               // Slave ID: 1
    req[1] = 0x03;                                           // Read Holding Registers
    req[2] = (uint8_t)((MODBUS_START_ADDR >> 8) & 0xFF);    // Register 1006 High byte (0x03)
    req[3] = (uint8_t)(MODBUS_START_ADDR & 0xFF);           // Register 1006 Low byte  (0xEE)
    req[4] = (uint8_t)((MODBUS_REG_COUNT >> 8) & 0xFF);     // Count 22 High byte      (0x00)
    req[5] = (uint8_t)(MODBUS_REG_COUNT & 0xFF);            // Count 22 Low byte       (0x16)

    uint16_t req_crc = modbus_crc16(req, 6);
    req[6] = (uint8_t)(req_crc & 0xFF);                     // CRC Low
    req[7] = (uint8_t)((req_crc >> 8) & 0xFF);              // CRC High

    uint8_t rx_buf[RS485_BUF_SIZE];
    const int expected_rx_len = 3 + (MODBUS_REG_COUNT * 2) + 2; // 3 header + 44 data + 2 CRC = 49 bytes

    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        // Flush any lingering RX bytes before sending new request
        uart_flush_input(RS485_UART_PORT);

        // Send Modbus RTU Query
        uart_write_bytes(RS485_UART_PORT, (const char *)req, sizeof(req));

        // Read response with 60ms timeout
        int len = uart_read_bytes(RS485_UART_PORT, rx_buf, expected_rx_len, pdMS_TO_TICKS(60));

        bool valid_packet = false;
        float voltage_val = 0.0f;
        float current_val = 0.0f;

        if (len >= expected_rx_len) {
            // Check Slave ID, Function Code, and Byte Count
            if (rx_buf[0] == MODBUS_SLAVE_ID && rx_buf[1] == 0x03 && rx_buf[2] == (MODBUS_REG_COUNT * 2)) {
                // Verify CRC-16
                uint16_t rx_crc = modbus_crc16(rx_buf, expected_rx_len - 2);
                uint16_t pkt_crc = (uint16_t)rx_buf[expected_rx_len - 2] | ((uint16_t)rx_buf[expected_rx_len - 1] << 8);

                if (rx_crc == pkt_crc) {
                    // Extract Average Current (Registers 1006 & 1007 -> offset 0 & 1 -> data bytes 0..3)
                    uint32_t current_raw = ((uint32_t)rx_buf[3] << 24) |
                                           ((uint32_t)rx_buf[4] << 16) |
                                           ((uint32_t)rx_buf[5] << 8)  |
                                           ((uint32_t)rx_buf[6]);
                    memcpy(&current_val, &current_raw, sizeof(float));

                    // Extract Average Voltage (Registers 1026 & 1027 -> offset 20 & 21 -> data bytes 40..43 -> rx_buf[43..46])
                    uint32_t voltage_raw = ((uint32_t)rx_buf[43] << 24) |
                                           ((uint32_t)rx_buf[44] << 16) |
                                           ((uint32_t)rx_buf[45] << 8)  |
                                           ((uint32_t)rx_buf[46]);
                    memcpy(&voltage_val, &voltage_raw, sizeof(float));

                    // Sanity check numeric values (not NaN or Inf)
                    if (!isnan(current_val) && !isinf(current_val) &&
                        !isnan(voltage_val) && !isinf(voltage_val)) {
                        valid_packet = true;
                    }
                } else {
                    ESP_LOGW(TAG, "Modbus CRC mismatch (calc: 0x%04X, pkt: 0x%04X)", rx_crc, pkt_crc);
                }
            } else {
                ESP_LOGW(TAG, "Modbus response header invalid [0x%02X, 0x%02X, 0x%02X]", rx_buf[0], rx_buf[1], rx_buf[2]);
            }
        } else {
            ESP_LOGW(TAG, "Modbus read timeout or partial packet (rx len: %d / %d)", len, expected_rx_len);
        }

        // Failsafe Logic: If packet dropped or invalid, retain last known good values
        if (valid_packet) {
            g_last_voltage = voltage_val;
            g_last_current = current_val;
        } else {
            ESP_LOGW(TAG, "Retaining last known good values (V: %.1f, A: %.1f)", g_last_voltage, g_last_current);
        }

        // Format strings using standard C snprintf
        char v_str[32];
        char a_str[32];
        snprintf(v_str, sizeof(v_str), "%.1fV", g_last_voltage);
        snprintf(a_str, sizeof(a_str), "%.1fA", g_last_current);

        // Update LVGL UI with thread-safety lock
        if (voltage_label != NULL && current_label != NULL) {
            if (bsp_display_lock(100)) {
                lv_label_set_text(voltage_label, v_str);
                lv_label_set_text(current_label, a_str);
                bsp_display_unlock();
            }
        }

        // Maintain strict 10Hz (100ms) polling rate
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MODBUS_POLL_INTERVAL_MS));
    }
}

// Setup Main User Interface: 2 equal rows dividing full screen height, centered + 20px down offset, 320px bold font
static void create_main_ui(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);

    // Dark background for maximum contrast
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // Top Row Container (Voltage) - 50% screen height
    lv_obj_t *row_v = lv_obj_create(scr);
    lv_obj_set_size(row_v, LV_PCT(100), LV_PCT(50));
    lv_obj_align(row_v, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(row_v, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row_v, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(row_v, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(row_v, 0, LV_PART_MAIN);
    lv_obj_clear_flag(row_v, LV_OBJ_FLAG_SCROLLABLE);

    // Row 1: Voltage Label (Yellow text #FFFF00, native crisp 320px font, centered + 20px down)
    voltage_label = lv_label_create(row_v);
    lv_obj_set_style_text_color(voltage_label, lv_color_make(0xFF, 0xFF, 0x00), LV_PART_MAIN);
    lv_obj_set_style_text_font(voltage_label, &font_bold_320, LV_PART_MAIN);
    lv_label_set_text(voltage_label, "0.0V");
    lv_obj_align(voltage_label, LV_ALIGN_CENTER, 0, 20);

    // Bottom Row Container (Current) - 50% screen height
    lv_obj_t *row_a = lv_obj_create(scr);
    lv_obj_set_size(row_a, LV_PCT(100), LV_PCT(50));
    lv_obj_align(row_a, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(row_a, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row_a, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(row_a, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(row_a, 0, LV_PART_MAIN);
    lv_obj_clear_flag(row_a, LV_OBJ_FLAG_SCROLLABLE);

    // Row 2: Current Label (Cyan text #00FFFF, native crisp 320px font, centered + 20px down)
    current_label = lv_label_create(row_a);
    lv_obj_set_style_text_color(current_label, lv_color_make(0x00, 0xFF, 0xFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(current_label, &font_bold_320, LV_PART_MAIN);
    lv_label_set_text(current_label, "0.0A");
    lv_obj_align(current_label, LV_ALIGN_CENTER, 0, 20);

    ESP_LOGI(TAG, "Main UI initialized: 2 equal centered rows (320px font, +20px vertical offset)");
}

// Startup Sequence Task: RGB Spill Test -> Fade In Splash -> Hold 5s -> Fade Out Splash -> Main UI
static void startup_sequence_task(void *arg)
{
    // PHASE 1: RGB Screen Spill Test (0.5s Red, 0.5s Green, 0.5s Blue)
    ESP_LOGI(TAG, "Starting RGB Screen Spill Test...");
    lv_obj_t *scr = lv_scr_act();

    // 1A. Red Screen (0.5s)
    if (bsp_display_lock(-1)) {
        lv_obj_set_style_bg_color(scr, lv_color_make(0xFF, 0x00, 0x00), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
        bsp_display_unlock();
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    // 1B. Green Screen (0.5s)
    if (bsp_display_lock(-1)) {
        lv_obj_set_style_bg_color(scr, lv_color_make(0x00, 0xFF, 0x00), LV_PART_MAIN);
        bsp_display_unlock();
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    // 1C. Blue Screen (0.5s)
    if (bsp_display_lock(-1)) {
        lv_obj_set_style_bg_color(scr, lv_color_make(0x00, 0x00, 0xFF), LV_PART_MAIN);
        bsp_display_unlock();
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    // PHASE 2: Splash Screen ("INDUSTRIAL ELECTRONIC SYSTEMS" in 3 rows in white, size 140px)
    ESP_LOGI(TAG, "Starting Splash Screen Fade In (140px font)...");

    lv_obj_t *splash_cnt = NULL;
    if (bsp_display_lock(-1)) {
        // Reset screen background to Black
        lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
        lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

        // Container for Splash Screen
        splash_cnt = lv_obj_create(scr);
        lv_obj_set_size(splash_cnt, LV_PCT(100), LV_PCT(100));
        lv_obj_align(splash_cnt, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_color(splash_cnt, lv_color_black(), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(splash_cnt, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_width(splash_cnt, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(splash_cnt, 0, LV_PART_MAIN);
        lv_obj_clear_flag(splash_cnt, LV_OBJ_FLAG_SCROLLABLE);

        // Center 3 rows using Flex Column
        lv_obj_set_flex_flow(splash_cnt, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(splash_cnt, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_gap(splash_cnt, 8, LV_PART_MAIN);

        // Row 1: INDUSTRIAL
        lv_obj_t *lbl1 = lv_label_create(splash_cnt);
        lv_obj_set_style_text_color(lbl1, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_text_font(lbl1, &font_splash_140, LV_PART_MAIN);
        lv_label_set_text(lbl1, "INDUSTRIAL");

        // Row 2: ELECTRONIC
        lv_obj_t *lbl2 = lv_label_create(splash_cnt);
        lv_obj_set_style_text_color(lbl2, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_text_font(lbl2, &font_splash_140, LV_PART_MAIN);
        lv_label_set_text(lbl2, "ELECTRONIC");

        // Row 3: SYSTEMS
        lv_obj_t *lbl3 = lv_label_create(splash_cnt);
        lv_obj_set_style_text_color(lbl3, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_text_font(lbl3, &font_splash_140, LV_PART_MAIN);
        lv_label_set_text(lbl3, "SYSTEMS");

        // Start fully transparent
        lv_obj_set_style_opa(splash_cnt, LV_OPA_TRANSP, LV_PART_MAIN);

        bsp_display_unlock();
    }

    // Smooth Fade In (1.0s)
    for (int opa = 0; opa <= 255; opa += 15) {
        if (bsp_display_lock(-1)) {
            lv_obj_set_style_opa(splash_cnt, (lv_opa_t)opa, LV_PART_MAIN);
            bsp_display_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (bsp_display_lock(-1)) {
        lv_obj_set_style_opa(splash_cnt, LV_OPA_COVER, LV_PART_MAIN);
        bsp_display_unlock();
    }

    // Hold Splash Screen for 5 seconds
    ESP_LOGI(TAG, "Splash Screen visible (holding 5s)...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Smooth Fade Out (1.0s)
    ESP_LOGI(TAG, "Starting Splash Screen Fade Out...");
    for (int opa = 255; opa >= 0; opa -= 15) {
        if (bsp_display_lock(-1)) {
            lv_obj_set_style_opa(splash_cnt, (lv_opa_t)opa, LV_PART_MAIN);
            bsp_display_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Clean up Splash Screen container
    if (bsp_display_lock(-1)) {
        lv_obj_del(splash_cnt);

        // PHASE 3: Initialize Main Measurement UI
        create_main_ui();

        bsp_display_unlock();
    }

    // PHASE 4: Start Modbus Reader Task (Priority 5, Stack 4096)
    ESP_LOGI(TAG, "Startup sequence complete. Starting Modbus Reader task...");
    xTaskCreate(modbus_reader_task, "modbus_reader", 4096, NULL, 5, NULL);

    // Delete startup task
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Waveshare ESP32-P4 Meatrol ME231 Display Project");

    // Enable power LDO VO4 (3.3V) for onboard RS485 transceiver
    bsp_enable_ldo_vo4();

    // Initialize RS485 UART (19200 Baud, 8N1, TX: 27, RX: 26)
    init_rs485_uart();

    // Initialize Display BSP for Waveshare 7-inch LCD
    lv_disp_t *disp = bsp_display_start();
    (void)disp;
    bsp_display_backlight_on();

    // Launch Startup Task (RGB Test -> Splash Screen 5s -> Main UI)
    xTaskCreate(startup_sequence_task, "startup_task", 4096, NULL, 5, NULL);
}
