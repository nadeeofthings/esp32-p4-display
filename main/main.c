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
#include "nvs_flash.h"
#include "nvs.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "bsp_board_extra.h"
#include "lvgl.h"

static const char *TAG = "ME231_DISPLAY";
#define NVS_NAMESPACE "disp_cfg"

// Native high-resolution fonts
extern const lv_font_t font_bold_320;
extern const lv_font_t font_splash_140;
extern const lv_font_t font_menu_32;

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

// Global LVGL UI elements and color configurations
static lv_obj_t *voltage_label = NULL;
static lv_obj_t *current_label = NULL;
static lv_obj_t *settings_modal = NULL;

static float g_last_voltage = 0.0f;
static float g_last_current = 0.0f;

// Color configurations (24-bit RGB hex values)
static uint32_t g_voltage_rgb = 0xFFFF00; // Default Yellow
static uint32_t g_current_rgb = 0x00FFFF; // Default Cyan

// Temporary colors edited in settings menu
static uint32_t temp_voltage_rgb = 0xFFFF00;
static uint32_t temp_current_rgb = 0x00FFFF;

// Triple-tap detection variables
static uint32_t g_tap_count = 0;
static uint32_t g_last_tap_ms = 0;

// High-saturation color palette (9 choices)
static const uint32_t PALETTE_COLORS[] = {
    0xFF0000, // Red
    0xFF7F00, // Orange
    0xFFFF00, // Yellow
    0x00FF00, // Green
    0x00FFFF, // Cyan
    0x0000FF, // Blue
    0xFF00FF, // Magenta
    0xFFBF00, // Amber
    0xFFFFFF  // White
};
#define PALETTE_SIZE (sizeof(PALETTE_COLORS) / sizeof(PALETTE_COLORS[0]))

// Forward declarations
static void create_settings_modal(void);
static void screen_tap_cb(lv_event_t *e);

// Load colors from NVS storage
static void load_colors_from_nvs(void)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        nvs_get_u32(my_handle, "v_rgb", &g_voltage_rgb);
        nvs_get_u32(my_handle, "a_rgb", &g_current_rgb);
        nvs_close(my_handle);
        ESP_LOGI(TAG, "Loaded colors from NVS: Voltage=0x%06X, Current=0x%06X",
                 (unsigned int)g_voltage_rgb, (unsigned int)g_current_rgb);
    } else {
        ESP_LOGI(TAG, "No NVS color config found. Using default palette.");
    }
}

// Save colors to NVS storage
static void save_colors_to_nvs(void)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        nvs_set_u32(my_handle, "v_rgb", g_voltage_rgb);
        nvs_set_u32(my_handle, "a_rgb", g_current_rgb);
        nvs_commit(my_handle);
        nvs_close(my_handle);
        ESP_LOGI(TAG, "Saved colors to NVS: Voltage=0x%06X, Current=0x%06X",
                 (unsigned int)g_voltage_rgb, (unsigned int)g_current_rgb);
    } else {
        ESP_LOGE(TAG, "Failed to open NVS for writing: %s", esp_err_to_name(err));
    }
}

// Helper to convert uint32_t RGB to lv_color_t
static inline lv_color_t rgb32_to_lv_color(uint32_t rgb)
{
    uint8_t r = (rgb >> 16) & 0xFF;
    uint8_t g = (rgb >> 8) & 0xFF;
    uint8_t b = rgb & 0xFF;
    return lv_color_make(r, g, b);
}

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
            if (rx_buf[0] == MODBUS_SLAVE_ID && rx_buf[1] == 0x03 && rx_buf[2] == (MODBUS_REG_COUNT * 2)) {
                uint16_t rx_crc = modbus_crc16(rx_buf, expected_rx_len - 2);
                uint16_t pkt_crc = (uint16_t)rx_buf[expected_rx_len - 2] | ((uint16_t)rx_buf[expected_rx_len - 1] << 8);

                if (rx_crc == pkt_crc) {
                    uint32_t current_raw = ((uint32_t)rx_buf[3] << 24) |
                                           ((uint32_t)rx_buf[4] << 16) |
                                           ((uint32_t)rx_buf[5] << 8)  |
                                           ((uint32_t)rx_buf[6]);
                    memcpy(&current_val, &current_raw, sizeof(float));

                    uint32_t voltage_raw = ((uint32_t)rx_buf[43] << 24) |
                                           ((uint32_t)rx_buf[44] << 16) |
                                           ((uint32_t)rx_buf[45] << 8)  |
                                           ((uint32_t)rx_buf[46]);
                    memcpy(&voltage_val, &voltage_raw, sizeof(float));

                    if (!isnan(current_val) && !isinf(current_val) &&
                        !isnan(voltage_val) && !isinf(voltage_val)) {
                        valid_packet = true;
                    }
                }
            }
        }

        // Failsafe Logic: Retain last good value if packet is invalid
        if (valid_packet) {
            g_last_voltage = voltage_val;
            g_last_current = current_val;
        }

        // Format strings
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

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MODBUS_POLL_INTERVAL_MS));
    }
}

// Screen Triple-Tap Event Callback
static void screen_tap_cb(lv_event_t *e)
{
    if (settings_modal != NULL) {
        return; // Settings menu is already open
    }

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now - g_last_tap_ms < 600) {
        g_tap_count++;
    } else {
        g_tap_count = 1;
    }
    g_last_tap_ms = now;

    if (g_tap_count >= 3) {
        g_tap_count = 0;
        ESP_LOGI(TAG, "Triple-tap detected! Opening Settings Menu...");
        create_settings_modal();
    }
}

// Setup Main User Interface
static void create_main_ui(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);

    // Dark background
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(scr, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(scr, screen_tap_cb, LV_EVENT_CLICKED, NULL);

    // Top Row Container (Voltage) - 50% screen height
    lv_obj_t *row_v = lv_obj_create(scr);
    lv_obj_set_size(row_v, LV_PCT(100), LV_PCT(50));
    lv_obj_align(row_v, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(row_v, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row_v, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(row_v, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(row_v, 0, LV_PART_MAIN);
    lv_obj_clear_flag(row_v, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(row_v, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(row_v, screen_tap_cb, LV_EVENT_CLICKED, NULL);

    // Row 1: Voltage Label (320px font, +20px vertical offset)
    voltage_label = lv_label_create(row_v);
    lv_obj_set_style_text_color(voltage_label, rgb32_to_lv_color(g_voltage_rgb), LV_PART_MAIN);
    lv_obj_set_style_text_font(voltage_label, &font_bold_320, LV_PART_MAIN);
    lv_label_set_text(voltage_label, "0.0V");
    lv_obj_align(voltage_label, LV_ALIGN_CENTER, 0, 20);
    lv_obj_add_flag(voltage_label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(voltage_label, screen_tap_cb, LV_EVENT_CLICKED, NULL);

    // Bottom Row Container (Current) - 50% screen height
    lv_obj_t *row_a = lv_obj_create(scr);
    lv_obj_set_size(row_a, LV_PCT(100), LV_PCT(50));
    lv_obj_align(row_a, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(row_a, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row_a, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(row_a, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(row_a, 0, LV_PART_MAIN);
    lv_obj_clear_flag(row_a, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(row_a, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(row_a, screen_tap_cb, LV_EVENT_CLICKED, NULL);

    // Row 2: Current Label (320px font, +20px vertical offset)
    current_label = lv_label_create(row_a);
    lv_obj_set_style_text_color(current_label, rgb32_to_lv_color(g_current_rgb), LV_PART_MAIN);
    lv_obj_set_style_text_font(current_label, &font_bold_320, LV_PART_MAIN);
    lv_label_set_text(current_label, "0.0A");
    lv_obj_align(current_label, LV_ALIGN_CENTER, 0, 20);
    lv_obj_add_flag(current_label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(current_label, screen_tap_cb, LV_EVENT_CLICKED, NULL);

    ESP_LOGI(TAG, "Main UI created with triple-tap gesture listener enabled.");
}

// Event structure for color swatches
typedef struct {
    uint32_t color_rgb;
    uint8_t section; // 0 = Voltage, 1 = Current
    lv_obj_t *preview_label;
} color_swatch_user_data_t;

static color_swatch_user_data_t swatch_data_pool[2 * PALETTE_SIZE];
static int swatch_data_idx = 0;

static void swatch_click_cb(lv_event_t *e)
{
    color_swatch_user_data_t *ud = (color_swatch_user_data_t *)lv_event_get_user_data(e);
    if (!ud) return;

    if (ud->section == 0) {
        temp_voltage_rgb = ud->color_rgb;
        if (ud->preview_label) lv_obj_set_style_text_color(ud->preview_label, rgb32_to_lv_color(temp_voltage_rgb), LV_PART_MAIN);
    } else if (ud->section == 1) {
        temp_current_rgb = ud->color_rgb;
        if (ud->preview_label) lv_obj_set_style_text_color(ud->preview_label, rgb32_to_lv_color(temp_current_rgb), LV_PART_MAIN);
    }
}

static void save_btn_cb(lv_event_t *e)
{
    g_voltage_rgb = temp_voltage_rgb;
    g_current_rgb = temp_current_rgb;

    save_colors_to_nvs();

    if (voltage_label) {
        lv_obj_set_style_text_color(voltage_label, rgb32_to_lv_color(g_voltage_rgb), LV_PART_MAIN);
    }
    if (current_label) {
        lv_obj_set_style_text_color(current_label, rgb32_to_lv_color(g_current_rgb), LV_PART_MAIN);
    }

    if (settings_modal) {
        lv_obj_del(settings_modal);
        settings_modal = NULL;
    }
    ESP_LOGI(TAG, "Settings saved and modal closed.");
}

static void cancel_btn_cb(lv_event_t *e)
{
    if (settings_modal) {
        lv_obj_del(settings_modal);
        settings_modal = NULL;
    }
    ESP_LOGI(TAG, "Settings cancelled.");
}

// Create Touch Settings Modal Dialog
static void create_settings_modal(void)
{
    temp_voltage_rgb = g_voltage_rgb;
    temp_current_rgb = g_current_rgb;
    swatch_data_idx = 0;

    lv_obj_t *scr = lv_scr_act();

    // Fullscreen translucent backdrop
    settings_modal = lv_obj_create(scr);
    lv_obj_set_size(settings_modal, LV_PCT(100), LV_PCT(100));
    lv_obj_align(settings_modal, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(settings_modal, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(settings_modal, LV_OPA_80, LV_PART_MAIN);
    lv_obj_set_style_border_width(settings_modal, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(settings_modal, 0, LV_PART_MAIN);
    lv_obj_clear_flag(settings_modal, LV_OBJ_FLAG_SCROLLABLE);

    // Dialog Card (940x440)
    lv_obj_t *card = lv_obj_create(settings_modal);
    lv_obj_set_size(card, 940, 440);
    lv_obj_align(card, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(card, lv_color_make(0x18, 0x18, 0x18), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(card, lv_color_make(0x44, 0x44, 0x44), LV_PART_MAIN);
    lv_obj_set_style_border_width(card, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(card, 16, LV_PART_MAIN);
    lv_obj_set_style_pad_all(card, 16, LV_PART_MAIN);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    // Flex layout for card contents
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(card, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(card, 16, LV_PART_MAIN);

    // Header Title
    lv_obj_t *title = lv_label_create(card);
    lv_obj_set_style_text_color(title, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &font_menu_32, LV_PART_MAIN);
    lv_label_set_text(title, "FONT COLOR PALETTE SETTINGS");

    // Divider Line
    lv_obj_t *line = lv_obj_create(card);
    lv_obj_set_size(line, 900, 2);
    lv_obj_set_style_bg_color(line, lv_color_make(0x44, 0x44, 0x44), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(line, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(line, 0, LV_PART_MAIN);

    // --- SECTION 1: Voltage Color ---
    lv_obj_t *sec1 = lv_obj_create(card);
    lv_obj_set_size(sec1, 900, 100);
    lv_obj_set_style_bg_color(sec1, lv_color_make(0x22, 0x22, 0x22), LV_PART_MAIN);
    lv_obj_set_style_border_width(sec1, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(sec1, 10, LV_PART_MAIN);
    lv_obj_set_flex_flow(sec1, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(sec1, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(sec1, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *lbl_v_title = lv_label_create(sec1);
    lv_obj_set_style_text_color(lbl_v_title, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_v_title, &font_menu_32, LV_PART_MAIN);
    lv_label_set_text(lbl_v_title, "Voltage:");

    // Container for color buttons
    lv_obj_t *swatches_v = lv_obj_create(sec1);
    lv_obj_set_size(swatches_v, 580, 80);
    lv_obj_set_style_bg_opa(swatches_v, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(swatches_v, 0, LV_PART_MAIN);
    lv_obj_set_flex_flow(swatches_v, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(swatches_v, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(swatches_v, 8, LV_PART_MAIN);

    lv_obj_t *prev_v = lv_label_create(sec1);
    lv_obj_set_style_text_font(prev_v, &font_menu_32, LV_PART_MAIN);
    lv_obj_set_style_text_color(prev_v, rgb32_to_lv_color(temp_voltage_rgb), LV_PART_MAIN);
    lv_label_set_text(prev_v, "230.5V");

    for (int i = 0; i < PALETTE_SIZE; i++) {
        lv_obj_t *btn = lv_btn_create(swatches_v);
        lv_obj_set_size(btn, 52, 52);
        lv_obj_set_style_bg_color(btn, rgb32_to_lv_color(PALETTE_COLORS[i]), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_color(btn, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_border_width(btn, 2, LV_PART_MAIN);
        lv_obj_set_style_radius(btn, 8, LV_PART_MAIN);

        swatch_data_pool[swatch_data_idx].color_rgb = PALETTE_COLORS[i];
        swatch_data_pool[swatch_data_idx].section = 0;
        swatch_data_pool[swatch_data_idx].preview_label = prev_v;
        lv_obj_add_event_cb(btn, swatch_click_cb, LV_EVENT_CLICKED, &swatch_data_pool[swatch_data_idx]);
        swatch_data_idx++;
    }

    // --- SECTION 2: Current Color ---
    lv_obj_t *sec2 = lv_obj_create(card);
    lv_obj_set_size(sec2, 900, 100);
    lv_obj_set_style_bg_color(sec2, lv_color_make(0x22, 0x22, 0x22), LV_PART_MAIN);
    lv_obj_set_style_border_width(sec2, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(sec2, 10, LV_PART_MAIN);
    lv_obj_set_flex_flow(sec2, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(sec2, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(sec2, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *lbl_a_title = lv_label_create(sec2);
    lv_obj_set_style_text_color(lbl_a_title, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_a_title, &font_menu_32, LV_PART_MAIN);
    lv_label_set_text(lbl_a_title, "Current:");

    lv_obj_t *swatches_a = lv_obj_create(sec2);
    lv_obj_set_size(swatches_a, 580, 80);
    lv_obj_set_style_bg_opa(swatches_a, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(swatches_a, 0, LV_PART_MAIN);
    lv_obj_set_flex_flow(swatches_a, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(swatches_a, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(swatches_a, 8, LV_PART_MAIN);

    lv_obj_t *prev_a = lv_label_create(sec2);
    lv_obj_set_style_text_font(prev_a, &font_menu_32, LV_PART_MAIN);
    lv_obj_set_style_text_color(prev_a, rgb32_to_lv_color(temp_current_rgb), LV_PART_MAIN);
    lv_label_set_text(prev_a, "12.4A");

    for (int i = 0; i < PALETTE_SIZE; i++) {
        lv_obj_t *btn = lv_btn_create(swatches_a);
        lv_obj_set_size(btn, 52, 52);
        lv_obj_set_style_bg_color(btn, rgb32_to_lv_color(PALETTE_COLORS[i]), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_color(btn, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_border_width(btn, 2, LV_PART_MAIN);
        lv_obj_set_style_radius(btn, 8, LV_PART_MAIN);

        swatch_data_pool[swatch_data_idx].color_rgb = PALETTE_COLORS[i];
        swatch_data_pool[swatch_data_idx].section = 1;
        swatch_data_pool[swatch_data_idx].preview_label = prev_a;
        lv_obj_add_event_cb(btn, swatch_click_cb, LV_EVENT_CLICKED, &swatch_data_pool[swatch_data_idx]);
        swatch_data_idx++;
    }

    // Action buttons row
    lv_obj_t *actions = lv_obj_create(card);
    lv_obj_set_size(actions, 900, 70);
    lv_obj_set_style_bg_opa(actions, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(actions, 0, LV_PART_MAIN);
    lv_obj_set_flex_flow(actions, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(actions, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(actions, 40, LV_PART_MAIN);
    lv_obj_clear_flag(actions, LV_OBJ_FLAG_SCROLLABLE);

    // Save Button
    lv_obj_t *btn_save = lv_btn_create(actions);
    lv_obj_set_size(btn_save, 260, 56);
    lv_obj_set_style_bg_color(btn_save, lv_color_make(0x00, 0xAA, 0x00), LV_PART_MAIN);
    lv_obj_set_style_radius(btn_save, 10, LV_PART_MAIN);
    lv_obj_add_event_cb(btn_save, save_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lbl_save = lv_label_create(btn_save);
    lv_obj_set_style_text_color(lbl_save, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_save, &font_menu_32, LV_PART_MAIN);
    lv_label_set_text(lbl_save, "SAVE & EXIT");
    lv_obj_align(lbl_save, LV_ALIGN_CENTER, 0, 0);

    // Cancel Button
    lv_obj_t *btn_cancel = lv_btn_create(actions);
    lv_obj_set_size(btn_cancel, 260, 56);
    lv_obj_set_style_bg_color(btn_cancel, lv_color_make(0xAA, 0x00, 0x00), LV_PART_MAIN);
    lv_obj_set_style_radius(btn_cancel, 10, LV_PART_MAIN);
    lv_obj_add_event_cb(btn_cancel, cancel_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lbl_cancel = lv_label_create(btn_cancel);
    lv_obj_set_style_text_color(lbl_cancel, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_cancel, &font_menu_32, LV_PART_MAIN);
    lv_label_set_text(lbl_cancel, "CANCEL");
    lv_obj_align(lbl_cancel, LV_ALIGN_CENTER, 0, 0);
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

    // PHASE 2: Splash Screen ("INDUSTRIAL ELECTRONIC SYSTEMS" in 3 rows, size 140px, White)
    ESP_LOGI(TAG, "Starting Splash Screen Fade In (140px font, White)...");

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

        // Splash text color locked to White #FFFFFF
        lv_color_t splash_color = lv_color_white();

        // Row 1: INDUSTRIAL
        lv_obj_t *lbl1 = lv_label_create(splash_cnt);
        lv_obj_set_style_text_color(lbl1, splash_color, LV_PART_MAIN);
        lv_obj_set_style_text_font(lbl1, &font_splash_140, LV_PART_MAIN);
        lv_label_set_text(lbl1, "INDUSTRIAL");

        // Row 2: ELECTRONIC
        lv_obj_t *lbl2 = lv_label_create(splash_cnt);
        lv_obj_set_style_text_color(lbl2, splash_color, LV_PART_MAIN);
        lv_obj_set_style_text_font(lbl2, &font_splash_140, LV_PART_MAIN);
        lv_label_set_text(lbl2, "ELECTRONIC");

        // Row 3: SYSTEMS
        lv_obj_t *lbl3 = lv_label_create(splash_cnt);
        lv_obj_set_style_text_color(lbl3, splash_color, LV_PART_MAIN);
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

    // Initialize NVS Flash for storing color preferences across reboots
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Load saved color preferences
    load_colors_from_nvs();

    // Enable power LDO VO4 (3.3V) for onboard RS485 transceiver
    bsp_enable_ldo_vo4();

    // Initialize RS485 UART (19200 Baud, 8N1, TX: 27, RX: 26)
    init_rs485_uart();

    // Initialize Display BSP for Waveshare 7-inch LCD (mirror_x = 0, mirror_y = 0)
    bsp_display_cfg_t disp_cfg = {
        .lv_adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG(),
        .rotation = ESP_LV_ADAPTER_ROTATE_180,
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_PARTIAL,
        .touch_flags = {
            .swap_xy = 0,
            .mirror_x = 0, // Fix horizontal touch mirroring
            .mirror_y = 0, // Fix vertical touch mirroring
        },
    };
    lv_disp_t *disp = bsp_display_start_with_config(&disp_cfg);
    (void)disp;
    bsp_display_backlight_on();

    // Launch Startup Task (RGB Test -> Splash Screen 5s -> Main UI)
    xTaskCreate(startup_sequence_task, "startup_task", 4096, NULL, 5, NULL);
}
