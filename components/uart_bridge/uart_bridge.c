#include "uart_bridge.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "uart_bridge";

// RX only — there's no command channel back to the sender yet. Pick a GPIO
// that's free on this board (not the LED pin, not console UART0, not
// USB D+/D-, not the octal PSRAM/flash pins) — change if it doesn't suit
// your wiring.
#define UART_PORT     UART_NUM_1
#define UART_RX_PIN   18
#define UART_BAUD     115200
#define UART_RX_BUF_SIZE 512

#define LINE_MAX_LEN 48

// Sender frame period is 700ms — anything under a few missed frames counts
// as "still active" so a single dropped byte doesn't flap the BLE central
// pause on and off.
#define ACTIVE_TIMEOUT_US (3 * 1000 * 1000)

static uart_bridge_data_cb_t s_data_cb;
static volatile int64_t s_last_frame_us;

static void handle_line(const char *line)
{
    int speed_001kmh, incline_01pct, distance_m, flags;

    if (sscanf(line, "[TM:%d:%d:%d:%x]", &speed_001kmh, &incline_01pct,
               &distance_m, &flags) != 4) {
        ESP_LOGW(TAG, "Malformed frame: %s", line);
        return;
    }

    ftms_treadmill_data_t ftms = {
        .speed_001kmh = (uint16_t)speed_001kmh,
        .incline_01pct = (int16_t)incline_01pct,
        .total_distance_m = (uint32_t)distance_m,
        .has_incline = (flags & 0x1) != 0,
        .has_distance = (flags & 0x2) != 0,
    };

    s_last_frame_us = esp_timer_get_time();

    if (s_data_cb) {
        s_data_cb(&ftms);
    }
}

static void uart_bridge_task(void *arg)
{
    char line[LINE_MAX_LEN];
    size_t line_len = 0;
    uint8_t byte;

    while (1) {
        int n = uart_read_bytes(UART_PORT, &byte, 1, pdMS_TO_TICKS(1000));
        if (n <= 0) continue;

        if (byte == '\n') {
            if (line_len > 0) {
                line[line_len] = '\0';
                handle_line(line);
                line_len = 0;
            }
            continue;
        }

        if (byte == '\r') continue;

        if (line_len >= LINE_MAX_LEN - 1) {
            // Line too long (noise/corruption) — discard and resync on
            // the next newline instead of overflowing the buffer.
            ESP_LOGW(TAG, "Line too long, discarding");
            line_len = 0;
            continue;
        }

        line[line_len++] = (char)byte;
    }
}

esp_err_t uart_bridge_init(uart_bridge_data_cb_t data_cb)
{
    s_data_cb = data_cb;

    uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(UART_PORT, UART_RX_BUF_SIZE, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %d", err);
        return err;
    }

    err = uart_param_config(UART_PORT, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %d", err);
        return err;
    }

    err = uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %d", err);
        return err;
    }

    // Generous stack: the data callback chain reaches into rsc_server's
    // BLE notify call, same as every other task that ends up there.
    xTaskCreate(uart_bridge_task, "uart_bridge", 6144, NULL, 5, NULL);

    ESP_LOGI(TAG, "UART bridge listening on RX=GPIO%d @ %d baud", UART_RX_PIN, UART_BAUD);
    return ESP_OK;
}

bool uart_bridge_is_active(void)
{
    if (s_last_frame_us == 0) return false; // never received a frame
    return (esp_timer_get_time() - s_last_frame_us) < ACTIVE_TIMEOUT_US;
}
