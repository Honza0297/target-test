/*
 * Target-side test firmware for esp-usb-bridge SPI/I2C bridging.
 *
 * This emulates the "target" chip that the bridge talks to over SPI or I2C.
 * It implements the bridge's framing protocol and simply ECHOES every payload
 * back: bytes the host sends down the USB serial port travel
 *
 *     host --USB--> bridge --(SPI/I2C)--> THIS target
 *
 * are buffered, and returned on the next poll
 *
 *     THIS target --(SPI/I2C)--> bridge --USB--> host
 *
 * so anything you type into the bridge's serial terminal comes back to you.
 *
 * Framing (identical to the bridge, see serial_handler.c):
 *     Frame = [LEN_H][LEN_L][PAYLOAD...]
 *     LEN is a big-endian uint16. LEN == 0 means "no data".
 *
 * Build for ESP32-C6. Select the bus under "Target Test Configuration" so it
 * matches the bridge's configured interface.
 */

#include <string.h>
#include <stdint.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#define FRAME_HEADER_SIZE   2
#define MAX_PAYLOAD         1024
// Total framing buffer, rounded up to a multiple of 4 for SPI slave DMA.
#define FRAME_BUF_SIZE      ((FRAME_HEADER_SIZE + MAX_PAYLOAD + 3) & ~3)

static const char *TAG = "target-test";

/* ═══════════════════════════════════════════════════════════════════════
 *  SPI slave echo
 *
 *  One CS assertion from the bridge carries either:
 *    - a WRITE: master clocks out [LEN][payload]; the LEN field is non-zero.
 *    - a POLL:  master clocks out zeros to read our pending frame; LEN field
 *               received from the master is therefore zero.
 *  We arm a single full-duplex transaction (tx = our pending frame,
 *  rx = receive buffer) and decide what happened after it completes.
 * ═══════════════════════════════════════════════════════════════════════ */

#if CONFIG_TARGET_TEST_INTERFACE_SPI

#include "driver/spi_slave.h"
#include "esp_heap_caps.h"

#define SPI_SLAVE_HOST  SPI2_HOST

static uint8_t *s_spi_tx;   // pending outgoing frame (DMA-capable)
static uint8_t *s_spi_rx;   // receive buffer (DMA-capable)

// Stage a [LEN][payload] frame into the TX buffer (len == 0 -> "no data").
static void spi_set_pending(const uint8_t *payload, uint16_t len)
{
    s_spi_tx[0] = len >> 8;
    s_spi_tx[1] = len & 0xFF;
    if (len) {
        memcpy(s_spi_tx + FRAME_HEADER_SIZE, payload, len);
    }
}

static void spi_slave_task(void *arg)
{
    while (1) {
        memset(s_spi_rx, 0, FRAME_BUF_SIZE);
        spi_slave_transaction_t t = {
            .length    = FRAME_BUF_SIZE * 8,
            .tx_buffer = s_spi_tx,
            .rx_buffer = s_spi_rx,
        };

        if (spi_slave_transmit(SPI_SLAVE_HOST, &t, portMAX_DELAY) != ESP_OK) {
            continue;
        }

        const uint16_t rx_len = ((uint16_t)s_spi_rx[0] << 8) | s_spi_rx[1];

        if (rx_len > 0 && rx_len <= MAX_PAYLOAD) {
            const uint8_t *payload = s_spi_rx + FRAME_HEADER_SIZE;
            uint16_t echo_len = (rx_len * 2 <= MAX_PAYLOAD) ? rx_len * 2 : rx_len;
            ESP_LOGI(TAG, "[console] SPI RX %u bytes from bridge: %.*s", rx_len, rx_len, payload);
            memcpy(s_spi_tx + FRAME_HEADER_SIZE, payload, rx_len);
            if (echo_len > rx_len) {
                memcpy(s_spi_tx + FRAME_HEADER_SIZE + rx_len, payload, rx_len);
            }
            s_spi_tx[0] = echo_len >> 8;
            s_spi_tx[1] = echo_len & 0xFF;
        } else {
            // Master POLLED. If we had a pending frame, it was just clocked out.
            const uint16_t pending = ((uint16_t)s_spi_tx[0] << 8) | s_spi_tx[1];
            if (pending > 0) {
                ESP_LOGI(TAG, "TX %u bytes to bridge (poll served)", pending);
                spi_set_pending(NULL, 0);
            }
        }
    }
}

static void transport_start(void)
{
    const spi_bus_config_t bus_cfg = {
        .mosi_io_num     = CONFIG_TARGET_TEST_SPI_MOSI,
        .miso_io_num     = CONFIG_TARGET_TEST_SPI_MISO,
        .sclk_io_num     = CONFIG_TARGET_TEST_SPI_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = FRAME_BUF_SIZE,
    };
    const spi_slave_interface_config_t slv_cfg = {
        .mode         = 0,
        .spics_io_num = CONFIG_TARGET_TEST_SPI_CS,
        .queue_size   = 1,
        .flags        = 0,
    };
    ESP_ERROR_CHECK(spi_slave_initialize(SPI_SLAVE_HOST, &bus_cfg, &slv_cfg, SPI_DMA_CH_AUTO));

    s_spi_tx = heap_caps_malloc(FRAME_BUF_SIZE, MALLOC_CAP_DMA);
    s_spi_rx = heap_caps_malloc(FRAME_BUF_SIZE, MALLOC_CAP_DMA);
    assert(s_spi_tx && s_spi_rx);
    spi_set_pending(NULL, 0);

    ESP_LOGI(TAG, "SPI slave ready (MOSI=%d MISO=%d SCLK=%d CS=%d)",
             CONFIG_TARGET_TEST_SPI_MOSI, CONFIG_TARGET_TEST_SPI_MISO,
             CONFIG_TARGET_TEST_SPI_SCLK, CONFIG_TARGET_TEST_SPI_CS);

    xTaskCreate(spi_slave_task, "spi_slave", 4096, NULL, 10, NULL);
}

#endif /* CONFIG_TARGET_TEST_INTERFACE_SPI */

/* ═══════════════════════════════════════════════════════════════════════
 *  I2C slave echo
 *
 *  The bridge polls in two separate read transactions: first it reads the
 *  2-byte length header, then (if LEN > 0) it reads LEN payload bytes. We
 *  therefore answer read requests in two phases. Writes from the bridge
 *  arrive as a single [LEN][payload] frame on the receive callback.
 *
 *  Callbacks run in ISR context, so they only enqueue events; the actual
 *  i2c_slave_write() happens in a task (the canonical IDF pattern).
 * ═══════════════════════════════════════════════════════════════════════ */

#if CONFIG_TARGET_TEST_INTERFACE_I2C

#include "driver/i2c_slave.h"

typedef enum { EVT_RX, EVT_TX } i2c_evt_t;
typedef enum { PHASE_HEADER, PHASE_PAYLOAD } i2c_phase_t;

static i2c_slave_dev_handle_t s_i2c_dev;
static QueueHandle_t          s_i2c_evt_q;

static i2c_phase_t s_phase = PHASE_HEADER;
static uint8_t     s_pending[MAX_PAYLOAD];
static uint16_t    s_pending_len;

// Staging buffer filled by the (ISR) receive callback.
static uint8_t  s_rx_stage[FRAME_BUF_SIZE];
static uint16_t s_rx_stage_len;

static IRAM_ATTR bool i2c_on_receive(i2c_slave_dev_handle_t dev,
                                     const i2c_slave_rx_done_event_data_t *evt, void *arg)
{
    BaseType_t hp_task_woken = pdFALSE;
    uint16_t n = evt->length;
    if (n > FRAME_BUF_SIZE) {
        n = FRAME_BUF_SIZE;
    }
    memcpy(s_rx_stage, evt->buffer, n);
    s_rx_stage_len = n;

    i2c_evt_t ev = EVT_RX;
    xQueueSendFromISR(s_i2c_evt_q, &ev, &hp_task_woken);
    return hp_task_woken == pdTRUE;
}

static IRAM_ATTR bool i2c_on_request(i2c_slave_dev_handle_t dev,
                                     const i2c_slave_request_event_data_t *evt, void *arg)
{
    BaseType_t hp_task_woken = pdFALSE;
    i2c_evt_t ev = EVT_TX;
    xQueueSendFromISR(s_i2c_evt_q, &ev, &hp_task_woken);
    return hp_task_woken == pdTRUE;
}

static void i2c_slave_task(void *arg)
{
    uint32_t written;

    while (1) {
        i2c_evt_t ev;
        if (xQueueReceive(s_i2c_evt_q, &ev, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGI(TAG, "[console] I2C: no data");
            continue;
        }

        if (ev == EVT_RX) {
            const uint16_t n = s_rx_stage_len;
            if (n >= FRAME_HEADER_SIZE) {
                const uint16_t len = ((uint16_t)s_rx_stage[0] << 8) | s_rx_stage[1];
                if (len > 0 && len <= MAX_PAYLOAD && n >= FRAME_HEADER_SIZE + len) {
                    const uint8_t *payload = s_rx_stage + FRAME_HEADER_SIZE;
                    memcpy(s_pending, payload, len);
                    if (len * 2 <= MAX_PAYLOAD) {
                        memcpy(s_pending + len, payload, len);
                        s_pending_len = len * 2;
                    } else {
                        s_pending_len = len;
                    }
                    s_phase = PHASE_HEADER;
                    ESP_LOGI(TAG, "[console] I2C RX %u bytes from bridge: %.*s", len, len, payload);
                }
            }
        } else { /* EVT_TX: master is reading; the bus is clock-stretched until we write */
            if (s_phase == PHASE_HEADER) {
                const uint8_t hdr[FRAME_HEADER_SIZE] = { s_pending_len >> 8, s_pending_len & 0xFF };
                i2c_slave_write(s_i2c_dev, hdr, FRAME_HEADER_SIZE, &written, 100);
                if (s_pending_len > 0) {
                    s_phase = PHASE_PAYLOAD;
                }
            } else {
                i2c_slave_write(s_i2c_dev, s_pending, s_pending_len, &written, 100);
                ESP_LOGI(TAG, "[console] I2C TX %u bytes to bridge (poll served)", s_pending_len);
                s_pending_len = 0;
                s_phase = PHASE_HEADER;
            }
        }
    }
}

static void transport_start(void)
{
    const i2c_slave_config_t cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = CONFIG_TARGET_TEST_I2C_SDA,
        .scl_io_num        = CONFIG_TARGET_TEST_I2C_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .send_buf_depth    = FRAME_BUF_SIZE,
        .receive_buf_depth = FRAME_BUF_SIZE,
        .slave_addr        = CONFIG_TARGET_TEST_I2C_ADDR,
        .addr_bit_len      = I2C_ADDR_BIT_LEN_7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_slave_device(&cfg, &s_i2c_dev));

    s_i2c_evt_q = xQueueCreate(16, sizeof(i2c_evt_t));
    assert(s_i2c_evt_q);

    const i2c_slave_event_callbacks_t cbs = {
        .on_receive = i2c_on_receive,
        .on_request = i2c_on_request,
    };
    ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(s_i2c_dev, &cbs, NULL));

    ESP_LOGI(TAG, "I2C slave ready (SDA=%d SCL=%d addr=0x%02X)",
             CONFIG_TARGET_TEST_I2C_SDA, CONFIG_TARGET_TEST_I2C_SCL, CONFIG_TARGET_TEST_I2C_ADDR);

    xTaskCreate(i2c_slave_task, "i2c_slave", 4096, NULL, 10, NULL);
}

#endif /* CONFIG_TARGET_TEST_INTERFACE_I2C */

/* ═══════════════════════════════════════════════════════════════════════
 *  UART echo
 *
 *  The bridge's UART transport sends/receives RAW bytes (no [LEN] framing),
 *  so the target just reads whatever arrives and writes it straight back.
 *  UART is symmetric (no master/slave): the bridge TX pin drives our RX pin
 *  and our TX pin drives the bridge RX pin.
 * ═══════════════════════════════════════════════════════════════════════ */

#if CONFIG_TARGET_TEST_INTERFACE_UART

#include "driver/uart.h"

#define UART_PORT       UART_NUM_1
#define UART_BUF_SIZE   1024

static void uart_echo_task(void *arg)
{
    uint8_t *buf = malloc(UART_BUF_SIZE);
    uint8_t echo[UART_BUF_SIZE * 2 + 22];
    assert(buf);

    while (1) {
        const int n = uart_read_bytes(UART_PORT, buf, UART_BUF_SIZE, pdMS_TO_TICKS(200));
        if (n > 0) {
            ESP_LOGI(TAG, "[console] RX %d bytes: %.*s", n, n, buf);
            int written = snprintf((char *)echo, sizeof(echo), "UART1 echo: %.*s%.*s", n, buf, n, buf);
            uart_write_bytes(UART_PORT, echo, written);
        }
        else {
            ESP_LOGI(TAG, "[console] no data");
            int written = snprintf((char *)echo, sizeof(echo), "UART1 echo: no data\n");
            uart_write_bytes(UART_PORT, echo, written);
        }

    }
}

static void transport_start(void)
{
    const uart_config_t cfg = {
        .baud_rate  = CONFIG_TARGET_TEST_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, CONFIG_TARGET_TEST_UART_TX, CONFIG_TARGET_TEST_UART_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "[console] UART1 ready (RX=%d TX=%d %d baud)",
             CONFIG_TARGET_TEST_UART_RX, CONFIG_TARGET_TEST_UART_TX, CONFIG_TARGET_TEST_UART_BAUD);

    xTaskCreate(uart_echo_task, "uart_echo", 4096, NULL, 10, NULL);
}

#endif /* CONFIG_TARGET_TEST_INTERFACE_UART */

void app_main(void)
{
    ESP_LOGI(TAG, "esp-usb-bridge target test (echo slave)");
    transport_start();
}
