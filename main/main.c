#include <stdio.h>
#include <string.h>
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"

#define BUFFER_SIZE 1024
#define TX_SAMPLE_SIZE (I2S_DATA_BIT_WIDTH_16BIT * BUFFER_SIZE)
#define RX_SAMPLE_SIZE (I2S_DATA_BIT_WIDTH_32BIT * BUFFER_SIZE)
#define MY_LOG_TAG "I2S_PA"

i2s_chan_handle_t tx_handle;
i2s_chan_handle_t rx_handle;

char i2s_rx_buff[RX_SAMPLE_SIZE];
char i2s_tx_buff[TX_SAMPLE_SIZE];
size_t bytes_read;
size_t bytes_written;

// This is our DAC
void init_tx_i2s_chan()
{
i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_new_channel(&tx_chan_cfg, &tx_handle, NULL);
    i2s_std_config_t std_tx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_26,
            .ws = GPIO_NUM_17,
            .dout = GPIO_NUM_25,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    /* Initialize the channel */
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_tx_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
}

// This is our 2 INMP441 mems microphones. 1 with l/r pin high; 1 low.
void init_rx_i2s_chan()
{
    /* RX channel will be registered on our second I2S (for now)*/
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    i2s_new_channel(&rx_chan_cfg, NULL, &rx_handle);
    i2s_std_config_t std_rx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = GPIO_NUM_16,
                .ws   = GPIO_NUM_27,
                .dout = I2S_GPIO_UNUSED,
                .din  = GPIO_NUM_14,
                .invert_flags = {
                        .mclk_inv = false,
                        .bclk_inv = false,
                        .ws_inv   = false,
                },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_rx_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
}

void app_main(void)
{
    init_tx_i2s_chan();
    init_rx_i2s_chan();

    while (true) {
        char *buf_ptr_read = i2s_rx_buff;
        char *buf_ptr_write = i2s_tx_buff;
        // Read the RAW samples from the microphone
        if (i2s_channel_read(rx_handle, i2s_rx_buff, RX_SAMPLE_SIZE, &bytes_read, 1000) == ESP_OK) {
            ESP_LOGI(MY_LOG_TAG, "bytes read: %d; raw buf size: %d; tx buf size: %d", bytes_read, sizeof(i2s_rx_buff) / sizeof(i2s_rx_buff[0]), sizeof(i2s_tx_buff) / sizeof(i2s_tx_buff[0]));
            int raw_samples_read = bytes_read / 2 / (I2S_DATA_BIT_WIDTH_32BIT / 8);
            ESP_LOGI(MY_LOG_TAG, "raw samples read: %d", raw_samples_read);
            for (int i = 0; i < raw_samples_read; i++)
            {
                // let's inspect our input once in a while
                if (i % 10000 == 0) { // the first sample of every new buffer
                    ESP_LOGI(MY_LOG_TAG, "rx packet looks like %04x %04x %04x %04x", i2s_rx_buff[i], i2s_rx_buff[i+1], i2s_rx_buff[i+2], i2s_rx_buff[i+3]);
                }
                
                // left
                buf_ptr_write[0] = buf_ptr_read[6]; // mid
                buf_ptr_write[1] = buf_ptr_read[7]; // high
                // right
                buf_ptr_write[2] = buf_ptr_read[2]; // mid
                buf_ptr_write[3] = buf_ptr_read[3]; // high

                buf_ptr_write += 2 * (I2S_DATA_BIT_WIDTH_16BIT / 8);
                buf_ptr_read += 2 * (I2S_DATA_BIT_WIDTH_32BIT / 8);
            }
            i2s_channel_write(tx_handle, (const char*)i2s_tx_buff, sizeof(i2s_tx_buff), &bytes_written, portMAX_DELAY);
        } else {
            ESP_LOGI(MY_LOG_TAG, "Read Failed!");
        }
    }
}