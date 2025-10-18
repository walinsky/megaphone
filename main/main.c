#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "codec.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"

#define TAG "MEGAPHONE"
#define MSBC_SAMPLES 240

// Configuration
#define SAMPLE_RATE     16000
#define MSBC_SAMPLES    240
#define I2S_DATA_BIT_WIDTH  I2S_DATA_BIT_WIDTH_32BIT
#define I2S_SLOT_BIT_WIDTH  I2S_SLOT_BIT_WIDTH_32BIT

// GPIO pins for INMP441
#define I2S_MIC_BCK     GPIO_NUM_16
#define I2S_MIC_WS      GPIO_NUM_27
#define I2S_MIC_DIN     GPIO_NUM_14

// GPIO pins for speaker (adjust as needed)
#define I2S_SPK_BCK     GPIO_NUM_26
#define I2S_SPK_WS      GPIO_NUM_17
#define I2S_SPK_DOUT    GPIO_NUM_25

// Channel handles
i2s_chan_handle_t rx_handle = NULL;
i2s_chan_handle_t tx_handle = NULL;

esp_err_t setup_i2s_microphone(void)
{
    // RX channel on I2S_NUM_1
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    
    esp_err_t ret = i2s_new_channel(&rx_chan_cfg, NULL, &rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S RX channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // PHILIPS mode with MONO and 32-bit
    i2s_std_config_t std_rx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_MIC_BCK,
            .ws = I2S_MIC_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_MIC_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    
    ret = i2s_channel_init_std_mode(rx_handle, &std_rx_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S RX standard mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2s_channel_enable(rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S RX channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2S microphone (INMP441) initialized successfully");
    return ESP_OK;
}

esp_err_t setup_i2s_speaker(void)
{
    // TX channel on I2S_NUM_0
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    
    esp_err_t ret = i2s_new_channel(&tx_chan_cfg, &tx_handle, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S TX channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // MSB mode with MONO and 16-bit, write to both slots
    i2s_std_config_t std_tx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_SPK_BCK,
            .ws = I2S_SPK_WS,
            .dout = I2S_SPK_DOUT,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    
    // Write to both slots for proper speaker output
    std_tx_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
    
    ret = i2s_channel_init_std_mode(tx_handle, &std_tx_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S TX standard mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2s_channel_enable(tx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S TX channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2S speaker initialized successfully");
    return ESP_OK;
}

void cleanup_i2s(void)
{
    if (rx_handle) {
        i2s_channel_disable(rx_handle);
        i2s_del_channel(rx_handle);
    }
    if (tx_handle) {
        i2s_channel_disable(tx_handle);
        i2s_del_channel(tx_handle);
    }
}

#define MSBC_FRAME_SAMPLES 120  // mSBC uses 120 samples per frame

void app_main(void)
{
    ESP_ERROR_CHECK(setup_i2s_microphone());
    ESP_ERROR_CHECK(setup_i2s_speaker());
    
    if (msbc_enc_open() != 0 || msbc_dec_open() != 0) {
        ESP_LOGE(TAG, "Failed to initialize codecs");
        return;
    }
    
    int32_t *i2s_buffer = malloc(MSBC_FRAME_SAMPLES * sizeof(int32_t));
    uint8_t *pcm_buffer = malloc(MSBC_FRAME_SAMPLES * 2);
    uint8_t *encoded_buffer = malloc(120);
    uint8_t *decoded_buffer = malloc(MSBC_FRAME_SAMPLES * 2);
    
    if (!i2s_buffer || !pcm_buffer || !encoded_buffer || !decoded_buffer) {
        ESP_LOGE(TAG, "Failed to allocate buffers");
        return;
    }
    
    ESP_LOGI(TAG, "Starting megaphone processing loop");
    
    size_t bytes_read, bytes_written;
    
    while (1) {
        esp_err_t ret = i2s_channel_read(rx_handle, i2s_buffer, 
                                         MSBC_FRAME_SAMPLES * sizeof(int32_t), 
                                         &bytes_read, portMAX_DELAY);
        
        if (ret == ESP_OK && bytes_read > 0) {
            i2s_32bit_to_16bit_pcm(i2s_buffer, pcm_buffer, MSBC_FRAME_SAMPLES);
            
            size_t encoded_len;
            if (msbc_enc_data(pcm_buffer, MSBC_FRAME_SAMPLES * 2, 
                             encoded_buffer, &encoded_len) == 0) {
                
                size_t decoded_len;
                if (msbc_dec_data(encoded_buffer, encoded_len, 
                                 decoded_buffer, &decoded_len) == 0) {
                    
                    i2s_channel_write(tx_handle, decoded_buffer, decoded_len, 
                                     &bytes_written, portMAX_DELAY);
                }
            }
        }
    }
    
    free(i2s_buffer);
    free(pcm_buffer);
    free(encoded_buffer);
    free(decoded_buffer);
    msbc_dec_close();
    msbc_enc_close();
    cleanup_i2s();
}
