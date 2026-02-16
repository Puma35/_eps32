/**
 * @file mic_i2s.c
 * @brief Implémentation du driver pour microphone INMP441 via I2S
 */

#include "mic_i2s.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "MIC_I2S";
static i2s_chan_handle_t rx_handle = NULL;

esp_err_t mic_i2s_init(void)
{
    esp_err_t ret;

    // Configuration du channel I2S
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_MIC_PORT, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear DMA buffer

    ret = i2s_new_channel(&chan_cfg, NULL, &rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur création channel I2S: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configuration standard I2S (format Philips pour INMP441)
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_MIC_SCK,
            .ws = I2S_MIC_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_MIC_SD,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    // Le INMP441 est MSB (Most Significant Bit first)
    std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_MONO;
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT; // L/R = GND = canal gauche

    ret = i2s_channel_init_std_mode(rx_handle, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur init mode standard: %s", esp_err_to_name(ret));
        i2s_del_channel(rx_handle);
        rx_handle = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "Microphone INMP441 initialisé (sample rate: %d Hz)", I2S_SAMPLE_RATE);
    return ESP_OK;
}

esp_err_t mic_i2s_start(void)
{
    if (rx_handle == NULL) {
        ESP_LOGE(TAG, "Channel I2S non initialisé");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = i2s_channel_enable(rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur démarrage I2S: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Enregistrement démarré");
    return ESP_OK;
}

esp_err_t mic_i2s_stop(void)
{
    if (rx_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = i2s_channel_disable(rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur arrêt I2S: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Enregistrement arrêté");
    return ESP_OK;
}

esp_err_t mic_i2s_read(void *buffer, size_t buffer_size, size_t *bytes_read)
{
    if (rx_handle == NULL) {
        ESP_LOGE(TAG, "Channel I2S non initialisé");
        return ESP_ERR_INVALID_STATE;
    }

    if (buffer == NULL || bytes_read == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Timeout de 1 seconde
    esp_err_t ret = i2s_channel_read(rx_handle, buffer, buffer_size, bytes_read, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "Erreur lecture I2S: %s", esp_err_to_name(ret));
    }

    return ret;
}

uint32_t mic_i2s_calculate_rms(int32_t *samples, size_t num_samples)
{
    if (samples == NULL || num_samples == 0) {
        return 0;
    }

    uint64_t sum_squares = 0;
    for (size_t i = 0; i < num_samples; i++) {
        // Convertir 32-bit signé en valeur absolue, puis normaliser
        int32_t sample = samples[i] >> 14; // Shift pour avoir une plage raisonnable
        sum_squares += (uint64_t)(sample * sample);
    }

    uint32_t mean_square = (uint32_t)(sum_squares / num_samples);
    uint32_t rms = (uint32_t)sqrt((double)mean_square);

    return rms;
}

esp_err_t mic_i2s_deinit(void)
{
    if (rx_handle == NULL) {
        return ESP_OK;
    }

    esp_err_t ret = i2s_del_channel(rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur suppression channel I2S: %s", esp_err_to_name(ret));
        return ret;
    }

    rx_handle = NULL;
    ESP_LOGI(TAG, "Microphone déinitialisé");
    return ESP_OK;
}
