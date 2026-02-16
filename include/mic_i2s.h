/**
 * @file mic_i2s.h
 * @brief Interface pour microphone INMP441 via I2S
 */

#ifndef MIC_I2S_H
#define MIC_I2S_H

#include "esp_err.h"
#include "driver/i2s_std.h"

// Configuration des pins pour INMP441 sur ESP32-CAM
#define I2S_MIC_SCK     GPIO_NUM_14  // Bit clock
#define I2S_MIC_WS      GPIO_NUM_15  // Word select (LR clock)
#define I2S_MIC_SD      GPIO_NUM_32  // Serial data
#define I2S_MIC_PORT    I2S_NUM_0

// Configuration audio
#define I2S_SAMPLE_RATE 16000        // Hz - bon pour reconnaissance vocale
#define I2S_SAMPLE_BITS 32           // ESP32 nécessite 32 bits même si INMP441 = 24 bits
#define I2S_BUFFER_SIZE 1024         // Samples par buffer

/**
 * @brief Initialiser le microphone I2S INMP441
 * @return ESP_OK si succès, code d'erreur sinon
 */
esp_err_t mic_i2s_init(void);

/**
 * @brief Démarrer l'enregistrement audio
 * @return ESP_OK si succès
 */
esp_err_t mic_i2s_start(void);

/**
 * @brief Arrêter l'enregistrement audio
 * @return ESP_OK si succès
 */
esp_err_t mic_i2s_stop(void);

/**
 * @brief Lire des échantillons audio depuis le microphone
 * @param buffer Buffer pour stocker les données (int32_t array)
 * @param buffer_size Taille du buffer en bytes
 * @param bytes_read Nombre de bytes effectivement lus
 * @return ESP_OK si succès
 */
esp_err_t mic_i2s_read(void *buffer, size_t buffer_size, size_t *bytes_read);

/**
 * @brief Calculer le niveau audio RMS (Root Mean Square)
 * @param samples Tableau d'échantillons int32_t
 * @param num_samples Nombre d'échantillons
 * @return Niveau RMS (0-32767 typique)
 */
uint32_t mic_i2s_calculate_rms(int32_t *samples, size_t num_samples);

/**
 * @brief Libérer les ressources du microphone
 * @return ESP_OK si succès
 */
esp_err_t mic_i2s_deinit(void);

#endif // MIC_I2S_H
