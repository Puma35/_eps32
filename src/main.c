/**
 * @file main.c
 * @brief Test du microphone INMP441 sur ESP32-CAM
 * 
 * Ce code teste l'enregistrement audio et affiche les niveaux sonores.
 * Connecter INMP441:
 *   VCC → 3.3V
 *   GND → GND
 *   SCK → GPIO 14
 *   WS  → GPIO 15
 *   SD  → GPIO 32
 *   L/R → GND
 */

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "mic_i2s.h"

static const char *TAG = "MAIN";

#define AUDIO_BUFFER_SIZE (I2S_BUFFER_SIZE * sizeof(int32_t))

/**
 * @brief Tâche d'enregistrement audio continu
 */
void audio_task(void *pvParameters)
{
    int32_t *audio_buffer = malloc(AUDIO_BUFFER_SIZE);
    if (audio_buffer == NULL) {
        ESP_LOGE(TAG, "Erreur allocation mémoire pour buffer audio");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Tâche audio démarrée - écoute en cours...");
    ESP_LOGI(TAG, "Parlez dans le micro pour voir les niveaux audio !");

    while (1) {
        size_t bytes_read = 0;
        esp_err_t ret = mic_i2s_read(audio_buffer, AUDIO_BUFFER_SIZE, &bytes_read);
        
        if (ret == ESP_OK && bytes_read > 0) {
            size_t num_samples = bytes_read / sizeof(int32_t);
            uint32_t rms = mic_i2s_calculate_rms(audio_buffer, num_samples);
            
            // Afficher un indicateur visuel de niveau sonore
            int bars = rms / 500; // Ajuster le diviseur selon votre environnement
            if (bars > 40) bars = 40;
            
            if (bars > 2) { // Ne afficher que si du son détecté
                printf("Audio [");
                for (int i = 0; i < bars; i++) {
                    printf("█");
                }
                printf("] RMS: %lu\n", (unsigned long)rms);
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Timeout lecture I2S");
        }
        
        // Petit délai pour ne pas surcharger le CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    free(audio_buffer);
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Démarrage ESP32-CAM + INMP441 ===");
    ESP_LOGI(TAG, "Projet: picsetvoc");
    ESP_LOGI(TAG, "Framework: ESP-IDF %s", esp_get_idf_version());

    // Afficher infos hardware
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Chip ESP32 avec %d cœurs, WiFi%s%s",
             chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    // Initialiser le microphone I2S
    ESP_LOGI(TAG, "Initialisation du microphone INMP441...");
    esp_err_t ret = mic_i2s_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Échec initialisation microphone: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Vérifiez le câblage !");
        return;
    }

    // Démarrer l'enregistrement
    ret = mic_i2s_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Échec démarrage enregistrement: %s", esp_err_to_name(ret));
        mic_i2s_deinit();
        return;
    }

    // Créer la tâche d'enregistrement audio
    BaseType_t task_ret = xTaskCreate(
        audio_task,
        "audio_task",
        4096,  // Stack size
        NULL,
        5,     // Priorité
        NULL
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Échec création tâche audio");
        mic_i2s_stop();
        mic_i2s_deinit();
        return;
    }

    ESP_LOGI(TAG, "Système prêt ! Surveillez les niveaux audio dans le terminal.");
    ESP_LOGI(TAG, "Appuyez sur Ctrl+C pour arrêter.");

    // La boucle principale peut faire autre chose ou juste attendre
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Sleep 10 secondes
    }
}