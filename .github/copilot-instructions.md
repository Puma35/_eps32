# Copilot Instructions - picsetvoc

## Project Overview
Projet ESP32-CAM utilisant **ESP-IDF** (Espressif IoT Development Framework) via PlatformIO. Ce n'est **PAS** un projet Arduino Framework.

## Hardware Target
- **Board**: ESP32-CAM
- **Chipset**: ESP32 (dual-core, WiFi, Bluetooth)
- **Caméra**: OV2640 intégrée (interface DVP parallèle)
- **Microphone**: INMP441 externe (interface I2S)
  - VCC → 3.3V, GND → GND
  - SCK → GPIO 14, WS → GPIO 15, SD → GPIO 32, L/R → GND
- **Sample rate**: 16 kHz (reconnaissance vocale)
- **Format I2S**: 32-bit, mono (canal gauche)

## Architecture & Build System

### Framework
- **Framework**: ESP-IDF 5.5.0 (pas Arduino)
- **Point d'entrée**: `void app_main()` dans [src/main.c](../src/main.c)
- **Langage**: C (pas C++ Arduino)
- **Build**: CMake + PlatformIO

### Structure du Projet
```
src/
  main.c      - Point d'entrée (app_main + tâche audio)
  mic_i2s.c   - Driver microphone INMP441
include/
  mic_i2s.h   - Interface publique du driver microphone
lib/          - Bibliothèques personnalisées (vide actuellement)
platformio.ini - Configuration PlatformIO
sdkconfig.esp32cam - Configuration ESP-IDF (généré automatiquement)
```

## Workflows de Développement

### Build & Upload
```bash
# Build
pio run -e esp32cam

# Upload
pio run -e esp32cam --target upload

# Monitor série
pio device monitor

# Build + Upload + Monitor
pio run -e esp32cam --target upload && pio device monitor
```

### ESP-IDF vs Arduino
⚠️ **Important**: Ce projet utilise ESP-IDF, pas Arduino Framework:
- Utiliser `esp_err_t` pour les retours d'erreur (pas `bool` ou `int`)
- Utiliser les APIs ESP-IDF (`esp_wifi_*`, `esp_camera_*`, etc.)
- Pas de `setup()` et `loop()` - utiliser `app_main()` et FreeRTOS tasks
- Inclure les headers ESP-IDF: `#include "esp_log.h"`, `#include "freertos/FreeRTOS.h"`, etc.

### Logging
Utiliser le système de logMODULE_NAME"; // TAG en majuscules
ESP_LOGI(TAG, "Info message");
ESP_LOGW(TAG, "Warning message");
ESP_LOGE(TAG, "Error message");
```
Exemples du projet: `TAG = "MAIN"`, `TAG = "MIC_I2S"`

### Gestion d'Erreurs
TOUJOURS vérifier `esp_err_t` et logger les erreurs:
```c
esp_err_t ret = mic_i2s_init();
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Erreur init: %s", esp_err_to_name(ret));
    return ret;
}TVOC";
ESP_LOGI(TAG, "Info message");
ESP_LOGW(TAG, "Warning message");
ESP_LOGE(TAG, "Error message");
```

## Conventions Spécifiques

### Code Style
- Langage C (pas C++)
- Convention de nommage: snake_case pour les fonctions et variables
- Constantes en SCREAMING_SNAKE_CASE
- Le TAG de logging devrait être le nom du module en majuscules

### Gestion Mémoire
- Utiliser heap_caps pour l'allocation mémoire spécifique (PSRAM, DMA, etc.)
- Privilégier les allocations statiques quand possible pour ESP32-CAM (mémoire limitée)
- Libérer toujours la mémoire avec `free()` après `malloc()`

### FreeRTOS
- Créer des tasks avec `xTaskCreate()` ou `xTaskCreatePinnedToCore()`
- Utiliser `vTaskDelay(pdMS_TO_TICKS(ms))` pour les délais
- Les queues, semaphores, et mutexes pour la synchronisation

## Configuration

###Modules du Projet

### Microphone I2S (mic_i2s.c/h)
Driver pour INMP441 utilisant ESP-IDF I2S standard driver:
- `mic_i2s_init()` - Configurer le channel I2S
- `mic_i2s_start()` / `mic_i2s_stop()` - Contrôler l'enregistrement
- `mic_i2s_read()` - Lire buffer audio (int32_t samples)
- `mic_i2s_calculate_rms()` - Calculer niveau sonore RMS

Pattern typique:
```c
mic_i2s_init();
mic_i2s_start();
int32_t buffer[1024];
size_t bytes_read;
mic_i2s_read(buffer, sizeof(buffer), &bytes_read);
uint32_t level = mic_i2s_calculate_rms(buffer, bytes_read/4);
```

### Main (main.c)
- Crée une FreeRTOS task pour lecture audio continue
- Affiche indicateur visuel des niveaux audio
- Pattern: init → start → xTaskCreate pour processing continu

## Ressources ESP32-CAM
- La caméra utilise l'interface DVP (pas MIPI CSI) - non déportable facilement
- PSRAM disponible pour stocker les frames caméra
- WiFi et caméra partagent des ressources - attention aux conflits
- GPIO 12-15, 32 utilisés pour I2S microphone - ne pas utiliser pour autre chose

### sdkconfig
Fichier de configuration ESP-IDF auto-généré. Pour modifier:
```bash
pio run -e esp32cam --target menuconfig
```

## Ressources ESP32-CAM
- La caméra utilise l'interface DVP (pas MIPI CSI)
- PSRAM disponible pour stocker les frames caméra
- WiFi et caméra partagent des ressources - attention aux conflits
