# picsetvoc — VAD Audio + Photo (XIAO ESP32S3 Sense)

Firmware PlatformIO pour la **Seeed Studio XIAO ESP32S3 Sense**.  
Enregistre automatiquement des sessions audio/photo déclenchées par la voix (VAD).

---

## Fonctionnalité principale

Chaque détection de voix crée un dossier horodaté sur la carte SD :

```
/session_YYYYMMDD_HHMMSS/
    audio.wav   — enregistrement WAV 16 kHz / 16 bit / mono
    photo.jpg   — capture OV2640 UXGA (1600×1200) JPEG
```

La photo est prise **après** la fin de l'enregistrement audio pour ne pas bloquer l'I2S.

---

## Matériel requis

| Composant | Référence |
|-----------|-----------|
| Microcontrôleur | Seeed Studio XIAO ESP32S3 Sense |
| MCU | ESP32-S3R8 — Xtensa LX7 dual-core 32 bits @ 240 MHz |
| Mémoire | 8 MB Flash on-chip + 8 MB PSRAM OPI on-chip |
| Microphone | PDM intégré sur la carte Sense |
| Caméra | OV2640 (module Sense) |
| Stockage | Carte SD (FAT, jusqu'à 32 GB) via SPI2/HSPI |

---

## Brochage

### Microphone PDM (intégré Sense)

| Signal | GPIO |
|--------|------|
| CLK    | 42   |
| DATA   | 41   |

### Carte SD (SPI)

| Signal | GPIO |
|--------|------|
| CS     | 21   |
| SCK    | 7    |
| MISO   | 8    |
| MOSI   | 9    |

### Caméra OV2640 (intégrée Sense)

| Signal  | GPIO |
|---------|------|
| XCLK    | 10   |
| SIOD    | 40   |
| SIOC    | 39   |
| VSYNC   | 38   |
| HREF    | 47   |
| PCLK    | 13   |
| D0–D7   | 15, 17, 18, 16, 14, 12, 11, 48 |

---

## Algorithme VAD

La détection d'activité vocale (Voice Activity Detection) repose sur trois critères cumulés :

1. **Filtre passe-bande 300–3400 Hz** — biquad Butterworth 2nd ordre (Direct Form II), élimine le bruit basse fréquence et les composantes hors spectre vocal.
2. **RMS AC** (DC offset soustrait) — mesure de l'énergie du signal filtré, comparée à un seuil adaptatif.
3. **ZCR (Zero Crossing Rate)** — discrimine la voix des bruits impulsionnels (claquements, chocs).

### Paramètres VAD

| Paramètre | Valeur | Description |
|-----------|--------|-------------|
| `VAD_TRIGGER_FACTOR` | 6.0× | Seuil déclenchement = `noise_ema × factor` |
| `VAD_SILENCE_FACTOR` | 3.0× | Seuil silence en cours d'enregistrement |
| `VAD_VOTE_NEEDED`    | 3 chunks | Nombre de chunks consécutifs pour valider |
| `VAD_CHUNK_MS`       | 30 ms | Durée d'un chunk d'analyse |
| `EMA_ALPHA`          | 0.990 | Lissage exponentiel du bruit de fond |
| `SILENCE_TIMEOUT_MS` | 5000 ms | Silence prolongé → fin de session |
| `MAX_RECORD_SEC`     | 120 s | Durée maximale d'enregistrement |
| `PRE_ROLL_MS`        | 300 ms | Audio capturé avant le déclenchement |
| `ZCR_MIN / ZCR_MAX`  | 3 / 180 | Plage ZCR acceptable pour la voix |

### Démarrage EMA (seed)
À l'initialisation, 1500 ms de silence sont analysés pour établir le plancher de bruit (`noise_floor_abs`). Les seuils `vad_trigger` et `vad_silence` sont calculés automatiquement à partir de ce plancher.

---

## Format audio WAV

| Paramètre | Valeur |
|-----------|--------|
| Format    | PCM 16 bits signé |
| Canaux    | 1 (mono) |
| Fréquence d'échantillonnage | 16 000 Hz |
| Gain appliqué | ×8 (amplification DC-free) |

Le signal est **centré sur 0** (DC soustrait) et amplifié avant écriture pour maximiser la dynamique WAV.

---

## Synchronisation horloge (WiFi + NTP)

Au démarrage, le firmware tente de se connecter au WiFi pour synchroniser l'heure via NTP (`pool.ntp.org`).

- **Fuseau horaire** : France — `CET-1CEST,M3.5.0,M10.5.0/3` (gestion DST automatique)
- **Timeout WiFi** : 8 secondes
- Après sync, le WiFi est **coupé** : la RTC interne ESP32-S3 continue seule
- **Fallback sans WiFi** : noms de sessions de la forme `session_19700101_XXXXXX` (epoch identifiable)

> **Sécurité** : les identifiants WiFi (SSID / mot de passe) sont définis directement dans `src/main.cpp`.  
> ⚠️ **Ne jamais commiter ce fichier avec de vraies credentials dans un dépôt public.**  
> Utiliser des variables d'environnement, un fichier `credentials.h` listé dans `.gitignore`, ou l'API NVS de l'ESP-IDF.

---

## Configuration caméra

| Réglage | Valeur | Note |
|---------|--------|------|
| Résolution | UXGA (1600×1200) | Maximum OV2640 |
| Format | JPEG (qualité 10) | 0 = max qualité |
| Framebuffer | PSRAM | Requis pour UXGA |
| Rotation | 180° (vflip + hmirror) | Module monté à l'envers |
| Exposition | Manuelle (150/1200) | Anti-flou de bougé |
| Gain | Manuel ×20 | Compense expo courte |
| AWB | Auto | Balance des blancs |

---

## Installation & compilation

### Prérequis

- [Visual Studio Code](https://code.visualstudio.com/) + extension [PlatformIO](https://platformio.org/)
- Pilote USB CDC si nécessaire (XIAO se présente comme port COM)

### Étapes

```bash
# 1. Cloner le dépôt
git clone <url-du-repo>
cd picsetvoc

# 2. Ajouter les credentials WiFi dans src/main.cpp
#    Remplacer les valeurs des defines WIFI_SSID et WIFI_PASSWORD

# 3. Compiler et flasher via PlatformIO
pio run --target upload --environment xiaos3sense

# 4. Moniteur série
pio device monitor --environment xiaos3sense
```

### Environnement PlatformIO (`platformio.ini`)

```ini
platform  = espressif32
board     = seeed_xiao_esp32s3
framework = arduino

board_build.flash_size        = 8MB
board_build.arduino.memory_type = qio_opi   ; PSRAM OPI requis pour esp_camera

build_flags =
    -DBOARD_HAS_PSRAM
    -DARDUINO_USB_CDC_ON_BOOT=1

monitor_speed   = 115200
monitor_port    = COM4           ; adapter selon votre système
monitor_filters = esp32_exception_decoder
```

---

## Sortie série (exemples)

```
========================================
=== XIAO ESP32S3 Sense - VAD + Photo ===
========================================
CPU: 240 MHz  Flash: 8MB  PSRAM: 8192KB

[INIT 1/4] WiFi + NTP...
  Connecte !  IP=192.168.x.x  RSSI=-55dBm
  Heure synchronisee : 26/02/2026 14:32:10
  WiFi deconnecte (RTC continue seule)
[INIT 2/4] Carte SD...
  SD OK - Taille: 15193MB  Type: 3
[INIT 3/4] Camera OV2640...
  Camera OK (OV2640 UXGA 1600x1200 JPEG, rotation 180deg, expo courte)
[INIT 4/4] Microphone PDM...
  Micro PDM OK (GPIO42=CLK, GPIO41=DATA, 16kHz)

[SEED] Mesure plancher bruit (1500ms) -- silence svp...
[SEED] min=18 moy=24 plancher=23 | trigger=138 | silence=69
[VAD] En attente de voix...

[VEILLE] raw=  21 filt=  19 EMA=23 trig=138  13% zcr= 12      vote=0/3
[20260226_143215] Demarre -> /session_20260226_143215/audio.wav
[20260226_143215]   3s | rms_filt= 312 sil=69 | [==..................] 0.1/5.0s
[20260226_143215] Termine : 6.4s | 204800 bytes | SILENCE 5s
[CAM] /session_20260226_143215/photo.jpg -- 142.3 kB
[VAD] En attente de voix...
```

---

## Structure du projet

```
picsetvoc/
├── platformio.ini          # Configuration PlatformIO
├── sdkconfig.esp32cam      # Config SDK caméra ESP32
├── src/
│   └── main.cpp            # Code source principal
└── README.md
```

---

## Licence

Ce projet est fourni tel quel, sans garantie. Libre d'utilisation et de modification.
