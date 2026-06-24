# picsetvoc — Liflogger VAD + Photo (XIAO ESP32S3 Sense)

Firmware PlatformIO pour la **Seeed Studio XIAO ESP32S3 Sense**.  
Capture en continu des sessions audio/photo déclenchées automatiquement par la voix (VAD),
des photos périodiques de veille, et expose une interface web complète pour supervision,
configuration et consultation des données.

---

## Vue d'ensemble

```
┌─────────────────────────────────────────────────────┐
│                XIAO ESP32S3 Sense                   │
│                                                     │
│  Micro PDM ──► VAD (Core1) ──► Session SD           │
│  Caméra    ──► Photo task (Core0) ──► SD            │
│  SD FAT32  ──► /session_*/ + /idle_*.jpg            │
│  WiFi AP   ──► Back-office web (192.168.4.1)        │
└─────────────────────────────────────────────────────┘
```

**Cas d'usage principal : liflogger portable.**  
L'appareil tourne en autonomie, capturant tout ce qui se passe autour de lui — voix, images
d'ambiance — sans intervention. L'interface web permet de consulter la timeline, configurer
les paramètres et télécharger les fichiers depuis un smartphone via WiFi direct.

---

## Architecture des données sur la SD

La SD est au format **FAT32** (obligatoire — exFAT non supporté). Fréquence SPI : 4 MHz.

### Structure des fichiers

```
/                                    ← racine SD
│
├── config.json                      ← configuration persistante
│
├── idle_YYYYMMDD_HHMMSS.jpg         ← photos de veille (rythme configurable)
├── idle_YYYYMMDD_HHMMSS.jpg
├── ...
│
├── session_YYYYMMDD_HHMMSS/         ← session audio (1 par détection voix)
│   ├── audio.wav                    ← enregistrement audio
│   ├── photo_001.jpg                ← photo au déclenchement VAD
│   ├── photo_002.jpg                ← photo pendant enregistrement (+10s)
│   ├── photo_003.jpg                ← photo pendant enregistrement (+20s)
│   ├── ...
│   ├── photo.jpg                    ← photo de fin de session
│   └── meta.json                    ← coordonnées GPS (si module branché)
│
└── session_YYYYMMDD_HHMMSS/
    └── ...
```

L'horodatage `YYYYMMDD_HHMMSS` provient de la RTC interne synchronisée par NTP au démarrage.
Sans WiFi disponible, le fallback est `19700101_XXXXXX` (epoch, identifiable).

---

## Formats de fichiers

### audio.wav

Format PCM standard, directement lisible par tout lecteur audio.

| Champ | Valeur |
|-------|--------|
| Conteneur | RIFF/WAV (header 44 octets) |
| Codec | PCM 16 bits signé, little-endian |
| Canaux | 1 (mono) |
| Fréquence | 16 000 Hz |
| Débit | 32 000 octets/seconde = **32 KB/s** |
| Gain | ×8 (amplifié, DC-free avant écriture) |
| Pré-roll | 300 ms capturés avant le déclenchement |
| Durée min/max | quelques secondes — 120 s (configurable) |

**Tailles typiques :**

| Durée session | Taille WAV |
|--------------|------------|
| 10 s | ~320 KB |
| 30 s | ~960 KB |
| 60 s | ~1,9 MB |
| 120 s (max) | ~3,8 MB |

### photo_NNN.jpg / photo.jpg / idle_*.jpg

Captures de la caméra OV2640 en JPEG.

| Champ | Valeur par défaut |
|-------|------------------|
| Résolution | UXGA — 1600 × 1200 px |
| Format | JPEG |
| Qualité JPEG | 10 (0 = max, 63 = min) |
| Framebuffer | PSRAM (requis pour UXGA) |
| Orientation | Corrigée (vflip + hmirror) |
| Exposition | Manuelle (150/1200) |
| Gain | Manuel ×20 |
| Taille typique | **100 – 300 KB** (≈ 150 KB en moyenne) |

### meta.json

Présent uniquement si le module GPS est branché et a un fix au moment de la fin de session.

```json
{
  "session": "20260624_143215",
  "lat": 48.856600,
  "lon": 2.352200,
  "fix": true
}
```

| Champ | Type | Description |
|-------|------|-------------|
| `session` | string | ID de la session parente |
| `lat` | float (6 décimales) | Latitude WGS84 |
| `lon` | float (6 décimales) | Longitude WGS84 |
| `fix` | bool | `true` si le GPS avait un fix valide |

Taille : ~70 octets.

### config.json

Configuration complète de l'appareil, écrite par l'interface web.

```json
{
  "wifi_ssid": "...", "wifi_pass": "...",
  "ap_ssid": "ESP32-Debug", "ap_pass": "admin1234",
  "vad_trigger_factor": 6.0, "vad_silence_factor": 3.0,
  "vad_min_trigger": 40, "vad_min_silence": 15,
  "ema_alpha": 0.990, "vad_vote_needed": 3,
  "silence_timeout_ms": 5000, "max_record_sec": 120,
  "mic_gain": 8, "zcr_min": 3, "zcr_max": 180,
  "cam_quality": 10, "cam_framesize": 13,
  "cam_aec_value": 150, "cam_agc_gain": 20,
  "gps_enabled": false, "gps_rx_pin": 43, "gps_tx_pin": 44,
  "idle_photo_interval_s": 60,
  "rec_photo_interval_s": 10
}
```

Rechargé au boot depuis la SD. Si absent, les valeurs par défaut s'appliquent.

---

## Déclencheurs d'écriture sur SD

| Événement | Fichiers créés | Fréquence |
|-----------|---------------|-----------|
| Voix détectée | `session_*/audio.wav` ouvert | À chaque déclenchement VAD |
| Déclenchement VAD | `session_*/photo_001.jpg` | À chaque déclenchement VAD |
| Pendant enregistrement | `session_*/photo_NNN.jpg` | Toutes les `rec_photo_interval_s` s (défaut 10 s) |
| Fin de session (silence/timeout) | `session_*/photo.jpg`, `session_*/meta.json` | À chaque fin de session |
| Mode veille | `/idle_YYYYMMDD_HHMMSS.jpg` | Toutes les `idle_photo_interval_s` s (défaut 60 s) |
| Changement config via web | `/config.json` | À chaque sauvegarde manuelle |

**Architecture asynchrone :** les prises de vue sont déléguées à une tâche FreeRTOS dédiée
(Core 0), ce qui garantit que l'écriture JPEG ne bloque jamais la détection vocale (Core 1).

---

## Estimation du volume de données

Hypothèses (usage liflogger continu) :
- Photo JPEG : **150 KB** en moyenne (UXGA qualité 10)
- WAV : **32 KB/s**
- Session moyenne : **45 secondes** → 5 photos (001 + 3×interval + fin) + 1,4 MB WAV

### Par heure d'utilisation

| Activité vocale | Sessions | WAV | Photos session | Photos veille (60 s) | **Total/heure** |
|-----------------|----------|-----|----------------|----------------------|----------------|
| Faible (2 min/h) | 3 × 45 s | 4,3 MB | 3 × 750 KB = 2,3 MB | 60 × 150 KB = 9 MB | **≈ 16 MB/h** |
| Moyenne (5 min/h) | 7 × 45 s | 9,4 MB | 7 × 750 KB = 5,2 MB | 60 × 150 KB = 9 MB | **≈ 24 MB/h** |
| Forte (15 min/h) | 20 × 45 s | 27 MB | 20 × 750 KB = 15 MB | 60 × 150 KB = 9 MB | **≈ 51 MB/h** |

### Par jour (24 h en continu)

| Scénario | Total/jour | Carte 32 GB | Carte 64 GB |
|----------|-----------|-------------|-------------|
| Faible activité vocale | ≈ 380 MB | **~84 jours** | ~168 jours |
| Activité vocale moyenne | ≈ 575 MB | ~55 jours | ~110 jours |
| Forte activité vocale | ≈ 1,2 GB | ~26 jours | ~52 jours |

> Ces estimations supposent la configuration par défaut (photo veille 60 s, photo enreg. 10 s).
> Réduire `idle_photo_interval_s` à 300 s (5 min) divise par 5 le volume de photos veille
> et allonge la durée d'autonomie SD de manière significative.

### Levier principal : intervalle des photos veille

| `idle_photo_interval_s` | Photos/24 h | Volume veille/jour | Autonomie 32 GB (activité moyenne) |
|------------------------|------------|-------------------|-------------------------------------|
| 30 s | 2 880 | 432 MB | ~28 jours |
| 60 s (défaut) | 1 440 | 216 MB | ~55 jours |
| 300 s (5 min) | 288 | 43 MB | ~120 jours |
| 600 s (10 min) | 144 | 22 MB | ~140 jours |
| 0 (désactivé) | 0 | 0 | ~200 jours |

---

## Interface web (back-office)

Accessible via WiFi direct : **SSID `ESP32-Debug` / mot de passe `admin1234`**  
Adresse : **`http://192.168.4.1`**

L'AP démarre systématiquement au boot, même en cas d'échec SD ou caméra.

### 5 onglets

| Onglet | Contenu |
|--------|---------|
| **Dashboard** | Indicateur VEILLE/REC, jauge RMS vs seuil, espace SD, uptime |
| **Timeline** | Sessions VAD + photos veille, triées du plus récent au plus ancien, suppression |
| **Config** | Tous les paramètres (VAD, caméra, GPS, WiFi, intervalles photos) — sauvegarde vers `/config.json` |
| **GPS** | Coordonnées courantes, statut fix, carte canvas offline |
| **Terminal** | Flux de logs en temps réel (polling toutes les secondes, 200 lignes, pause/clear) |

### Routes HTTP

| Méthode | Route | Description |
|---------|-------|-------------|
| GET | `/` | Page HTML complète (PROGMEM) |
| GET | `/stats` | JSON stats VAD live (RMS, EMA, trigger, session en cours) |
| GET | `/sessions` | JSON timeline mixte `[{t:"s",id:...},{t:"i",id:...},...]` |
| GET | `/file?p=<path>` | Stream fichier SD (WAV, JPEG, JSON) |
| GET | `/sdinfo` | JSON espace SD `{total, used, free, error}` |
| GET | `/log?from=N` | JSON lignes de log depuis l'index N `{total, lines:[...]}` |
| GET | `/gps` | JSON coordonnées GPS courantes |
| GET | `/config` | JSON configuration complète |
| POST | `/config` | Mise à jour config + sauvegarde `/config.json` |
| POST | `/reboot` | Redémarrage ESP32 |
| DELETE | `/session?id=` | Supprime un dossier session (tous les fichiers) |
| DELETE | `/idle?id=` | Supprime une photo veille |

---

## Algorithme VAD

La détection repose sur trois critères cumulés par chunk de **30 ms** :

1. **Filtre passe-bande 300–3400 Hz** — biquad Butterworth 2e ordre, élimine bruit BF et composantes hors spectre vocal.
2. **RMS AC** sur signal filtré — comparé à un seuil adaptatif `noise_ema × trigger_factor`.
3. **ZCR (Zero Crossing Rate)** — discrimine voix/bruit impulsionnel (plage 3–180 ZCR/chunk valide).

Un vote de 3 chunks consécutifs valide le déclenchement. Le plancher de bruit est estimé
sur 1,5 s de silence au boot via EMA (α = 0,990).

### Paramètres VAD (configurables via `/config`)

| Paramètre | Défaut | Description |
|-----------|--------|-------------|
| `vad_trigger_factor` | 6,0× | Seuil déclenchement = `noise_ema × factor` |
| `vad_silence_factor` | 3,0× | Seuil silence pendant enregistrement |
| `vad_vote_needed` | 3 | Chunks consécutifs requis |
| `ema_alpha` | 0,990 | Lissage exponentiel du fond sonore |
| `silence_timeout_ms` | 5 000 ms | Silence prolongé → fin de session |
| `max_record_sec` | 120 s | Durée maximale d'une session |
| `mic_gain` | ×8 | Gain appliqué avant écriture WAV |
| `zcr_min` / `zcr_max` | 3 / 180 | Plage ZCR valide pour la voix |

---

## Matériel

| Composant | Détails |
|-----------|---------|
| MCU | ESP32-S3R8 — Xtensa LX7 dual-core @ 240 MHz |
| RAM | 512 KB SRAM + 8 MB PSRAM OPI (on-chip) |
| Flash | 8 MB (on-chip) |
| Microphone | PDM intégré — GPIO42 (CLK), GPIO41 (DATA) |
| Caméra | OV2640 intégrée — UXGA 1600×1200 JPEG |
| Stockage | Micro-SD via SPI2 — CS:21 SCK:7 MISO:8 MOSI:9 |
| GPS (optionnel) | UART Neo-6M/GT-U7 — GPIO43 (RX), GPIO44 (TX) |
| WiFi | AP 802.11 b/g/n 2,4 GHz — channel 6, 4 clients max |

---

## Installation

### Prérequis

- [Visual Studio Code](https://code.visualstudio.com/) + extension [PlatformIO](https://platformio.org/)
- Pilote USB CDC si nécessaire (XIAO se présente comme port COM)

### Étapes

```bash
git clone <url-du-repo>
cd picsetvoc

# Copier et compléter le fichier credentials
cp src/credentials.h.example src/credentials.h
# Éditer src/credentials.h avec le SSID/password de votre WiFi (pour NTP)

# Compiler et flasher
pio run --target upload

# Moniteur série (optionnel)
pio device monitor
```

### `platformio.ini`

```ini
platform  = espressif32
board     = seeed_xiao_esp32s3
framework = arduino

board_build.flash_size          = 8MB
board_build.arduino.memory_type = qio_opi   ; PSRAM OPI requis pour esp_camera

build_flags =
    -DBOARD_HAS_PSRAM
    -DARDUINO_USB_CDC_ON_BOOT=1

lib_deps =
    https://github.com/me-no-dev/AsyncTCP.git
    https://github.com/me-no-dev/ESPAsyncWebServer.git
    bblanchon/ArduinoJson @ ^7.0.0

monitor_speed   = 115200
monitor_filters = esp32_exception_decoder
```

### Carte SD

- Format **FAT32** obligatoire (exFAT non supporté par la lib Arduino SD)
- Cartes > 32 GB : utiliser `diskpart` (Windows) ou Rufus pour forcer FAT32
- Cartes ≤ 32 GB : formatage standard Windows FAT32 fonctionne

---

## Architecture logicielle

```
Core 0 (FreeRTOS)                Core 1 (Arduino setup task)
─────────────────────────        ──────────────────────────────
AsyncWebServer                   record_vad()  ← boucle bloquante
  ├─ HTTP handlers               │  i2s_read() toutes les 30 ms
  └─ Connexions AP               │  Filtre biquad + RMS + ZCR
                                 │  Écriture WAV sur SD
Photo task (priority 2)          │  enqueue_photo() → queue
  └─ esp_camera_fb_get()         └─ EMA noise floor adaptatif
     + écriture JPEG SD
```

**Points clés :**
- `record_vad()` ne s'arrête jamais — `loop()` ne fait rien
- Les prises de vue sont **asynchrones** (queue FreeRTOS 4 slots) pour ne jamais bloquer l'audio
- Un mutex FreeRTOS protège tous les accès SD entre Core 0 et Core 1
- L'AP WiFi démarre avant la SD : l'interface est toujours accessible

---

## Logs série (exemples)

```
=== XIAO ESP32S3 - VAD Back-Office ===
[1/6] NTP...  STA 'DoubleBanana'...  NTP OK : 24/06/2026 10:15:32
[2/6] AP + WebServer...  softAP 'ESP32-Debug': OK  IP=192.168.4.1
[3/6] SD + Config...  SD OK 29696MB type=3  Config: /config.json charge
[4/6] Camera...  Camera OK  Tache photo async OK (Core0)
[5/6] Micro PDM...  Micro PDM OK
[6/6] GPS: desactive (config)
=== TOUT OK -- demarrage VAD ===

[SEED] 1500ms silence svp...
[SEED] plancher=24 trigger=144 silence=72
[VAD] trigger=144 silence=72 NTP=OK
[VEI] raw=  18 filt=   5 EMA=21 trig=125   4% zcr=141 v=0
[20260624_101542] DEBUT rms=312 trig=125
[CAM] /session_20260624_101542/photo_001.jpg 142.3kB
[20260624_101542]  15s rms= 287 sil=0/5000ms
[CAM] /session_20260624_101542/photo_002.jpg 138.7kB
[20260624_101542] FIN 18.4s 589824b SILENCE
[CAM] /session_20260624_101542/photo.jpg 145.1kB
[VAD] En attente de voix...
[CAM] /idle_20260624_101642.jpg 131.2kB
```

---

## Structure du projet

```
picsetvoc/
├── platformio.ini              # Configuration PlatformIO
├── sdkconfig.esp32cam          # Config SDK caméra ESP32
├── src/
│   ├── main.cpp                # Code source principal (~1400 lignes)
│   ├── credentials.h           # WiFi STA credentials (gitignore)
│   └── credentials.h.example  # Modèle credentials
└── README.md
```

---

## Licence

Ce projet est fourni tel quel, sans garantie. Libre d'utilisation et de modification.
