/**
 * ============================================================
 *  Projet  : picsetvoc
 *  Carte   : Seeed Studio XIAO ESP32S3 Sense
 *  MCU     : ESP32-S3R8 - Xtensa LX7 dual-core 32-bit @ 240 MHz
 *  Memoire : 8 MB PSRAM OPI on-chip + 8 MB Flash on-chip
 *  Outil   : PlatformIO / Framework Arduino (env: xiaos3sense)
 * ============================================================
 *
 *  Description
 *  -----------
 *  VAD (Voice Activity Detection) base sur :
 *    - RMS AC (DC offset soustrait) sur signal filtre passe-bande 300-3400 Hz
 *    - ZCR (Zero Crossing Rate) pour discriminer voix vs bruits parasites
 *    - EMA adaptative du bruit de fond
 *
 *  Chaque declenchement vocal cree une session horodatee :
 *    /session_YYYYMMDD_HHMMSS/audio.wav  -- enregistrement WAV 16kHz/16bit/mono
 *    /session_YYYYMMDD_HHMMSS/photo.jpg  -- capture OV2640 XGA (1024x768) JPEG
 *
 *  WiFi + NTP : synchronisation heure au demarrage (timeout 8s).
 *  La RTC interne ESP32S3 continue apres deconnexion WiFi.
 *  Fallback si pas de WiFi : session_19700101_XXXXXX (epoch identifiable).
 *
 *  Brochage (XIAO ESP32S3 Sense)
 *  -------------------------------------------------------------------
 *  Micro  PDM CLK=GPIO42  DATA=GPIO41
 *  SD     CS=GPIO21 SCK=GPIO7 MISO=GPIO8 MOSI=GPIO9
 *  Camera XCLK=GPIO10 SIOD=GPIO40 SIOC=GPIO39
 *         D0-D7=GPIO15/17/18/16/14/12/11/48
 *         VSYNC=GPIO38 HREF=GPIO47 PCLK=GPIO13
 * ============================================================
 */

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <time.h>
#include "driver/i2s.h"
#include "esp_camera.h"

// ---- WiFi + NTP ----
#define WIFI_SSID        "DoubleBanana"
#define WIFI_PASSWORD    "LveuLft#*35"
#define NTP_SERVER       "pool.ntp.org"
#define NTP_TIMEOUT_MS   8000
// Timezone France : CET-1 hiver, CEST-2 ete (DST auto)
#define TZ_FR            "CET-1CEST,M3.5.0,M10.5.0/3"

// ---- Pins micro PDM ----
#define MIC_CLK_PIN   42
#define MIC_DATA_PIN  41

// ---- Pins SD (SPI) ----
#define SD_CS_PIN     21
#define SD_SCK_PIN    7
#define SD_MISO_PIN   8
#define SD_MOSI_PIN   9

// ---- Pins camera OV2640 (XIAO ESP32S3 Sense) ----
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    10
#define SIOD_GPIO_NUM    40
#define SIOC_GPIO_NUM    39
#define Y9_GPIO_NUM      48
#define Y8_GPIO_NUM      11
#define Y7_GPIO_NUM      12
#define Y6_GPIO_NUM      14
#define Y5_GPIO_NUM      16
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM      17
#define Y2_GPIO_NUM      15
#define VSYNC_GPIO_NUM   38
#define HREF_GPIO_NUM    47
#define PCLK_GPIO_NUM    13

// ---- Audio config ----
#define SAMPLE_RATE      16000
#define MIC_GAIN         8        // x8 sur signal AC -> amplitude WAV audible

// ---- VAD config ----
#define VAD_TRIGGER_FACTOR   6.0f
#define VAD_SILENCE_FACTOR   3.0f
#define VAD_MIN_TRIGGER      40
#define VAD_MIN_SILENCE      15
#define EMA_ALPHA            0.990f
#define VAD_VOTE_NEEDED      3
#define VAD_SEED_MS          1500
#define SILENCE_TIMEOUT_MS   5000
#define MAX_RECORD_SEC       120
#define VAD_CHUNK_MS         30
#define ZCR_MIN              3
#define ZCR_MAX              180

#define VAD_CHUNK_SAMPLES    (SAMPLE_RATE * VAD_CHUNK_MS / 1000)
#define VAD_CHUNK_BYTES      (VAD_CHUNK_SAMPLES * 2)

// Pre-roll : capture les N ms d'audio AVANT le declenchement (rattrapage premiers mots)
#define PRE_ROLL_MS      1500
#define PRE_ROLL_CHUNKS  (PRE_ROLL_MS / VAD_CHUNK_MS)  // 50 chunks x 30ms = 1500ms

// ---- Globals VAD ----
float    noise_ema       = 1000.0f;
float    noise_floor_abs = 1000.0f;
uint16_t vad_trigger     = VAD_MIN_TRIGGER;
uint16_t vad_silence     = VAD_MIN_SILENCE;

// ---- Global NTP sync flag ----
static bool ntp_synced = false;

// ============================================================
//  WIFI + NTP
// ============================================================
void wifi_ntp_init() {
    Serial.printf("  Connexion a '%s' (timeout %ds)...\n", WIFI_SSID, NTP_TIMEOUT_MS/1000);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < NTP_TIMEOUT_MS) {
        delay(500);
        Serial.printf("  . status=%d  (%lums)\n", WiFi.status(), millis()-t0);
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.printf("  ECHEC WiFi (status=%d) -- sessions epoch\n", WiFi.status());
        WiFi.disconnect(true);
        return;
    }

    Serial.printf("  Connecte !  IP=%s  RSSI=%ddBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());

    Serial.printf("  NTP sync depuis %s (TZ France)...\n", NTP_SERVER);
    configTzTime(TZ_FR, NTP_SERVER);

    struct tm timeinfo;
    t0 = millis();
    while (!getLocalTime(&timeinfo, 100) && millis() - t0 < 5000) {
        delay(300);
        Serial.printf("  . attente NTP (%lums)\n", millis()-t0);
    }

    if (getLocalTime(&timeinfo, 0)) {
        ntp_synced = true;
        char buf[32];
        strftime(buf, sizeof(buf), "%d/%m/%Y %H:%M:%S", &timeinfo);
        Serial.printf("  Heure synchronisee : %s\n", buf);
    } else {
        Serial.println("  NTP timeout -- sessions epoch");
    }

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("  WiFi deconnecte (RTC continue seule)");
}

// Retourne le timestamp actuel sous forme "YYYYMMDD_HHMMSS"
// Fallback : "19700101_XXXXXX" (epoch visible) si NTP non synce
void get_timestamp(char *out, size_t len) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 100)) {
        strftime(out, len, "%Y%m%d_%H%M%S", &timeinfo);
    } else {
        snprintf(out, len, "19700101_%06lu", millis() / 1000);
    }
}

// ============================================================
//  FILTRE BIQUAD -- Direct Form II
//  HP 300 Hz (Butterworth 2nd order, Q=0.7071, Fs=16 kHz)
//  LP 3400 Hz (Butterworth 2nd order, Q=0.7071, Fs=16 kHz)
// ============================================================
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float x1, x2, y1, y2;
} biquad_t;

static biquad_t hp_filt = {
    .b0=0.92007f, .b1=-1.84015f, .b2=0.92007f,
    .a1=-1.83388f, .a2=0.84657f,
    .x1=0,.x2=0,.y1=0,.y2=0
};

static biquad_t lp_filt = {
    .b0=0.36182f, .b1=0.72363f, .b2=0.36182f,
    .a1=0.26343f, .a2=0.18393f,
    .x1=0,.x2=0,.y1=0,.y2=0
};

inline float biquad_step(biquad_t *f, float x) {
    float y = f->b0*x + f->b1*f->x1 + f->b2*f->x2
              - f->a1*f->y1 - f->a2*f->y2;
    f->x2 = f->x1; f->x1 = x;
    f->y2 = f->y1; f->y1 = y;
    return y;
}

void apply_bandpass(const int16_t *src, int16_t *dst, size_t n) {
    for (size_t i = 0; i < n; i++) {
        float s = biquad_step(&hp_filt, (float)src[i]);
        s       = biquad_step(&lp_filt, s);
        if (s >  32767.0f) s =  32767.0f;
        if (s < -32768.0f) s = -32768.0f;
        dst[i] = (int16_t)s;
    }
}

// ============================================================
//  RMS AC (DC offset soustrait)
// ============================================================
uint16_t compute_rms(const int16_t *samples, size_t n) {
    if (n == 0) return 0;
    int64_t mean_sum = 0;
    for (size_t i = 0; i < n; i++) mean_sum += samples[i];
    int32_t dc = (int32_t)(mean_sum / (int64_t)n);
    uint64_t sum = 0;
    for (size_t i = 0; i < n; i++) {
        int32_t s = (int32_t)samples[i] - dc;
        sum += (uint64_t)(s * s);
    }
    return (uint16_t)sqrtf((float)sum / n);
}

// ============================================================
//  ZCR -- Zero Crossing Rate (crossings par chunk)
// ============================================================
uint16_t compute_zcr(const int16_t *buf, size_t n) {
    uint16_t count = 0;
    for (size_t i = 1; i < n; i++) {
        if ((buf[i] >= 0) != (buf[i-1] >= 0)) count++;
    }
    return count;
}

// ============================================================
//  WAV HEADER
// ============================================================
typedef struct __attribute__((packed)) {
    char     riff[4];
    uint32_t file_size;
    char     wave[4];
    char     fmt[4];
    uint32_t fmt_size;
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char     data[4];
    uint32_t data_size;
} wav_header_t;

void write_wav_header(File &f, uint32_t data_size) {
    wav_header_t h = {
        .riff            = {'R','I','F','F'},
        .file_size       = data_size + 36,
        .wave            = {'W','A','V','E'},
        .fmt             = {'f','m','t',' '},
        .fmt_size        = 16,
        .audio_format    = 1,
        .num_channels    = 1,
        .sample_rate     = SAMPLE_RATE,
        .byte_rate       = SAMPLE_RATE * 2,
        .block_align     = 2,
        .bits_per_sample = 16,
        .data            = {'d','a','t','a'},
        .data_size       = data_size,
    };
    f.seek(0);
    f.write((uint8_t*)&h, sizeof(h));
}

// ============================================================
//  CAMERA
// ============================================================
static bool cam_ok = false;

bool camera_init() {
    camera_config_t cfg;
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.pin_d0       = Y2_GPIO_NUM;
    cfg.pin_d1       = Y3_GPIO_NUM;
    cfg.pin_d2       = Y4_GPIO_NUM;
    cfg.pin_d3       = Y5_GPIO_NUM;
    cfg.pin_d4       = Y6_GPIO_NUM;
    cfg.pin_d5       = Y7_GPIO_NUM;
    cfg.pin_d6       = Y8_GPIO_NUM;
    cfg.pin_d7       = Y9_GPIO_NUM;
    cfg.pin_xclk     = XCLK_GPIO_NUM;
    cfg.pin_pclk     = PCLK_GPIO_NUM;
    cfg.pin_vsync    = VSYNC_GPIO_NUM;
    cfg.pin_href     = HREF_GPIO_NUM;
    cfg.pin_sscb_sda = SIOD_GPIO_NUM;
    cfg.pin_sscb_scl = SIOC_GPIO_NUM;
    cfg.pin_pwdn     = PWDN_GPIO_NUM;
    cfg.pin_reset    = RESET_GPIO_NUM;
    cfg.xclk_freq_hz = 20000000;
    cfg.pixel_format = PIXFORMAT_JPEG;
    cfg.frame_size   = FRAMESIZE_XGA;
    cfg.jpeg_quality = 12;
    cfg.fb_count     = 1;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("ERREUR camera: 0x%x\n", err);
        return false;
    }
    Serial.println("Camera OK (OV2640 XGA 1024x768 JPEG)");
    return true;
}

void take_photo(const char *path) {
    if (!cam_ok) return;
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("[CAM] ERREUR: capture echouee");
        return;
    }
    File f = SD.open(path, FILE_WRITE);
    if (!f) {
        Serial.printf("[CAM] ERREUR: impossible d'ouvrir %s\n", path);
        esp_camera_fb_return(fb);
        return;
    }
    f.write(fb->buf, fb->len);
    f.close();
    Serial.printf("[CAM] %s -- %.1f kB\n", path, fb->len / 1024.0f);
    esp_camera_fb_return(fb);
}

// ============================================================
//  MICRO PDM
// ============================================================
bool mic_init() {
    i2s_config_t i2s_config = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_PCM_SHORT,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 8,
        .dma_buf_len          = 64,
        .use_apll             = false,
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0,
    };
    if (i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL) != ESP_OK) {
        Serial.println("ERREUR: i2s_driver_install");
        return false;
    }
    i2s_pin_config_t pin_config = {
        .mck_io_num   = I2S_PIN_NO_CHANGE,
        .bck_io_num   = I2S_PIN_NO_CHANGE,
        .ws_io_num    = MIC_CLK_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = MIC_DATA_PIN,
    };
    if (i2s_set_pin(I2S_NUM_0, &pin_config) != ESP_OK) {
        Serial.println("ERREUR: i2s_set_pin");
        return false;
    }
    Serial.println("Micro PDM OK (GPIO42=CLK, GPIO41=DATA, 16kHz)");
    return true;
}

// ============================================================
//  SEED EMA -- amorcage sur signal filtre (rechauffe le filtre)
// ============================================================
void seed_noise_ema(uint8_t *buf, int16_t *fbuf) {
    Serial.printf("[SEED] Mesure plancher bruit (%dms) -- silence svp...\n", VAD_SEED_MS);
    uint32_t start    = millis();
    float    rms_sum  = 0.0f;
    float    rms_min  = 99999.0f;
    uint32_t n_chunks = 0;
    while (millis() - start < VAD_SEED_MS) {
        size_t bytes_read = 0;
        i2s_read(I2S_NUM_0, buf, VAD_CHUNK_BYTES, &bytes_read, pdMS_TO_TICKS(200));
        if (bytes_read == 0) continue;
        size_t n = bytes_read / 2;
        apply_bandpass((int16_t*)buf, fbuf, n);
        float rms = compute_rms(fbuf, n);
        rms_sum += rms;
        if (rms < rms_min) rms_min = rms;
        n_chunks++;
    }
    float rms_avg   = (n_chunks > 0) ? (rms_sum / n_chunks) : 20.0f;
    noise_floor_abs = rms_min * 0.2f + rms_avg * 0.8f;
    noise_ema       = noise_floor_abs;
    vad_trigger     = max((uint16_t)VAD_MIN_TRIGGER, (uint16_t)(noise_floor_abs * VAD_TRIGGER_FACTOR));
    vad_silence     = max((uint16_t)VAD_MIN_SILENCE,  (uint16_t)(noise_floor_abs * VAD_SILENCE_FACTOR));
    Serial.printf("[SEED] min=%.0f moy=%.0f plancher=%.0f | trigger=%u | silence=%u\n",
                  rms_min, rms_avg, noise_floor_abs, vad_trigger, vad_silence);
}

// ============================================================
//  BOUCLE VAD + ENREGISTREMENT + SESSION HORODATEE
// ============================================================
void record_vad() {
    uint8_t  *buf  = (uint8_t  *)malloc(VAD_CHUNK_BYTES);
    int16_t  *fbuf = (int16_t  *)malloc(VAD_CHUNK_BYTES);
    // Pre-roll buffer circulaire en PSRAM (50 chunks x 960 bytes = ~47 KB)
    uint8_t  *preroll = (uint8_t *)ps_malloc((size_t)PRE_ROLL_CHUNKS * VAD_CHUNK_BYTES);
    if (!buf || !fbuf || !preroll) {
        Serial.printf("ERREUR: malloc echoue (buf=%p fbuf=%p preroll=%p)\n", buf, fbuf, preroll);
        return;
    }
    int preroll_head  = 0;   // prochain slot a ecrire
    int preroll_count = 0;   // nb de chunks valides (0..PRE_ROLL_CHUNKS)

    File     wav;
    uint32_t written      = 0;
    uint32_t silence_ms   = 0;
    uint32_t rec_start_ms = 0;
    uint32_t last_print   = 0;
    uint32_t last_veille  = 0;
    uint16_t rms_peak     = 0;
    int      vote_count   = 0;
    bool     recording    = false;
    char     session_id[24] = "";   // "YYYYMMDD_HHMMSS"

    seed_noise_ema(buf, fbuf);

    Serial.printf("[VAD] EMA=%.2f | trigger=%u | silence=%u | GainX%d | ZCR[%d-%d] | NTP=%s\n",
                  EMA_ALPHA, vad_trigger, vad_silence, MIC_GAIN, ZCR_MIN, ZCR_MAX,
                  ntp_synced ? "OK" : "NON");
    Serial.println("[VAD] En attente de voix...");

    while (true) {
        size_t bytes_read = 0;
        i2s_read(I2S_NUM_0, buf, VAD_CHUNK_BYTES, &bytes_read, pdMS_TO_TICKS(500));
        if (bytes_read == 0) continue;

        int16_t *samples = (int16_t *)buf;
        size_t   n_samp  = bytes_read / 2;

        // RMS brut AC (logs + EMA)
        uint16_t rms_raw  = compute_rms(samples, n_samp);
        if (rms_raw > rms_peak) rms_peak = rms_raw;

        // Bandpass -> RMS filtre + ZCR (decision VAD)
        apply_bandpass(samples, fbuf, n_samp);
        uint16_t rms_filt = compute_rms(fbuf, n_samp);
        uint16_t zcr      = compute_zcr(fbuf, n_samp);
        bool     zcr_ok   = (zcr >= ZCR_MIN && zcr <= ZCR_MAX);

        // EMA en veille uniquement
        if (!recording) {
            noise_ema   = EMA_ALPHA * noise_ema + (1.0f - EMA_ALPHA) * (float)rms_filt;
            vad_trigger = max((uint16_t)VAD_MIN_TRIGGER, (uint16_t)(noise_ema * VAD_TRIGGER_FACTOR));
            vad_silence = max((uint16_t)VAD_MIN_SILENCE,  (uint16_t)(noise_ema * VAD_SILENCE_FACTOR));
        }

        // Decision voix
        bool chunk_voice = recording
            ? (rms_filt >= vad_silence)
            : (rms_filt >= vad_trigger && zcr_ok);
        if (!recording)
            vote_count = chunk_voice ? (vote_count + 1) : 0;
        bool voice = recording ? chunk_voice : (vote_count >= VAD_VOTE_NEEDED);

        // Gain WAV
        if (MIC_GAIN != 1) {
            for (size_t i = 0; i < n_samp; i++) {
                int32_t s = (int32_t)samples[i] * MIC_GAIN;
                if (s >  32767) s =  32767;
                if (s < -32768) s = -32768;
                samples[i] = (int16_t)s;
            }
        }

        // Stocker le chunk gaine dans le buffer pre-roll (ecrase le plus vieux si plein)
        if (!recording) {
            memcpy(preroll + preroll_head * VAD_CHUNK_BYTES, buf, bytes_read);
            preroll_head  = (preroll_head + 1) % PRE_ROLL_CHUNKS;
            if (preroll_count < PRE_ROLL_CHUNKS) preroll_count++;
        }

        uint32_t now = millis();

        // ---- MODE VEILLE ----
        if (!recording) {
            if (now - last_veille >= 500) {
                Serial.printf("[VEILLE] raw=%4u filt=%4u EMA=%.0f trig=%u %3.0f%% zcr=%3u%s vote=%d/%d%s\n",
                    rms_peak, rms_filt, noise_ema, vad_trigger,
                    vad_trigger > 0 ? (float)rms_filt * 100.0f / vad_trigger : 0.0f,
                    zcr, zcr_ok ? "" : "(KO)",
                    vote_count, VAD_VOTE_NEEDED,
                    chunk_voice ? " *" : "");
                rms_peak    = 0;
                last_veille = now;
            }

            if (voice) {
                // Generer le nom de session horodate
                get_timestamp(session_id, sizeof(session_id));

                char dir[48], wav_path[64], photo_path[64];
                snprintf(dir,        sizeof(dir),        "/session_%s",           session_id);
                snprintf(wav_path,   sizeof(wav_path),   "/session_%s/audio.wav", session_id);
                snprintf(photo_path, sizeof(photo_path), "/session_%s/photo.jpg", session_id);

                SD.mkdir(dir);
                take_photo(photo_path);

                wav = SD.open(wav_path, FILE_WRITE);
                if (!wav) {
                    Serial.printf("ERREUR: impossible de creer %s\n", wav_path);
                    continue;
                }
                write_wav_header(wav, 0);
                written = 0;

                // Flush pre-roll : ecrire les chunks captures avant le declenchement
                if (preroll_count > 0) {
                    int start_slot = (preroll_head - preroll_count + PRE_ROLL_CHUNKS) % PRE_ROLL_CHUNKS;
                    for (int i = 0; i < preroll_count; i++) {
                        int slot = (start_slot + i) % PRE_ROLL_CHUNKS;
                        wav.write(preroll + slot * VAD_CHUNK_BYTES, VAD_CHUNK_BYTES);
                        written += VAD_CHUNK_BYTES;
                    }
                    Serial.printf("  [PRE-ROLL] %d chunks (%.0fms) prepends\n",
                                  preroll_count, preroll_count * (float)VAD_CHUNK_MS);
                    preroll_count = 0;
                    preroll_head  = 0;
                }
                silence_ms   = 0;
                vote_count   = 0;
                rms_peak     = 0;
                rec_start_ms = now;
                last_print   = now;
                recording    = true;
                Serial.printf("\n[%s] Demarre -> %s  (rms_filt=%u trigger=%u)\n",
                              session_id, wav_path, rms_filt, vad_trigger);
            }
        }
        // ---- MODE ENREGISTREMENT ----
        else {
            wav.write(buf, bytes_read);
            written += bytes_read;

            if (voice) {
                silence_ms = 0;
            } else {
                silence_ms += VAD_CHUNK_MS;
            }

            if (now - last_print >= 1000) {
                uint32_t rec_sec = (now - rec_start_ms) / 1000;
                char bar[21];
                uint8_t filled = (silence_ms * 20) / SILENCE_TIMEOUT_MS;
                if (filled > 20) filled = 20;
                for (uint8_t i = 0; i < 20; i++)
                    bar[i] = (i < filled) ? '=' : '.';
                bar[20] = '\0';
                Serial.printf("[%s] %3lus | rms_filt=%4u sil=%u | [%s] %.1f/%.0fs\n",
                              session_id, rec_sec, rms_filt, vad_silence,
                              bar, silence_ms / 1000.0f, SILENCE_TIMEOUT_MS / 1000.0f);
                last_print = now;
            }

            bool timeout = ((now - rec_start_ms) >= (uint32_t)MAX_RECORD_SEC * 1000);
            if (silence_ms >= SILENCE_TIMEOUT_MS || timeout) {
                write_wav_header(wav, written);
                wav.close();
                float dur = written / (float)(SAMPLE_RATE * 2);
                Serial.printf("[%s] Termine : %.1fs | %lu bytes | %s\n",
                              session_id, dur, written,
                              timeout ? "TIMEOUT MAX" : "SILENCE 5s");
                Serial.println("[VAD] En attente de voix...\n");
                recording = false;
            }
        }
    }

    free(buf);
    free(fbuf);
    free(preroll);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);  // ecriture non-bloquante -- jamais bloquer si moniteur absent
    delay(500);
    Serial.println("\n\n========================================");
    Serial.println("=== XIAO ESP32S3 Sense - VAD + Photo ===");
    Serial.println("========================================");
    Serial.printf("CPU: %lu MHz  Flash: %luMB  PSRAM: %luKB\n",
                  getCpuFrequencyMhz(),
                  spi_flash_get_chip_size() / (1024*1024),
                  ESP.getPsramSize() / 1024);

    // WiFi + NTP
    Serial.println("\n[INIT 1/4] WiFi + NTP...");
    wifi_ntp_init();

    // SD
    Serial.println("[INIT 2/4] Carte SD...");
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    delay(100);
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    delay(100);
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("  ERREUR: SD non detectee ! (carte inseree ?)");
        Serial.println("  --> Blocage. Inserer la SD et redemarrer.");
        while (1) delay(1000);
    }
    Serial.printf("  SD OK - Taille: %lluMB  Type: %d\n",
                  SD.cardSize() / (1024*1024), SD.cardType());

    // Camera
    Serial.println("[INIT 3/4] Camera OV2640...");
    cam_ok = camera_init();
    if (!cam_ok)
        Serial.println("  AVERTISSEMENT: camera indisponible, photos desactivees");

    // Micro PDM
    Serial.println("[INIT 4/4] Microphone PDM...");
    if (!mic_init()) {
        Serial.println("  ERREUR: micro impossible a initialiser. Blocage.");
        while (1) delay(1000);
    }

    Serial.println("\n[INIT] Tout OK -- demarrage VAD\n");
    record_vad();
}

// ============================================================
//  LOOP -- inactif
// ============================================================
void loop() {
    delay(5000);
}
