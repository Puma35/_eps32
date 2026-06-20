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
 *    /session_YYYYMMDD_HHMMSS/photo.jpg  -- capture OV2640 UXGA (1600x1200) JPEG
 *    /session_YYYYMMDD_HHMMSS/meta.json  -- coordonnees GPS + satellites + UTC GPS
 *
 *  WiFi : mode AP+STA simultane.
 *    STA : connexion au reseau maison pour NTP (reste actif — serveur web accessible).
 *    AP  : reseau "ESP32-VAD" toujours disponible pour acces direct terrain/dev.
 *  Interface admin : http://192.168.4.1 (AP) ou http://<IP_STA>
 *    - Carte GPS temps reel (Leaflet / OpenStreetMap)
 *    - Terminal serie mirore via WebSocket
 *    - Statut systeme (heap, PSRAM, EMA, enregistrement en cours)
 *
 *  GPS BN-220 : UART1 GPIO44(RX=D7) / GPIO43(TX=D6), 9600 baud.
 *    - Fournit les coordonnees a chaque session (meta.json).
 *    - Synchronise la RTC si le NTP a echoue (UTC GPS).
 *
 *  Brochage (XIAO ESP32S3 Sense)
 *  -------------------------------------------------------------------
 *  Micro  PDM  CLK=GPIO42  DATA=GPIO41
 *  SD     CS=GPIO21 SCK=GPIO7 MISO=GPIO8 MOSI=GPIO9
 *  Camera XCLK=GPIO10 SIOD=GPIO40 SIOC=GPIO39
 *         D0-D7=GPIO15/17/18/16/14/12/11/48
 *         VSYNC=GPIO38 HREF=GPIO47 PCLK=GPIO13
 *  GPS    RX=GPIO44(D7) TX=GPIO43(D6)
 * ============================================================
 */

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <time.h>
#include "driver/i2s.h"
#include "esp_camera.h"
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include "credentials.h"    // WIFI_SSID, WIFI_PASSWORD, AP_SSID, AP_PASS
#include "web_dashboard.h"  // DASHBOARD_HTML

// ---- WiFi + NTP ----
#define NTP_SERVER       "pool.ntp.org"
#define NTP_TIMEOUT_MS   8000
#define TZ_FR            "CET-1CEST,M3.5.0,M10.5.0/3"

// ---- GPS ----
#define GPS_RX_PIN  44   // D7 — reçoit TX du BN-220
#define GPS_TX_PIN  43   // D6 — envoie vers RX du BN-220 (optionnel)
#define GPS_BAUD    9600

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
#define MIC_GAIN         8

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

#define PRE_ROLL_MS      300
#define PRE_ROLL_CHUNKS  (PRE_ROLL_MS / VAD_CHUNK_MS)

// ---- Globals VAD ----
float    noise_ema       = 1000.0f;
float    noise_floor_abs = 1000.0f;
uint16_t vad_trigger     = VAD_MIN_TRIGGER;
uint16_t vad_silence     = VAD_MIN_SILENCE;
static bool ntp_synced   = false;
static bool recording_flag = false;  // true pendant une session — lu par le serveur web

// ============================================================
//  CONFIG RUNTIME — réglable via /api/config, persistée en NVS
// ============================================================
static Preferences prefs;

// VAD / audio
float    cfg_trigger_factor  = VAD_TRIGGER_FACTOR;
float    cfg_silence_factor  = VAD_SILENCE_FACTOR;
int      cfg_zcr_min         = ZCR_MIN;
int      cfg_zcr_max         = ZCR_MAX;
int      cfg_vote_needed     = VAD_VOTE_NEEDED;
uint32_t cfg_silence_timeout = SILENCE_TIMEOUT_MS;
uint32_t cfg_max_record_sec  = MAX_RECORD_SEC;
int      cfg_mic_gain        = MIC_GAIN;
// Camera — framesize_t : 8=VGA 9=SVGA 10=XGA 12=SXGA 13=UXGA
int      cfg_cam_framesize   = 9;    // SVGA : bon compromis aperçu/qualité
int      cfg_cam_quality     = 12;   // 10..40 (plus haut = plus compressé)
int      cfg_cam_aec         = 150;
int      cfg_cam_agc         = 20;
int      cfg_cam_vflip       = 1;
int      cfg_cam_hmirror     = 1;
// Identifiants WiFi STA — défaut = credentials.h, surchargeables via /api/config (NVS)
char     cfg_wifi_ssid[33]   = WIFI_SSID;
char     cfg_wifi_pass[65]   = WIFI_PASSWORD;

// Test de connexion WiFi (déclenché par /api/wifi/test, exécuté dans la boucle VAD)
static volatile bool wifi_test_req   = false;
static volatile int  wifi_test_state = 0;   // 0=idle 1=en cours 2=termine
static volatile bool wifi_test_ok    = false;
static volatile int  wifi_test_rssi  = 0;
static char wifi_test_ssid[33] = "";
static char wifi_test_pass[65] = "";
static char wifi_test_ip[20]   = "";

// Aperçu caméra : double buffer PSRAM, servi par /api/photo (hors enregistrement)
#define PREVIEW_BUF_SZ      (120 * 1024)
static uint8_t          *preview_buf[2]     = { nullptr, nullptr };
static volatile int      preview_active     = -1;
static volatile size_t   preview_len        = 0;
static volatile uint32_t preview_want_until = 0;
static uint32_t          preview_last_ms    = 0;

static volatile bool     g_restart_req      = false;

void config_load() {
    prefs.begin("vadcfg", true);
    cfg_trigger_factor  = prefs.getFloat("trigf",  cfg_trigger_factor);
    cfg_silence_factor  = prefs.getFloat("silf",   cfg_silence_factor);
    cfg_zcr_min         = prefs.getInt  ("zcrmin", cfg_zcr_min);
    cfg_zcr_max         = prefs.getInt  ("zcrmax", cfg_zcr_max);
    cfg_vote_needed     = prefs.getInt  ("vote",   cfg_vote_needed);
    cfg_silence_timeout = prefs.getULong("siltmo", cfg_silence_timeout);
    cfg_max_record_sec  = prefs.getULong("maxrec", cfg_max_record_sec);
    cfg_mic_gain        = prefs.getInt  ("gain",   cfg_mic_gain);
    cfg_cam_framesize   = prefs.getInt  ("cfs",    cfg_cam_framesize);
    cfg_cam_quality     = prefs.getInt  ("cq",     cfg_cam_quality);
    cfg_cam_aec         = prefs.getInt  ("caec",   cfg_cam_aec);
    cfg_cam_agc         = prefs.getInt  ("cagc",   cfg_cam_agc);
    cfg_cam_vflip       = prefs.getInt  ("cvf",    cfg_cam_vflip);
    cfg_cam_hmirror     = prefs.getInt  ("chm",    cfg_cam_hmirror);
    String ss = prefs.getString("wssid", cfg_wifi_ssid);
    String sp = prefs.getString("wpass", cfg_wifi_pass);
    strlcpy(cfg_wifi_ssid, ss.c_str(), sizeof(cfg_wifi_ssid));
    strlcpy(cfg_wifi_pass, sp.c_str(), sizeof(cfg_wifi_pass));
    prefs.end();
}

void config_save() {
    prefs.begin("vadcfg", false);
    prefs.putFloat("trigf",  cfg_trigger_factor);
    prefs.putFloat("silf",   cfg_silence_factor);
    prefs.putInt  ("zcrmin", cfg_zcr_min);
    prefs.putInt  ("zcrmax", cfg_zcr_max);
    prefs.putInt  ("vote",   cfg_vote_needed);
    prefs.putULong("siltmo", cfg_silence_timeout);
    prefs.putULong("maxrec", cfg_max_record_sec);
    prefs.putInt  ("gain",   cfg_mic_gain);
    prefs.putInt  ("cfs",    cfg_cam_framesize);
    prefs.putInt  ("cq",     cfg_cam_quality);
    prefs.putInt  ("caec",   cfg_cam_aec);
    prefs.putInt  ("cagc",   cfg_cam_agc);
    prefs.putInt  ("cvf",    cfg_cam_vflip);
    prefs.putInt  ("chm",    cfg_cam_hmirror);
    prefs.putString("wssid", cfg_wifi_ssid);
    prefs.putString("wpass", cfg_wifi_pass);
    prefs.end();
}

// ---- GPS ----
static HardwareSerial gpsSerial(1);
static TinyGPSPlus    gps;

struct GpsSnapshot {
    bool    valid;
    double  lat, lng;
    float   alt_m;
    uint8_t satellites;
    int year; uint8_t month, day, hour, minute, second;
};
static GpsSnapshot last_gps = {};

// ---- Log ring buffer + WebSocket ----
#define LOG_BUF 8192
static char              log_ring[LOG_BUF];
static volatile uint32_t log_head = 0;
static SemaphoreHandle_t log_mutex = nullptr;

// ---- Serveur web (Core 0) ----
static AsyncWebServer webServer(80);
static AsyncWebSocket ws("/ws");

// ============================================================
//  LOG — miroir Serial + ring buffer WebSocket
// ============================================================
void term_printf(const char *fmt, ...) {
    char tmp[512];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);
    if (n <= 0) return;
    if (n >= (int)sizeof(tmp)) n = (int)sizeof(tmp) - 1;
    Serial.print(tmp);
    if (log_mutex && xSemaphoreTake(log_mutex, 0) == pdTRUE) {
        for (int i = 0; i < n; i++) {
            log_ring[log_head % LOG_BUF] = tmp[i];
            log_head++;
        }
        xSemaphoreGive(log_mutex);
    }
}

// ============================================================
//  WIFI + NTP
// ============================================================
void wifi_ntp_init() {
    // Démarrage immédiat du point d'accès (disponible même sans WiFi maison)
    WiFi.mode(WIFI_AP_STA);

    // Sondes diagnostic : log quand un client rejoint l'AP et reçoit une IP
    WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t) {
        term_printf("  [AP] >>> station connectee (total=%d)\n", WiFi.softAPgetStationNum());
    }, ARDUINO_EVENT_WIFI_AP_STACONNECTED);
    WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t info) {
        term_printf("  [AP] >>> IP attribuee au client : %s\n",
                   IPAddress(info.wifi_ap_staipassigned.ip.addr).toString().c_str());
    }, ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED);

    bool ap_ok = WiFi.softAP(AP_SSID, AP_PASS);
    term_printf("  AP demarre : %s (softAP=%s)  ->  http://%s\n",
               AP_SSID, ap_ok ? "OK" : "ECHEC", WiFi.softAPIP().toString().c_str());

    term_printf("  Connexion a '%s' (timeout %ds)...\n", cfg_wifi_ssid, NTP_TIMEOUT_MS/1000);
    WiFi.begin(cfg_wifi_ssid, cfg_wifi_pass);

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < NTP_TIMEOUT_MS) {
        delay(500);
        term_printf("  . status=%d  (%lums)\n", WiFi.status(), millis()-t0);
    }

    if (WiFi.status() != WL_CONNECTED) {
        term_printf("  ECHEC WiFi (status=%d) -- sessions epoch\n", WiFi.status());
        // Garde l'AP actif — abandonne seulement la connexion STA
        WiFi.disconnect(false);
        return;
    }

    term_printf("  Connecte !  IP=%s  RSSI=%ddBm  ->  http://%s\n",
               WiFi.localIP().toString().c_str(), WiFi.RSSI(),
               WiFi.localIP().toString().c_str());

    term_printf("  NTP sync depuis %s (TZ France)...\n", NTP_SERVER);
    configTzTime(TZ_FR, NTP_SERVER);

    struct tm timeinfo;
    t0 = millis();
    while (!getLocalTime(&timeinfo, 100) && millis() - t0 < 5000) {
        delay(300);
        term_printf("  . attente NTP (%lums)\n", millis()-t0);
    }

    if (getLocalTime(&timeinfo, 0)) {
        ntp_synced = true;
        char buf[32];
        strftime(buf, sizeof(buf), "%d/%m/%Y %H:%M:%S", &timeinfo);
        term_printf("  Heure synchronisee : %s\n", buf);
    } else {
        term_printf("  NTP timeout -- sessions epoch\n");
    }
    // WiFi STA reste actif pour le serveur web
}

// Retourne le timestamp actuel sous forme "YYYYMMDD_HHMMSS"
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
//  HP 300 Hz + LP 3400 Hz (Butterworth 2nd order, Fs=16 kHz)
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
//  RMS AC
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

void prepare_wav_chunk(int16_t *samples, size_t n) {
    if (n == 0) return;
    int64_t mean_sum = 0;
    for (size_t i = 0; i < n; i++) mean_sum += samples[i];
    int32_t dc = (int32_t)(mean_sum / (int64_t)n);
    for (size_t i = 0; i < n; i++) {
        int32_t s = ((int32_t)samples[i] - dc) * cfg_mic_gain;
        if (s >  32767) s =  32767;
        if (s < -32768) s = -32768;
        samples[i] = (int16_t)s;
    }
}

// ============================================================
//  ZCR
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

void apply_camera_settings();   // défini après take_photo()

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
    cfg.frame_size   = (framesize_t)cfg_cam_framesize;
    cfg.jpeg_quality = cfg_cam_quality;
    cfg.fb_count     = 1;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        term_printf("ERREUR camera: 0x%x\n", err);
        return false;
    }
    apply_camera_settings();
    term_printf("Camera OK (OV2640 framesize=%d quality=%d, reglages appliques)\n",
               cfg_cam_framesize, cfg_cam_quality);
    return true;
}

// Applique les réglages caméra runtime (à l'init et après /api/config)
void apply_camera_settings() {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return;
    s->set_framesize(s, (framesize_t)cfg_cam_framesize);
    s->set_quality  (s, cfg_cam_quality);
    s->set_vflip    (s, cfg_cam_vflip   ? 1 : 0);
    s->set_hmirror  (s, cfg_cam_hmirror ? 1 : 0);
    s->set_exposure_ctrl(s, 0);
    s->set_aec_value(s, cfg_cam_aec);
    s->set_gain_ctrl(s, 0);
    s->set_agc_gain (s, cfg_cam_agc);
    s->set_aec2     (s, 0);
    s->set_ae_level (s, 0);
    s->set_whitebal (s, 1);
    s->set_awb_gain (s, 1);
    s->set_bpc      (s, 1);
    s->set_wpc      (s, 1);
}

// Capture une image dans le buffer d'aperçu (PSRAM, double buffer). Core 1 / hors enregistrement.
void preview_capture() {
    if (!cam_ok) return;
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return;
    int next = (preview_active == 0) ? 1 : 0;
    if (fb->len <= PREVIEW_BUF_SZ && preview_buf[next]) {
        memcpy(preview_buf[next], fb->buf, fb->len);
        preview_len    = fb->len;
        preview_active = next;
    }
    esp_camera_fb_return(fb);
}

void take_photo(const char *path) {
    if (!cam_ok) return;
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        term_printf("[CAM] ERREUR: capture echouee\n");
        return;
    }
    File f = SD.open(path, FILE_WRITE);
    if (!f) {
        term_printf("[CAM] ERREUR: impossible d'ouvrir %s\n", path);
        esp_camera_fb_return(fb);
        return;
    }
    f.write(fb->buf, fb->len);
    f.close();
    term_printf("[CAM] %s -- %.1f kB\n", path, fb->len / 1024.0f);
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
        .dma_buf_count        = 16,
        .dma_buf_len          = 256,
        .use_apll             = false,
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0,
    };
    if (i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL) != ESP_OK) {
        term_printf("ERREUR: i2s_driver_install\n");
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
        term_printf("ERREUR: i2s_set_pin\n");
        return false;
    }
    term_printf("Micro PDM OK (GPIO42=CLK, GPIO41=DATA, 16kHz)\n");
    return true;
}

// ============================================================
//  SEED EMA
// ============================================================
void seed_noise_ema(uint8_t *buf, int16_t *fbuf) {
    term_printf("[SEED] Mesure plancher bruit (%dms) -- silence svp...\n", VAD_SEED_MS);
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
    vad_trigger     = max((uint16_t)VAD_MIN_TRIGGER, (uint16_t)(noise_floor_abs * cfg_trigger_factor));
    vad_silence     = max((uint16_t)VAD_MIN_SILENCE,  (uint16_t)(noise_floor_abs * cfg_silence_factor));
    term_printf("[SEED] min=%.0f moy=%.0f plancher=%.0f | trigger=%u | silence=%u\n",
               rms_min, rms_avg, noise_floor_abs, vad_trigger, vad_silence);
}

// ============================================================
//  UTILITAIRES SD
// ============================================================
bool sd_has_space(uint64_t min_bytes = 200ULL * 1024 * 1024) {
    return (SD.totalBytes() - SD.usedBytes()) >= min_bytes;
}

// ============================================================
//  GPS — polling non bloquant, appele dans la boucle VAD
// ============================================================
void gps_poll() {
    while (gpsSerial.available()) gps.encode(gpsSerial.read());

    if (!gps.location.isUpdated()) return;

    last_gps.valid      = gps.location.isValid();
    last_gps.lat        = gps.location.lat();
    last_gps.lng        = gps.location.lng();
    last_gps.alt_m      = gps.altitude.isValid() ? gps.altitude.meters() : 0.0f;
    last_gps.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
    last_gps.year       = gps.date.year();
    last_gps.month      = gps.date.month();
    last_gps.day        = gps.date.day();
    last_gps.hour       = gps.time.hour();
    last_gps.minute     = gps.time.minute();
    last_gps.second     = gps.time.second();

    // Synchronise la RTC si NTP n'a pas réussi
    if (!ntp_synced && gps.location.isValid() &&
        gps.date.isValid() && gps.time.isValid())
    {
        struct tm t = {};
        t.tm_year = last_gps.year  - 1900;
        t.tm_mon  = last_gps.month - 1;
        t.tm_mday = last_gps.day;
        t.tm_hour = last_gps.hour;
        t.tm_min  = last_gps.minute;
        t.tm_sec  = last_gps.second;
        time_t epoch = mktime(&t);
        struct timeval tv = { epoch, 0 };
        settimeofday(&tv, nullptr);
        ntp_synced = true;
        term_printf("[GPS] RTC synchronisee via GPS (UTC) : %04d-%02d-%02d %02d:%02d:%02d\n",
                   last_gps.year, last_gps.month, last_gps.day,
                   last_gps.hour, last_gps.minute, last_gps.second);
    }
}

// ============================================================
//  GPS — fichier sidecar meta.json dans le dossier session
// ============================================================
void write_meta(const char *session_dir, const GpsSnapshot &s) {
    char path[64];
    snprintf(path, sizeof(path), "%s/meta.json", session_dir);
    File f = SD.open(path, FILE_WRITE);
    if (!f) return;
    if (s.valid)
        f.printf("{\"gps_fix\":true,\"lat\":%.6f,\"lng\":%.6f,"
                 "\"alt_m\":%.1f,\"satellites\":%u,"
                 "\"utc\":\"%04d-%02d-%02dT%02d:%02d:%02dZ\"}\n",
                 s.lat, s.lng, s.alt_m, s.satellites,
                 s.year, s.month, s.day, s.hour, s.minute, s.second);
    else
        f.print("{\"gps_fix\":false}\n");
    f.close();
}

// ============================================================
//  SERVEUR WEB + WEBSOCKET (tourne sur Core 0)
// ============================================================
void webserver_task(void *) {
    webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *r) {
        term_printf("  [WEB] >>> GET / depuis %s\n", r->client()->remoteIP().toString().c_str());
        r->send(200, "text/html", DASHBOARD_HTML);
    });

    webServer.on("/api/gps", HTTP_GET, [](AsyncWebServerRequest *r) {
        char buf[256];
        snprintf(buf, sizeof(buf),
                 "{\"valid\":%s,\"lat\":%.6f,\"lng\":%.6f,\"alt\":%.1f,\"sats\":%u}",
                 last_gps.valid ? "true" : "false",
                 last_gps.lat, last_gps.lng, last_gps.alt_m, last_gps.satellites);
        r->send(200, "application/json", buf);
    });

    webServer.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *r) {
        char buf[256];
        int rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
        snprintf(buf, sizeof(buf),
                 "{\"heap\":%lu,\"psram\":%lu,\"ntp_synced\":%s,"
                 "\"recording\":%s,\"noise_ema\":%.0f,\"rssi\":%d,\"sta_ip\":\"%s\"}",
                 (unsigned long)ESP.getFreeHeap(),
                 (unsigned long)ESP.getFreePsram(),
                 ntp_synced     ? "true" : "false",
                 recording_flag ? "true" : "false",
                 noise_ema, rssi,
                 (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString().c_str() : "");
        r->send(200, "application/json", buf);
    });

    // Aperçu caméra : sert la dernière image capturée (PSRAM). Demande aussi une capture fraîche.
    webServer.on("/api/photo", HTTP_GET, [](AsyncWebServerRequest *r) {
        preview_want_until = millis() + 8000;   // garde l'aperçu actif ~8s
        int    idx = preview_active;
        size_t len = preview_len;
        if (idx < 0 || len == 0 || !preview_buf[idx]) {
            r->send(503, "text/plain", "apercu en preparation");
            return;
        }
        uint8_t *b = preview_buf[idx];
        AsyncWebServerResponse *resp = r->beginResponse("image/jpeg", len,
            [b, len](uint8_t *out, size_t maxLen, size_t index) -> size_t {
                size_t rem = len - index;
                size_t n   = rem < maxLen ? rem : maxLen;
                memcpy(out, b + index, n);
                return n;
            });
        resp->addHeader("Cache-Control", "no-store");
        r->send(resp);
    });

    // Config : lecture
    webServer.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *r) {
        char buf[512];
        snprintf(buf, sizeof(buf),
            "{\"trigger_factor\":%.2f,\"silence_factor\":%.2f,\"zcr_min\":%d,\"zcr_max\":%d,"
            "\"vote_needed\":%d,\"silence_timeout_ms\":%lu,\"max_record_sec\":%lu,\"mic_gain\":%d,"
            "\"cam_framesize\":%d,\"cam_quality\":%d,\"cam_aec\":%d,\"cam_agc\":%d,"
            "\"cam_vflip\":%d,\"cam_hmirror\":%d,\"wifi_ssid\":\"%s\"}",
            cfg_trigger_factor, cfg_silence_factor, cfg_zcr_min, cfg_zcr_max,
            cfg_vote_needed, (unsigned long)cfg_silence_timeout, (unsigned long)cfg_max_record_sec,
            cfg_mic_gain, cfg_cam_framesize, cfg_cam_quality, cfg_cam_aec, cfg_cam_agc,
            cfg_cam_vflip, cfg_cam_hmirror, cfg_wifi_ssid);
        r->send(200, "application/json", buf);
    });

    // Config : écriture (form urlencoded), validée puis persistée en NVS
    webServer.on("/api/config", HTTP_POST, [](AsyncWebServerRequest *r) {
        auto pf = [&](const char *k, float d) -> float {
            auto p = r->getParam(k, true); return p ? p->value().toFloat() : d;
        };
        auto pi = [&](const char *k, int d) -> int {
            auto p = r->getParam(k, true); return p ? (int)p->value().toInt() : d;
        };
        cfg_trigger_factor  = constrain(pf("trigger_factor", cfg_trigger_factor), 1.5f, 30.0f);
        cfg_silence_factor  = constrain(pf("silence_factor", cfg_silence_factor), 1.0f, 20.0f);
        cfg_zcr_min         = constrain(pi("zcr_min", cfg_zcr_min), 0, 400);
        cfg_zcr_max         = constrain(pi("zcr_max", cfg_zcr_max), 1, 800);
        cfg_vote_needed     = constrain(pi("vote_needed", cfg_vote_needed), 1, 20);
        cfg_silence_timeout = (uint32_t)constrain(pi("silence_timeout_ms", (int)cfg_silence_timeout), 1000, 60000);
        cfg_max_record_sec  = (uint32_t)constrain(pi("max_record_sec", (int)cfg_max_record_sec), 5, 600);
        cfg_mic_gain        = constrain(pi("mic_gain", cfg_mic_gain), 1, 64);
        cfg_cam_framesize   = constrain(pi("cam_framesize", cfg_cam_framesize), 5, 13);
        cfg_cam_quality     = constrain(pi("cam_quality", cfg_cam_quality), 8, 40);
        cfg_cam_aec         = constrain(pi("cam_aec", cfg_cam_aec), 0, 1200);
        cfg_cam_agc         = constrain(pi("cam_agc", cfg_cam_agc), 0, 30);
        cfg_cam_vflip       = pi("cam_vflip", cfg_cam_vflip) ? 1 : 0;
        cfg_cam_hmirror     = pi("cam_hmirror", cfg_cam_hmirror) ? 1 : 0;
        // WiFi : SSID toujours pris si fourni ; mot de passe seulement si non vide (vide = inchangé)
        { auto p = r->getParam("wifi_ssid", true);
          if (p && p->value().length()) strlcpy(cfg_wifi_ssid, p->value().c_str(), sizeof(cfg_wifi_ssid)); }
        { auto p = r->getParam("wifi_pass", true);
          if (p && p->value().length()) strlcpy(cfg_wifi_pass, p->value().c_str(), sizeof(cfg_wifi_pass)); }
        config_save();
        apply_camera_settings();
        term_printf("[CFG] Configuration mise a jour via web\n");
        r->send(200, "application/json", "{\"ok\":true}");
    });

    // Redémarrage logiciel
    webServer.on("/api/restart", HTTP_POST, [](AsyncWebServerRequest *r) {
        r->send(200, "application/json", "{\"ok\":true}");
        g_restart_req = true;
    });

    // Test de connexion WiFi : utilise les identifiants fournis (sinon ceux sauvegardés)
    webServer.on("/api/wifi/test", HTTP_POST, [](AsyncWebServerRequest *r) {
        if (recording_flag) {
            r->send(409, "application/json", "{\"error\":\"enregistrement en cours\"}");
            return;
        }
        auto ps = r->getParam("wifi_ssid", true);
        auto pp = r->getParam("wifi_pass", true);
        strlcpy(wifi_test_ssid, (ps && ps->value().length()) ? ps->value().c_str() : cfg_wifi_ssid,
                sizeof(wifi_test_ssid));
        strlcpy(wifi_test_pass, (pp && pp->value().length()) ? pp->value().c_str() : cfg_wifi_pass,
                sizeof(wifi_test_pass));
        wifi_test_ok    = false;
        wifi_test_rssi  = 0;
        wifi_test_ip[0] = '\0';
        wifi_test_state = 1;
        wifi_test_req   = true;
        r->send(200, "application/json", "{\"started\":true}");
    });

    // Résultat du dernier test WiFi
    webServer.on("/api/wifi/status", HTTP_GET, [](AsyncWebServerRequest *r) {
        char buf[160];
        snprintf(buf, sizeof(buf),
                 "{\"state\":%d,\"ok\":%s,\"rssi\":%d,\"ip\":\"%s\"}",
                 wifi_test_state, wifi_test_ok ? "true" : "false",
                 wifi_test_rssi, wifi_test_ip);
        r->send(200, "application/json", buf);
    });

    // À la connexion d'un client, envoie les logs déjà accumulés
    ws.onEvent([](AsyncWebSocket *, AsyncWebSocketClient *client,
                  AwsEventType type, void *, uint8_t *, size_t) {
        if (type != WS_EVT_CONNECT || !log_mutex) return;
        if (xSemaphoreTake(log_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
        uint32_t head = log_head;
        uint32_t avail = (head > LOG_BUF) ? (uint32_t)LOG_BUF : head;
        if (avail > 0) {
            uint32_t tail = head - avail;
            char *hist = (char *)malloc(avail + 1);
            if (hist) {
                for (uint32_t i = 0; i < avail; i++)
                    hist[i] = log_ring[(tail + i) % LOG_BUF];
                hist[avail] = '\0';
                client->text(hist);
                free(hist);
            }
        }
        xSemaphoreGive(log_mutex);
    });

    webServer.addHandler(&ws);
    webServer.begin();
    term_printf("  [WEB] webServer.begin() execute -- port 80 en ecoute (heap libre=%lu o)\n",
               (unsigned long)ESP.getFreeHeap());

    // Boucle : pousse les nouveaux logs vers tous les clients WS
    static uint32_t ws_tail = 0;
    for (;;) {
        if (ws.count() > 0 && log_head != ws_tail && log_mutex &&
            xSemaphoreTake(log_mutex, 0) == pdTRUE)
        {
            uint32_t head  = log_head;
            uint32_t avail = head - ws_tail;
            if (avail > LOG_BUF) { ws_tail = head - LOG_BUF; avail = LOG_BUF; }
            char *out = (char *)malloc(avail + 1);
            if (out) {
                for (uint32_t i = 0; i < avail; i++)
                    out[i] = log_ring[(ws_tail + i) % LOG_BUF];
                out[avail] = '\0';
                ws.textAll(out);
                free(out);
            }
            ws_tail = head;
            xSemaphoreGive(log_mutex);
        }
        ws.cleanupClients();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ============================================================
//  BOUCLE VAD + ENREGISTREMENT + SESSION HORODATEE
// ============================================================
void record_vad() {
    uint8_t  *buf     = (uint8_t  *)malloc(VAD_CHUNK_BYTES);
    int16_t  *fbuf    = (int16_t  *)malloc(VAD_CHUNK_BYTES);
    uint8_t  *preroll = (uint8_t  *)ps_malloc((size_t)PRE_ROLL_CHUNKS * VAD_CHUNK_BYTES);
    if (!buf || !fbuf || !preroll) {
        term_printf("ERREUR: malloc echoue (buf=%p fbuf=%p preroll=%p)\n", buf, fbuf, preroll);
        return;
    }
    int preroll_head  = 0;
    int preroll_count = 0;

    File     wav;
    uint32_t written        = 0;
    uint32_t silence_ms     = 0;
    uint32_t rec_start_ms   = 0;
    uint32_t last_print     = 0;
    uint32_t last_veille    = 0;
    uint32_t last_header_ms = 0;
    uint16_t rms_peak       = 0;
    int      vote_count     = 0;
    char     session_id[24] = "";
    char     photo_path[64] = "";
    char     session_dir[48]= "";
    GpsSnapshot session_snap = {};

    seed_noise_ema(buf, fbuf);

    term_printf("[VAD] EMA=%.2f | trigger=%u | silence=%u | GainX%d | ZCR[%d-%d] | NTP=%s\n",
               EMA_ALPHA, vad_trigger, vad_silence, cfg_mic_gain, cfg_zcr_min, cfg_zcr_max,
               ntp_synced ? "OK" : "NON");
    term_printf("[VAD] En attente de voix...\n");

    while (true) {
        size_t bytes_read = 0;
        i2s_read(I2S_NUM_0, buf, VAD_CHUNK_BYTES, &bytes_read, pdMS_TO_TICKS(500));
        if (bytes_read == 0) continue;

        // Polling GPS non bloquant
        gps_poll();

        // Redémarrage demandé depuis l'interface web
        if (g_restart_req) {
            term_printf("[SYS] Redemarrage demande via interface web...\n");
            delay(200);
            ESP.restart();
        }

        // Test de connexion WiFi demandé depuis l'interface (hors enregistrement)
        if (wifi_test_req && !recording_flag) {
            wifi_test_req = false;
            term_printf("[WIFI] Test connexion a '%s'...\n", wifi_test_ssid);
            WiFi.begin(wifi_test_ssid, wifi_test_pass);
            uint32_t t0 = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) delay(200);
            wifi_test_ok   = (WiFi.status() == WL_CONNECTED);
            wifi_test_rssi = wifi_test_ok ? WiFi.RSSI() : 0;
            strlcpy(wifi_test_ip, wifi_test_ok ? WiFi.localIP().toString().c_str() : "",
                    sizeof(wifi_test_ip));
            wifi_test_state = 2;
            term_printf("[WIFI] Test termine : %s  RSSI=%d  IP=%s\n",
                       wifi_test_ok ? "OK" : "ECHEC", wifi_test_rssi, wifi_test_ip);
        }

        // Aperçu caméra : capture périodique quand un client regarde, hors enregistrement.
        // Co-localisée avec le micro sur le même core pour éviter le conflit I2S/caméra.
        if (!recording_flag && cam_ok && millis() < preview_want_until &&
            millis() - preview_last_ms >= 1200) {
            preview_capture();
            preview_last_ms = millis();
        }

        int16_t *samples = (int16_t *)buf;
        size_t   n_samp  = bytes_read / 2;

        uint16_t rms_raw  = compute_rms(samples, n_samp);
        if (!recording_flag && rms_raw > rms_peak) rms_peak = rms_raw;

        apply_bandpass(samples, fbuf, n_samp);
        uint16_t rms_filt = compute_rms(fbuf, n_samp);
        uint16_t zcr      = compute_zcr(fbuf, n_samp);
        bool     zcr_ok   = (zcr >= cfg_zcr_min && zcr <= cfg_zcr_max);

        if (!recording_flag) {
            noise_ema   = EMA_ALPHA * noise_ema + (1.0f - EMA_ALPHA) * (float)rms_filt;
            noise_ema   = max(noise_ema, noise_floor_abs * 0.5f);
            vad_trigger = max((uint16_t)VAD_MIN_TRIGGER, (uint16_t)(noise_ema * cfg_trigger_factor));
            vad_silence = max((uint16_t)VAD_MIN_SILENCE,  (uint16_t)(noise_ema * cfg_silence_factor));
        }

        bool chunk_voice = recording_flag
            ? (rms_filt >= vad_silence)
            : (rms_filt >= vad_trigger && zcr_ok);
        if (!recording_flag)
            vote_count = chunk_voice ? (vote_count + 1) : 0;
        bool voice = recording_flag ? chunk_voice : (vote_count >= cfg_vote_needed);

        prepare_wav_chunk(samples, n_samp);

        if (!recording_flag) {
            memcpy(preroll + preroll_head * VAD_CHUNK_BYTES, buf, bytes_read);
            preroll_head  = (preroll_head + 1) % PRE_ROLL_CHUNKS;
            if (preroll_count < PRE_ROLL_CHUNKS) preroll_count++;
        }

        uint32_t now = millis();

        // ---- MODE VEILLE ----
        if (!recording_flag) {
            if (now - last_veille >= 500) {
                term_printf("[VEILLE] raw=%4u filt=%4u EMA=%.0f trig=%u %3.0f%% zcr=%3u%s vote=%d/%d%s\n",
                    rms_peak, rms_filt, noise_ema, vad_trigger,
                    vad_trigger > 0 ? (float)rms_filt * 100.0f / vad_trigger : 0.0f,
                    zcr, zcr_ok ? "" : "(KO)",
                    vote_count, cfg_vote_needed,
                    chunk_voice ? " *" : "");
                rms_peak    = 0;
                last_veille = now;
            }

            if (voice) {
                get_timestamp(session_id, sizeof(session_id));
                char wav_path[64];
                snprintf(session_dir, sizeof(session_dir), "/session_%s",           session_id);
                snprintf(wav_path,    sizeof(wav_path),    "/session_%s/audio.wav", session_id);
                snprintf(photo_path,  sizeof(photo_path),  "/session_%s/photo.jpg", session_id);

                // Snapshot GPS au moment du déclenchement
                session_snap = last_gps;

                if (!sd_has_space()) {
                    term_printf("[VAD] AVERTISSEMENT: espace SD faible (<200MB) -- session ignoree\n");
                    vote_count = 0;
                    continue;
                }
                SD.mkdir(session_dir);

                wav = SD.open(wav_path, FILE_WRITE);
                if (!wav) {
                    term_printf("ERREUR: impossible de creer %s\n", wav_path);
                    continue;
                }
                write_wav_header(wav, 0);
                written = 0;

                if (preroll_count > 0) {
                    int start_slot = (preroll_head - preroll_count + PRE_ROLL_CHUNKS) % PRE_ROLL_CHUNKS;
                    for (int i = 0; i < preroll_count; i++) {
                        int slot = (start_slot + i) % PRE_ROLL_CHUNKS;
                        wav.write(preroll + slot * VAD_CHUNK_BYTES, VAD_CHUNK_BYTES);
                        written += VAD_CHUNK_BYTES;
                    }
                    term_printf("  [PRE-ROLL] %d chunks (%.0fms) prepends\n",
                               preroll_count, preroll_count * (float)VAD_CHUNK_MS);
                    preroll_count = 0;
                    preroll_head  = 0;
                }
                silence_ms      = 0;
                vote_count      = 0;
                rms_peak        = 0;
                rec_start_ms    = now;
                last_print      = now;
                last_header_ms  = now;
                recording_flag  = true;
                term_printf("\n[%s] Demarre -> %s  (rms_filt=%u trigger=%u  GPS:%s)\n",
                           session_id, wav_path, rms_filt, vad_trigger,
                           session_snap.valid ? "fix" : "no-fix");
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
                uint8_t filled = (silence_ms * 20) / cfg_silence_timeout;
                if (filled > 20) filled = 20;
                for (uint8_t i = 0; i < 20; i++)
                    bar[i] = (i < filled) ? '=' : '.';
                bar[20] = '\0';
                term_printf("[%s] %3lus | rms_filt=%4u sil=%u | [%s] %.1f/%.0fs\n",
                           session_id, rec_sec, rms_filt, vad_silence,
                           bar, silence_ms / 1000.0f, cfg_silence_timeout / 1000.0f);
                last_print = now;
            }

            if (now - last_header_ms >= 10000) {
                write_wav_header(wav, written);
                wav.seek(sizeof(wav_header_t) + written);
                last_header_ms = now;
            }

            bool timeout = ((now - rec_start_ms) >= cfg_max_record_sec * 1000);
            if (silence_ms >= cfg_silence_timeout || timeout) {
                write_wav_header(wav, written);
                wav.close();
                float dur = written / (float)(SAMPLE_RATE * 2);
                term_printf("[%s] Termine : %.1fs | %lu bytes | %s\n",
                           session_id, dur, written,
                           timeout ? "TIMEOUT MAX" : "SILENCE 5s");

                take_photo(photo_path);
                write_meta(session_dir, session_snap);

                term_printf("[VAD] En attente de voix...\n\n");
                recording_flag = false;
            }
        }
    }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);
    delay(500);

    // Mutex ring buffer — créé en premier pour que tous les term_printf() le trouvent
    log_mutex = xSemaphoreCreateMutex();

    // Config persistée (NVS) chargée avant tout — pilote VAD + caméra
    config_load();
    // Buffers d'aperçu caméra en PSRAM (double buffer pour /api/photo)
    preview_buf[0] = (uint8_t *)ps_malloc(PREVIEW_BUF_SZ);
    preview_buf[1] = (uint8_t *)ps_malloc(PREVIEW_BUF_SZ);

    term_printf("\n\n========================================\n");
    term_printf("=== XIAO ESP32S3 Sense - VAD + Photo ===\n");
    term_printf("========================================\n");
    term_printf("CPU: %lu MHz  Flash: %luMB  PSRAM: %luKB\n",
               getCpuFrequencyMhz(),
               spi_flash_get_chip_size() / (1024*1024),
               ESP.getPsramSize() / 1024);

    // WiFi + NTP + AP
    term_printf("\n[INIT 1/5] WiFi + NTP + AP...\n");
    wifi_ntp_init();

    // SD
    term_printf("[INIT 2/5] Carte SD...\n");
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    delay(100);
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    delay(100);
    if (!SD.begin(SD_CS_PIN)) {
        term_printf("  ERREUR: SD non detectee ! (carte inseree ?)\n");
        term_printf("  --> Blocage. Inserer la SD et redemarrer.\n");
        while (1) delay(1000);
    }
    term_printf("  SD OK - Taille: %lluMB  Type: %d\n",
               SD.cardSize() / (1024*1024), SD.cardType());

    // Camera
    term_printf("[INIT 3/5] Camera OV2640...\n");
    cam_ok = camera_init();
    if (!cam_ok)
        term_printf("  AVERTISSEMENT: camera indisponible, photos desactivees\n");

    // Micro PDM
    term_printf("[INIT 4/5] Microphone PDM...\n");
    if (!mic_init()) {
        term_printf("  ERREUR: micro impossible a initialiser. Blocage.\n");
        while (1) delay(1000);
    }

    // GPS + serveur web
    term_printf("[INIT 5/5] GPS + Serveur web...\n");
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    term_printf("  GPS UART1 demarre (RX=GPIO%d/D7, TX=GPIO%d/D6, %d baud)\n",
               GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
    xTaskCreatePinnedToCore(webserver_task, "webserver", 8192, nullptr, 1, nullptr, 0);
    term_printf("  Serveur admin port 80\n");
    term_printf("  -> AP  : http://192.168.4.1  (reseau %s)\n", AP_SSID);
    if (WiFi.status() == WL_CONNECTED)
        term_printf("  -> STA : http://%s\n", WiFi.localIP().toString().c_str());

    term_printf("\n[INIT] Tout OK -- demarrage VAD\n\n");
    record_vad();
}

// ============================================================
//  LOOP -- inactif (record_vad est une boucle infinie)
// ============================================================
void loop() {
    delay(5000);
}
