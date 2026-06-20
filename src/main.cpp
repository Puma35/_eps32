/**
 * ============================================================
 *  Projet  : picsetvoc
 *  Carte   : Seeed Studio XIAO ESP32S3 Sense
 *  MCU     : ESP32-S3R8 - Xtensa LX7 dual-core 32-bit @ 240 MHz
 *  Memoire : 8 MB PSRAM OPI on-chip + 8 MB Flash on-chip
 *  Outil   : PlatformIO / Framework Arduino (env: xiaos3sense)
 * ============================================================
 *
 *  VAD (Voice Activity Detection) :
 *    - RMS AC filtre passe-bande 300-3400 Hz
 *    - ZCR pour discriminer voix vs bruits parasites
 *    - EMA adaptative du bruit de fond
 *
 *  Chaque declenchement : /session_YYYYMMDD_HHMMSS/audio.wav + photo.jpg
 *
 *  WiFi AP "ESP32-Debug" / admin1234  ->  http://192.168.4.1/
 *  Interface web : timeline sessions, photos, audio, terminal live.
 * ============================================================
 */

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <time.h>
#include "driver/i2s.h"
#include "esp_camera.h"
#include "credentials.h"

// ---- Access Point ----
#define AP_SSID  "ESP32-Debug"
#define AP_PASS  "admin1234"

// ---- NTP ----
#define NTP_SERVER      "pool.ntp.org"
#define NTP_TIMEOUT_MS  8000
#define TZ_FR           "CET-1CEST,M3.5.0,M10.5.0/3"

// ---- Pins ----
#define MIC_CLK_PIN   42
#define MIC_DATA_PIN  41
#define SD_CS_PIN     21
#define SD_SCK_PIN    7
#define SD_MISO_PIN   8
#define SD_MOSI_PIN   9

// ---- Camera ----
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

// ---- Audio ----
#define SAMPLE_RATE  16000
#define MIC_GAIN     8

// ---- VAD ----
#define VAD_TRIGGER_FACTOR  6.0f
#define VAD_SILENCE_FACTOR  3.0f
#define VAD_MIN_TRIGGER     40
#define VAD_MIN_SILENCE     15
#define EMA_ALPHA           0.990f
#define VAD_VOTE_NEEDED     3
#define VAD_SEED_MS         1500
#define SILENCE_TIMEOUT_MS  5000
#define MAX_RECORD_SEC      120
#define VAD_CHUNK_MS        30
#define ZCR_MIN             3
#define ZCR_MAX             180
#define VAD_CHUNK_SAMPLES   (SAMPLE_RATE * VAD_CHUNK_MS / 1000)
#define VAD_CHUNK_BYTES     (VAD_CHUNK_SAMPLES * 2)
#define PRE_ROLL_MS         300
#define PRE_ROLL_CHUNKS     (PRE_ROLL_MS / VAD_CHUNK_MS)

// ---- Globals VAD ----
static float    noise_ema       = 1000.0f;
static float    noise_floor_abs = 1000.0f;
static uint16_t vad_trigger     = VAD_MIN_TRIGGER;
static uint16_t vad_silence     = VAD_MIN_SILENCE;
static bool     ntp_synced      = false;

// ---- Debug stats (partages VAD -> WebServer) ----
static volatile bool     g_recording     = false;
static volatile uint16_t g_rms_raw_d     = 0;
static volatile uint16_t g_rms_filt_d    = 0;
static volatile uint16_t g_zcr_d         = 0;
static volatile float    g_noise_ema_d   = 0.0f;
static volatile uint16_t g_trigger_d     = 0;
static volatile uint16_t g_vad_sil_d     = 0;
static volatile uint32_t g_wav_written_d = 0;
static volatile uint32_t g_sil_ms_d      = 0;
static char              g_session_d[24] = "---";
static char              g_last_photo[64]= "---";

// Mutex SD : VAD writes vs WebServer reads
static SemaphoreHandle_t g_sd_mutex = nullptr;

// ============================================================
//  PAGE HTML (interface complete)
// ============================================================
static const char HTML_PAGE[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 VAD</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:monospace;background:#0a0a0a;color:#0f0;height:100vh;display:flex;flex-direction:column;overflow:hidden}
header{background:#111;border-bottom:1px solid #2a2a2a;padding:8px 14px;display:flex;align-items:center;gap:12px;flex-shrink:0}
h1{color:#ff0;font-size:13px;letter-spacing:1px}
#dot{width:10px;height:10px;border-radius:50%;background:#0f0;flex-shrink:0}
#dot.rec{background:#f80;animation:blink 0.8s infinite}
@keyframes blink{50%{opacity:0}}
#mode{font-size:12px;color:#0f0}
#mode.rec{color:#f80;font-weight:bold}
#up{color:#555;font-size:11px;margin-left:auto}
.main{display:flex;flex:1;overflow:hidden}
.left{width:280px;border-right:1px solid #1a1a1a;display:flex;flex-direction:column;flex-shrink:0}
.right{flex:1;overflow-y:auto;padding:10px}
.pane{padding:10px;overflow-y:auto;flex:1}
.sec{color:#555;font-size:10px;text-transform:uppercase;letter-spacing:1.5px;margin-bottom:6px;padding-bottom:4px;border-bottom:1px solid #1a1a1a}
/* Stats */
.sr{display:flex;justify-content:space-between;padding:3px 0;border-bottom:1px dotted #151515;font-size:12px}
.sk{color:#666}.sv{color:#ccc}.sv.hot{color:#f80;font-weight:bold}
/* Terminal */
.term{background:#050505;border:1px solid #1a1a1a;border-radius:3px;padding:6px;height:140px;overflow-y:auto;font-size:10px;color:#0a0;flex-shrink:0;margin:0 10px 10px}
/* Sessions */
.sgrid{display:flex;flex-direction:column;gap:6px}
.card{background:#0e0e0e;border:1px solid #222;border-radius:4px;overflow:hidden}
.card:hover{border-color:#333}
.ch{padding:7px 10px;cursor:pointer;display:flex;justify-content:space-between;align-items:center;user-select:none}
.ch:hover{background:#141414}
.ct{color:#ff0;font-size:11px}.ca{color:#444;font-size:11px}
.cb{display:none;padding:8px;border-top:1px solid #1a1a1a}
.cb.open{display:block}
.cimg{width:100%;max-width:480px;border-radius:3px;cursor:zoom-in;display:block;margin-bottom:6px}
.cimg[data-err]{display:none}
audio{width:100%;height:30px}
.no-sess{color:#333;font-size:12px;text-align:center;padding:30px}
.cnt{color:#888;font-size:11px}
</style>
</head>
<body>
<header>
  <div id="dot"></div>
  <h1>XIAO ESP32S3 &mdash; Debug VAD</h1>
  <span id="mode">VEILLE</span>
  <span id="up" title="uptime">0s</span>
</header>
<div class="main">
  <div class="left">
    <div class="pane">
      <div class="sec">Stats en direct</div>
      <div id="stbl"></div>
    </div>
    <div class="sec" style="margin:0 10px 4px">Journal</div>
    <div class="term" id="term"></div>
  </div>
  <div class="right">
    <div class="sec">Sessions &mdash; <span class="cnt" id="scnt">0</span> &mdash; <span class="cnt" id="srfr"></span></div>
    <div class="sgrid" id="sgrid"><div class="no-sess">Chargement...</div></div>
  </div>
</div>
<script>
const FIELDS=[
  ['RMS brut','rms_raw',''],['RMS filtre','rms_filt',''],['ZCR','zcr',''],
  ['EMA bruit','ema',''],['Seuil trigger','trigger',''],['Seuil silence','silence',''],
  ['Session','session',''],['WAV ecrit','wav_kb',' kB'],['Silence acc.','sil_ms',' ms'],
];
let known=[], openCards=new Set();

function log(msg){
  const t=document.getElementById('term');
  const d=new Date(), ts=d.toLocaleTimeString('fr',{hour12:false});
  const ln=document.createElement('div');
  ln.textContent=`${ts} ${msg}`;
  t.appendChild(ln);
  while(t.children.length>80)t.removeChild(t.firstChild);
  t.scrollTop=t.scrollHeight;
}

async function refreshStats(){
  try{
    const d=await(await fetch('/stats')).json();
    const rec=d.rec;
    document.getElementById('dot').className=rec?'rec':'';
    const m=document.getElementById('mode');
    m.textContent=rec?'ENREGISTREMENT':'VEILLE';
    m.className=rec?'rec':'';
    document.getElementById('up').textContent=d.uptime+'s';
    let h='';
    for(const[k,f,u]of FIELDS){
      const v=d[f]??'';
      const hot=(f==='wav_kb'||f==='sil_ms')&&rec;
      h+=`<div class="sr"><span class="sk">${k}</span><span class="sv${hot?' hot':''}">${v}${u}</span></div>`;
    }
    document.getElementById('stbl').innerHTML=h;
    if(rec)log(`REC rms=${d.rms_filt} sil=${d.sil_ms}ms wav=${d.wav_kb}kB`);
  }catch(e){log('stats: '+e.message);}
}

async function refreshSessions(){
  try{
    const list=await(await fetch('/sessions')).json();
    list.sort().reverse();
    const n=list.length;
    document.getElementById('scnt').textContent=n+(n>1?' sessions':' session');
    document.getElementById('srfr').textContent='mis a jour '+new Date().toLocaleTimeString('fr',{hour12:false});
    const fresh=list.filter(s=>!known.includes(s));
    if(fresh.length||known.length!==list.length){
      fresh.forEach(s=>log('Nouvelle session: '+s));
      known=list;
      renderSessions(list);
    }
  }catch(e){}
}

function renderSessions(list){
  const g=document.getElementById('sgrid');
  if(!list.length){g.innerHTML='<div class="no-sess">Aucune session</div>';return;}
  g.innerHTML='';
  list.forEach(sess=>{
    const open=openCards.has(sess);
    const c=document.createElement('div');
    c.className='card';
    c.innerHTML=`
      <div class="ch" onclick="toggle('${sess}',this)">
        <span class="ct">${sess}</span>
        <span class="ca">${open?'&#9650;':'&#9660;'}</span>
      </div>
      <div class="cb${open?' open':''}" id="c_${sess}">
        <img class="cimg" src="/file?p=/session_${sess}/photo.jpg"
             onerror="this.setAttribute('data-err','1')"
             onclick="window.open(this.src)">
        <audio controls preload="none">
          <source src="/file?p=/session_${sess}/audio.wav" type="audio/wav">
        </audio>
      </div>`;
    g.appendChild(c);
  });
}

function toggle(sess,hdr){
  const b=document.getElementById('c_'+sess);
  const o=b.classList.toggle('open');
  hdr.querySelector('.ca').innerHTML=o?'&#9650;':'&#9660;';
  if(o)openCards.add(sess);else openCards.delete(sess);
}

log('Interface demarree');
refreshStats(); refreshSessions();
setInterval(refreshStats,1000);
setInterval(refreshSessions,5000);
</script>
</body>
</html>
)rawhtml";

// ============================================================
//  WEB SERVER ASYNC
// ============================================================
static AsyncWebServer g_ws(80);

static void handle_root(AsyncWebServerRequest *req) {
    req->send_P(200, "text/html", HTML_PAGE);
}

static void handle_stats(AsyncWebServerRequest *req) {
    char buf[512];
    snprintf(buf, sizeof(buf),
        "{\"rec\":%s,\"rms_raw\":%u,\"rms_filt\":%u,\"zcr\":%u,"
        "\"ema\":%.0f,\"trigger\":%u,\"silence\":%u,"
        "\"session\":\"%s\",\"wav_kb\":%.1f,\"sil_ms\":%lu,\"uptime\":%lu}",
        g_recording ? "true" : "false",
        (unsigned)g_rms_raw_d, (unsigned)g_rms_filt_d, (unsigned)g_zcr_d,
        (double)g_noise_ema_d, (unsigned)g_trigger_d, (unsigned)g_vad_sil_d,
        g_session_d, (double)(g_wav_written_d / 1024.0f),
        (unsigned long)g_sil_ms_d, (unsigned long)(millis() / 1000)
    );
    req->send(200, "application/json", buf);
}

static void handle_sessions(AsyncWebServerRequest *req) {
    if (!g_sd_mutex || xSemaphoreTake(g_sd_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        req->send(503, "text/plain", "SD busy");
        return;
    }
    String json = "[";
    bool first = true;
    File root = SD.open("/");
    if (root) {
        File entry = root.openNextFile();
        while (entry) {
            if (entry.isDirectory()) {
                String name = entry.name();
                // name peut inclure le slash initial selon la version de la lib
                if (name.startsWith("/")) name = name.substring(1);
                if (name.startsWith("session_")) {
                    if (!first) json += ",";
                    json += "\"" + name + "\"";
                    first = false;
                }
            }
            entry.close();
            entry = root.openNextFile();
        }
        root.close();
    }
    xSemaphoreGive(g_sd_mutex);
    json += "]";
    req->send(200, "application/json", json);
}

static void handle_file(AsyncWebServerRequest *req) {
    if (!req->hasParam("p")) { req->send(400, "text/plain", "missing p"); return; }
    String path = req->getParam("p")->value();
    String mime = "application/octet-stream";
    if (path.endsWith(".jpg") || path.endsWith(".jpeg")) mime = "image/jpeg";
    else if (path.endsWith(".wav"))  mime = "audio/wav";
    else if (path.endsWith(".html")) mime = "text/html";
    // Lecture sur fichiers deja fermes par VAD -- pas besoin du mutex
    req->send(SD, path, mime);
}

void ap_webserver_init() {
    WiFi.mode(WIFI_AP);
    delay(200);
    bool ok = WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 4);
    delay(500);
    Serial.printf("  softAP: %s  IP=%s\n", ok ? "OK" : "ECHEC",
                  WiFi.softAPIP().toString().c_str());

    g_ws.on("/",        HTTP_GET, handle_root);
    g_ws.on("/stats",   HTTP_GET, handle_stats);
    g_ws.on("/sessions",HTTP_GET, handle_sessions);
    g_ws.on("/file",    HTTP_GET, handle_file);
    g_ws.onNotFound([](AsyncWebServerRequest *r){ r->send(404,"text/plain","404"); });
    g_ws.begin();
    Serial.println("  AsyncWebServer OK (/, /stats, /sessions, /file)");
}

// ============================================================
//  NTP (appele avant ap_webserver_init si WiFi STA dispo)
// ============================================================
void wifi_ntp_init() {
    Serial.printf("  STA '%s'...\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < NTP_TIMEOUT_MS) {
        delay(500);
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("  ECHEC WiFi -- pas de NTP");
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        delay(200);
        return;
    }
    Serial.printf("  IP=%s\n", WiFi.localIP().toString().c_str());
    configTzTime(TZ_FR, NTP_SERVER);
    struct tm ti;
    t0 = millis();
    while (!getLocalTime(&ti, 100) && millis() - t0 < 5000) delay(300);
    if (getLocalTime(&ti, 0)) {
        ntp_synced = true;
        char buf[32]; strftime(buf, sizeof(buf), "%d/%m/%Y %H:%M:%S", &ti);
        Serial.printf("  NTP OK : %s\n", buf);
    } else {
        Serial.println("  NTP timeout");
    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(200);
}

void get_timestamp(char *out, size_t len) {
    struct tm ti;
    if (getLocalTime(&ti, 100)) strftime(out, len, "%Y%m%d_%H%M%S", &ti);
    else snprintf(out, len, "19700101_%06lu", millis() / 1000);
}

// ============================================================
//  FILTRE BIQUAD
// ============================================================
typedef struct { float b0,b1,b2,a1,a2,x1,x2,y1,y2; } biquad_t;
static biquad_t hp_filt={0.92007f,-1.84015f,0.92007f,-1.83388f,0.84657f,0,0,0,0};
static biquad_t lp_filt={0.36182f,0.72363f,0.36182f,0.26343f,0.18393f,0,0,0,0};

inline float biquad_step(biquad_t *f, float x) {
    float y = f->b0*x + f->b1*f->x1 + f->b2*f->x2 - f->a1*f->y1 - f->a2*f->y2;
    f->x2=f->x1; f->x1=x; f->y2=f->y1; f->y1=y; return y;
}
void apply_bandpass(const int16_t *src, int16_t *dst, size_t n) {
    for (size_t i=0;i<n;i++) {
        float s=biquad_step(&hp_filt,(float)src[i]);
        s=biquad_step(&lp_filt,s);
        if(s>32767)s=32767; if(s<-32768)s=-32768;
        dst[i]=(int16_t)s;
    }
}

// ============================================================
//  RMS / ZCR
// ============================================================
uint16_t compute_rms(const int16_t *s, size_t n) {
    if(!n) return 0;
    int64_t m=0; for(size_t i=0;i<n;i++) m+=s[i]; int32_t dc=(int32_t)(m/(int64_t)n);
    uint64_t sum=0;
    for(size_t i=0;i<n;i++){int32_t v=(int32_t)s[i]-dc; sum+=(uint64_t)(v*v);}
    return (uint16_t)sqrtf((float)sum/n);
}
void prepare_wav_chunk(int16_t *s, size_t n) {
    if(!n) return;
    int64_t m=0; for(size_t i=0;i<n;i++) m+=s[i]; int32_t dc=(int32_t)(m/(int64_t)n);
    for(size_t i=0;i<n;i++){int32_t v=((int32_t)s[i]-dc)*MIC_GAIN; v=v>32767?32767:(v<-32768?-32768:v); s[i]=(int16_t)v;}
}
uint16_t compute_zcr(const int16_t *b, size_t n) {
    uint16_t c=0; for(size_t i=1;i<n;i++) if((b[i]>=0)!=(b[i-1]>=0)) c++; return c;
}

// ============================================================
//  WAV HEADER
// ============================================================
typedef struct __attribute__((packed)) {
    char riff[4]; uint32_t file_size; char wave[4]; char fmt[4];
    uint32_t fmt_size; uint16_t audio_fmt; uint16_t channels;
    uint32_t sample_rate; uint32_t byte_rate; uint16_t block_align;
    uint16_t bits; char data[4]; uint32_t data_size;
} wav_header_t;

void write_wav_header(File &f, uint32_t data_size) {
    wav_header_t h={{'R','I','F','F'},data_size+36,{'W','A','V','E'},
        {'f','m','t',' '},16,1,1,SAMPLE_RATE,SAMPLE_RATE*2,2,16,{'d','a','t','a'},data_size};
    f.seek(0); f.write((uint8_t*)&h,sizeof(h));
}

// ============================================================
//  CAMERA
// ============================================================
static bool cam_ok = false;
bool camera_init() {
    camera_config_t cfg={};
    cfg.ledc_channel=LEDC_CHANNEL_0; cfg.ledc_timer=LEDC_TIMER_0;
    cfg.pin_d0=Y2_GPIO_NUM; cfg.pin_d1=Y3_GPIO_NUM; cfg.pin_d2=Y4_GPIO_NUM;
    cfg.pin_d3=Y5_GPIO_NUM; cfg.pin_d4=Y6_GPIO_NUM; cfg.pin_d5=Y7_GPIO_NUM;
    cfg.pin_d6=Y8_GPIO_NUM; cfg.pin_d7=Y9_GPIO_NUM;
    cfg.pin_xclk=XCLK_GPIO_NUM; cfg.pin_pclk=PCLK_GPIO_NUM;
    cfg.pin_vsync=VSYNC_GPIO_NUM; cfg.pin_href=HREF_GPIO_NUM;
    cfg.pin_sscb_sda=SIOD_GPIO_NUM; cfg.pin_sscb_scl=SIOC_GPIO_NUM;
    cfg.pin_pwdn=PWDN_GPIO_NUM; cfg.pin_reset=RESET_GPIO_NUM;
    cfg.xclk_freq_hz=20000000; cfg.pixel_format=PIXFORMAT_JPEG;
    cfg.frame_size=FRAMESIZE_UXGA; cfg.jpeg_quality=10;
    cfg.fb_count=1; cfg.fb_location=CAMERA_FB_IN_PSRAM;
    cfg.grab_mode=CAMERA_GRAB_WHEN_EMPTY;
    if(esp_camera_init(&cfg)!=ESP_OK){Serial.println("ERREUR camera");return false;}
    sensor_t *s=esp_camera_sensor_get();
    if(s){s->set_vflip(s,1);s->set_hmirror(s,1);
          s->set_exposure_ctrl(s,0);s->set_aec_value(s,150);
          s->set_gain_ctrl(s,0);s->set_agc_gain(s,20);
          s->set_aec2(s,0);s->set_ae_level(s,0);
          s->set_whitebal(s,1);s->set_awb_gain(s,1);
          s->set_bpc(s,1);s->set_wpc(s,1);}
    Serial.println("Camera OK");
    return true;
}

void take_photo(const char *path) {
    if(!cam_ok) return;
    camera_fb_t *fb=esp_camera_fb_get();
    if(!fb){Serial.println("[CAM] echec capture");return;}
    if(g_sd_mutex) xSemaphoreTake(g_sd_mutex,portMAX_DELAY);
    File f=SD.open(path,FILE_WRITE);
    if(f){f.write(fb->buf,fb->len);f.close();
          Serial.printf("[CAM] %s %.1fkB\n",path,fb->len/1024.0f);}
    else Serial.printf("[CAM] ERREUR ouverture %s\n",path);
    if(g_sd_mutex) xSemaphoreGive(g_sd_mutex);
    esp_camera_fb_return(fb);
    strncpy(g_last_photo,path,sizeof(g_last_photo)-1);
}

// ============================================================
//  MICRO PDM
// ============================================================
bool mic_init() {
    i2s_config_t cfg={
        .mode=(i2s_mode_t)(I2S_MODE_MASTER|I2S_MODE_RX|I2S_MODE_PDM),
        .sample_rate=SAMPLE_RATE,.bits_per_sample=I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format=I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format=I2S_COMM_FORMAT_STAND_PCM_SHORT,
        .intr_alloc_flags=ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count=16,.dma_buf_len=256,
        .use_apll=false,.tx_desc_auto_clear=false,.fixed_mclk=0};
    if(i2s_driver_install(I2S_NUM_0,&cfg,0,NULL)!=ESP_OK) return false;
    i2s_pin_config_t pins={I2S_PIN_NO_CHANGE,I2S_PIN_NO_CHANGE,MIC_CLK_PIN,I2S_PIN_NO_CHANGE,MIC_DATA_PIN};
    if(i2s_set_pin(I2S_NUM_0,&pins)!=ESP_OK) return false;
    Serial.println("Micro PDM OK");
    return true;
}

// ============================================================
//  SEED EMA
// ============================================================
void seed_noise_ema(uint8_t *buf, int16_t *fbuf) {
    Serial.printf("[SEED] %dms silence svp...\n",VAD_SEED_MS);
    uint32_t t0=millis(); float rs=0,rm=99999; uint32_t nc=0;
    while(millis()-t0<VAD_SEED_MS){
        size_t br=0;
        i2s_read(I2S_NUM_0,buf,VAD_CHUNK_BYTES,&br,pdMS_TO_TICKS(200));
        if(!br) continue;
        apply_bandpass((int16_t*)buf,fbuf,br/2);
        float r=compute_rms(fbuf,br/2);
        rs+=r; if(r<rm) rm=r; nc++;
    }
    float ra=(nc>0)?(rs/nc):20.0f;
    noise_floor_abs=rm*0.2f+ra*0.8f;
    noise_ema=noise_floor_abs;
    vad_trigger=max((uint16_t)VAD_MIN_TRIGGER,(uint16_t)(noise_floor_abs*VAD_TRIGGER_FACTOR));
    vad_silence=max((uint16_t)VAD_MIN_SILENCE,(uint16_t)(noise_floor_abs*VAD_SILENCE_FACTOR));
    Serial.printf("[SEED] plancher=%.0f trigger=%u silence=%u\n",noise_floor_abs,vad_trigger,vad_silence);
}

// ============================================================
//  BOUCLE VAD
// ============================================================
void record_vad() {
    uint8_t  *buf     = (uint8_t *)malloc(VAD_CHUNK_BYTES);
    int16_t  *fbuf    = (int16_t *)malloc(VAD_CHUNK_BYTES);
    uint8_t  *preroll = (uint8_t *)ps_malloc((size_t)PRE_ROLL_CHUNKS*VAD_CHUNK_BYTES);
    if(!buf||!fbuf||!preroll){Serial.println("ERREUR malloc");return;}
    int pr_head=0,pr_count=0;
    File wav; uint32_t written=0,silence_ms=0,rec_start=0,last_print=0,last_veil=0;
    uint16_t rms_peak=0; int votes=0; bool recording=false;
    char session_id[24]="", photo_path[64]="";
    seed_noise_ema(buf,fbuf);
    Serial.printf("[VAD] trigger=%u silence=%u NTP=%s\n",vad_trigger,vad_silence,ntp_synced?"OK":"NON");
    Serial.printf("[VAD] http://%s/\n",WiFi.softAPIP().toString().c_str());
    Serial.println("[VAD] En attente de voix...");

    while(true){
        size_t br=0;
        i2s_read(I2S_NUM_0,buf,VAD_CHUNK_BYTES,&br,pdMS_TO_TICKS(500));
        if(!br) continue;
        int16_t *smp=(int16_t*)buf; size_t ns=br/2;
        uint16_t rms_raw=compute_rms(smp,ns);
        if(rms_raw>rms_peak) rms_peak=rms_raw;
        apply_bandpass(smp,fbuf,ns);
        uint16_t rms_filt=compute_rms(fbuf,ns);
        uint16_t zcr=compute_zcr(fbuf,ns);
        bool zcr_ok=(zcr>=ZCR_MIN&&zcr<=ZCR_MAX);
        if(!recording){
            noise_ema=EMA_ALPHA*noise_ema+(1-EMA_ALPHA)*(float)rms_filt;
            vad_trigger=max((uint16_t)VAD_MIN_TRIGGER,(uint16_t)(noise_ema*VAD_TRIGGER_FACTOR));
            vad_silence=max((uint16_t)VAD_MIN_SILENCE,(uint16_t)(noise_ema*VAD_SILENCE_FACTOR));
        }
        bool cv=recording?(rms_filt>=vad_silence):(rms_filt>=vad_trigger&&zcr_ok);
        if(!recording) votes=cv?(votes+1):0;
        bool voice=recording?cv:(votes>=VAD_VOTE_NEEDED);
        prepare_wav_chunk(smp,ns);
        if(!recording){
            memcpy(preroll+pr_head*VAD_CHUNK_BYTES,buf,br);
            pr_head=(pr_head+1)%PRE_ROLL_CHUNKS;
            if(pr_count<PRE_ROLL_CHUNKS) pr_count++;
        }
        uint32_t now=millis();
        if(!recording){
            g_rms_raw_d=rms_peak; g_rms_filt_d=rms_filt; g_zcr_d=zcr;
            g_noise_ema_d=noise_ema; g_trigger_d=vad_trigger; g_vad_sil_d=vad_silence;
            g_recording=false;
            if(now-last_veil>=500){
                Serial.printf("[VEI] raw=%4u filt=%4u EMA=%.0f trig=%u %3.0f%% zcr=%3u%s v=%d\n",
                    rms_peak,rms_filt,noise_ema,vad_trigger,
                    vad_trigger?(float)rms_filt*100/vad_trigger:0,
                    zcr,zcr_ok?"":"(KO)",votes);
                rms_peak=0; last_veil=now;
            }
            if(voice){
                get_timestamp(session_id,sizeof(session_id));
                char dir[48],wav_path[64];
                snprintf(dir,sizeof(dir),"/session_%s",session_id);
                snprintf(wav_path,sizeof(wav_path),"/session_%s/audio.wav",session_id);
                snprintf(photo_path,sizeof(photo_path),"/session_%s/photo.jpg",session_id);
                if(g_sd_mutex) xSemaphoreTake(g_sd_mutex,portMAX_DELAY);
                SD.mkdir(dir);
                wav=SD.open(wav_path,FILE_WRITE);
                if(g_sd_mutex) xSemaphoreGive(g_sd_mutex);
                if(!wav){Serial.printf("ERREUR open %s\n",wav_path);continue;}
                write_wav_header(wav,0); written=0;
                if(pr_count>0){
                    int sl=(pr_head-pr_count+PRE_ROLL_CHUNKS)%PRE_ROLL_CHUNKS;
                    for(int i=0;i<pr_count;i++){
                        int s=(sl+i)%PRE_ROLL_CHUNKS;
                        wav.write(preroll+s*VAD_CHUNK_BYTES,VAD_CHUNK_BYTES);
                        written+=VAD_CHUNK_BYTES;
                    }
                    pr_count=0; pr_head=0;
                }
                silence_ms=0; votes=0; rms_peak=0; rec_start=now; last_print=now; recording=true;
                strncpy(g_session_d,session_id,sizeof(g_session_d)-1);
                g_wav_written_d=0; g_sil_ms_d=0; g_recording=true;
                Serial.printf("\n[%s] DEBUT rms=%u trig=%u\n",session_id,rms_filt,vad_trigger);
            }
        } else {
            wav.write(buf,br); written+=br;
            if(voice) silence_ms=0; else silence_ms+=VAD_CHUNK_MS;
            g_wav_written_d=written; g_sil_ms_d=silence_ms;
            if(now-last_print>=1000){
                uint32_t sec=(now-rec_start)/1000;
                Serial.printf("[%s] %3lus rms=%4u sil=%lu/%lums\n",
                    session_id,sec,rms_filt,(unsigned long)silence_ms,
                    (unsigned long)SILENCE_TIMEOUT_MS);
                last_print=now;
            }
            bool tout=((now-rec_start)>=(uint32_t)MAX_RECORD_SEC*1000);
            if(silence_ms>=SILENCE_TIMEOUT_MS||tout){
                write_wav_header(wav,written); wav.close();
                Serial.printf("[%s] FIN %.1fs %lub %s\n",session_id,
                    written/(float)(SAMPLE_RATE*2),written,tout?"TIMEOUT":"SILENCE");
                take_photo(photo_path);
                g_recording=false; recording=false;
                Serial.println("[VAD] En attente de voix...\n");
            }
        }
    }
    free(buf); free(fbuf); free(preroll);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);
    delay(500);
    Serial.println("\n=== XIAO ESP32S3 - VAD Debug ===");
    Serial.printf("CPU:%luMHz PSRAM:%luKB Flash:%luMB\n",
        getCpuFrequencyMhz(),ESP.getPsramSize()/1024,
        spi_flash_get_chip_size()/(1024*1024));

    g_sd_mutex = xSemaphoreCreateMutex();

    Serial.println("\n[1/5] NTP...");
    wifi_ntp_init();

    Serial.println("[2/5] AP + WebServer...");
    ap_webserver_init();

    Serial.println("[3/5] SD...");
    pinMode(SD_CS_PIN,OUTPUT); digitalWrite(SD_CS_PIN,HIGH); delay(100);
    SPI.begin(SD_SCK_PIN,SD_MISO_PIN,SD_MOSI_PIN,SD_CS_PIN); delay(100);
    if(!SD.begin(SD_CS_PIN)){
        Serial.println("ERREUR SD -- blocage"); while(1) delay(1000);
    }
    Serial.printf("  SD OK %lluMB type=%d\n",SD.cardSize()/(1024*1024),SD.cardType());

    Serial.println("[4/5] Camera...");
    cam_ok=camera_init();
    if(!cam_ok) Serial.println("  AVERTISSEMENT: camera KO");

    Serial.println("[5/5] Micro PDM...");
    if(!mic_init()){Serial.println("ERREUR micro -- blocage");while(1) delay(1000);}

    Serial.println("\n=== TOUT OK -- demarrage VAD ===\n");
    record_vad();
}

void loop() { delay(1000); }
