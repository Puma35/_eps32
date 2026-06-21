/**
 * ============================================================
 *  Projet  : picsetvoc — VAD Back-Office
 *  Carte   : Seeed Studio XIAO ESP32S3 Sense
 *  MCU     : ESP32-S3R8, LX7 dual-core 240 MHz, 8MB PSRAM, 8MB Flash
 *  WiFi AP : g_cfg.ap_ssid / g_cfg.ap_pass  ->  http://192.168.4.1/
 * ============================================================
 */

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <time.h>
#include "driver/i2s.h"
#include "esp_camera.h"
#include "credentials.h"

// ---- Access Point defaults ----
#define AP_SSID_DEFAULT  "ESP32-Debug"
#define AP_PASS_DEFAULT  "admin1234"

// ---- NTP ----
#define NTP_SERVER     "pool.ntp.org"
#define NTP_TIMEOUT_MS 8000
#define TZ_FR          "CET-1CEST,M3.5.0,M10.5.0/3"

// ---- Pins hardware ----
#define MIC_CLK_PIN   42
#define MIC_DATA_PIN  41
#define SD_CS_PIN     21
#define SD_SCK_PIN     7
#define SD_MISO_PIN    8
#define SD_MOSI_PIN    9

// ---- Camera pins ----
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

// ---- Audio constantes compile-time ----
#define SAMPLE_RATE       16000
#define VAD_CHUNK_MS      30
#define VAD_SEED_MS       1500
#define VAD_CHUNK_SAMPLES (SAMPLE_RATE * VAD_CHUNK_MS / 1000)
#define VAD_CHUNK_BYTES   (VAD_CHUNK_SAMPLES * 2)
#define PRE_ROLL_MS       300
#define PRE_ROLL_CHUNKS   (PRE_ROLL_MS / VAD_CHUNK_MS)

// ============================================================
//  CONFIGURATION PERSISTANTE (chargee depuis /config.json SD)
// ============================================================
struct AppConfig {
    char     wifi_ssid[64];
    char     wifi_pass[64];
    char     ap_ssid[32];
    char     ap_pass[32];
    float    vad_trigger_factor;
    float    vad_silence_factor;
    uint16_t vad_min_trigger;
    uint16_t vad_min_silence;
    float    ema_alpha;
    uint8_t  vad_vote_needed;
    uint32_t silence_timeout_ms;
    uint16_t max_record_sec;
    uint8_t  mic_gain;
    uint8_t  zcr_min;
    uint8_t  zcr_max;
    uint8_t  cam_quality;
    uint8_t  cam_framesize;
    uint16_t cam_aec_value;
    uint8_t  cam_agc_gain;
    bool     gps_enabled;
    uint8_t  gps_rx_pin;
    uint8_t  gps_tx_pin;
};
static AppConfig g_cfg;

// ---- Debug stats (VAD -> WebServer, volatile) ----
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

// ---- GPS globals ----
static volatile double g_gps_lat = 0.0;
static volatile double g_gps_lon = 0.0;
static volatile bool   g_gps_fix = false;

// ---- SD mutex (VAD writes vs WebServer reads) ----
static SemaphoreHandle_t g_sd_mutex = nullptr;

// ---- NTP ----
static bool g_ntp_synced = false;

// ---- VAD state partagee ----
static float    g_noise_ema   = 1000.0f;
static uint16_t g_vad_trigger = 40;
static uint16_t g_vad_silence = 15;
static bool     g_cam_ok      = false;
static bool     g_sd_ok       = false;

// ---- Log ring buffer (ecrit Core1, lu Core0 via /log) ----
#define LOG_RING  80
#define LOG_MAX  128
static char              g_log_ring[LOG_RING][LOG_MAX];
static volatile int      g_log_total = 0;
static SemaphoreHandle_t g_log_mutex = nullptr;

// AsyncEventSource conserve pour l'avenir (non utilise pour les logs)
static AsyncEventSource g_events("/events");

static void log_line(const char *msg) {
    Serial.println(msg);
    if (g_log_mutex && xSemaphoreTake(g_log_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        strlcpy(g_log_ring[g_log_total % LOG_RING], msg, LOG_MAX);
        g_log_total++;
        xSemaphoreGive(g_log_mutex);
    }
}
static void log_linef(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void log_linef(const char *fmt, ...) {
    char tmp[160];
    va_list ap; va_start(ap, fmt); vsnprintf(tmp, sizeof(tmp), fmt, ap); va_end(ap);
    log_line(tmp);
}

// Buffer body pour POST /config
static char   g_post_buf[2048];
static size_t g_post_len = 0;

// ============================================================
//  PAGE HTML  (back-office 5 onglets)
// ============================================================
static const char HTML_PAGE[] PROGMEM = R"WEBUI(<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 VAD</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:monospace;background:#080808;color:#0f0;height:100vh;display:flex;flex-direction:column;overflow:hidden}
header{background:#101010;border-bottom:1px solid #1e1e1e;padding:7px 12px;display:flex;align-items:center;gap:10px;flex-shrink:0;min-height:36px}
h1{color:#ff0;font-size:12px;letter-spacing:1px;white-space:nowrap}
.dot{width:9px;height:9px;border-radius:50%;background:#0a0;flex-shrink:0}
.dot.rec{background:#f80;animation:bl .8s infinite}
@keyframes bl{50%{opacity:0}}
#hmode{font-size:11px;color:#0a0}
#hmode.rec{color:#f80;font-weight:bold}
#hup{color:#444;font-size:10px;margin-left:auto}
#hsd{color:#333;font-size:10px;padding-left:8px;border-left:1px solid #222;margin-left:4px}
nav{display:flex;background:#0d0d0d;border-bottom:1px solid #1a1a1a;flex-shrink:0}
.tab{flex:1;padding:7px 2px;text-align:center;cursor:pointer;font-size:10px;color:#555;border-bottom:2px solid transparent;transition:.1s;user-select:none}
.tab:hover{color:#888;background:#111}
.tab.on{color:#ff0;border-bottom-color:#ff0;background:#111}
.content{flex:1;overflow:hidden;position:relative}
.pan{display:none;position:absolute;inset:0;overflow-y:auto;padding:10px;gap:8px;flex-direction:column}
.pan.on{display:flex}
.card{background:#0d0d0d;border:1px solid #1d1d1d;border-radius:3px;padding:9px}
.card h3{color:#ff0;font-size:11px;margin-bottom:7px;letter-spacing:.5px}
.sec{color:#444;font-size:9px;text-transform:uppercase;letter-spacing:1.5px;margin-bottom:5px}
.sr{display:flex;justify-content:space-between;padding:2px 0;border-bottom:1px dotted #131313;font-size:11px}
.sk{color:#555}.sv{color:#bbb}
.bar-bg{background:#111;border-radius:2px;height:5px;margin:5px 0}
.bar{background:#0f0;height:100%;border-radius:2px;transition:.3s;max-width:100%}
.bar.w{background:#f80}.bar.c{background:#f44}
.scard{background:#0d0d0d;border:1px solid #1d1d1d;border-radius:3px;overflow:hidden;margin-bottom:5px}
.sh{padding:7px 9px;cursor:pointer;display:flex;justify-content:space-between;align-items:center}
.sh:hover{background:#131313}
.st{color:#ff0;font-size:11px}.sa{color:#333;font-size:11px}
.sb{display:none;padding:7px;border-top:1px solid #1a1a1a}
.sb.on{display:block}
.si{width:100%;max-width:480px;border-radius:2px;cursor:zoom-in;display:block;margin-bottom:5px}
audio{width:100%;height:28px;margin-bottom:4px}
.gline{font-size:10px;color:#666;margin-bottom:3px}
.bdel{background:#1a0000;color:#f44;border:1px solid #400;padding:2px 7px;border-radius:2px;cursor:pointer;font-size:10px;font-family:monospace}
.bdel:hover{background:#300}
.nosess{color:#333;text-align:center;padding:25px;font-size:11px}
.cfgsec{margin-bottom:9px}
.cfgsec>summary{cursor:pointer;color:#777;font-size:10px;text-transform:uppercase;letter-spacing:1px;padding:3px 0;user-select:none;list-style:none}
.cfgsec>summary::before{content:"▶ "}
.cfgsec[open]>summary::before{content:"▼ "}
.cfgsec>summary:hover{color:#bbb}
.crow{display:flex;align-items:center;gap:7px;padding:3px 0;font-size:11px;flex-wrap:wrap}
.crow label{color:#555;min-width:150px;flex-shrink:0}
.crow input[type=text],.crow input[type=password],.crow input[type=number],.crow select{background:#0f0f0f;color:#bbb;border:1px solid #252525;border-radius:2px;padding:3px 5px;font-family:monospace;font-size:11px;width:170px}
.crow input[type=range]{flex:1;max-width:160px;accent-color:#0f0}
.cval{color:#ff0;font-size:11px;min-width:38px}
.crow input[type=checkbox]{width:14px;height:14px;cursor:pointer;accent-color:#0f0}
.btn{background:#151515;color:#999;border:1px solid #2a2a2a;padding:5px 13px;border-radius:2px;cursor:pointer;font-family:monospace;font-size:11px}
.btn:hover{background:#1d1d1d;border-color:#444}
.btnp{background:#0a1f0a;color:#0f0;border-color:#0f0}
.btnp:hover{background:#0f2f0f}
.btnd{background:#1f0a0a;color:#f44;border-color:#500}
.btnd:hover{background:#2f0a0a}
.toast{position:fixed;bottom:16px;right:16px;padding:6px 14px;border-radius:3px;font-size:11px;display:none;z-index:99;pointer-events:none}
.tok{background:#0a1f0a;color:#0f0;border:1px solid #0f0}
.terr{background:#1f0a0a;color:#f44;border:1px solid #f44}
#gpscvs{background:#0a0a0a;border:1px solid #1a1a1a;border-radius:2px;display:none;width:100%}
#term{background:#020202;border:1px solid #1a1a1a;border-radius:2px;padding:7px;flex:1;overflow-y:auto;font-size:10px;color:#0a0;min-height:150px;white-space:pre-wrap;word-break:break-all}
.tctr{display:flex;gap:7px;align-items:center;margin-bottom:5px;flex-shrink:0}
</style>
</head>
<body>
<header>
  <div class="dot" id="dot"></div>
  <h1>XIAO ESP32S3 &mdash; VAD</h1>
  <span id="hmode">VEILLE</span>
  <span id="hup">0s</span>
  <span id="hsd"></span>
</header>
<nav>
  <div class="tab on"  onclick="go('dash',this)">&#128200; Dashboard</div>
  <div class="tab"     onclick="go('sess',this)">&#128247; Sessions</div>
  <div class="tab"     onclick="go('conf',this)">&#9881; Config</div>
  <div class="tab"     onclick="go('gps', this)">&#127759; GPS</div>
  <div class="tab"     onclick="go('term',this)">&#128196; Terminal</div>
</nav>
<div class="content">

<div id="pan-dash" class="pan on">
  <div class="card">
    <h3>&#9679; Etat</h3>
    <div id="dmode" style="font-size:16px;text-align:center;padding:8px;color:#0f0">&#9711; VEILLE</div>
    <div class="sec" style="margin-top:7px">Niveau VAD vs seuil</div>
    <div class="bar-bg"><div class="bar" id="dbar" style="width:0%"></div></div>
    <div style="font-size:9px;color:#333;display:flex;justify-content:space-between"><span>0</span><span id="dpct">0%</span><span>seuil</span></div>
  </div>
  <div class="card"><h3>&#128266; Audio</h3><div id="daudio"></div></div>
  <div class="card"><h3>&#128190; Stockage SD</h3><div id="dsd"></div></div>
</div>

<div id="pan-sess" class="pan">
  <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:4px">
    <div class="sec" style="margin:0">Sessions &mdash; <span id="scnt">0</span></div>
    <button class="btn" onclick="loadSess()">&#8635;</button>
  </div>
  <canvas id="sesscvs" style="display:none;width:100%;height:180px;border:1px solid #1a1a1a;border-radius:2px;background:#0a0a0a;margin-bottom:6px"></canvas>
  <div id="sgrid"></div>
</div>

<div id="pan-conf" class="pan">
  <div class="card">
    <h3>&#9881; Configuration</h3>
    <form id="cform">
      <details class="cfgsec" open>
        <summary>WiFi STA (NTP)</summary>
        <div style="margin-top:5px">
          <div class="crow"><label>SSID</label><input type="text" name="wifi_ssid" maxlength="63"></div>
          <div class="crow"><label>Mot de passe</label><input type="password" name="wifi_pass" maxlength="63"></div>
        </div>
      </details>
      <details class="cfgsec" open>
        <summary>Access Point</summary>
        <div style="margin-top:5px">
          <div class="crow"><label>SSID AP</label><input type="text" name="ap_ssid" maxlength="31"></div>
          <div class="crow"><label>Mot de passe AP</label><input type="password" name="ap_pass" maxlength="31"></div>
        </div>
      </details>
      <details class="cfgsec" open>
        <summary>VAD</summary>
        <div style="margin-top:5px" id="vad-f"></div>
      </details>
      <details class="cfgsec">
        <summary>Camera</summary>
        <div style="margin-top:5px" id="cam-f">
          <div class="crow"><label>Resolution</label>
            <select name="cam_framesize">
              <option value="5">VGA 640x480</option>
              <option value="7">SVGA 800x600</option>
              <option value="9">XGA 1024x768</option>
              <option value="10">HD 1280x720</option>
              <option value="12">SXGA 1280x1024</option>
              <option value="13">UXGA 1600x1200</option>
            </select>
          </div>
        </div>
      </details>
      <details class="cfgsec">
        <summary>GPS</summary>
        <div style="margin-top:5px">
          <div class="crow"><label>GPS actif</label><input type="checkbox" name="gps_enabled"></div>
          <div class="crow"><label>Pin RX</label><input type="number" name="gps_rx_pin" min="0" max="48" style="width:70px"></div>
          <div class="crow"><label>Pin TX</label><input type="number" name="gps_tx_pin" min="0" max="48" style="width:70px"></div>
        </div>
      </details>
      <div style="display:flex;gap:9px;margin-top:12px;padding-top:9px;border-top:1px solid #1a1a1a">
        <button type="button" class="btn btnp" onclick="saveCfg()">&#128190; Enregistrer</button>
        <button type="button" class="btn btnd" onclick="doReboot()">&#8635; Redemarrer</button>
      </div>
    </form>
  </div>
</div>

<div id="pan-gps" class="pan">
  <div class="card">
    <h3>&#127759; Fix GPS</h3>
    <div id="gpsinfo"></div>
  </div>
  <div class="card" style="flex:1">
    <h3>&#128205; Carte sessions</h3>
    <canvas id="gpscvs" height="280"></canvas>
    <div id="gpsno" style="color:#333;font-size:11px;text-align:center;padding:20px">Aucune session avec GPS</div>
  </div>
</div>

<div id="pan-term" class="pan">
  <div class="tctr">
    <button class="btn" onclick="termClr()">&#128465; Clear</button>
    <label style="font-size:10px;color:#555;cursor:pointer"><input type="checkbox" id="tpause"> Pause</label>
    <label style="font-size:10px;color:#555;cursor:pointer"><input type="checkbox" id="tscroll" checked> Auto-scroll</label>
    <span id="tcnt" style="font-size:9px;color:#333;margin-left:auto">0 lignes</span>
  </div>
  <div id="term"></div>
</div>
</div>

<div class="toast" id="toast"></div>
<script>
// ===== TABS =====
function go(id,el){
  document.querySelectorAll('.pan').forEach(p=>p.classList.remove('on'));
  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('on'));
  document.getElementById('pan-'+id).classList.add('on');
  el.classList.add('on');
  if(id==='conf'&&!cfgOk)loadCfg();
  if(id==='gps')loadGPS();
  if(id==='sess')loadSess();
}

// ===== TOAST =====
function toast(msg,ok=true){
  const t=document.getElementById('toast');
  t.textContent=msg; t.className='toast '+(ok?'tok':'terr');
  t.style.display='block';
  clearTimeout(t._t); t._t=setTimeout(()=>t.style.display='none',3000);
}

// ===== DASHBOARD =====
async function refreshStats(){
  try{
    const d=await(await fetch('/stats')).json();
    const rec=d.rec;
    document.getElementById('dot').className='dot'+(rec?' rec':'');
    const m=document.getElementById('hmode');
    m.textContent=rec?'ENREGISTREMENT':'VEILLE'; m.className=rec?'rec':'';
    document.getElementById('hup').textContent=fmtUp(d.uptime);
    const dm=document.getElementById('dmode');
    dm.textContent=rec?'⏺ ENREGISTREMENT':'◯ VEILLE';
    dm.style.color=rec?'#f80':'#0f0';
    const pct=d.trigger>0?Math.min(100,Math.round(d.rms_filt*100/d.trigger)):0;
    const bar=document.getElementById('dbar');
    bar.style.width=pct+'%';
    bar.className='bar'+(pct>=100?' c':pct>=70?' w':'');
    document.getElementById('dpct').textContent=pct+'%';
    const rows=[
      ['RMS brut',d.rms_raw],['RMS filtre',d.rms_filt],['ZCR',d.zcr],
      ['EMA bruit',Math.round(d.ema)],['Seuil trigger',d.trigger],['Seuil silence',d.silence],
    ];
    if(rec) rows.push(['Session',d.session],['WAV ecrit',d.wav_kb.toFixed(1)+' kB'],['Silence acc.',d.sil_ms+' ms']);
    document.getElementById('daudio').innerHTML=rows.map(([k,v])=>
      `<div class="sr"><span class="sk">${k}</span><span class="sv">${v}</span></div>`).join('');
  }catch(e){}
}

async function refreshSD(){
  try{
    const d=await(await fetch('/sdinfo')).json();
    if(d.error){
      document.getElementById('hsd').textContent='SD ERR';
      document.getElementById('dsd').innerHTML='<div style="color:#f44;font-size:11px;padding:4px">Carte SD non initialisee -- verifier la carte</div>';
      return;
    }
    const pct=Math.round(d.used*100/d.total);
    document.getElementById('hsd').textContent=fmtB(d.free)+' libre';
    document.getElementById('dsd').innerHTML=
      `<div class="sr"><span class="sk">Total</span><span class="sv">${fmtB(d.total)}</span></div>`+
      `<div class="sr"><span class="sk">Utilise</span><span class="sv">${fmtB(d.used)} (${pct}%)</span></div>`+
      `<div class="sr"><span class="sk">Libre</span><span class="sv">${fmtB(d.free)}</span></div>`+
      `<div class="bar-bg"><div class="bar${pct>85?' w':''}" style="width:${pct}%"></div></div>`;
  }catch(e){}
}

function fmtUp(s){return s<60?s+'s':s<3600?Math.floor(s/60)+'m'+s%60+'s':Math.floor(s/3600)+'h'+Math.floor(s%3600/60)+'m'}
function fmtB(b){return b>1e9?(b/1e9).toFixed(1)+'GB':b>1e6?(b/1e6).toFixed(0)+'MB':(b/1e3).toFixed(0)+'KB'}

// ===== SESSIONS =====
let knownS=[],openC=new Set();
async function loadSess(){
  try{
    const list=await(await fetch('/sessions')).json();
    list.sort().reverse();
    document.getElementById('scnt').textContent=list.length;
    if(JSON.stringify(list)!==JSON.stringify(knownS)){knownS=list;renderSess(list);}
  }catch(e){}
}
function renderSess(list){
  const g=document.getElementById('sgrid');
  if(!list.length){g.innerHTML='<div class="nosess">Aucune session enregistree</div>';return;}
  g.innerHTML='';
  list.forEach(sess=>{
    const open=openC.has(sess);
    const el=document.createElement('div');
    el.className='scard';
    el.innerHTML=
      `<div class="sh" onclick="togS('${sess}',this)"><span class="st">${fmtId(sess)}</span><span class="sa">${open?'&#9650;':'&#9660;'}</span></div>`+
      `<div class="sb${open?' on':''}" id="sc_${sess}">`+
        `<img class="si" src="/file?p=/session_${sess}/photo.jpg" onerror="this.style.display='none'" onclick="window.open(this.src)">`+
        `<audio controls preload="none"><source src="/file?p=/session_${sess}/audio.wav" type="audio/wav"></audio>`+
        `<div class="gline" id="gm_${sess}">GPS: ---</div>`+
        `<button class="bdel" onclick="delS('${sess}')">&#128465; Supprimer</button>`+
      `</div>`;
    g.appendChild(el);
    if(open) loadMeta(sess);
  });
  loadSessMap();
}
function togS(s,hdr){
  const b=document.getElementById('sc_'+s);
  const o=b.classList.toggle('on');
  hdr.querySelector('.sa').innerHTML=o?'&#9650;':'&#9660;';
  if(o){openC.add(s);loadMeta(s);}else openC.delete(s);
}
async function loadMeta(sess){
  const el=document.getElementById('gm_'+sess); if(!el) return;
  try{
    const r=await fetch('/file?p=/session_'+sess+'/meta.json');
    if(!r.ok){el.textContent='GPS: non disponible';return;}
    const m=await r.json();
    if(m.fix) el.innerHTML=`GPS: <a href="https://maps.google.com/?q=${m.lat},${m.lon}" target="_blank" style="color:#88f">${m.lat.toFixed(5)}, ${m.lon.toFixed(5)}</a>`;
    else el.textContent='GPS: pas de fix';
  }catch(e){el.textContent='GPS: -';}
}
async function delS(sess){
  if(!confirm('Supprimer la session '+sess+' ?')) return;
  try{
    const r=await fetch('/session?id='+sess,{method:'DELETE'});
    if(r.ok){knownS=[];loadSess();toast('Session supprimee');}
    else toast('Erreur suppression',false);
  }catch(e){toast('Erreur: '+e.message,false);}
}
function fmtId(id){
  const m=id.match(/^(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})$/);
  return m?`${m[3]}/${m[2]}/${m[1]} ${m[4]}:${m[5]}:${m[6]}`:id;
}
async function loadSessMap(){
  const pts=await getGPSPts(knownS);
  drawMap('sesscvs',pts,180);
}

// ===== CONFIG =====
let cfgOk=false;
const VAD_SLIDERS=[
  ['vad_trigger_factor','Facteur trigger',1,20,0.1],
  ['vad_silence_factor','Facteur silence',1,10,0.1],
  ['vad_min_trigger','RMS trigger min',10,200,1],
  ['vad_min_silence','RMS silence min',5,100,1],
  ['ema_alpha','EMA alpha',0.9,0.999,0.001],
  ['vad_vote_needed','Votes requis',1,10,1],
  ['silence_timeout_ms','Timeout silence ms',500,30000,500],
  ['max_record_sec','Duree max s',10,300,10],
  ['mic_gain','Gain micro',1,32,1],
  ['zcr_min','ZCR min',1,50,1],
  ['zcr_max','ZCR max',20,300,5],
];
const CAM_SLIDERS=[
  ['cam_quality','Qualite JPEG',0,63,1],
  ['cam_aec_value','Exposition AEC',0,1200,10],
  ['cam_agc_gain','Gain AGC',0,30,1],
];
function buildSliders(id,arr){
  document.getElementById(id).innerHTML+=arr.map(([n,l,mn,mx,st])=>
    `<div class="crow"><label>${l}</label><input type="range" name="${n}" min="${mn}" max="${mx}" step="${st}" oninput="document.getElementById('v_${n}').textContent=this.value"><span class="cval" id="v_${n}">-</span></div>`
  ).join('');
}
async function loadCfg(){
  buildSliders('vad-f',VAD_SLIDERS);
  buildSliders('cam-f',CAM_SLIDERS);
  try{
    const c=await(await fetch('/config')).json();
    const f=document.getElementById('cform');
    for(const[k,v]of Object.entries(c)){
      const el=f.elements[k]; if(!el) continue;
      if(el.type==='checkbox') el.checked=!!v;
      else el.value=v;
      const vs=document.getElementById('v_'+k); if(vs) vs.textContent=v;
    }
    cfgOk=true;
  }catch(e){toast('Erreur config',false);}
}
async function saveCfg(){
  const f=document.getElementById('cform'),obj={};
  for(const el of f.elements){
    if(!el.name) continue;
    if(el.type==='checkbox') obj[el.name]=el.checked;
    else if(el.type==='range'||el.type==='number') obj[el.name]=parseFloat(el.value);
    else obj[el.name]=el.value;
  }
  try{
    const r=await fetch('/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(obj)});
    const d=await r.json();
    toast(d.ok?'Sauvegarde OK (reboot pour WiFi/GPS/Camera)':'Erreur save',d.ok);
  }catch(e){toast('Erreur: '+e.message,false);}
}
async function doReboot(){
  if(!confirm('Redemarrer le ESP32 ?')) return;
  try{await fetch('/reboot',{method:'POST'});toast('Redemarrage...');}catch(e){}
}

// ===== GPS =====
async function loadGPS(){
  try{
    const d=await(await fetch('/gps')).json();
    document.getElementById('gpsinfo').innerHTML=
      `<div class="sr"><span class="sk">Fix</span><span class="sv" style="color:${d.fix?'#0f0':'#f44'}">${d.fix?'OUI':'NON'}</span></div>`+
      `<div class="sr"><span class="sk">Latitude</span><span class="sv">${d.fix?d.lat.toFixed(6):'-'}</span></div>`+
      `<div class="sr"><span class="sk">Longitude</span><span class="sv">${d.fix?d.lon.toFixed(6):'-'}</span></div>`+
      `<div class="sr"><span class="sk">Module</span><span class="sv">${d.enabled?'Actif':'Inactif'}</span></div>`+
      (d.fix?`<div style="margin-top:6px"><a href="https://maps.google.com/?q=${d.lat},${d.lon}" target="_blank" style="color:#88f;font-size:11px">&#128205; Google Maps</a></div>`:'');
  }catch(e){document.getElementById('gpsinfo').innerHTML='<div style="color:#333;font-size:11px">GPS non disponible</div>';}
  if(knownS.length===0){try{const l=await(await fetch('/sessions')).json();knownS=l.sort().reverse();}catch(e){}}
  const pts=await getGPSPts(knownS.slice(0,50));
  drawMap('gpscvs',pts,280);
  document.getElementById('gpsno').style.display=pts.length?'none':'block';
}
async function getGPSPts(list){
  if(!list.length) return [];
  const results=await Promise.all(list.slice(0,50).map(async sess=>{
    try{
      const r=await fetch('/file?p=/session_'+sess+'/meta.json');
      if(!r.ok) return null;
      const m=await r.json();
      return m.fix?{sess,lat:m.lat,lon:m.lon}:null;
    }catch(e){return null;}
  }));
  return results.filter(Boolean);
}
function drawMap(id,pts,h){
  const cvs=document.getElementById(id);
  if(!pts.length){cvs.style.display='none';return;}
  cvs.style.display='block';
  const W=cvs.parentElement.clientWidth||380;
  cvs.width=W; cvs.height=h;
  const ctx=cvs.getContext('2d');
  ctx.fillStyle='#0a0a0a'; ctx.fillRect(0,0,W,h);
  const lats=pts.map(p=>p.lat),lons=pts.map(p=>p.lon);
  const la0=Math.min(...lats),la1=Math.max(...lats);
  const lo0=Math.min(...lons),lo1=Math.max(...lons);
  const pd=28;
  const sx=lo1===lo0?1:(W-2*pd)/(lo1-lo0);
  const sy=la1===la0?1:(h-2*pd)/(la1-la0);
  pts.forEach((p,i)=>{
    const x=lo1===lo0?W/2:pd+(p.lon-lo0)*sx;
    const y=la1===la0?h/2:h-pd-(p.lat-la0)*sy;
    ctx.beginPath(); ctx.arc(x,y,5,0,Math.PI*2);
    ctx.fillStyle=i===0?'#f80':'#0f0'; ctx.fill();
    ctx.strokeStyle='#222'; ctx.lineWidth=1; ctx.stroke();
    ctx.fillStyle='#555'; ctx.font='8px monospace';
    ctx.fillText(fmtId(p.sess).slice(0,8),x+7,y+3);
  });
}

// ===== TERMINAL (polling /log) =====
let tLines=0,tPaused=false,tAuto=true,logIdx=0;
document.getElementById('tpause').onchange=e=>tPaused=e.target.checked;
document.getElementById('tscroll').onchange=e=>tAuto=e.target.checked;
function termClr(){document.getElementById('term').innerHTML='';tLines=0;logIdx=0;updT();}
function updT(){document.getElementById('tcnt').textContent=tLines+' lignes';}
function appendLog(msg){
  const t=document.getElementById('term');
  const d=document.createElement('div');
  const ts=new Date().toLocaleTimeString('fr',{hour12:false});
  d.textContent='['+ts+'] '+msg;
  if(/ERREUR|ERROR/.test(msg)) d.style.color='#f44';
  else if(/DEBUT|REC\b/.test(msg)) d.style.color='#f80';
  else if(/FIN\b|OK\b/.test(msg)) d.style.color='#8f8';
  t.appendChild(d);
  while(t.children.length>200){t.removeChild(t.firstChild);tLines--;}
  tLines++; updT();
  if(tAuto) t.scrollTop=t.scrollHeight;
}
async function pollLog(){
  if(tPaused) return;
  try{
    const d=await(await fetch('/log?from='+logIdx)).json();
    if(d.lines&&d.lines.length){d.lines.forEach(appendLog);logIdx=d.total;}
  }catch(e){}
}

// ===== INIT =====
refreshStats(); refreshSD(); loadSess(); pollLog();
setInterval(refreshStats,1000);
setInterval(refreshSD,15000);
setInterval(loadSess,8000);
setInterval(pollLog,1000);
</script>
</body>
</html>)WEBUI";

// ============================================================
//  HANDLERS HTTP
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
        (unsigned long)g_sil_ms_d, (unsigned long)(millis() / 1000));
    req->send(200, "application/json", buf);
}

static void handle_sessions(AsyncWebServerRequest *req) {
    if (!g_sd_mutex || xSemaphoreTake(g_sd_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        req->send(503, "text/plain", "SD busy"); return;
    }
    String json = "["; bool first = true;
    File root = SD.open("/");
    if (root) {
        File entry = root.openNextFile();
        while (entry) {
            if (entry.isDirectory()) {
                String name = entry.name();
                if (name.startsWith("/")) name = name.substring(1);
                if (name.startsWith("session_")) {
                    if (!first) json += ",";
                    json += "\"" + name.substring(8) + "\"";  // strip "session_" prefix
                    first = false;
                }
            }
            entry.close(); entry = root.openNextFile();
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
    if (path.indexOf("..") >= 0) { req->send(403, "text/plain", "forbidden"); return; }
    String mime = "application/octet-stream";
    if (path.endsWith(".jpg") || path.endsWith(".jpeg")) mime = "image/jpeg";
    else if (path.endsWith(".wav"))  mime = "audio/wav";
    else if (path.endsWith(".json")) mime = "application/json";
    else if (path.endsWith(".html")) mime = "text/html";
    req->send(SD, path, mime);
}

static void handle_sdinfo(AsyncWebServerRequest *req) {
    if (!g_sd_ok) {
        req->send(200, "application/json", "{\"total\":0,\"used\":0,\"free\":0,\"error\":true}");
        return;
    }
    char buf[128];
    uint64_t tot = SD.totalBytes(), used = SD.usedBytes();
    snprintf(buf, sizeof(buf), "{\"total\":%llu,\"used\":%llu,\"free\":%llu,\"error\":false}", tot, used, tot - used);
    req->send(200, "application/json", buf);
}

static void handle_log(AsyncWebServerRequest *req) {
    int from = 0;
    if (req->hasParam("from")) from = req->getParam("from")->value().toInt();
    int total = g_log_total;
    int start = (total - LOG_RING > from) ? (total - LOG_RING) : from;
    if (start < 0) start = 0;
    String json = "{\"total\":" + String(total) + ",\"lines\":[";
    bool first = true;
    for (int i = start; i < total; i++) {
        const char *s = g_log_ring[i % LOG_RING];
        if (!first) json += ',';
        json += '"';
        for (; *s; s++) {
            if (*s == '"') { json += "\\\""; }
            else if (*s == '\\') { json += "\\\\"; }
            else json += *s;
        }
        json += '"';
        first = false;
    }
    json += "]}";
    req->send(200, "application/json", json);
}

static void handle_gps(AsyncWebServerRequest *req) {
    char buf[128];
    snprintf(buf, sizeof(buf),
        "{\"fix\":%s,\"lat\":%.6f,\"lon\":%.6f,\"enabled\":%s}",
        g_gps_fix ? "true" : "false",
        (double)g_gps_lat, (double)g_gps_lon,
        g_cfg.gps_enabled ? "true" : "false");
    req->send(200, "application/json", buf);
}

static void handle_config_get(AsyncWebServerRequest *req) {
    JsonDocument doc;
    doc["wifi_ssid"] = g_cfg.wifi_ssid;
    doc["wifi_pass"] = g_cfg.wifi_pass;
    doc["ap_ssid"]   = g_cfg.ap_ssid;
    doc["ap_pass"]   = g_cfg.ap_pass;
    doc["vad_trigger_factor"]  = g_cfg.vad_trigger_factor;
    doc["vad_silence_factor"]  = g_cfg.vad_silence_factor;
    doc["vad_min_trigger"]     = g_cfg.vad_min_trigger;
    doc["vad_min_silence"]     = g_cfg.vad_min_silence;
    doc["ema_alpha"]           = g_cfg.ema_alpha;
    doc["vad_vote_needed"]     = g_cfg.vad_vote_needed;
    doc["silence_timeout_ms"]  = g_cfg.silence_timeout_ms;
    doc["max_record_sec"]      = g_cfg.max_record_sec;
    doc["mic_gain"]            = g_cfg.mic_gain;
    doc["zcr_min"]             = g_cfg.zcr_min;
    doc["zcr_max"]             = g_cfg.zcr_max;
    doc["cam_quality"]         = g_cfg.cam_quality;
    doc["cam_framesize"]       = g_cfg.cam_framesize;
    doc["cam_aec_value"]       = g_cfg.cam_aec_value;
    doc["cam_agc_gain"]        = g_cfg.cam_agc_gain;
    doc["gps_enabled"]         = g_cfg.gps_enabled;
    doc["gps_rx_pin"]          = g_cfg.gps_rx_pin;
    doc["gps_tx_pin"]          = g_cfg.gps_tx_pin;
    String out; serializeJson(doc, out);
    req->send(200, "application/json", out);
}

static void handle_session_delete(AsyncWebServerRequest *req) {
    if (!req->hasParam("id")) { req->send(400, "text/plain", "missing id"); return; }
    String id = req->getParam("id")->value();
    if (id.indexOf("..") >= 0 || id.indexOf("/") >= 0) { req->send(403, "text/plain", "invalid"); return; }
    if (g_recording && strncmp(g_session_d, id.c_str(), 23) == 0) {
        req->send(409, "text/plain", "recording"); return;
    }
    String base = "/session_" + id;
    if (g_sd_mutex) xSemaphoreTake(g_sd_mutex, portMAX_DELAY);
    SD.remove((base + "/audio.wav").c_str());
    SD.remove((base + "/photo.jpg").c_str());
    SD.remove((base + "/meta.json").c_str());
    bool ok = SD.rmdir(base.c_str());
    if (g_sd_mutex) xSemaphoreGive(g_sd_mutex);
    req->send(ok ? 200 : 500, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

// ============================================================
//  CONFIG LOAD / SAVE
// ============================================================
static void config_defaults() {
    strlcpy(g_cfg.wifi_ssid, WIFI_SSID,       sizeof(g_cfg.wifi_ssid));
    strlcpy(g_cfg.wifi_pass, WIFI_PASSWORD,    sizeof(g_cfg.wifi_pass));
    strlcpy(g_cfg.ap_ssid,   AP_SSID_DEFAULT,  sizeof(g_cfg.ap_ssid));
    strlcpy(g_cfg.ap_pass,   AP_PASS_DEFAULT,  sizeof(g_cfg.ap_pass));
    g_cfg.vad_trigger_factor = 6.0f;
    g_cfg.vad_silence_factor = 3.0f;
    g_cfg.vad_min_trigger    = 40;
    g_cfg.vad_min_silence    = 15;
    g_cfg.ema_alpha          = 0.990f;
    g_cfg.vad_vote_needed    = 3;
    g_cfg.silence_timeout_ms = 5000;
    g_cfg.max_record_sec     = 120;
    g_cfg.mic_gain           = 8;
    g_cfg.zcr_min            = 3;
    g_cfg.zcr_max            = 180;
    g_cfg.cam_quality        = 10;
    g_cfg.cam_framesize      = 13;  // FRAMESIZE_UXGA
    g_cfg.cam_aec_value      = 150;
    g_cfg.cam_agc_gain       = 20;
    g_cfg.gps_enabled        = false;
    g_cfg.gps_rx_pin         = 43;
    g_cfg.gps_tx_pin         = 44;
}

static bool config_load() {
    if (!SD.exists("/config.json")) return false;
    File f = SD.open("/config.json", FILE_READ);
    if (!f) return false;
    JsonDocument doc;
    auto err = deserializeJson(doc, f);
    f.close();
    if (err) return false;
    strlcpy(g_cfg.wifi_ssid, doc["wifi_ssid"] | g_cfg.wifi_ssid, sizeof(g_cfg.wifi_ssid));
    strlcpy(g_cfg.wifi_pass, doc["wifi_pass"] | g_cfg.wifi_pass, sizeof(g_cfg.wifi_pass));
    strlcpy(g_cfg.ap_ssid,   doc["ap_ssid"]   | g_cfg.ap_ssid,   sizeof(g_cfg.ap_ssid));
    strlcpy(g_cfg.ap_pass,   doc["ap_pass"]   | g_cfg.ap_pass,   sizeof(g_cfg.ap_pass));
    g_cfg.vad_trigger_factor = doc["vad_trigger_factor"] | g_cfg.vad_trigger_factor;
    g_cfg.vad_silence_factor = doc["vad_silence_factor"] | g_cfg.vad_silence_factor;
    g_cfg.vad_min_trigger    = doc["vad_min_trigger"]    | (int)g_cfg.vad_min_trigger;
    g_cfg.vad_min_silence    = doc["vad_min_silence"]    | (int)g_cfg.vad_min_silence;
    g_cfg.ema_alpha          = doc["ema_alpha"]          | g_cfg.ema_alpha;
    g_cfg.vad_vote_needed    = doc["vad_vote_needed"]    | (int)g_cfg.vad_vote_needed;
    g_cfg.silence_timeout_ms = doc["silence_timeout_ms"] | (int)g_cfg.silence_timeout_ms;
    g_cfg.max_record_sec     = doc["max_record_sec"]     | (int)g_cfg.max_record_sec;
    g_cfg.mic_gain           = doc["mic_gain"]           | (int)g_cfg.mic_gain;
    g_cfg.zcr_min            = doc["zcr_min"]            | (int)g_cfg.zcr_min;
    g_cfg.zcr_max            = doc["zcr_max"]            | (int)g_cfg.zcr_max;
    g_cfg.cam_quality        = doc["cam_quality"]        | (int)g_cfg.cam_quality;
    g_cfg.cam_framesize      = doc["cam_framesize"]      | (int)g_cfg.cam_framesize;
    g_cfg.cam_aec_value      = doc["cam_aec_value"]      | (int)g_cfg.cam_aec_value;
    g_cfg.cam_agc_gain       = doc["cam_agc_gain"]       | (int)g_cfg.cam_agc_gain;
    g_cfg.gps_enabled        = doc["gps_enabled"]        | g_cfg.gps_enabled;
    g_cfg.gps_rx_pin         = doc["gps_rx_pin"]         | (int)g_cfg.gps_rx_pin;
    g_cfg.gps_tx_pin         = doc["gps_tx_pin"]         | (int)g_cfg.gps_tx_pin;
    return true;
}

static void config_save() {
    JsonDocument doc;
    doc["wifi_ssid"] = g_cfg.wifi_ssid;
    doc["wifi_pass"] = g_cfg.wifi_pass;
    doc["ap_ssid"]   = g_cfg.ap_ssid;
    doc["ap_pass"]   = g_cfg.ap_pass;
    doc["vad_trigger_factor"]  = g_cfg.vad_trigger_factor;
    doc["vad_silence_factor"]  = g_cfg.vad_silence_factor;
    doc["vad_min_trigger"]     = g_cfg.vad_min_trigger;
    doc["vad_min_silence"]     = g_cfg.vad_min_silence;
    doc["ema_alpha"]           = g_cfg.ema_alpha;
    doc["vad_vote_needed"]     = g_cfg.vad_vote_needed;
    doc["silence_timeout_ms"]  = g_cfg.silence_timeout_ms;
    doc["max_record_sec"]      = g_cfg.max_record_sec;
    doc["mic_gain"]            = g_cfg.mic_gain;
    doc["zcr_min"]             = g_cfg.zcr_min;
    doc["zcr_max"]             = g_cfg.zcr_max;
    doc["cam_quality"]         = g_cfg.cam_quality;
    doc["cam_framesize"]       = g_cfg.cam_framesize;
    doc["cam_aec_value"]       = g_cfg.cam_aec_value;
    doc["cam_agc_gain"]        = g_cfg.cam_agc_gain;
    doc["gps_enabled"]         = g_cfg.gps_enabled;
    doc["gps_rx_pin"]          = g_cfg.gps_rx_pin;
    doc["gps_tx_pin"]          = g_cfg.gps_tx_pin;
    String out; serializeJson(doc, out);
    if (g_sd_mutex) xSemaphoreTake(g_sd_mutex, portMAX_DELAY);
    SD.remove("/config.json");
    File f = SD.open("/config.json", FILE_WRITE);
    if (f) { f.print(out); f.close(); }
    if (g_sd_mutex) xSemaphoreGive(g_sd_mutex);
}

// ============================================================
//  AP + WEBSERVER
// ============================================================
void ap_webserver_init() {
    WiFi.mode(WIFI_AP);
    delay(200);
    bool ok = WiFi.softAP(g_cfg.ap_ssid, g_cfg.ap_pass, 6, 0, 4);
    delay(500);
    Serial.printf("  softAP '%s': %s  IP=%s\n", g_cfg.ap_ssid,
                  ok ? "OK" : "ECHEC", WiFi.softAPIP().toString().c_str());

    g_ws.on("/",        HTTP_GET,    handle_root);
    g_ws.on("/stats",   HTTP_GET,    handle_stats);
    g_ws.on("/sessions",HTTP_GET,    handle_sessions);
    g_ws.on("/file",    HTTP_GET,    handle_file);
    g_ws.on("/sdinfo",  HTTP_GET,    handle_sdinfo);
    g_ws.on("/log",     HTTP_GET,    handle_log);
    g_ws.on("/gps",     HTTP_GET,    handle_gps);
    g_ws.on("/config",  HTTP_GET,    handle_config_get);
    g_ws.on("/session", HTTP_DELETE, handle_session_delete);

    // POST /config — accumulate body, then parse
    g_ws.on("/config", HTTP_POST,
        [](AsyncWebServerRequest *req) {
            JsonDocument doc;
            if (deserializeJson(doc, g_post_buf, g_post_len) == DeserializationError::Ok) {
                strlcpy(g_cfg.wifi_ssid, doc["wifi_ssid"] | g_cfg.wifi_ssid, sizeof(g_cfg.wifi_ssid));
                strlcpy(g_cfg.wifi_pass, doc["wifi_pass"] | g_cfg.wifi_pass, sizeof(g_cfg.wifi_pass));
                strlcpy(g_cfg.ap_ssid,   doc["ap_ssid"]   | g_cfg.ap_ssid,   sizeof(g_cfg.ap_ssid));
                strlcpy(g_cfg.ap_pass,   doc["ap_pass"]   | g_cfg.ap_pass,   sizeof(g_cfg.ap_pass));
                g_cfg.vad_trigger_factor = doc["vad_trigger_factor"] | g_cfg.vad_trigger_factor;
                g_cfg.vad_silence_factor = doc["vad_silence_factor"] | g_cfg.vad_silence_factor;
                g_cfg.vad_min_trigger    = doc["vad_min_trigger"]    | (int)g_cfg.vad_min_trigger;
                g_cfg.vad_min_silence    = doc["vad_min_silence"]    | (int)g_cfg.vad_min_silence;
                g_cfg.ema_alpha          = doc["ema_alpha"]          | g_cfg.ema_alpha;
                g_cfg.vad_vote_needed    = doc["vad_vote_needed"]    | (int)g_cfg.vad_vote_needed;
                g_cfg.silence_timeout_ms = doc["silence_timeout_ms"] | (int)g_cfg.silence_timeout_ms;
                g_cfg.max_record_sec     = doc["max_record_sec"]     | (int)g_cfg.max_record_sec;
                g_cfg.mic_gain           = doc["mic_gain"]           | (int)g_cfg.mic_gain;
                g_cfg.zcr_min            = doc["zcr_min"]            | (int)g_cfg.zcr_min;
                g_cfg.zcr_max            = doc["zcr_max"]            | (int)g_cfg.zcr_max;
                g_cfg.cam_quality        = doc["cam_quality"]        | (int)g_cfg.cam_quality;
                g_cfg.cam_framesize      = doc["cam_framesize"]      | (int)g_cfg.cam_framesize;
                g_cfg.cam_aec_value      = doc["cam_aec_value"]      | (int)g_cfg.cam_aec_value;
                g_cfg.cam_agc_gain       = doc["cam_agc_gain"]       | (int)g_cfg.cam_agc_gain;
                g_cfg.gps_enabled        = doc["gps_enabled"]        | g_cfg.gps_enabled;
                g_cfg.gps_rx_pin         = doc["gps_rx_pin"]         | (int)g_cfg.gps_rx_pin;
                g_cfg.gps_tx_pin         = doc["gps_tx_pin"]         | (int)g_cfg.gps_tx_pin;
                // VAD params prennent effet immediatement (record_vad lit g_cfg)
                config_save();
                req->send(200, "application/json", "{\"ok\":true}");
            } else {
                req->send(400, "application/json", "{\"ok\":false,\"e\":\"parse\"}");
            }
            g_post_len = 0;
        },
        nullptr,
        [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total) {
            if (index == 0) g_post_len = 0;
            size_t rem = sizeof(g_post_buf) - g_post_len - 1;
            size_t cp = len < rem ? len : rem;
            memcpy(g_post_buf + g_post_len, data, cp);
            g_post_len += cp;
            g_post_buf[g_post_len] = 0;
        }
    );

    g_ws.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *req) {
        req->send(200, "text/plain", "OK");
        delay(200);
        ESP.restart();
    });

    g_ws.addHandler(&g_events);
    g_ws.onNotFound([](AsyncWebServerRequest *r){ r->send(404, "text/plain", "404"); });
    g_ws.begin();
    Serial.println("  AsyncWebServer OK");
}

// ============================================================
//  NTP
// ============================================================
void wifi_ntp_init() {
    Serial.printf("  STA '%s'...\n", g_cfg.wifi_ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(g_cfg.wifi_ssid, g_cfg.wifi_pass);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < NTP_TIMEOUT_MS) delay(400);
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("  ECHEC WiFi -- pas de NTP");
        WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(200); return;
    }
    Serial.printf("  IP=%s\n", WiFi.localIP().toString().c_str());
    configTzTime(TZ_FR, NTP_SERVER);
    struct tm ti; t0 = millis();
    while (!getLocalTime(&ti, 100) && millis() - t0 < 5000) delay(300);
    if (getLocalTime(&ti, 0)) {
        g_ntp_synced = true;
        char buf[32]; strftime(buf, sizeof(buf), "%d/%m/%Y %H:%M:%S", &ti);
        Serial.printf("  NTP OK : %s\n", buf);
    } else {
        Serial.println("  NTP timeout");
    }
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(200);
}

static void get_timestamp(char *out, size_t len) {
    struct tm ti;
    if (getLocalTime(&ti, 100)) strftime(out, len, "%Y%m%d_%H%M%S", &ti);
    else snprintf(out, len, "19700101_%06lu", millis() / 1000);
}

// ============================================================
//  FILTRE BIQUAD  (passe-bande 300-3400 Hz @ 16kHz)
// ============================================================
typedef struct { float b0,b1,b2,a1,a2,x1,x2,y1,y2; } biquad_t;
static biquad_t hp_filt = {0.92007f,-1.84015f,0.92007f,-1.83388f,0.84657f,0,0,0,0};
static biquad_t lp_filt = {0.36182f, 0.72363f,0.36182f, 0.26343f,0.18393f,0,0,0,0};

inline float biquad_step(biquad_t *f, float x) {
    float y = f->b0*x + f->b1*f->x1 + f->b2*f->x2 - f->a1*f->y1 - f->a2*f->y2;
    f->x2=f->x1; f->x1=x; f->y2=f->y1; f->y1=y; return y;
}
static void apply_bandpass(const int16_t *src, int16_t *dst, size_t n) {
    for (size_t i = 0; i < n; i++) {
        float s = biquad_step(&hp_filt, (float)src[i]);
        s = biquad_step(&lp_filt, s);
        if (s > 32767) s = 32767; else if (s < -32768) s = -32768;
        dst[i] = (int16_t)s;
    }
}

// ============================================================
//  RMS / ZCR
// ============================================================
static uint16_t compute_rms(const int16_t *s, size_t n) {
    if (!n) return 0;
    int64_t m = 0;
    for (size_t i = 0; i < n; i++) m += s[i];
    int32_t dc = (int32_t)(m / (int64_t)n);
    uint64_t sum = 0;
    for (size_t i = 0; i < n; i++) { int32_t v = (int32_t)s[i] - dc; sum += (uint64_t)(v*v); }
    return (uint16_t)sqrtf((float)sum / n);
}
static void prepare_wav_chunk(int16_t *s, size_t n) {
    if (!n) return;
    int64_t m = 0;
    for (size_t i = 0; i < n; i++) m += s[i];
    int32_t dc = (int32_t)(m / (int64_t)n);
    for (size_t i = 0; i < n; i++) {
        int32_t v = ((int32_t)s[i] - dc) * g_cfg.mic_gain;
        s[i] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
    }
}
static uint16_t compute_zcr(const int16_t *b, size_t n) {
    uint16_t c = 0;
    for (size_t i = 1; i < n; i++) if ((b[i] >= 0) != (b[i-1] >= 0)) c++;
    return c;
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

static void write_wav_header(File &f, uint32_t data_size) {
    wav_header_t h = {{'R','I','F','F'}, data_size+36, {'W','A','V','E'},
        {'f','m','t',' '}, 16, 1, 1, SAMPLE_RATE, SAMPLE_RATE*2, 2, 16,
        {'d','a','t','a'}, data_size};
    f.seek(0); f.write((uint8_t*)&h, sizeof(h));
}

// ============================================================
//  CAMERA
// ============================================================
static bool camera_init() {
    camera_config_t cfg = {};
    cfg.ledc_channel = LEDC_CHANNEL_0; cfg.ledc_timer = LEDC_TIMER_0;
    cfg.pin_d0=Y2_GPIO_NUM; cfg.pin_d1=Y3_GPIO_NUM; cfg.pin_d2=Y4_GPIO_NUM;
    cfg.pin_d3=Y5_GPIO_NUM; cfg.pin_d4=Y6_GPIO_NUM; cfg.pin_d5=Y7_GPIO_NUM;
    cfg.pin_d6=Y8_GPIO_NUM; cfg.pin_d7=Y9_GPIO_NUM;
    cfg.pin_xclk=XCLK_GPIO_NUM; cfg.pin_pclk=PCLK_GPIO_NUM;
    cfg.pin_vsync=VSYNC_GPIO_NUM; cfg.pin_href=HREF_GPIO_NUM;
    cfg.pin_sscb_sda=SIOD_GPIO_NUM; cfg.pin_sscb_scl=SIOC_GPIO_NUM;
    cfg.pin_pwdn=PWDN_GPIO_NUM; cfg.pin_reset=RESET_GPIO_NUM;
    cfg.xclk_freq_hz = 20000000; cfg.pixel_format = PIXFORMAT_JPEG;
    cfg.frame_size   = (framesize_t)g_cfg.cam_framesize;
    cfg.jpeg_quality = g_cfg.cam_quality;
    cfg.fb_count = 1; cfg.fb_location = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    if (esp_camera_init(&cfg) != ESP_OK) { Serial.println("ERREUR camera"); return false; }
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_vflip(s, 1); s->set_hmirror(s, 1);
        s->set_exposure_ctrl(s, 0); s->set_aec_value(s, g_cfg.cam_aec_value);
        s->set_gain_ctrl(s, 0);    s->set_agc_gain(s, g_cfg.cam_agc_gain);
        s->set_aec2(s, 0); s->set_ae_level(s, 0);
        s->set_whitebal(s, 1); s->set_awb_gain(s, 1);
        s->set_bpc(s, 1); s->set_wpc(s, 1);
    }
    Serial.println("  Camera OK");
    return true;
}

static void take_photo(const char *path) {
    if (!g_cam_ok) return;
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { log_line("[CAM] echec capture"); return; }
    if (g_sd_mutex) xSemaphoreTake(g_sd_mutex, portMAX_DELAY);
    File f = SD.open(path, FILE_WRITE);
    if (f) { f.write(fb->buf, fb->len); f.close();
              log_linef("[CAM] %s %.1fkB", path, fb->len/1024.0f); }
    else log_linef("[CAM] ERREUR ouverture %s", path);
    if (g_sd_mutex) xSemaphoreGive(g_sd_mutex);
    esp_camera_fb_return(fb);
}

// ============================================================
//  MICRO PDM
// ============================================================
static bool mic_init() {
    i2s_config_t cfg = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER|I2S_MODE_RX|I2S_MODE_PDM),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_PCM_SHORT,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 16, .dma_buf_len = 256,
        .use_apll = false, .tx_desc_auto_clear = false, .fixed_mclk = 0
    };
    if (i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL) != ESP_OK) return false;
    i2s_pin_config_t pins = {I2S_PIN_NO_CHANGE, I2S_PIN_NO_CHANGE,
                              MIC_CLK_PIN, I2S_PIN_NO_CHANGE, MIC_DATA_PIN};
    if (i2s_set_pin(I2S_NUM_0, &pins) != ESP_OK) return false;
    Serial.println("  Micro PDM OK");
    return true;
}

// ============================================================
//  GPS  (optionnel, tache Core 0)
// ============================================================
static double nmea_to_deg(const char *s, char dir) {
    double v = atof(s);
    int d = (int)(v / 100);
    double r = d + (v - d * 100) / 60.0;
    return (dir == 'S' || dir == 'W') ? -r : r;
}
static void parse_gprmc(const char *line) {
    char buf[128]; strncpy(buf, line, 127); buf[127] = 0;
    char *tok[13] = {};
    int i = 0; char *p = strtok(buf, ",");
    while (p && i < 13) { tok[i++] = p; p = strtok(nullptr, ","); }
    if (i < 7) return;
    bool fix = (tok[2] && tok[2][0] == 'A');
    g_gps_fix = fix;
    if (fix && tok[3] && tok[4] && tok[5] && tok[6]) {
        g_gps_lat = nmea_to_deg(tok[3], tok[4][0]);
        g_gps_lon = nmea_to_deg(tok[5], tok[6][0]);
    }
}
static void gps_task_fn(void *arg) {
    Serial2.begin(9600, SERIAL_8N1, g_cfg.gps_rx_pin, g_cfg.gps_tx_pin);
    char line[128]; int li = 0;
    while (true) {
        while (Serial2.available()) {
            char c = (char)Serial2.read();
            if (c == '\n' || li >= 126) {
                line[li] = 0;
                if (strncmp(line, "$GPRMC", 6) == 0) parse_gprmc(line);
                li = 0;
            } else if (c != '\r') {
                line[li++] = c;
            }
        }
        delay(10);
    }
}
static void write_session_meta(const char *session_id) {
    char path[64];
    snprintf(path, sizeof(path), "/session_%s/meta.json", session_id);
    char buf[128];
    snprintf(buf, sizeof(buf),
        "{\"session\":\"%s\",\"lat\":%.6f,\"lon\":%.6f,\"fix\":%s}",
        session_id, (double)g_gps_lat, (double)g_gps_lon,
        g_gps_fix ? "true" : "false");
    if (g_sd_mutex) xSemaphoreTake(g_sd_mutex, portMAX_DELAY);
    File f = SD.open(path, FILE_WRITE);
    if (f) { f.print(buf); f.close(); }
    if (g_sd_mutex) xSemaphoreGive(g_sd_mutex);
}

// ============================================================
//  SEED EMA
// ============================================================
static void seed_noise_ema(uint8_t *buf, int16_t *fbuf) {
    log_linef("[SEED] %dms silence svp...", VAD_SEED_MS);
    uint32_t t0 = millis(); float rs = 0, rm = 99999; uint32_t nc = 0;
    while (millis() - t0 < VAD_SEED_MS) {
        size_t br = 0;
        i2s_read(I2S_NUM_0, buf, VAD_CHUNK_BYTES, &br, pdMS_TO_TICKS(200));
        if (!br) continue;
        apply_bandpass((int16_t*)buf, fbuf, br/2);
        float r = compute_rms(fbuf, br/2);
        rs += r; if (r < rm) rm = r; nc++;
    }
    float ra = (nc > 0) ? (rs / nc) : 20.0f;
    float floor_abs = rm * 0.2f + ra * 0.8f;
    g_noise_ema    = floor_abs;
    g_vad_trigger  = max((uint16_t)g_cfg.vad_min_trigger,
                         (uint16_t)(floor_abs * g_cfg.vad_trigger_factor));
    g_vad_silence  = max((uint16_t)g_cfg.vad_min_silence,
                         (uint16_t)(floor_abs * g_cfg.vad_silence_factor));
    log_linef("[SEED] plancher=%.0f trigger=%u silence=%u", floor_abs, g_vad_trigger, g_vad_silence);
}

// ============================================================
//  BOUCLE VAD  (bloquante dans setup)
// ============================================================
void record_vad() {
    uint8_t  *buf     = (uint8_t *)malloc(VAD_CHUNK_BYTES);
    int16_t  *fbuf    = (int16_t *)malloc(VAD_CHUNK_BYTES);
    uint8_t  *preroll = (uint8_t *)ps_malloc((size_t)PRE_ROLL_CHUNKS * VAD_CHUNK_BYTES);
    if (!buf || !fbuf || !preroll) { log_line("ERREUR malloc VAD"); return; }

    int pr_head = 0, pr_count = 0;
    File wav;
    uint32_t written = 0, silence_ms = 0, rec_start = 0, last_print = 0, last_veil = 0;
    uint16_t rms_peak = 0;
    int votes = 0;
    bool recording = false;
    char session_id[24] = "", photo_path[64] = "";

    seed_noise_ema(buf, fbuf);
    log_linef("[VAD] trigger=%u silence=%u NTP=%s", g_vad_trigger, g_vad_silence, g_ntp_synced?"OK":"NON");
    log_linef("[VAD] http://%s/", WiFi.softAPIP().toString().c_str());
    log_line("[VAD] En attente de voix...");

    while (true) {
        size_t br = 0;
        i2s_read(I2S_NUM_0, buf, VAD_CHUNK_BYTES, &br, pdMS_TO_TICKS(500));
        if (!br) continue;

        int16_t *smp = (int16_t*)buf; size_t ns = br / 2;
        uint16_t rms_raw  = compute_rms(smp, ns);
        if (rms_raw > rms_peak) rms_peak = rms_raw;
        apply_bandpass(smp, fbuf, ns);
        uint16_t rms_filt = compute_rms(fbuf, ns);
        uint16_t zcr      = compute_zcr(fbuf, ns);
        bool zcr_ok = (zcr >= g_cfg.zcr_min && zcr <= g_cfg.zcr_max);

        if (!recording) {
            g_noise_ema   = g_cfg.ema_alpha * g_noise_ema + (1 - g_cfg.ema_alpha) * (float)rms_filt;
            g_vad_trigger = max((uint16_t)g_cfg.vad_min_trigger,
                                (uint16_t)(g_noise_ema * g_cfg.vad_trigger_factor));
            g_vad_silence = max((uint16_t)g_cfg.vad_min_silence,
                                (uint16_t)(g_noise_ema * g_cfg.vad_silence_factor));
        }

        bool cv    = recording ? (rms_filt >= g_vad_silence) : (rms_filt >= g_vad_trigger && zcr_ok);
        if (!recording) votes = cv ? (votes + 1) : 0;
        bool voice = recording ? cv : (votes >= g_cfg.vad_vote_needed);

        prepare_wav_chunk(smp, ns);

        if (!recording) {
            memcpy(preroll + pr_head * VAD_CHUNK_BYTES, buf, br);
            pr_head = (pr_head + 1) % PRE_ROLL_CHUNKS;
            if (pr_count < PRE_ROLL_CHUNKS) pr_count++;
        }

        uint32_t now = millis();
        if (!recording) {
            g_rms_raw_d = rms_peak; g_rms_filt_d = rms_filt; g_zcr_d = zcr;
            g_noise_ema_d = g_noise_ema; g_trigger_d = g_vad_trigger; g_vad_sil_d = g_vad_silence;
            g_recording = false;

            if (now - last_veil >= 500) {
                log_linef("[VEI] raw=%4u filt=%4u EMA=%.0f trig=%u %3.0f%% zcr=%3u%s v=%d",
                    rms_peak, rms_filt, g_noise_ema, g_vad_trigger,
                    g_vad_trigger ? (float)rms_filt * 100 / g_vad_trigger : 0.f,
                    zcr, zcr_ok ? "" : "(KO)", votes);
                rms_peak = 0; last_veil = now;
            }

            if (voice) {
                get_timestamp(session_id, sizeof(session_id));
                char dir[48], wav_path[64];
                snprintf(dir,      sizeof(dir),      "/session_%s", session_id);
                snprintf(wav_path, sizeof(wav_path), "/session_%s/audio.wav", session_id);
                snprintf(photo_path, sizeof(photo_path), "/session_%s/photo.jpg", session_id);
                if (g_sd_mutex) xSemaphoreTake(g_sd_mutex, portMAX_DELAY);
                SD.mkdir(dir);
                wav = SD.open(wav_path, FILE_WRITE);
                if (g_sd_mutex) xSemaphoreGive(g_sd_mutex);
                if (!wav) { log_linef("ERREUR open %s", wav_path); continue; }
                write_wav_header(wav, 0); written = 0;
                if (pr_count > 0) {
                    int sl = (pr_head - pr_count + PRE_ROLL_CHUNKS) % PRE_ROLL_CHUNKS;
                    for (int i = 0; i < pr_count; i++) {
                        int s = (sl + i) % PRE_ROLL_CHUNKS;
                        wav.write(preroll + s * VAD_CHUNK_BYTES, VAD_CHUNK_BYTES);
                        written += VAD_CHUNK_BYTES;
                    }
                    pr_count = 0; pr_head = 0;
                }
                silence_ms = 0; votes = 0; rms_peak = 0; rec_start = now; last_print = now;
                recording = true;
                strncpy(g_session_d, session_id, sizeof(g_session_d) - 1);
                g_wav_written_d = 0; g_sil_ms_d = 0; g_recording = true;
                log_linef("[%s] DEBUT rms=%u trig=%u", session_id, rms_filt, g_vad_trigger);
            }
        } else {
            wav.write(buf, br); written += br;
            if (voice) silence_ms = 0; else silence_ms += VAD_CHUNK_MS;
            g_wav_written_d = written; g_sil_ms_d = silence_ms;

            if (now - last_print >= 1000) {
                uint32_t sec = (now - rec_start) / 1000;
                log_linef("[%s] %3lus rms=%4u sil=%lu/%lums",
                    session_id, sec, rms_filt,
                    (unsigned long)silence_ms, (unsigned long)g_cfg.silence_timeout_ms);
                last_print = now;
            }

            bool tout = ((now - rec_start) >= (uint32_t)g_cfg.max_record_sec * 1000);
            if (silence_ms >= g_cfg.silence_timeout_ms || tout) {
                write_wav_header(wav, written); wav.close();
                log_linef("[%s] FIN %.1fs %lub %s",
                    session_id, written / (float)(SAMPLE_RATE * 2),
                    written, tout ? "TIMEOUT" : "SILENCE");
                take_photo(photo_path);
                if (g_cfg.gps_enabled) write_session_meta(session_id);
                g_recording = false; recording = false;
                log_line("[VAD] En attente de voix...");
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
    Serial.println("\n=== XIAO ESP32S3 - VAD Back-Office ===");
    Serial.printf("CPU:%luMHz PSRAM:%luKB Flash:%luMB\n",
        getCpuFrequencyMhz(), ESP.getPsramSize()/1024,
        spi_flash_get_chip_size()/(1024*1024));

    g_sd_mutex  = xSemaphoreCreateMutex();
    g_log_mutex = xSemaphoreCreateMutex();
    config_defaults();  // toujours avant NTP pour avoir les creds WiFi

    Serial.println("\n[1/6] NTP...");
    wifi_ntp_init();

    Serial.println("[2/6] AP + WebServer...");
    ap_webserver_init();
    // L'AP est UP ici : la carte est joignable meme si SD ou camera plante

    Serial.println("[3/6] SD + Config...");
    pinMode(SD_CS_PIN, OUTPUT); digitalWrite(SD_CS_PIN, HIGH); delay(100);
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN); delay(100);
    if (!SD.begin(SD_CS_PIN)) {
        log_line("ERREUR SD -- sessions desactivees (verif carte)");
        // pas de blocage : l'AP reste accessible
    } else {
        g_sd_ok = true;
        log_linef("SD OK %lluMB type=%d", SD.cardSize()/(1024*1024), SD.cardType());
        if (config_load()) log_line("Config: /config.json charge");
        else log_line("Config: valeurs par defaut");
    }

    Serial.println("[4/6] Camera...");
    g_cam_ok = camera_init();
    if (!g_cam_ok) Serial.println("  AVERTISSEMENT: camera KO");

    Serial.println("[5/6] Micro PDM...");
    if (!mic_init()) { Serial.println("ERREUR micro -- blocage"); while (1) delay(1000); }

    if (g_cfg.gps_enabled) {
        Serial.println("[6/6] GPS UART...");
        xTaskCreatePinnedToCore(gps_task_fn, "gps", 2048, nullptr, 1, nullptr, 0);
    } else {
        Serial.println("[6/6] GPS: desactive (config)");
    }

    Serial.println("\n=== TOUT OK -- demarrage VAD ===\n");
    record_vad();
}

void loop() { delay(1000); }
