#pragma once

// Dashboard admin — 100% servi depuis la carte (RAM), fonctionne SANS Internet.
// Leaflet (carte) est chargé en option, de façon non bloquante : s'il n'arrive pas
// (client sans Internet, ex. sur l'AP terrain), le reste de l'interface marche quand même.

static const char DASHBOARD_HTML[] = R"rawhtml(
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-VAD Admin</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,monospace;background:#0d1117;color:#c9d1d9;font-size:14px}
header{position:sticky;top:0;z-index:5;background:#161b22;padding:10px 14px;display:flex;justify-content:space-between;align-items:center;border-bottom:1px solid #30363d}
h1{font-size:1rem;color:#58a6ff}
h2{font-size:.8rem;color:#58a6ff;margin:0 0 8px;text-transform:uppercase;letter-spacing:.5px}
#dot{width:9px;height:9px;border-radius:50%;background:#484f58;display:inline-block;margin-right:6px;vertical-align:middle}
#dot.live{background:#3fb950;box-shadow:0 0 6px #3fb950}
.wrap{max-width:920px;margin:0 auto;padding:12px;display:grid;gap:12px}
.card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:12px}
.row{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
button{background:#21262d;color:#c9d1d9;border:1px solid #30363d;border-radius:6px;padding:7px 12px;cursor:pointer;font-size:.8rem}
button:hover{background:#30363d}
button.primary{background:#238636;border-color:#2ea043;color:#fff}
button.danger{background:#6e1414;border-color:#da3633;color:#fff}
#cam{width:100%;max-height:50vh;object-fit:contain;background:#010409;border-radius:6px;display:block;min-height:120px}
.gps b,.sys b{color:#79c0ff}
.gps div,.sys div{line-height:1.8}
#map{height:200px;border-radius:6px;margin-top:8px;display:none;background:#1c2128}
#term{height:34vh;overflow-y:auto;font-family:monospace;font-size:.72rem;line-height:1.4;white-space:pre-wrap;word-break:break-all;background:#010409;color:#79c0ff;padding:8px;border-radius:6px}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px}
label{display:flex;flex-direction:column;font-size:.72rem;color:#8b949e;gap:3px}
input,select{background:#0d1117;color:#c9d1d9;border:1px solid #30363d;border-radius:5px;padding:6px;font-size:.8rem}
#msg{font-size:.75rem;color:#3fb950;min-height:16px}
.muted{color:#8b949e;font-size:.72rem}
</style>
</head>
<body>
<header>
  <h1>&#128225; ESP32-VAD Admin</h1>
  <span><span id="dot"></span><span id="wss">Connexion&hellip;</span></span>
</header>

<div class="wrap">

  <div class="card">
    <h2>&#128247; Cam&eacute;ra</h2>
    <img id="cam" alt="apercu camera" src="">
    <div class="row" style="margin-top:8px">
      <button id="camToggle" class="primary">&#9208; Pause</button>
      <button onclick="camOnce()">&#128260; Rafra&icirc;chir</button>
      <span class="muted">Aper&ccedil;u captur&eacute; hors enregistrement uniquement.</span>
    </div>
  </div>

  <div class="card gps">
    <h2>&#128205; GPS</h2>
    <div>Lat : <b id="glat">--</b>&nbsp;&nbsp; Lng : <b id="glng">--</b></div>
    <div>Altitude : <b id="galt">--</b> m &nbsp;&nbsp; Satellites : <b id="gsat">--</b></div>
    <div>Fix : <b id="gfix">--</b></div>
    <div id="map"></div>
  </div>

  <div class="card sys">
    <h2>&#9881; Syst&egrave;me</h2>
    <div id="sys">Chargement&hellip;</div>
    <div class="row" style="margin-top:8px">
      <button class="danger" onclick="doRestart()">&#8635; Red&eacute;marrer la carte</button>
    </div>
  </div>

  <div class="card">
    <h2>&#128246; R&eacute;seau WiFi</h2>
    <div class="grid2">
      <label>SSID (r&eacute;seau maison)<input type="text" data-cfg="wifi_ssid" autocomplete="off"></label>
      <label>Mot de passe<input type="password" data-cfg="wifi_pass" placeholder="laisser vide = inchang&eacute;" autocomplete="off"></label>
    </div>
    <div class="row" style="margin-top:10px">
      <button onclick="testWifi()">&#128246; Tester la connexion</button>
      <button class="primary" onclick="saveCfg()">&#128190; Enregistrer</button>
      <span id="wifiRes" class="muted"></span>
    </div>
    <div class="muted" style="margin-top:6px">Le test reconnecte le WiFi (~10 s) : testez de pr&eacute;f&eacute;rence depuis l'AP. Le changement s'applique au prochain red&eacute;marrage.</div>
  </div>

  <div class="card">
    <h2>&#128196; Terminal temps r&eacute;el</h2>
    <div id="term"></div>
  </div>

  <div class="card">
    <h2>&#128295; R&eacute;glages</h2>
    <div class="grid2">
      <label>Facteur trigger (&times;EMA)<input type="number" step="0.1" data-cfg="trigger_factor"></label>
      <label>Facteur silence (&times;EMA)<input type="number" step="0.1" data-cfg="silence_factor"></label>
      <label>ZCR min<input type="number" data-cfg="zcr_min"></label>
      <label>ZCR max<input type="number" data-cfg="zcr_max"></label>
      <label>Votes requis<input type="number" data-cfg="vote_needed"></label>
      <label>Gain micro (&times;)<input type="number" data-cfg="mic_gain"></label>
      <label>Timeout silence (ms)<input type="number" data-cfg="silence_timeout_ms"></label>
      <label>Dur&eacute;e max session (s)<input type="number" data-cfg="max_record_sec"></label>
      <label>R&eacute;solution cam&eacute;ra
        <select data-cfg="cam_framesize">
          <option value="8">VGA 640&times;480</option>
          <option value="9">SVGA 800&times;600</option>
          <option value="10">XGA 1024&times;768</option>
          <option value="12">SXGA 1280&times;1024</option>
          <option value="13">UXGA 1600&times;1200</option>
        </select>
      </label>
      <label>Qualit&eacute; JPEG (10=top)<input type="number" data-cfg="cam_quality"></label>
      <label>Exposition (0-1200)<input type="number" data-cfg="cam_aec"></label>
      <label>Gain cam&eacute;ra (0-30)<input type="number" data-cfg="cam_agc"></label>
      <label>Flip vertical<select data-cfg="cam_vflip"><option value="1">Oui</option><option value="0">Non</option></select></label>
      <label>Miroir horizontal<select data-cfg="cam_hmirror"><option value="1">Oui</option><option value="0">Non</option></select></label>
    </div>
    <div class="row" style="margin-top:10px">
      <button class="primary" onclick="saveCfg()">&#128190; Enregistrer</button>
      <button onclick="loadCfg()">&#8635; Recharger</button>
      <span id="msg"></span>
    </div>
    <div class="muted" style="margin-top:6px">R&eacute;solution/qualit&eacute; s'appliquent aussi aux photos de session. Enregistr&eacute; en m&eacute;moire flash (persiste apr&egrave;s reboot).</div>
  </div>

</div>

<script>
(function(){
  // ---------- WebSocket terminal ----------
  var term=document.getElementById('term');
  var dot=document.getElementById('dot');
  var wsEl=document.getElementById('wss');
  function connectWS(){
    var ws=new WebSocket('ws://'+location.host+'/ws');
    ws.onopen=function(){dot.classList.add('live');wsEl.textContent='LIVE';};
    ws.onclose=function(){dot.classList.remove('live');wsEl.textContent='Hors ligne';setTimeout(connectWS,2000);};
    ws.onerror=function(){wsEl.textContent='Erreur WS';};
    ws.onmessage=function(e){
      term.textContent+=e.data;
      term.scrollTop=term.scrollHeight;
      if(term.textContent.length>60000) term.textContent=term.textContent.slice(-50000);
    };
  }
  connectWS();

  // ---------- Camera preview ----------
  var cam=document.getElementById('cam');
  var camOn=true;
  function camOnce(){ cam.src='/api/photo?t='+Date.now(); }
  window.camOnce=camOnce;
  function camTick(){ if(camOn && !document.hidden) camOnce(); }
  setInterval(camTick,2500);
  document.getElementById('camToggle').onclick=function(){
    camOn=!camOn;
    this.innerHTML=camOn?'&#9208; Pause':'&#9654; Reprendre';
    this.className=camOn?'primary':'';
  };
  camOnce();

  // ---------- GPS + map (Leaflet optionnel, non bloquant) ----------
  var map=null,mk=null;
  function tryLeaflet(){
    var css=document.createElement('link');css.rel='stylesheet';
    css.href='https://unpkg.com/leaflet@1.9.4/dist/leaflet.css';document.head.appendChild(css);
    var s=document.createElement('script');s.src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js';
    s.onload=function(){
      try{
        document.getElementById('map').style.display='block';
        map=L.map('map').setView([46.8,2.3],5);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19}).addTo(map);
      }catch(e){}
    };
    document.head.appendChild(s); // si echec (pas d'Internet) : ignore, le reste marche
  }
  tryLeaflet();

  function updGps(){
    fetch('/api/gps').then(function(r){return r.json();}).then(function(d){
      document.getElementById('glat').textContent=d.valid?d.lat.toFixed(6):'--';
      document.getElementById('glng').textContent=d.valid?d.lng.toFixed(6):'--';
      document.getElementById('galt').textContent=d.valid?d.alt.toFixed(1):'--';
      document.getElementById('gsat').textContent=d.valid?d.sats:'--';
      document.getElementById('gfix').textContent=d.valid?'✓ actif':'✗ pas de fix';
      if(d.valid && map){
        if(!mk){mk=L.marker([d.lat,d.lng]).addTo(map);map.setView([d.lat,d.lng],15);}
        else mk.setLatLng([d.lat,d.lng]);
      }
    }).catch(function(){});
  }

  function updSys(){
    fetch('/api/status').then(function(r){return r.json();}).then(function(d){
      document.getElementById('sys').innerHTML=
        'Heap libre : <b>'+(d.heap/1024|0)+' ko</b> &nbsp; PSRAM : <b>'+(d.psram/1024|0)+' ko</b><br>'
        +'NTP : <b>'+(d.ntp_synced?'✓':'✗')+'</b> &nbsp; '
        +'WiFi : <b>'+(d.rssi?d.rssi+' dBm':'AP seul')+'</b>'+(d.sta_ip?' ('+d.sta_ip+')':'')+' &nbsp; '
        +'État : <b>'+(d.recording?'● ENREGISTRE':'○ veille')+'</b> &nbsp; '
        +'EMA bruit : <b>'+Math.round(d.noise_ema)+'</b>';
    }).catch(function(){});
  }
  setInterval(updGps,3000);
  setInterval(updSys,2000);
  updGps();updSys();

  // ---------- Config ----------
  var msg=document.getElementById('msg');
  function showMsg(t,ok){msg.textContent=t;msg.style.color=ok?'#3fb950':'#f85149';setTimeout(function(){msg.textContent='';},3000);}
  function loadCfg(){
    fetch('/api/config').then(function(r){return r.json();}).then(function(c){
      document.querySelectorAll('[data-cfg]').forEach(function(el){
        var k=el.getAttribute('data-cfg');
        if(k in c) el.value=c[k];
      });
    }).catch(function(){showMsg('Lecture config impossible',false);});
  }
  window.loadCfg=loadCfg;
  function saveCfg(){
    var body=[];
    document.querySelectorAll('[data-cfg]').forEach(function(el){
      body.push(el.getAttribute('data-cfg')+'='+encodeURIComponent(el.value));
    });
    fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body.join('&')})
      .then(function(r){return r.json();})
      .then(function(){showMsg('✓ Configuration enregistrée',true);})
      .catch(function(){showMsg('Echec enregistrement',false);});
  }
  window.saveCfg=saveCfg;
  loadCfg();

  // ---------- Test WiFi ----------
  function wEl(){return document.getElementById('wifiRes');}
  window.testWifi=function(){
    var ssid=document.querySelector('[data-cfg="wifi_ssid"]').value;
    var pass=document.querySelector('[data-cfg="wifi_pass"]').value;
    var el=wEl();el.style.color='#8b949e';el.textContent='Test en cours… (jusqu’à 10 s)';
    fetch('/api/wifi/test',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
      body:'wifi_ssid='+encodeURIComponent(ssid)+'&wifi_pass='+encodeURIComponent(pass)})
      .then(function(r){
        if(r.status===409){el.style.color='#f85149';el.textContent='Impossible : enregistrement en cours';return null;}
        return r.json();
      })
      .then(function(j){ if(j) setTimeout(pollWifi,1200); })
      .catch(function(){el.style.color='#f85149';el.textContent='Échec du lancement';});
  };
  function pollWifi(){
    fetch('/api/wifi/status').then(function(r){return r.json();}).then(function(s){
      var el=wEl();
      if(s.state===2){
        if(s.ok){el.style.color='#3fb950';el.innerHTML='✓ Connecté — RSSI '+s.rssi+' dBm — IP '+s.ip;}
        else{el.style.color='#f85149';el.textContent='✗ Échec de connexion (SSID/mot de passe ?)';}
      } else { setTimeout(pollWifi,1200); }
    }).catch(function(){setTimeout(pollWifi,1500);});
  }

  // ---------- Restart ----------
  window.doRestart=function(){
    if(!confirm('Redémarrer la carte ?')) return;
    fetch('/api/restart',{method:'POST'}).then(function(){showMsg('Redémarrage en cours…',true);}).catch(function(){});
  };
})();
</script>
</body>
</html>
)rawhtml";
