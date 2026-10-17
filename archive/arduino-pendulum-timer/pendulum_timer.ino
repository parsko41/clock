// Pendulum timer - ESP32 + IR sensor on GPIO22, with a live stats webpage.
//
// First boot (no Wi-Fi saved): the ESP32 starts an open hotspot "Pendulum-Setup".
//   Join it, browse to http://192.168.4.1, pick your home network and type its password.
//   The ESP32 saves it and reboots onto your network.
// After that: browse to http://pendulum.local (or the IP printed on Serial at 115200).
//   /data  -> JSON stats     /forget -> erase saved Wi-Fi and go back to setup mode
//
// Keep the beam clear at boot: blocked polarity is auto-detected from the idle level.

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>

const int      IR_PIN        = 22;
int            BLOCKED_LEVEL = HIGH;    // auto-set in setup()
const double   TARGET_BEAT_S = 1.0;     // 60 BPM
const uint32_t GLITCH_US     = 2000;    // ignore edges closer than 2 ms
const int      AVG_N         = 30;      // beats in the rolling average
const int      HIST_N        = 300;     // beats kept for the charts
const char*    HOSTNAME      = "pendulum";
const char*    AP_SSID       = "Pendulum-Setup";

WebServer   server(80);
Preferences prefs;
bool        apMode = false;

// ---- ISR ring buffer ----
struct Edge { uint32_t t; uint8_t lvl; };
volatile Edge     buf[64];
volatile uint32_t head = 0;
volatile uint32_t lastT = 0;
volatile int      lastLvl = -1;

void IRAM_ATTR onEdge() {
  uint32_t t = micros();
  int lvl = digitalRead(IR_PIN);
  if (lvl == lastLvl) return;
  if (t - lastT < GLITCH_US) return;
  lastLvl = lvl; lastT = t;
  buf[head & 63].t = t;
  buf[head & 63].lvl = lvl;
  head++;
}

// ---- stats ----
uint32_t tail = 0;
uint32_t prevBlockT = 0, blockT = 0;
bool     haveBlock = false;
double   ring[AVG_N]; int ringN = 0, ringI = 0;
double   lastIntervalMs = 0, prevIntervalMs = 0, lastBlockedMs = 0;
double   avgMs = 0, rateSpd = 0, beatErrMs = 0;
double   totalMs = 0;                    // sum of all intervals, for the long-run rate
uint32_t beatCount = 0;
uint32_t lastBeatMillis = 0;
float    histI[HIST_N], histB[HIST_N]; int histN = 0, histHead = 0;

void processEdges() {
  while (tail != head) {
    noInterrupts();
    Edge e = { buf[tail & 63].t, buf[tail & 63].lvl };
    interrupts();
    tail++;

    if (e.lvl == BLOCKED_LEVEL) {
      blockT = e.t;
      haveBlock = true;
    } else if (haveBlock) {
      lastBlockedMs = (e.t - blockT) / 1000.0;
      if (prevBlockT) {
        prevIntervalMs = lastIntervalMs;
        lastIntervalMs = (blockT - prevBlockT) / 1000.0;
        beatCount++;
        totalMs += lastIntervalMs;
        lastBeatMillis = millis();

        ring[ringI] = lastIntervalMs; ringI = (ringI + 1) % AVG_N;
        if (ringN < AVG_N) ringN++;
        double sum = 0; for (int i = 0; i < ringN; i++) sum += ring[i];
        avgMs = sum / ringN;
        rateSpd = -(avgMs / 1000.0 - TARGET_BEAT_S) / TARGET_BEAT_S * 86400.0;
        beatErrMs = prevIntervalMs > 0 ? fabs(lastIntervalMs - prevIntervalMs) / 2.0 : 0;

        histI[histHead] = lastIntervalMs; histB[histHead] = lastBlockedMs;
        histHead = (histHead + 1) % HIST_N;
        if (histN < HIST_N) histN++;

        Serial.printf("%lu\t%.2f\t\t%.1f\t\t%.2f\t%+.0f\t\t%.2f\n",
                      beatCount, lastIntervalMs, lastBlockedMs, avgMs, rateSpd, beatErrMs);
      }
      prevBlockT = blockT;
      haveBlock = false;
    }
  }
}

// ---- web pages ----
const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Pendulum Timer</title>
<style>
:root{--bg:#fcfcfb;--card:#ffffff;--ink:#0b0b0b;--ink2:#52514e;--grid:#e6e5e0;--s1:#2a78d6;--ref:#9a998f;--good:#0ca30c;--bad:#d03b3b}
@media (prefers-color-scheme:dark){:root{--bg:#1a1a19;--card:#232322;--ink:#f0efec;--ink2:#c3c2b7;--grid:#383835;--s1:#3987e5;--ref:#7d7c74}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.4 system-ui,sans-serif;padding:16px;max-width:1000px;margin:auto}
h1{font-size:20px;margin:0 0 4px}.sub{color:var(--ink2);font-size:13px;margin-bottom:16px}
.tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px;margin-bottom:16px}
.tile{background:var(--card);border:1px solid var(--grid);border-radius:8px;padding:10px 12px}
.tile .k{color:var(--ink2);font-size:12px}.tile .v{font-size:24px;font-variant-numeric:tabular-nums;font-weight:600}.tile .u{font-size:12px;color:var(--ink2)}
.card{background:var(--card);border:1px solid var(--grid);border-radius:8px;padding:12px;margin-bottom:14px;position:relative}
.card h2{font-size:14px;margin:0 0 6px;font-weight:600}canvas{width:100%;height:200px;display:block}
.tip{position:absolute;pointer-events:none;background:var(--ink);color:var(--bg);font-size:12px;padding:4px 7px;border-radius:4px;display:none;white-space:nowrap}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:5px;vertical-align:middle}
a{color:var(--s1)}
</style></head><body>
<h1>Pendulum Timer</h1><div class="sub" id="status">connecting...</div>
<div class="tiles">
<div class="tile"><div class="k">Beats</div><div class="v" id="n">-</div></div>
<div class="tile"><div class="k">Last interval</div><div class="v" id="last">-</div><div class="u">ms</div></div>
<div class="tile"><div class="k">Avg (last 30)</div><div class="v" id="avg">-</div><div class="u">ms</div></div>
<div class="tile"><div class="k">Rate (last 30)</div><div class="v" id="rate">-</div><div class="u">s/day</div></div>
<div class="tile"><div class="k">Rate (since start)</div><div class="v" id="lrate">-</div><div class="u">s/day</div></div>
<div class="tile"><div class="k">Beat error</div><div class="v" id="be">-</div><div class="u">ms (tick vs tock)</div></div>
<div class="tile"><div class="k">Beam blocked</div><div class="v" id="blk">-</div><div class="u">ms (shorter = bigger swing)</div></div>
</div>
<div class="card"><h2>Beat interval (ms) <span class="sub">- dashed line = 1000 ms target</span></h2><canvas id="c1"></canvas><div class="tip" id="t1"></div></div>
<div class="card"><h2>Beam blocked time (ms)</h2><canvas id="c2"></canvas><div class="tip" id="t2"></div></div>
<div class="sub">Raw data: <a href="/data">/data</a> &middot; <a href="/forget" onclick="return confirm('Forget saved Wi-Fi and reboot into setup mode?')">forget Wi-Fi</a></div>
<script>
const $=id=>document.getElementById(id);let D=null;
function css(v){return getComputedStyle(document.documentElement).getPropertyValue(v).trim()}
function fmt(x,d){return (x===null||x===undefined||isNaN(x))?'-':x.toFixed(d)}
function sgn(x){return isNaN(x)?'-':(x>0?'+':'')+x.toFixed(0)}
function draw(cv,arr,ref,tipEl,unit){
  const dpr=window.devicePixelRatio||1,W=cv.clientWidth,H=cv.clientHeight;cv.width=W*dpr;cv.height=H*dpr;
  const g=cv.getContext('2d');g.scale(dpr,dpr);g.clearRect(0,0,W,H);
  const L=52,R=10,T=8,B=22,pw=W-L-R,ph=H-T-B;
  if(!arr.length){g.fillStyle=css('--ink2');g.font='13px system-ui';g.fillText('waiting for beats...',L,T+20);cv._pts=[];return}
  let lo=Math.min(...arr),hi=Math.max(...arr);if(ref!=null){lo=Math.min(lo,ref);hi=Math.max(hi,ref)}
  let pad=(hi-lo)*0.1||Math.max(1,hi*0.02);lo-=pad;hi+=pad;
  const X=i=>L+(arr.length<2?pw/2:i*pw/(arr.length-1)),Y=v=>T+ph-(v-lo)/(hi-lo)*ph;
  g.font='11px system-ui';g.textAlign='right';g.textBaseline='middle';
  for(let k=0;k<=4;k++){const v=lo+(hi-lo)*k/4,y=Y(v);g.strokeStyle=css('--grid');g.lineWidth=1;g.beginPath();g.moveTo(L,y);g.lineTo(W-R,y);g.stroke();g.fillStyle=css('--ink2');g.fillText(v.toFixed(1),L-6,y)}
  g.textAlign='left';g.textBaseline='alphabetic';g.fillText('oldest',L,H-6);g.textAlign='right';g.fillText('newest',W-R,H-6);
  if(ref!=null){g.strokeStyle=css('--ref');g.setLineDash([4,4]);g.beginPath();g.moveTo(L,Y(ref));g.lineTo(W-R,Y(ref));g.stroke();g.setLineDash([])}
  g.strokeStyle=css('--s1');g.lineWidth=2;g.lineJoin='round';g.beginPath();arr.forEach((v,i)=>i?g.lineTo(X(i),Y(v)):g.moveTo(X(i),Y(v)));g.stroke();
  cv._pts=arr.map((v,i)=>({x:X(i),y:Y(v),v,i}));cv._T=T;cv._ph=ph;
  if(cv._hover!=null&&cv._pts[cv._hover]){const p=cv._pts[cv._hover];g.strokeStyle=css('--ink2');g.lineWidth=1;g.beginPath();g.moveTo(p.x,T);g.lineTo(p.x,T+ph);g.stroke();
    g.fillStyle=css('--card');g.beginPath();g.arc(p.x,p.y,6,0,7);g.fill();g.fillStyle=css('--s1');g.beginPath();g.arc(p.x,p.y,4,0,7);g.fill()}
}
function hook(cv,tip,unit,getArr,ref){
  cv.addEventListener('mousemove',e=>{const r=cv.getBoundingClientRect(),x=e.clientX-r.left;if(!cv._pts||!cv._pts.length)return;
    let best=0;cv._pts.forEach((p,i)=>{if(Math.abs(p.x-x)<Math.abs(cv._pts[best].x-x))best=i});cv._hover=best;const p=cv._pts[best];
    const n=D?D.n-(cv._pts.length-1-best):best+1;
    tip.innerHTML='<span class="dot" style="background:'+css('--s1')+'"></span>beat '+n+': <b>'+p.v.toFixed(2)+'</b> '+unit;tip.style.display='block';
    tip.style.left=Math.min(p.x+16,r.width-tip.offsetWidth-4)+'px';tip.style.top=(p.y+10)+'px';render()});
  cv.addEventListener('mouseleave',()=>{cv._hover=null;tip.style.display='none';render()});
}
function render(){if(!D)return;draw($('c1'),D.hi,1000,$('t1'),'ms');draw($('c2'),D.hb,null,$('t2'),'ms')}
hook($('c1'),$('t1'),'ms');hook($('c2'),$('t2'),'ms');window.addEventListener('resize',render);
async function tick(){
  try{const r=await fetch('/data');D=await r.json();
    $('n').textContent=D.n;$('last').textContent=fmt(D.last,1);$('avg').textContent=fmt(D.avg,2);
    $('rate').textContent=D.n?sgn(D.rate):'-';$('lrate').textContent=D.n?sgn(D.lrate):'-';
    $('be').textContent=fmt(D.be,1);$('blk').textContent=fmt(D.blk,1);
    const ago=D.ago<0?'no beats yet':('last beat '+(D.ago/1000).toFixed(1)+' s ago');
    $('status').textContent='Live - '+ago+' - uptime '+Math.floor(D.up/60000)+' min - beam '+(D.beam?'BLOCKED':'clear');
    render();
  }catch(e){$('status').textContent='connection lost, retrying...'}
}
tick();setInterval(tick,1000);
</script></body></html>)HTML";

void handleData() {
  processEdges();
  String j; j.reserve(6000);
  double lrate = beatCount ? -((totalMs / beatCount) / 1000.0 - TARGET_BEAT_S) / TARGET_BEAT_S * 86400.0 : 0;
  j += "{\"n\":";    j += beatCount;
  j += ",\"last\":"; j += String(lastIntervalMs, 2);
  j += ",\"avg\":";  j += String(avgMs, 3);
  j += ",\"rate\":"; j += String(rateSpd, 1);
  j += ",\"lrate\":";j += String(lrate, 1);
  j += ",\"be\":";   j += String(beatErrMs, 2);
  j += ",\"blk\":";  j += String(lastBlockedMs, 2);
  j += ",\"beam\":"; j += (digitalRead(IR_PIN) == BLOCKED_LEVEL) ? "true" : "false";
  j += ",\"ago\":";  j += beatCount ? (long)(millis() - lastBeatMillis) : -1;
  j += ",\"up\":";   j += millis();
  int start = (histHead - histN + HIST_N) % HIST_N;
  j += ",\"hi\":[";
  for (int k = 0; k < histN; k++) { if (k) j += ','; j += String(histI[(start + k) % HIST_N], 2); }
  j += "],\"hb\":[";
  for (int k = 0; k < histN; k++) { if (k) j += ','; j += String(histB[(start + k) % HIST_N], 1); }
  j += "]}";
  server.send(200, "application/json", j);
}

String htmlEscape(const String& s) {
  String o; for (char c : s) { if (c == '<') o += "&lt;"; else if (c == '>') o += "&gt;"; else if (c == '"') o += "&quot;"; else if (c == '&') o += "&amp;"; else o += c; } return o;
}

void handleSetup() {
  int n = WiFi.scanNetworks();
  String p = F("<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
               "<title>Pendulum Setup</title><style>body{font:16px system-ui;max-width:420px;margin:24px auto;padding:0 16px}"
               "input,select,button{width:100%;font-size:16px;padding:8px;margin:6px 0 14px}</style></head><body>"
               "<h2>Pendulum Timer - Wi-Fi setup</h2><form method='POST' action='/save'><label>Network</label><select name='s'>");
  for (int i = 0; i < n; i++) { String s = htmlEscape(WiFi.SSID(i)); p += "<option value=\"" + s + "\">" + s + " (" + WiFi.RSSI(i) + " dBm)</option>"; }
  p += F("</select><label>Password</label><input name='p' type='password'><button>Save and connect</button></form>"
         "<p>After saving, rejoin your home Wi-Fi and open <b>http://pendulum.local</b>.</p></body></html>");
  server.send(200, "text/html", p);
}

void handleSave() {
  prefs.putString("ssid", server.arg("s"));
  prefs.putString("pass", server.arg("p"));
  server.send(200, "text/html", "<p style='font:16px system-ui'>Saved. Rebooting and joining the network... then open <b>http://pendulum.local</b></p>");
  delay(1500);
  ESP.restart();
}

void handleForget() {
  prefs.remove("ssid"); prefs.remove("pass");
  server.send(200, "text/html", "<p style='font:16px system-ui'>Wi-Fi forgotten. Rebooting into setup mode (hotspot \"Pendulum-Setup\").</p>");
  delay(1500);
  ESP.restart();
}

bool connectWiFi() {
  String ssid = prefs.getString("ssid", "");
  if (ssid == "") return false;
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(ssid.c_str(), prefs.getString("pass", "").c_str());
  Serial.printf("Connecting to \"%s\"", ssid.c_str());
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) { delay(500); Serial.print('.'); }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

void setup() {
  Serial.begin(115200);
  pinMode(IR_PIN, INPUT_PULLUP);
  lastLvl = digitalRead(IR_PIN);
  BLOCKED_LEVEL = lastLvl ? LOW : HIGH;
  attachInterrupt(digitalPinToInterrupt(IR_PIN), onEdge, CHANGE);
  Serial.println();
  Serial.printf("Pendulum timer ready. GPIO%d idle level = %s, blocked = %s\n", IR_PIN,
                lastLvl ? "HIGH" : "LOW", BLOCKED_LEVEL ? "HIGH" : "LOW");

  prefs.begin("wifi", false);
  if (connectWiFi()) {
    MDNS.begin(HOSTNAME);
    MDNS.addService("http", "tcp", 80);
    Serial.printf("Stats page: http://%s.local  or  http://%s\n", HOSTNAME, WiFi.localIP().toString().c_str());
    server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  } else {
    apMode = true;
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID);
    Serial.printf("No Wi-Fi saved (or it failed). Join hotspot \"%s\" and open http://%s\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str());
    server.on("/", handleSetup);
    server.on("/stats", []() { server.send_P(200, "text/html", PAGE); });
  }
  server.on("/data", handleData);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/forget", handleForget);
  server.begin();
  Serial.println("beat\tinterval_ms\tblocked_ms\tavg_ms\trate_s/day\tbeat_err_ms");
}

void loop() {
  processEdges();
  server.handleClient();
  static uint32_t lastCheck = 0;           // re-join if the router drops us
  if (!apMode && millis() - lastCheck > 10000) {
    lastCheck = millis();
    if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
  }
}
