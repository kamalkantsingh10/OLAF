/**
 * Web-Based PID Tuner with Auto-Tune Implementation
 *
 * Auto-tune uses Ziegler-Nichols ultimate gain method:
 *   1. Set Ki=0, Kd=0
 *   2. Increase Kp from 5 in steps until sustained oscillation detected
 *   3. Record critical gain Ku and oscillation period Tu
 *   4. Compute PID: Kp=0.6*Ku, Ki=1.2*Ku/Tu, Kd=0.075*Ku*Tu
 */

#include "web_tuner.h"
#include "config.h"
#include <ESPAsyncWebServer.h>

WebTuner webTuner;

// Live-tunable gains
float tuneKp = kPitchKp;
float tuneKi = kPitchKi;
float tuneKd = kPitchKd;
float tuneTarget = kTargetPitchDeg;

static AsyncWebServer server(80);
static AsyncWebSocket wsSocket("/ws");

// ---- HTML dashboard ----
static const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OLAF PID Tuner</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:monospace;background:#f5f5f5;color:#222;padding:10px}
h1{color:#333;font-size:1.2em;margin-bottom:8px}
.row{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:8px}
.card{background:#fff;border:1px solid #ddd;border-radius:8px;padding:10px;flex:1;min-width:120px}
.card h3{font-size:0.8em;color:#888;margin-bottom:4px}
.val{font-size:1.6em;font-weight:bold;color:#007700}
.val.err{color:#cc0000}
canvas{width:100%;height:180px;background:#fff;border:1px solid #ddd;border-radius:8px;margin-bottom:8px}
.slider-row{display:flex;align-items:center;gap:8px;margin:6px 0}
.slider-row label{width:80px;font-size:0.9em;color:#444}
.slider-row input[type=range]{flex:1}
.slider-row .sv{width:70px;text-align:right;color:#0066cc;font-size:0.9em;font-weight:bold}
.btns{display:flex;gap:8px;margin:8px 0}
.btns button{flex:1;padding:12px;border:none;border-radius:8px;font-size:1em;
  font-weight:bold;cursor:pointer;font-family:monospace}
.btn-go{background:#22aa22;color:#fff}
.btn-stop{background:#cc2222;color:#fff}
.btn-tare{background:#2266bb;color:#fff}
.btn-auto{background:#9922cc;color:#fff}
.btn-go:active{background:#33cc33}
.btn-stop:active{background:#ee3333}
.btn-tare:active{background:#3388dd}
.btn-auto:active{background:#bb44ee}
#atStatus{background:#fff8e0;border:1px solid #eeb;border-radius:8px;padding:10px;margin:8px 0;
  font-size:0.9em;color:#885500;display:none}
.status{font-size:0.8em;color:#999;margin-top:4px}
</style>
</head>
<body>
<h1>OLAF Base — PID Tuner</h1>

<div class="row">
  <div class="card"><h3>PITCH</h3><div class="val" id="vPitch">--</div></div>
  <div class="card"><h3>PID OUT</h3><div class="val" id="vPid">--</div></div>
  <div class="card"><h3>L(A)</h3><div class="val" id="vM0">--</div></div>
  <div class="card"><h3>R(A)</h3><div class="val" id="vM1">--</div></div>
</div>

<div class="row">
  <div class="card"><h3>GYRO X</h3><div class="val" id="vGx" style="font-size:1em;color:#666">--</div></div>
  <div class="card"><h3>GYRO Y</h3><div class="val" id="vGy" style="font-size:1em;color:#666">--</div></div>
  <div class="card"><h3>GYRO Z</h3><div class="val" id="vGz" style="font-size:1em;color:#666">--</div></div>
  <div class="card"><h3>ROLL</h3><div class="val" id="vRoll" style="font-size:1em;color:#666">--</div></div>
</div>

<canvas id="chart"></canvas>

<div class="btns">
  <button class="btn-go" onclick="send('CMD:BALANCE')">BALANCE</button>
  <button class="btn-stop" onclick="send('CMD:STOP')">STOP</button>
  <button class="btn-tare" onclick="send('CMD:TARE')">TARE</button>
  <button class="btn-auto" onclick="send('CMD:AUTOTUNE')">AUTO-TUNE</button>
  <button class="btn-tare" onclick="send('CMD:SAVE')">SAVE</button>
</div>
<div class="btns">
  <button class="btn-go" id="btnLog" onclick="toggleLog()">REC LOG</button>
  <button class="btn-tare" onclick="copyLog()">COPY LOG</button>
  <button class="btn-stop" onclick="clearLog()">CLEAR</button>
</div>

<div id="atStatus"></div>
<div id="logStatus" style="font-size:0.8em;color:#885500;margin:4px 0"></div>
<textarea id="logArea" rows="10" style="width:100%;font-family:monospace;font-size:0.7em;
  background:#fff;border:1px solid #ddd;border-radius:8px;padding:8px;display:none"></textarea>

<div class="card">
  <h3>PID GAINS</h3>
  <div class="slider-row">
    <label>Kp</label>
    <input type="range" id="sKp" min="0" max="3" step="0.01" value="0.5">
    <span class="sv" id="svKp">0.500</span>
  </div>
  <div class="slider-row">
    <label>Ki</label>
    <input type="range" id="sKi" min="0" max="0.5" step="0.001" value="0">
    <span class="sv" id="svKi">0.000</span>
  </div>
  <div class="slider-row">
    <label>Kd</label>
    <input type="range" id="sKd" min="0" max="0.5" step="0.001" value="0.05">
    <span class="sv" id="svKd">0.050</span>
  </div>
  <div class="slider-row">
    <label>Target</label>
    <input type="range" id="sTarget" min="-5" max="5" step="0.1" value="0">
    <span class="sv" id="svTarget">0.0</span>
  </div>
</div>

<div class="status" id="wsStatus">Connecting...</div>

<script>
const N=200;
let pitch=[],pidOut=[],m0=[],m1=[];
for(let i=0;i<N;i++){pitch.push(0);pidOut.push(0);m0.push(0);m1.push(0)}

const canvas=document.getElementById('chart');
const ctx=canvas.getContext('2d');

function resize(){canvas.width=canvas.clientWidth;canvas.height=canvas.clientHeight}
resize();window.onresize=resize;

function drawChart(){
  const w=canvas.width,h=canvas.height;
  ctx.clearRect(0,0,w,h);

  ctx.strokeStyle='#ddd';ctx.lineWidth=1;
  const mid=h/2;
  ctx.beginPath();ctx.moveTo(0,mid);ctx.lineTo(w,mid);ctx.stroke();
  for(let y of[h*0.25,h*0.75]){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke()}

  function line(arr,color,scale){
    ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();
    for(let i=0;i<N;i++){
      const x=i/(N-1)*w;
      const y=mid-arr[i]/scale*mid;
      i===0?ctx.moveTo(x,y):ctx.lineTo(x,y);
    }
    ctx.stroke();
  }
  line(pitch,'#008800',30);
  line(pidOut,'#cc6600',10);
  line(m0,'#0066cc',10);
  line(m1,'#aa00aa',10);

  ctx.font='11px monospace';
  const labels=[['pitch','#008800'],['pid','#cc6600'],['L(A)','#0066cc'],['R(A)','#aa00aa']];
  let lx=8;
  for(const[t,c]of labels){
    ctx.fillStyle=c;ctx.fillText(t,lx,14);lx+=ctx.measureText(t).width+12;
  }

  ctx.fillStyle='#aaa';ctx.textAlign='right';
  ctx.fillText('+30',w-4,14);ctx.fillText('-30',w-4,h-4);ctx.fillText('0',w-4,mid-2);
  ctx.textAlign='left';
}

let ws;
function connect(){
  ws=new WebSocket('ws://'+location.hostname+'/ws');
  ws.onopen=()=>{document.getElementById('wsStatus').textContent='Connected';send('CMD:GETGAINS')};
  ws.onclose=()=>{document.getElementById('wsStatus').textContent='Disconnected — reconnecting...';setTimeout(connect,2000)};
  ws.onerror=()=>{ws.close()};
  ws.onmessage=(e)=>{
    const d=JSON.parse(e.data);
    // Initial gains sync
    if(d.gains){
      document.getElementById('sKp').value=d.kp;
      document.getElementById('svKp').textContent=d.kp.toFixed(1);
      document.getElementById('sKi').value=d.ki;
      document.getElementById('svKi').textContent=d.ki.toFixed(2);
      document.getElementById('sKd').value=d.kd;
      document.getElementById('svKd').textContent=d.kd.toFixed(1);
      document.getElementById('sTarget').value=d.tgt;
      document.getElementById('svTarget').textContent=d.tgt.toFixed(1);
      return;
    }
    // Auto-tune status messages
    if(d.at){
      const el=document.getElementById('atStatus');
      el.style.display='block';
      el.textContent=d.at;
      // Update sliders if gains changed
      if(d.kp!==undefined){
        document.getElementById('sKp').value=d.kp;
        document.getElementById('svKp').textContent=d.kp.toFixed(1);
        document.getElementById('sKi').value=d.ki;
        document.getElementById('svKi').textContent=d.ki.toFixed(2);
        document.getElementById('sKd').value=d.kd;
        document.getElementById('svKd').textContent=d.kd.toFixed(1);
      }
      return;
    }
    pitch.push(d.p);pitch.shift();
    pidOut.push(d.o);pidOut.shift();
    m0.push(d.m0);m0.shift();
    m1.push(d.m1);m1.shift();
    document.getElementById('vPitch').textContent=d.p.toFixed(1)+'\u00B0';
    document.getElementById('vPid').textContent=d.o.toFixed(2);
    document.getElementById('vM0').textContent=d.m0.toFixed(2);
    document.getElementById('vM1').textContent=d.m1.toFixed(2);
    if(d.gx!==undefined){
      document.getElementById('vGx').textContent=d.gx.toFixed(3);
      document.getElementById('vGy').textContent=d.gy.toFixed(3);
      document.getElementById('vGz').textContent=d.gz.toFixed(3);
      document.getElementById('vRoll').textContent=d.r.toFixed(1)+'\u00B0';
    }
    if(logging && (d.o!==0 || d.m0!==0)){
      logLines.push(Date.now()+','+d.p.toFixed(3)+','+d.o.toFixed(4)+','+d.m0.toFixed(4)+','+d.m1.toFixed(4)+','+(d.gx||0).toFixed(4)+','+(d.gy||0).toFixed(4)+','+(d.gz||0).toFixed(4));
    }
    const el=document.getElementById('vPitch');
    el.className=Math.abs(d.p)>20?'val err':'val';
    drawChart();
  };
}
connect();

function send(msg){if(ws&&ws.readyState===1)ws.send(msg)}

let logging=false;
let logLines=[];
function toggleLog(){
  logging=!logging;
  const b=document.getElementById('btnLog');
  const ta=document.getElementById('logArea');
  if(logging){
    logLines=['# Kp='+document.getElementById('svKp').textContent+' Ki='+document.getElementById('svKi').textContent+' Kd='+document.getElementById('svKd').textContent+' Target='+document.getElementById('svTarget').textContent];
    logLines.push('time_ms,pitch,pid_out,left_A,right_A,gyro_x,gyro_y,gyro_z');
    b.textContent='STOP LOG';b.className='btn-stop';
    ta.style.display='block';
    document.getElementById('logStatus').textContent='Recording...';
  }else{
    b.textContent='REC LOG';b.className='btn-go';
    ta.value=logLines.join('\n');
    document.getElementById('logStatus').textContent=logLines.length-2+' samples captured';
  }
}
function copyLog(){
  const ta=document.getElementById('logArea');
  ta.select();navigator.clipboard.writeText(ta.value);
  document.getElementById('logStatus').textContent='Copied to clipboard!';
}
function clearLog(){
  logLines=[];
  document.getElementById('logArea').value='';document.getElementById('logArea').style.display='none';
  document.getElementById('logStatus').textContent='';
}

function bindSlider(id,param,fmt){
  const s=document.getElementById(id);
  const v=document.getElementById('sv'+id.slice(1));
  s.oninput=()=>{v.textContent=parseFloat(s.value).toFixed(fmt);send('SET:'+param+'='+s.value)};
}
bindSlider('sKp','Kp',2);
bindSlider('sKi','Ki',3);
bindSlider('sKd','Kd',3);
bindSlider('sTarget','Target',1);
</script>
</body>
</html>
)rawliteral";

// ---- WebSocket event handler ----
static void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                       AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->opcode == WS_TEXT) {
            char msg[64];
            size_t n = len < sizeof(msg) - 1 ? len : sizeof(msg) - 1;
            memcpy(msg, data, n);
            msg[n] = '\0';

            if (strncmp(msg, "CMD:", 4) == 0) {
                const char* cmd = msg + 4;
                if (strcmp(cmd, "BALANCE") == 0) {
                    webTuner.pendingCmd_ = WEB_CMD_BALANCE;
                } else if (strcmp(cmd, "STOP") == 0) {
                    webTuner.pendingCmd_ = WEB_CMD_STOP;
                } else if (strcmp(cmd, "TARE") == 0) {
                    webTuner.pendingCmd_ = WEB_CMD_TARE;
                } else if (strcmp(cmd, "AUTOTUNE") == 0) {
                    webTuner.pendingCmd_ = WEB_CMD_AUTOTUNE;
                } else if (strcmp(cmd, "SAVE") == 0) {
                    webTuner.saveGains();
                    char json[64];
                    snprintf(json, sizeof(json), "{\"at\":\"PID gains saved to flash\"}");
                    client->text(json);
                } else if (strcmp(cmd, "LOGSTART") == 0) {
                    webTuner.startLog();
                } else if (strcmp(cmd, "LOGSTOP") == 0) {
                    webTuner.stopLog();
                } else if (strcmp(cmd, "GETGAINS") == 0) {
                    char json[128];
                    snprintf(json, sizeof(json),
                             "{\"gains\":1,\"kp\":%.1f,\"ki\":%.2f,\"kd\":%.1f,\"tgt\":%.1f}",
                             tuneKp, tuneKi, tuneKd, tuneTarget);
                    client->text(json);
                }
                Serial.printf("[Web] CMD: %s\n", cmd);
            }
            else if (strncmp(msg, "SET:", 4) == 0) {
                char* eq = strchr(msg + 4, '=');
                if (eq) {
                    *eq = '\0';
                    const char* param = msg + 4;
                    float val = atof(eq + 1);

                    if (strcmp(param, "Kp") == 0) { tuneKp = val; }
                    else if (strcmp(param, "Ki") == 0) { tuneKi = val; }
                    else if (strcmp(param, "Kd") == 0) { tuneKd = val; }
                    else if (strcmp(param, "Target") == 0) { tuneTarget = val; }

                    Serial.printf("[Web] %s = %.2f\n", param, val);
                }
            }
        }
    } else if (type == WS_EVT_CONNECT) {
        Serial.printf("[Web] Client #%u connected\n", client->id());
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[Web] Client #%u disconnected\n", client->id());
    }
}

// ---- Auto-tune implementation ----

void WebTuner::autoTuneStart() {
    atState_ = AT_FINDING_KU;
    atKp_ = 0.01f;           // Start very gentle (current mode)
    atKpStep_ = 0.005f;     // Tiny increments
    atStepStartMs_ = millis();
    atStepDurationMs_ = 4000; // 4 seconds per step to settle
    atPeakCount_ = 0;
    atPrevPitch_ = 0.0f;
    atPrevDelta_ = 0.0f;
    atRising_ = true;
    atKu_ = 0.0f;
    atTu_ = 0.0f;

    // Keep a small Kd for damping during auto-tune — pure P-only is too violent
    // We're looking for the Kp where oscillation starts DESPITE the damping
    tuneKi = 0.0f;
    tuneKd = 0.002f;  // Light damping to prevent violent jerks
    tuneKp = atKp_;

    char msg[96];
    snprintf(msg, sizeof(msg), "Auto-tune started — Kp=%.1f, Ki=0, Kd=2 (damped)", atKp_);
    sendAutoTuneStatus(msg);
    Serial.printf("[AutoTune] Starting at Kp=%.1f (Kd=2 damping)\n", atKp_);
}

bool WebTuner::autoTuneStep(float pitch) {
    if (atState_ == AT_IDLE) return false;

    uint32_t now = millis();

    // Detect peaks (zero-crossing of pitch derivative)
    float delta = pitch - atPrevPitch_;
    bool wasRising = atRising_;
    atRising_ = (delta >= 0.0f);

    // Peak detected: was rising, now falling
    if (wasRising && !atRising_ && fabsf(pitch) > 0.3f) {
        if (atPeakCount_ < 20) {
            atPeaks_[atPeakCount_] = pitch;
            atPeakTimes_[atPeakCount_] = now;
            atPeakCount_++;
        }
    }
    // Trough detected: was falling, now rising
    if (!wasRising && atRising_ && fabsf(pitch) > 0.3f) {
        if (atPeakCount_ < 20) {
            atPeaks_[atPeakCount_] = pitch;
            atPeakTimes_[atPeakCount_] = now;
            atPeakCount_++;
        }
    }

    atPrevPitch_ = pitch;
    atPrevDelta_ = delta;

    if (atState_ == AT_FINDING_KU) {
        // Check if we have sustained oscillation at this Kp
        if (now - atStepStartMs_ >= atStepDurationMs_) {
            if (detectSustainedOscillation()) {
                // Found critical gain — move to measuring
                atState_ = AT_MEASURING;
                atKu_ = atKp_;

                // Calculate period from last few peaks
                // Use last 4+ peaks for period measurement
                if (atPeakCount_ >= 4) {
                    uint32_t totalPeriod = 0;
                    uint8_t periodCount = 0;
                    // Period = time between every other peak (peak-to-peak, not peak-to-trough)
                    for (uint8_t i = 2; i < atPeakCount_; i += 2) {
                        totalPeriod += atPeakTimes_[i] - atPeakTimes_[i - 2];
                        periodCount++;
                    }
                    if (periodCount > 0) {
                        atTu_ = (float)totalPeriod / (float)periodCount / 1000.0f;
                    }
                }

                if (atTu_ < 0.05f) {
                    // Period too short — probably noise, keep going
                    atState_ = AT_FINDING_KU;
                } else {
                    autoTuneApplyResults();
                    return false;
                }
            }

            // Not oscillating yet — increase Kp
            atKp_ += atKpStep_;
            if (atKp_ > 0.5f) {
                // Hit max — use current Kp as Ku estimate
                atKu_ = atKp_ * 0.8f;
                atTu_ = 0.5f;  // Assume 0.5s period as fallback
                autoTuneApplyResults();
                return false;
            }

            tuneKp = atKp_;
            atPeakCount_ = 0;
            atStepStartMs_ = now;

            char msg[96];
            snprintf(msg, sizeof(msg), "Testing Kp=%.1f — looking for oscillation...", atKp_);
            sendAutoTuneStatus(msg);
            Serial.printf("[AutoTune] Trying Kp=%.1f\n", atKp_);
        }
    }

    return true;  // Still tuning
}

bool WebTuner::detectSustainedOscillation() {
    // Need at least 6 peaks/troughs (3 full oscillation cycles)
    if (atPeakCount_ < 6) return false;

    // Check last 6 peaks: amplitude should be consistent (not growing/shrinking much)
    float minAmp = 999.0f, maxAmp = 0.0f;
    for (uint8_t i = atPeakCount_ - 6; i < atPeakCount_; i++) {
        float amp = fabsf(atPeaks_[i]);
        if (amp < minAmp) minAmp = amp;
        if (amp > maxAmp) maxAmp = amp;
    }

    // Sustained oscillation: amplitude between 1° and 15°, ratio < 2:1
    bool ampOk = (minAmp > 0.5f && maxAmp < 15.0f);
    bool consistent = (maxAmp < 2.5f * minAmp);

    if (ampOk && consistent) {
        Serial.printf("[AutoTune] Oscillation detected! amp=%.1f-%.1f deg\n", minAmp, maxAmp);
        return true;
    }

    return false;
}

void WebTuner::autoTuneApplyResults() {
    // Ziegler-Nichols PID formulas
    float newKp = 0.6f * atKu_;
    float newKi = 1.2f * atKu_ / atTu_;
    float newKd = 0.075f * atKu_ * atTu_;

    // Clamp to reasonable ranges
    newKp = constrain(newKp, 1.0f, 60.0f);
    newKi = constrain(newKi, 0.0f, 5.0f);
    newKd = constrain(newKd, 0.0f, 15.0f);

    tuneKp = newKp;
    tuneKi = newKi;
    tuneKd = newKd;

    atState_ = AT_IDLE;

    // Save to flash so they persist across reboots
    saveGains();

    char msg[128];
    snprintf(msg, sizeof(msg),
             "Auto-tune DONE! Ku=%.1f Tu=%.2fs -> Kp=%.1f Ki=%.2f Kd=%.1f (saved)",
             atKu_, atTu_, newKp, newKi, newKd);
    sendAutoTuneStatus(msg);
    Serial.printf("[AutoTune] Complete: Ku=%.1f Tu=%.3fs\n", atKu_, atTu_);
    Serial.printf("[AutoTune] Applied & saved: Kp=%.1f Ki=%.2f Kd=%.1f\n", newKp, newKi, newKd);

    // Send gains to web UI so sliders update
    char json[128];
    snprintf(json, sizeof(json),
             "{\"at\":\"%s\",\"kp\":%.1f,\"ki\":%.2f,\"kd\":%.1f}",
             msg, newKp, newKi, newKd);
    wsSocket.textAll(json);
}

void WebTuner::sendAutoTuneStatus(const char* msg) {
    char json[160];
    snprintf(json, sizeof(json), "{\"at\":\"%s\"}", msg);
    wsSocket.textAll(json);
}

// ---- Public API ----

void WebTuner::loadGains() {
    prefs_.begin("pid", true);  // Read-only
    if (prefs_.isKey("kp")) {
        tuneKp = prefs_.getFloat("kp", kPitchKp);
        tuneKi = prefs_.getFloat("ki", kPitchKi);
        tuneKd = prefs_.getFloat("kd", kPitchKd);
        tuneTarget = prefs_.getFloat("target", kTargetPitchDeg);
        Serial.printf("[Web] Loaded saved PID: Kp=%.1f Ki=%.2f Kd=%.1f Target=%.1f\n",
                      tuneKp, tuneKi, tuneKd, tuneTarget);
    } else {
        Serial.println("[Web] No saved PID — using defaults");
    }
    prefs_.end();
}

void WebTuner::saveGains() {
    prefs_.begin("pid", false);  // Read-write
    prefs_.putFloat("kp", tuneKp);
    prefs_.putFloat("ki", tuneKi);
    prefs_.putFloat("kd", tuneKd);
    prefs_.putFloat("target", tuneTarget);
    prefs_.end();
    Serial.printf("[Web] Saved PID: Kp=%.1f Ki=%.2f Kd=%.1f Target=%.1f\n",
                  tuneKp, tuneKi, tuneKd, tuneTarget);
}

// ---- Data logging ----

void WebTuner::startLog() {
    if (!logBuf_) {
        // Allocate in PSRAM if available, else heap
        logBuf_ = (LogRow*)ps_malloc(kLogMaxRows * sizeof(LogRow));
        if (!logBuf_) {
            logBuf_ = (LogRow*)malloc(kLogMaxRows * sizeof(LogRow));
        }
        if (!logBuf_) {
            Serial.println("[Log] Failed to allocate buffer!");
            char json[64];
            snprintf(json, sizeof(json), "{\"log\":\"ERROR: no memory for log\"}");
            wsSocket.textAll(json);
            return;
        }
        Serial.printf("[Log] Allocated %d rows (%d bytes)\n",
                      kLogMaxRows, kLogMaxRows * (int)sizeof(LogRow));
    }
    logCount_ = 0;
    logStartMs_ = millis();
    logging_ = true;
    Serial.println("[Log] Recording started");

    char json[64];
    snprintf(json, sizeof(json), "{\"log\":\"Recording... (max %ds)\"}", kLogMaxRows / 200);
    wsSocket.textAll(json);
}

void WebTuner::stopLog() {
    logging_ = false;
    Serial.printf("[Log] Stopped — %d samples (%.1fs)\n", logCount_, logCount_ / 200.0f);

    char json[96];
    snprintf(json, sizeof(json), "{\"log\":\"Stopped — %d samples (%.1fs). Hit DOWNLOAD.\"}",
             logCount_, logCount_ / 200.0f);
    wsSocket.textAll(json);
}

void WebTuner::logSample(uint32_t time_ms, float pitch, float pid_out,
                          float m0, float m1, float gx, float gy, float gz) {
    if (!logging_ || !logBuf_) return;

    if (logCount_ >= kLogMaxRows) {
        stopLog();
        return;
    }

    LogRow& row = logBuf_[logCount_++];
    row.time_ms = time_ms;
    row.pitch = pitch;
    row.pid_out = pid_out;
    row.m0 = m0;
    row.m1 = m1;
    row.gx = gx;
    row.gy = gy;
    row.gz = gz;
}

void WebTuner::begin() {
    // Load saved PID gains from flash
    loadGains();

    wsSocket.onEvent(onWsEvent);
    server.addHandler(&wsSocket);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send_P(200, "text/html", PAGE);
    });

    // CSV download endpoint — streams log buffer as CSV
    server.on("/log.csv", HTTP_GET, [this](AsyncWebServerRequest* request) {
        if (!logBuf_ || logCount_ == 0) {
            request->send(200, "text/plain", "No data. Press REC LOG first.\n");
            return;
        }

        uint16_t count = logCount_;  // Snapshot
        LogRow* buf = logBuf_;

        // Stream CSV using chunked response
        AsyncResponseStream* response = request->beginResponseStream("text/csv");
        response->addHeader("Content-Disposition", "attachment; filename=olaf_pid_log.csv");

        // Header with metadata
        response->printf("# OLAF PID Log — Kp=%.4f Ki=%.4f Kd=%.4f Target=%.1f\n",
                         tuneKp, tuneKi, tuneKd, tuneTarget);
        response->println("time_ms,pitch_deg,pid_out_amps,left_amps,right_amps,gyro_x,gyro_y,gyro_z");

        for (uint16_t i = 0; i < count; i++) {
            response->printf("%lu,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                             buf[i].time_ms, buf[i].pitch, buf[i].pid_out,
                             buf[i].m0, buf[i].m1,
                             buf[i].gx, buf[i].gy, buf[i].gz);
        }

        request->send(response);
        Serial.printf("[Log] Served %d rows as CSV\n", count);
    });

    server.begin();
    Serial.println("[Web] PID tuner at http://olaf-base.local");
}

void WebTuner::update() {
    wsSocket.cleanupClients();
}

void WebTuner::sendTelemetry(float pitch, float roll, float pid_out,
                              float m0_vel, float m1_vel, uint8_t accuracy,
                              float gyro_x, float gyro_y, float gyro_z) {
    if (wsSocket.count() == 0) return;

    char buf[200];
    snprintf(buf, sizeof(buf),
             "{\"p\":%.2f,\"r\":%.2f,\"o\":%.3f,\"m0\":%.3f,\"m1\":%.3f,\"a\":%d,"
             "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f}",
             pitch, roll, pid_out, m0_vel, m1_vel, accuracy,
             gyro_x, gyro_y, gyro_z);
    wsSocket.textAll(buf);
}

WebCommand WebTuner::getCommand() {
    WebCommand cmd = pendingCmd_;
    pendingCmd_ = WEB_CMD_NONE;
    return cmd;
}
