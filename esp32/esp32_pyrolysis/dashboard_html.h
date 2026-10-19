#pragma once

// Dashboard HTML (PROGMEM) - dipindah ke header terpisah supaya auto-prototype-generator
// Arduino (ctags, hanya memindai file .ino utama) tidak salah membaca function JS di
// dalam raw string ini sebagai prototype C++ (bug lama Arduino IDE/arduino-cli).

const char HTML[] PROGMEM = R"HTMLDIAG(
<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>Pyrolysis Diagnostic</title>
<style>
  :root {
    --bg: #1a1a2e;
    --card: #16213e;
    --border: #2a3358;
    --text: #eaeaea;
    --text-dim: #8f9bb3;
    --teal: #4ecca3;
    --red: #e94560;
    --yellow: #ffd700;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; background: var(--bg); color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    max-width: 460px; margin: 0 auto; padding: 12px 14px 30px 14px;
  }
  h1 { font-size: 17px; text-align: center; margin: 10px 0 4px 0; }
  #connBar {
    display: flex; align-items: center; justify-content: center; gap: 8px;
    font-size: 12px; padding: 6px; border-radius: 8px; margin-bottom: 14px;
    background: var(--card); border: 1px solid var(--border);
  }
  .dot { width: 9px; height: 9px; border-radius: 50%; background: var(--red); }
  .dot.on { background: var(--teal); }

  .card {
    background: var(--card); border: 1px solid var(--border);
    border-radius: 10px; padding: 14px; margin-bottom: 12px;
  }
  .card h3 { margin: 0 0 10px 0; font-size: 13px; text-transform: uppercase; letter-spacing: 0.5px; color: var(--text-dim); }

  button {
    border: none; border-radius: 7px; padding: 10px 14px; font-size: 13px; font-weight: 700;
    cursor: pointer; color: #fff; background: #33406e;
  }
  button.primary { background: var(--teal); color: #06251c; }
  button.danger { background: var(--red); }
  button.warn { background: var(--yellow); color: #3a2e00; }
  .btn-row { display: flex; gap: 8px; flex-wrap: wrap; margin-top: 8px; }
  .btn-row button { flex: 1; min-width: 60px; }

  input[type=range] { width: 100%; accent-color: var(--teal); margin: 8px 0; }

  .result {
    margin-top: 10px; padding: 10px; border-radius: 6px; font-size: 13px;
    background: #0d1229; border: 1px solid var(--border); min-height: 20px;
    white-space: pre-wrap; word-break: break-word;
  }
  .result.ok { border-color: var(--teal); color: var(--teal); }
  .result.err { border-color: var(--red); color: var(--red); }
  .result.info { color: var(--text); }

  .hint { font-size: 11px; color: var(--text-dim); margin-top: 6px; line-height: 1.4; }
  .status-line { font-size: 13px; margin-top: 6px; }
  .status-line b { color: var(--teal); }
  .status-line b.off { color: var(--text-dim); }

  /* --- Step & Relay Test (card 7 & 8) --- */
  .at-row { display: flex; gap: 8px; }
  .at-row label { flex: 1; font-size: 11px; color: var(--text-dim); display: flex; flex-direction: column; gap: 4px; }
  .at-row input {
    width: 100%; padding: 8px; border-radius: 6px; border: 1px solid var(--border);
    background: #0d1229; color: var(--text); font-size: 14px;
  }
  .badge { display: inline-block; padding: 2px 10px; border-radius: 10px; font-size: 12px; font-weight: 700; }
  .badge.idle { background: #33406e; color: var(--text); }
  .badge.running { background: var(--teal); color: #06251c; animation: atBlink 1s infinite; }
  .badge.done { background: var(--yellow); color: #3a2e00; }
  .badge.error { background: var(--red); color: #fff; }
  @keyframes atBlink { 50% { opacity: 0.35; } }
  .progress { height: 8px; background: #0d1229; border: 1px solid var(--border); border-radius: 5px; margin: 8px 0 4px 0; overflow: hidden; }
  .progress-bar { height: 100%; width: 0%; background: var(--teal); transition: width 0.4s; }
  .at-result {
    margin-top: 10px; padding: 10px; border-radius: 6px; font-size: 13px; line-height: 1.6;
    background: #2b2600; border: 1px solid var(--yellow); color: var(--yellow);
    font-family: monospace; white-space: pre-wrap;
  }
</style>
</head>
<body>

<h1>PYROLYSIS DIAGNOSTIC</h1>
<div id="connBar"><span class="dot" id="connDot"></span><span id="connText">Menghubungkan...</span></div>

<!-- 1. MAX6675 -->
<div class="card">
  <h3>1. MAX6675 Thermocouple</h3>
  <button class="primary" onclick="testTemp()">Baca Suhu</button>
  <div class="result info" id="tempResult">-- belum dites --</div>
  <div class="hint">Suhu auto-refresh setiap 5 detik.</div>
</div>

<!-- 2. Tipping Bucket -->
<div class="card">
  <h3>2. Tipping Bucket (GPIO34)</h3>
  <button class="danger" onclick="resetTip()" style="width:100%;">Reset Counter</button>
  <div class="result info" id="tipResult">Tips: -- | Volume: -- mL</div>
  <div class="hint">Terbaca otomatis setiap 5 detik.</div>
</div>

<!-- 3. Fan -->
<div class="card">
  <h3>3. Fan DC (GPIO25)</h3>
  <input type="range" id="fanSlider" min="0" max="100" step="10" value="0" oninput="onFanSlide(this.value)">
  <div class="btn-row">
    <button onclick="setFan(0)">OFF</button>
    <button onclick="setFan(30)">30%</button>
    <button onclick="setFan(50)">50%</button>
    <button class="warn" onclick="setFan(100)">100%</button>
  </div>
  <div class="result info" id="fanResult">PWM raw: 0 / 255</div>
</div>

<!-- 4. Oil Pump -->
<div class="card">
  <h3>4. Oil Pump (GPIO27)</h3>
  <div class="btn-row">
    <button class="primary" onclick="oilCmd('on')">ON</button>
    <button class="danger" onclick="oilCmd('off')">OFF</button>
  </div>
  <div class="status-line">Status: <b class="off" id="oilStatus">OFF</b></div>
  <div class="hint">Jika relay tidak bunyi "klik": cek tegangan 12V ke coil relay, kontinuitas PC817 (LED sisi ESP32, transistor sisi relay), BD139 terpasang benar (E-B-C), dan ground ESP32 nyambung ke ground rangkaian relay.</div>
</div>

<!-- 5. Water Pump -->
<div class="card">
  <h3>5. Water Pump (GPIO26)</h3>
  <div class="btn-row">
    <button class="primary" onclick="waterCmd('on')">ON</button>
    <button class="danger" onclick="waterCmd('off')">OFF</button>
  </div>
  <div class="status-line">Status: <b class="off" id="waterStatus">OFF</b></div>
  <div class="hint">Sama seperti Oil Pump: cek jalur PC817+BD139, tegangan coil relay 12V, dan pastikan common ground. Kalau relay bunyi tapi pompa tidak jalan, cek kabel motor/pompa dan sumber 12V/220V ke pompa.</div>
</div>

<!-- 6. Test All -->
<div class="card">
  <h3>6. Test Semua Sekaligus</h3>
  <button class="warn" onclick="testAll()" style="width:100%;">JALANKAN SEMUA TEST</button>
  <div class="hint">Urutan: Fan 50% (1s) &rarr; OFF, Oil ON (1s) &rarr; OFF, Water ON (1s) &rarr; OFF, baca suhu. Proses &plusmn;3 detik, tombol lain akan menunggu selesai.</div>
  <div class="result info" id="allResult">-- belum dites --</div>
</div>

<!-- 7. Step Test -->
<div class="card">
  <h3>7. Step Test (Dead Time &amp; Time Constant)</h3>
  <div class="at-row">
    <label>Fan STEP (%)<input type="number" id="stPct" value="60" min="10" max="100" step="1"></label>
    <label>Batas suhu (C)<input type="number" id="stLimit" value="500" min="50" max="800" step="10"></label>
    <label>Maks durasi (mnt)<input type="number" id="stMin" value="30" min="5" max="60" step="1"></label>
  </div>
  <div class="btn-row">
    <button class="primary" id="btnStStart" onclick="stStart()">MULAI STEP TEST</button>
    <button class="danger" id="btnStStop" onclick="stStop()" style="display:none;">STOP</button>
  </div>
  <div class="status-line">Status: <span class="badge idle" id="stBadge">IDLE</span> <span id="stPhase"></span></div>
  <div class="progress"><div class="progress-bar" id="stBar"></div></div>
  <div class="status-line" id="stProgText">0 s</div>
  <div class="status-line">Suhu: <b id="stTemp">--</b> C &nbsp;|&nbsp; T0: <b id="stT0">--</b> C &nbsp;|&nbsp; Fan: <b id="stFan">--</b></div>
  <canvas id="stChart" width="400" height="150" style="width:100%;height:150px;background:#0d1229;border:1px solid var(--border);border-radius:6px;margin-top:8px;"></canvas>
  <div class="at-result" id="stResult" style="display:none;"></div>
  <div class="btn-row" id="stCopyRow" style="display:none;">
    <button class="warn" id="btnStCopy" onclick="stCopy()">SALIN HASIL</button>
  </div>
  <div class="hint">Fan dijaga 0% selama 30 detik (baseline), lalu dinaikkan ke nilai STEP dan suhu direkam tiap detik sampai steady-state (biasanya 10-20 menit; maksimum sesuai durasi di atas). Mulai dari tungku dingin/stabil supaya baseline valid. Metode dua titik (28,3% &amp; 63,2%): T = 1,5(t63 &minus; t28), L = t63 &minus; T, K = &Delta;T / STEP. Fan otomatis OFF setelah selesai. Tidak bisa berjalan bersamaan dengan Relay Test.</div>
</div>

<!-- 8. Relay Test -->
<div class="card">
  <h3>8. Relay Test (Ziegler-Nichols)</h3>
  <div class="at-row">
    <label>Setpoint (C)<input type="number" id="atSp" value="200" min="50" max="800" step="1"></label>
    <label>Fan ON (%)<input type="number" id="atHigh" value="80" min="10" max="100" step="1"></label>
    <label>Fan OFF (%)<input type="number" id="atLow" value="0" min="0" max="50" step="1"></label>
  </div>
  <div class="btn-row">
    <button class="primary" id="btnAtStart" onclick="atStart()">MULAI AUTOTUNE</button>
    <button class="danger" id="btnAtStop" onclick="atStop()" style="display:none;">STOP</button>
  </div>
  <div class="status-line">Status: <span class="badge idle" id="atBadge">IDLE</span></div>
  <div class="progress"><div class="progress-bar" id="atBar"></div></div>
  <div class="status-line" id="atProgText">0 / 3 siklus</div>
  <div class="status-line">Suhu: <b id="atTemp">--</b> C &nbsp;|&nbsp; Relay: <b class="off" id="atRelay">Fan: --</b></div>
  <div class="status-line">Puncak: <span id="atPeaks">--</span></div>
  <div class="at-result" id="atResult" style="display:none;"></div>
  <div class="btn-row" id="atCopyRow" style="display:none;">
    <button class="warn" id="btnAtCopy" onclick="atCopy()">SALIN HASIL</button>
    <button class="primary" id="btnAtApply" onclick="pidApply()">TERAPKAN KE PID</button>
  </div>
  <div class="status-line">PID aktif: <b id="pidActive">--</b></div>
  <div class="btn-row">
    <button class="danger" id="btnPidReset" onclick="pidReset()" style="width:100%;">RESET PID KE DEFAULT (Kp=2.0 Ki=0.1 Kd=10.0)</button>
  </div>
  <div class="hint">Autotune berjalan ~3-10 menit tergantung thermal mass sistem. Jangan ubah setpoint atau matikan power saat proses berjalan. Hasil Kp/Ki/Kd ini adalah nilai AWAL untuk PID adaptive &mdash; gain scheduling akan menyesuaikan otomatis saat sistem berjalan. Mulai dari suhu di bawah setpoint (tungku dingin) supaya osilasi pertama valid, dan sebaiknya DEKAT suhu operasi nyata (bukan 60 C kalau target akhirnya 300 C) supaya amplitudo osilasi cukup besar untuk hasil yang andal. Saat Relay/Step/Uji Metode aktif, kontrol fan manual dan Test Semua dikunci. Satuan hasil: % PWM fan per &deg;C. "Terapkan ke PID" menyimpan Kp/Ki/Kd ke memori ESP32 (NVS, tetap ada setelah reboot); "Reset PID" mengembalikannya ke default kapan saja.</div>
</div>

<!-- 9. Uji Metode -->
<div class="card">
  <h3>9. Uji Metode (Adaptive PID)</h3>
  <div class="at-row">
    <label>Setpoint (C)<input type="number" id="mtSp" value="300" min="50" max="800" step="1"></label>
    <label>Maks durasi (mnt)<input type="number" id="mtMin" value="60" min="5" max="240" step="5"></label>
  </div>
  <label style="display:flex;align-items:center;gap:6px;font-size:12px;color:var(--text-dim);margin-top:6px;">
    <input type="checkbox" id="mtUnlimited" onchange="mtToggleUnlimited()" style="width:auto;margin:0;">
    Tanpa batas waktu (berhenti manual dengan tombol STOP)
  </label>
  <div class="btn-row">
    <button class="primary" id="btnMtStart" onclick="mtStart()">MULAI UJI</button>
    <button class="danger" id="btnMtStop" onclick="mtStop()" style="display:none;">STOP</button>
  </div>
  <div class="status-line">Status: <span class="badge idle" id="mtBadge">IDLE</span></div>
  <div class="progress"><div class="progress-bar" id="mtBar"></div></div>
  <div class="status-line" id="mtProgText">0 s</div>
  <div class="status-line">Suhu: <b id="mtTemp">--</b> C &nbsp;|&nbsp; Fan: <b id="mtFan">--</b> &nbsp;|&nbsp; Tetes: <b id="mtTips">--</b></div>
  <div class="status-line">Waktu tetes terakhir (detik sejak mulai): <span id="mtRecentTips">--</span></div>
  <div class="at-result" id="mtResult" style="display:none;"></div>
  <div class="btn-row" id="mtDataRow" style="display:none;">
    <button class="warn" onclick="window.open('/method/data','_blank')">UNDUH CSV WAKTU TETES</button>
  </div>
  <div class="hint">Menjalankan Adaptive PID (gain scheduling 3-zona sama seperti firmware utama: Kp x1.3 di bawah 200 C, x1.0 di 200-400 C, x0.7 di atas 400 C; Ki/Kd tetap) memakai Kp/Ki/Kd yang sedang aktif (lihat card 8, "PID aktif"). Pompa oli &amp; air dinyalakan OTOMATIS selama uji supaya oli benar-benar menetes, dan dimatikan otomatis saat uji selesai/distop -- kontrol manual pompa (card 4/5) terkunci selama uji berjalan. Dua mode durasi: <b>Waktu Tetap</b> (durasi ditentukan di atas; begitu habis, uji dimatikan otomatis, mau tidak mau) atau <b>Tanpa Batas</b> (centang kotak di atas; uji terus berjalan sampai Anda tekan STOP). Kedua mode tetap berhenti otomatis kalau suhu melebihi setpoint+100 C atau sensor gagal baca. Tidak bisa berjalan bersamaan dengan Step/Relay Test. Waktu tiap tetes dicatat (detik sejak mulai) untuk menghitung laju produksi -- unduh CSV-nya lewat tombol di atas atau logger.py di laptop.</div>
</div>

<!-- 10. Grafik Suhu & Setpoint -->
<div class="card">
  <h3>10. Grafik Suhu &amp; Setpoint</h3>
  <canvas id="chartTemp" style="width:100%;height:220px;background:#0d1229;border:1px solid var(--border);border-radius:6px;"></canvas>
  <div class="hint">Merah = suhu aktual, hijau putus-putus = setpoint aktif (0 saat idle/Step Test, karena Step Test tidak memakai setpoint). Polling /data/live tiap 2 detik, maksimum 300 titik (&plusmn;10 menit). Sumbu Y dibatasi 0-300&deg;C; untuk Relay/Step Test di atas 300&deg;C titik akan terpotong secara visual saja (bukan data error).</div>
</div>

<!-- 11. Grafik Blower (Fan %) -->
<div class="card">
  <h3>11. Grafik Blower (Fan %)</h3>
  <canvas id="chartFan" style="width:100%;height:180px;background:#0d1229;border:1px solid var(--border);border-radius:6px;"></canvas>
</div>

<!-- 12. Grafik Pompa Air & Oli -->
<div class="card">
  <h3>12. Grafik Pompa Air &amp; Oli</h3>
  <canvas id="chartPump" style="width:100%;height:160px;background:#0d1229;border:1px solid var(--border);border-radius:6px;"></canvas>
</div>

<script src="https://cdn.jsdelivr.net/npm/chart.js@4/dist/chart.umd.min.js"></script>
<script>
var connOk = false;

function setResult(id, ok, text) {
  var el = document.getElementById(id);
  el.className = "result " + (ok === null ? "info" : (ok ? "ok" : "err"));
  el.textContent = text;
}

function handleData(d) {
  if (!d || !d.type) return;
  switch (d.type) {
    case "TEMP": {
      if (d.error) setResult("tempResult", false, d.error + (d.value != null ? " (" + d.value.toFixed(1) + " C)" : ""));
      else setResult("tempResult", true, "Suhu: " + d.value.toFixed(1) + " C");
      break;
    }
    case "TIP": {
      var vol = (d.count * 1.42).toFixed(2);
      setResult("tipResult", null, "Tips: " + d.count + " | Volume: " + vol + " mL");
      break;
    }
    case "OIL": {
      var el = document.getElementById("oilStatus");
      el.textContent = d.state ? "ON" : "OFF";
      el.className = d.state ? "" : "off";
      break;
    }
    case "WATER": {
      var el = document.getElementById("waterStatus");
      el.textContent = d.state ? "ON" : "OFF";
      el.className = d.state ? "" : "off";
      break;
    }
    case "FAN": {
      if (d.error) { setResult("fanResult", false, "Ditolak: " + d.error); break; }
      document.getElementById("fanSlider").value = d.pct;
      setResult("fanResult", null, "PWM raw: " + d.raw + " / 255  (" + d.pct + "%)");
      break;
    }
    case "ALL": {
      if (d.error) { setResult("allResult", false, "Ditolak: " + d.error); break; }
      var lines = [];
      lines.push((d.tempOk ? "[OK] " : "[GAGAL] ") + "Suhu: " + d.temp.toFixed(1) + " C");
      lines.push("[INFO] Tipping bucket: " + d.tips + " tips");
      lines.push("[INFO] Fan/Oil/Water sudah dites ON 1 detik lalu OFF - cek suara relay & putaran fan secara fisik.");
      setResult("allResult", d.tempOk, lines.join("\n"));
      break;
    }
    case "AT": {
      atHandle(d);
      break;
    }
    case "ST": {
      stHandle(d);
      break;
    }
    case "PID": {
      pidHandle(d);
      break;
    }
    case "MT": {
      mtHandle(d);
      break;
    }
  }
}

function req(url, cb) {
  fetch(url).then(function(r){ return r.json(); }).then(function(d){ handleData(d); if (cb) cb(d); })
    .catch(function(e){ console.error(e); });
}

function testTemp() { req("/test/temp"); }
function resetTip() { req("/tip/reset"); }
function testAll() { setResult("allResult", null, "Menjalankan semua test (~3 detik)..."); req("/test/all"); }

function oilCmd(state) { req("/oil/" + state); }
function waterCmd(state) { req("/water/" + state); }

function setFan(pct) { req("/fan/" + pct); }
function onFanSlide(val) { req("/fan/" + val); }

// ---------------- Step Test (card 7) ----------------
var stTimer = null;
var stCopyText = "";

function stEl(id) { return document.getElementById(id); }

function stSetPolling(on) {
  if (on && !stTimer) stTimer = setInterval(function(){ req("/step/status"); }, 2000);
  if (!on && stTimer) { clearInterval(stTimer); stTimer = null; }
}

function stSetBadge(cls, text) {
  var b = stEl("stBadge");
  b.className = "badge " + cls;
  b.textContent = text;
}

function stSetRunningUi(running) {
  stEl("btnStStart").style.display = running ? "none" : "";
  stEl("btnStStop").style.display = running ? "" : "none";
  var ids = ["stPct", "stLimit", "stMin"];
  for (var i = 0; i < ids.length; i++) stEl(ids[i]).disabled = running;
}

function stShowBox(txt) {
  var r = stEl("stResult");
  r.style.display = "";
  r.textContent = txt;
}

function stStart() {
  var pct = parseFloat(stEl("stPct").value);
  var lim = parseFloat(stEl("stLimit").value);
  var mn  = parseFloat(stEl("stMin").value);
  if (isNaN(pct) || pct < 10 || pct > 100) { stShowBox("Fan STEP harus 10-100 %"); return; }
  if (isNaN(lim) || lim < 50 || lim > 800) { stShowBox("Batas suhu harus 50-800 C"); return; }
  if (isNaN(mn) || mn < 5 || mn > 60) { stShowBox("Durasi maksimum harus 5-60 menit"); return; }
  req("/step/start?pct=" + pct + "&limit=" + lim + "&min=" + mn);
}

function stStop() { req("/step/stop"); }

// Grafik suhu vs waktu (canvas). d.tr = titik yang di-downsample tiap d.dt detik,
// d.ts = detik saat step diberikan.
function stDraw(d, showSs) {
  var c = stEl("stChart");
  var g = c.getContext("2d");
  var w = c.width, h = c.height;
  g.clearRect(0, 0, w, h);
  var tr = d.tr || [];
  if (tr.length < 2) return;
  var lo = Math.min.apply(null, tr), hi = Math.max.apply(null, tr);
  if (d.T0 > 0) lo = Math.min(lo, d.T0);
  if (showSs && d.Tss) hi = Math.max(hi, d.Tss);
  if (hi - lo < 5) hi = lo + 5;
  var padL = 34, padR = 6, padT = 6, padB = 14;
  function X(i) { return padL + (w - padL - padR) * i / (tr.length - 1); }
  function Y(v) { return padT + (h - padT - padB) * (1 - (v - lo) / (hi - lo)); }
  g.lineWidth = 2; g.strokeStyle = "#4ecca3"; g.beginPath();
  for (var i = 0; i < tr.length; i++) { if (i === 0) g.moveTo(X(i), Y(tr[i])); else g.lineTo(X(i), Y(tr[i])); }
  g.stroke();
  g.lineWidth = 1;
  var si = d.ts / d.dt;                                  // indeks saat step
  if (si < tr.length - 1) {
    g.strokeStyle = "#ffd700"; g.setLineDash([4, 3]); g.beginPath();
    g.moveTo(X(si), padT); g.lineTo(X(si), h - padB); g.stroke();
  }
  if (showSs && d.Tss) {
    g.strokeStyle = "#e94560"; g.setLineDash([3, 3]); g.beginPath();
    g.moveTo(padL, Y(d.Tss)); g.lineTo(w - padR, Y(d.Tss)); g.stroke();
  }
  g.setLineDash([]);
  g.fillStyle = "#8f9bb3"; g.font = "10px sans-serif";
  g.fillText(hi.toFixed(0), 2, padT + 8);
  g.fillText(lo.toFixed(0), 2, h - padB);
}

function stHandle(d) {
  if (d.status === "started") {
    stEl("stResult").style.display = "none";
    stEl("stCopyRow").style.display = "none";
    stCopyText = "";
    stSetBadge("running", "RUNNING");
    stSetRunningUi(true);
    stEl("stBar").style.width = "0%";
    stEl("stProgText").textContent = "0 s";
    stEl("stChart").getContext("2d").clearRect(0, 0, 400, 150);
    stSetPolling(true);
    req("/step/status");
  } else if (d.status === "running") {
    stSetBadge("running", "RUNNING");
    stSetRunningUi(true);
    stSetPolling(true);
    stEl("stPhase").textContent = (d.phase === "step") ? ("STEP fan " + d.pct + "%") : "baseline (fan 0%)";
    stEl("stProgText").textContent = d.t + " s / maks " + d.max + " s";
    stEl("stBar").style.width = Math.min(100, d.t / d.max * 100) + "%";
    stEl("stTemp").textContent = d.temp.toFixed(1);
    stEl("stT0").textContent = (d.phase === "step") ? d.T0.toFixed(1) : "--";
    stEl("stFan").textContent = d.fan + "%";
    stDraw(d, false);
  } else if (d.status === "done") {
    stSetPolling(false);
    stSetRunningUi(false);
    stSetBadge("done", "DONE");
    stEl("stPhase").textContent = "";
    stEl("stBar").style.width = "100%";
    stEl("stFan").textContent = "0%";
    stDraw(d, true);
    stShowBox("L (dead time)     = " + d.L.toFixed(1) + " s\n" +
              "T (time constant) = " + d.T.toFixed(1) + " s\n" +
              "K (gain)          = " + d.K.toFixed(4) + " C/%\n" +
              "T0=" + d.T0.toFixed(1) + "  Tss=" + d.Tss.toFixed(1) + "  dT=" + d.dT.toFixed(1) + " C\n" +
              "t28,3%=" + d.t283.toFixed(1) + " s   t63,2%=" + d.t632.toFixed(1) + " s" +
              (d.warn ? ("\n! " + d.warn) : ""));
    stCopyText = "L=" + d.L.toFixed(1) + " T=" + d.T.toFixed(1) + " K=" + d.K.toFixed(4);
    stEl("stCopyRow").style.display = "";
  } else if (d.status === "stopped" || d.status === "idle") {
    stSetPolling(false);
    stSetRunningUi(false);
    stSetBadge("idle", "IDLE");
    stEl("stPhase").textContent = "";
    stEl("stBar").style.width = "0%";
    stEl("stProgText").textContent = (d.status === "stopped") ? "Dibatalkan" : "0 s";
    stEl("stFan").textContent = "--";
  } else if (d.status === "error") {
    stSetPolling(false);
    stSetRunningUi(false);
    stSetBadge("error", "ERROR");
    stShowBox("Step test gagal: " + (d.error || "tidak diketahui"));
  }
}

function copyText(txt, btn, label) {
  if (!txt) return;
  function done(ok) {
    btn.textContent = ok ? "TERSALIN!" : "GAGAL SALIN";
    setTimeout(function(){ btn.textContent = label; }, 1500);
  }
  if (navigator.clipboard && window.isSecureContext) {
    navigator.clipboard.writeText(txt).then(function(){ done(true); }, function(){ done(false); });
  } else {
    var ta = document.createElement("textarea");
    ta.value = txt;
    ta.style.position = "fixed";
    ta.style.opacity = "0";
    document.body.appendChild(ta);
    ta.select();
    var ok = false;
    try { ok = document.execCommand("copy"); } catch (e) { ok = false; }
    document.body.removeChild(ta);
    done(ok);
  }
}

function stCopy() { copyText(stCopyText, stEl("btnStCopy"), "SALIN HASIL"); }

// ---------------- Terapkan ke PID (card 8) ----------------
function pidApply() {
  if (!confirm("Terapkan Kp/Ki/Kd hasil Relay Test ke PID? Nilai lama akan ditimpa dan disimpan di memori ESP32.")) return;
  req("/pid/apply");
}

function pidReset() {
  if (!confirm("Reset Kp/Ki/Kd ke default (Kp=2.0 Ki=0.1 Kd=10.0)? Nilai hasil Relay Test yang tersimpan sekarang akan hilang.")) return;
  req("/pid/reset");
}

function pidHandle(d) {
  if (d.status === "error") { alert("Gagal: " + d.error); return; }
  document.getElementById("pidActive").textContent =
    "Kp=" + d.Kp.toFixed(4) + " Ki=" + d.Ki.toFixed(4) + " Kd=" + d.Kd.toFixed(4) +
    " (" + (d.src === "relay" ? "hasil relay test" : "default") + ")";
  if (d.status === "applied") {
    var b = document.getElementById("btnAtApply");
    b.textContent = "DITERAPKAN";
    setTimeout(function(){ b.textContent = "TERAPKAN KE PID"; }, 1500);
  } else if (d.status === "reset") {
    var rb = document.getElementById("btnPidReset");
    rb.textContent = "SUDAH DI-RESET";
    setTimeout(function(){ rb.textContent = "RESET PID KE DEFAULT (Kp=2.0 Ki=0.1 Kd=10.0)"; }, 1500);
  }
}

// ---------------- Uji Metode (card 9) ----------------
var mtTimer = null;

function mtEl(id) { return document.getElementById(id); }

function mtSetPolling(on) {
  if (on && !mtTimer) mtTimer = setInterval(function(){ req("/method/status"); }, 2000);
  if (!on && mtTimer) { clearInterval(mtTimer); mtTimer = null; }
}

function mtSetBadge(cls, text) {
  var b = mtEl("mtBadge");
  b.className = "badge " + cls;
  b.textContent = text;
}

function mtToggleUnlimited() {
  mtEl("mtMin").disabled = mtEl("mtUnlimited").checked;
}

function mtSetRunningUi(running) {
  mtEl("btnMtStart").style.display = running ? "none" : "";
  mtEl("btnMtStop").style.display = running ? "" : "none";
  mtEl("mtSp").disabled = running;
  mtEl("mtUnlimited").disabled = running;
  mtEl("mtMin").disabled = running || mtEl("mtUnlimited").checked;
}

function mtShowBox(txt) {
  var r = mtEl("mtResult");
  r.style.display = "";
  r.textContent = txt;
}

function mtStart() {
  var sp = parseFloat(mtEl("mtSp").value);
  var unlimited = mtEl("mtUnlimited").checked;
  var mn = unlimited ? 0 : parseFloat(mtEl("mtMin").value);
  if (isNaN(sp) || sp < 50 || sp > 800) { mtShowBox("Setpoint harus 50-800 C"); return; }
  if (!unlimited && (isNaN(mn) || mn < 5 || mn > 240)) { mtShowBox("Durasi maksimum harus 5-240 menit"); return; }
  req("/method/start?sp=" + sp + "&min=" + mn);
}

function mtStop() { req("/method/stop"); }

function mtHandle(d) {
  if (d.status === "started") {
    mtEl("mtResult").style.display = "none";
    mtEl("mtDataRow").style.display = "none";
    mtSetBadge("running", "RUNNING");
    mtSetRunningUi(true);
    mtEl("mtBar").style.width = "0%";
    mtEl("mtProgText").textContent = "0 s";
    mtSetPolling(true);
    req("/method/status");
  } else if (d.status === "running") {
    mtSetBadge("running", "RUNNING");
    mtSetRunningUi(true);
    mtSetPolling(true);
    if (d.max > 0) {
      mtEl("mtProgText").textContent = d.t + " s / maks " + d.max + " s";
      mtEl("mtBar").style.width = Math.min(100, d.t / d.max * 100) + "%";
    } else {
      mtEl("mtProgText").textContent = d.t + " s (tanpa batas waktu)";
      mtEl("mtBar").style.width = "100%";
    }
    mtEl("mtTemp").textContent = d.temp.toFixed(1);
    mtEl("mtFan").textContent = d.fan + "%";
    mtEl("mtTips").textContent = d.tips;
    mtEl("mtRecentTips").textContent = (d.recentTips && d.recentTips.length) ? d.recentTips.join(", ") : "--";
  } else if (d.status === "done") {
    mtSetPolling(false);
    mtSetRunningUi(false);
    mtSetBadge("done", "DONE");
    mtEl("mtBar").style.width = "100%";
    mtEl("mtFan").textContent = "0%";
    mtShowBox("Durasi     = " + d.durS + " s\n" +
              "Tetes      = " + d.tips + " (" + d.volumeMl.toFixed(2) + " mL)\n" +
              "RMS error  = " + d.rmsErr.toFixed(2) + " C");
    mtEl("mtDataRow").style.display = "";
  } else if (d.status === "stopped" || d.status === "idle") {
    mtSetPolling(false);
    mtSetRunningUi(false);
    mtSetBadge("idle", "IDLE");
    mtEl("mtBar").style.width = "0%";
    mtEl("mtProgText").textContent = (d.status === "stopped") ? "Dibatalkan" : "0 s";
    mtEl("mtFan").textContent = "--";
  } else if (d.status === "error") {
    mtSetPolling(false);
    mtSetRunningUi(false);
    mtSetBadge("error", "ERROR");
    mtShowBox("Uji metode gagal: " + (d.error || "tidak diketahui"));
  }
}

// ---------------- Relay Test (card 8) ----------------
var atTimer = null;
var atCopyText = "";

function atEl(id) { return document.getElementById(id); }

function atSetPolling(on) {
  if (on && !atTimer) atTimer = setInterval(function(){ req("/autotune/status"); }, 2000);
  if (!on && atTimer) { clearInterval(atTimer); atTimer = null; }
}

function atSetBadge(cls, text) {
  var b = atEl("atBadge");
  b.className = "badge " + cls;
  b.textContent = text;
}

function atSetRunningUi(running) {
  atEl("btnAtStart").style.display = running ? "none" : "";
  atEl("btnAtStop").style.display = running ? "" : "none";
  var ids = ["atSp", "atHigh", "atLow"];
  for (var i = 0; i < ids.length; i++) atEl(ids[i]).disabled = running;
}

function atShowBox(txt) {
  var r = atEl("atResult");
  r.style.display = "";
  r.textContent = txt;
}

function atStart() {
  var sp = parseFloat(atEl("atSp").value);
  var hi = parseFloat(atEl("atHigh").value);
  var lo = parseFloat(atEl("atLow").value);
  if (isNaN(sp) || sp < 50 || sp > 800) { atShowBox("Setpoint harus 50-800 C"); return; }
  if (isNaN(hi) || hi < 10 || hi > 100) { atShowBox("Fan ON harus 10-100 %"); return; }
  if (isNaN(lo) || lo < 0 || lo > 50)   { atShowBox("Fan OFF harus 0-50 %"); return; }
  if (lo >= hi) { atShowBox("Fan OFF harus lebih kecil dari Fan ON"); return; }
  req("/autotune/start?sp=" + sp + "&high=" + hi + "&low=" + lo);
}

function atStop() { req("/autotune/stop"); }

function atHandle(d) {
  if (d.status === "started") {
    atEl("atResult").style.display = "none";
    atEl("atCopyRow").style.display = "none";
    atCopyText = "";
    atSetBadge("running", "RUNNING");
    atSetRunningUi(true);
    atEl("atBar").style.width = "0%";
    atEl("atProgText").textContent = "0 / 3 siklus";
    atSetPolling(true);
    req("/autotune/status");
  } else if (d.status === "running") {
    atSetBadge("running", "RUNNING");
    atSetRunningUi(true);
    atSetPolling(true);
    atEl("atProgText").textContent = d.cycles + " / " + d.need + " siklus  (" + d.peaks + " puncak)";
    atEl("atBar").style.width = Math.min(100, d.peaks / (d.need * 2) * 100) + "%";
    atEl("atTemp").textContent = d.temp.toFixed(1);
    var rl = atEl("atRelay");
    rl.textContent = d.relay ? "Fan: ON" : "Fan: OFF";
    rl.className = d.relay ? "" : "off";
    atEl("atPeaks").textContent = (d.pk && d.pk.length) ? (d.pk.join(", ") + " C") : "--";
  } else if (d.status === "done") {
    atSetPolling(false);
    atSetRunningUi(false);
    atSetBadge("done", "DONE");
    atEl("atBar").style.width = "100%";
    atEl("atProgText").textContent = "Selesai";
    atEl("atRelay").textContent = "Fan: OFF";
    atEl("atRelay").className = "off";
    atShowBox("Ku  = " + d.Ku.toFixed(4) + "   Tu  = " + d.Tu.toFixed(1) + " s\n" +
              "Kp  = " + d.Kp.toFixed(4) + "   Ki  = " + d.Ki.toFixed(4) + "   Kd  = " + d.Kd.toFixed(4));
    atCopyText = "Kp=" + d.Kp.toFixed(4) + " Ki=" + d.Ki.toFixed(4) + " Kd=" + d.Kd.toFixed(4);
    atEl("atCopyRow").style.display = "";
  } else if (d.status === "stopped" || d.status === "idle") {
    atSetPolling(false);
    atSetRunningUi(false);
    atSetBadge("idle", "IDLE");
    atEl("atBar").style.width = "0%";
    atEl("atProgText").textContent = (d.status === "stopped") ? "Dibatalkan" : "0 / 3 siklus";
    atEl("atRelay").textContent = "Fan: --";
    atEl("atRelay").className = "off";
  } else if (d.status === "error") {
    atSetPolling(false);
    atSetRunningUi(false);
    atSetBadge("error", "ERROR");
    atShowBox("Autotune gagal: " + (d.error || "tidak diketahui"));
  }
}

// navigator.clipboard hanya ada di konteks aman (HTTPS/localhost); dashboard
// ini di http://192.168.4.1, jadi pakai fallback execCommand.
function atCopy() {
  if (!atCopyText) return;
  var btn = atEl("btnAtCopy");
  function done(ok) {
    btn.textContent = ok ? "TERSALIN!" : "GAGAL SALIN";
    setTimeout(function(){ btn.textContent = "SALIN HASIL"; }, 1500);
  }
  if (navigator.clipboard && window.isSecureContext) {
    navigator.clipboard.writeText(atCopyText).then(function(){ done(true); }, function(){ done(false); });
  } else {
    var ta = document.createElement("textarea");
    ta.value = atCopyText;
    ta.style.position = "fixed";
    ta.style.opacity = "0";
    document.body.appendChild(ta);
    ta.select();
    var ok = false;
    try { ok = document.execCommand("copy"); } catch (e) { ok = false; }
    document.body.removeChild(ta);
    done(ok);
  }
}

function autoTip() { req("/tip/get"); }

function checkConn() {
  fetch("/ping").then(function(r){ return r.json(); }).then(function(d){
    connOk = !!d.ok;
    document.getElementById("connDot").className = "dot on";
    document.getElementById("connText").textContent = "Terhubung ke ESP32";
  }).catch(function(){
    connOk = false;
    document.getElementById("connDot").className = "dot";
    document.getElementById("connText").textContent = "Terputus dari ESP32";
  });
}

checkConn();
setInterval(checkConn, 3000);
// Suhu auto-refresh tiap 5 detik (endpoint tanpa cetak Serial)
function autoTemp() { req("/temp/auto"); }
autoTemp();
setInterval(autoTemp, 5000);
// Tipping bucket: baca langsung tiap 5 detik (tanpa tombol Mulai/Stop Monitor)
autoTip();
setInterval(autoTip, 5000);
// Sinkronkan kartu 7/8 dengan kondisi ESP32 (mis. halaman di-refresh saat uji berjalan)
req("/step/status");
req("/autotune/status");
req("/pid/get");
req("/method/status");

// ---------------- Grafik real-time (card 10/11/12) ----------------
// SATU setInterval untuk ketiga grafik: tiap 2 detik fetch /data/live sekali,
// lalu dorong satu titik baru ke ketiganya sekaligus (bukan 1 interval/grafik).
var LIVE_MAX_POINTS = 300;
var liveStartMs = Date.now();
var liveLabels = [];
var liveTemp = [];
var liveSp = [];
var liveFan = [];
var liveWater = [];
var liveOil = [];

function fmtMMSS(sec) {
  var m = Math.floor(sec / 60);
  var s = Math.floor(sec % 60);
  return (m < 10 ? "0" : "") + m + ":" + (s < 10 ? "0" : "") + s;
}

var liveGridColor = "#2a3358";
var liveTickColor = "#8f9bb3";
var liveChartDefaults = {
  animation: false,
  responsive: true,
  maintainAspectRatio: false,
  interaction: { mode: "nearest", intersect: false },
  scales: { x: { ticks: { color: liveTickColor, maxTicksLimit: 8 }, grid: { color: liveGridColor } } },
  plugins: { legend: { labels: { color: "#eaeaea", boxWidth: 12, font: { size: 11 } } } },
};

function liveOpts(yAxis) {
  var o = JSON.parse(JSON.stringify(liveChartDefaults));
  o.scales.y = yAxis;
  return o;
}

var chartTemp = new Chart(document.getElementById("chartTemp"), {
  type: "line",
  data: {
    labels: liveLabels,
    datasets: [
      { label: "Suhu (C)", data: liveTemp, borderColor: "#e94560", backgroundColor: "transparent",
        borderWidth: 1.5, pointRadius: 0, tension: 0.15, spanGaps: false },
      { label: "Setpoint (C)", data: liveSp, borderColor: "#4ecca3", backgroundColor: "transparent",
        borderWidth: 1.5, borderDash: [6, 4], pointRadius: 0, tension: 0 },
    ],
  },
  options: liveOpts({ min: 0, max: 300, ticks: { color: liveTickColor }, grid: { color: liveGridColor } }),
});

var chartFan = new Chart(document.getElementById("chartFan"), {
  type: "line",
  data: {
    labels: liveLabels,
    datasets: [
      { label: "Fan (%)", data: liveFan, borderColor: "#4ea1e9", backgroundColor: "rgba(78,161,233,0.15)",
        fill: true, borderWidth: 1.5, pointRadius: 0, tension: 0.15 },
    ],
  },
  options: liveOpts({ min: 0, max: 100, ticks: { color: liveTickColor }, grid: { color: liveGridColor } }),
});

var chartPump = new Chart(document.getElementById("chartPump"), {
  type: "line",
  data: {
    labels: liveLabels,
    datasets: [
      { label: "Water", data: liveWater, borderColor: "#4ea1e9", backgroundColor: "transparent",
        borderWidth: 1.5, pointRadius: 0, stepped: true },
      { label: "Oil", data: liveOil, borderColor: "#ffd700", backgroundColor: "transparent",
        borderWidth: 1.5, pointRadius: 0, stepped: true },
    ],
  },
  options: liveOpts({
    min: -0.2, max: 1.2, ticks: {
      color: liveTickColor, stepSize: 1,
      callback: function (v) { if (v === 0) return "OFF"; if (v === 1) return "ON"; return ""; },
    },
    grid: { color: liveGridColor },
  }),
});

function liveFetch() {
  var ctrl = (typeof AbortController !== "undefined") ? new AbortController() : null;
  var timer = ctrl ? setTimeout(function () { ctrl.abort(); }, 1500) : null;
  fetch("/data/live", ctrl ? { signal: ctrl.signal } : {})
    .then(function (r) { return r.json(); })
    .then(function (d) {
      if (timer) clearTimeout(timer);
      var sec = (Date.now() - liveStartMs) / 1000;
      liveLabels.push(fmtMMSS(sec));
      liveTemp.push((d.temp === null || d.temp === undefined) ? null : d.temp);
      liveSp.push(typeof d.sp === "number" ? d.sp : null);
      liveFan.push(typeof d.fan === "number" ? d.fan : null);
      liveWater.push(d.water ? 1 : 0);
      liveOil.push(d.oil ? 1 : 0);

      while (liveLabels.length > LIVE_MAX_POINTS) {
        liveLabels.shift(); liveTemp.shift(); liveSp.shift();
        liveFan.shift(); liveWater.shift(); liveOil.shift();
      }
      chartTemp.update("none");
      chartFan.update("none");
      chartPump.update("none");
    })
    .catch(function () {
      if (timer) clearTimeout(timer);
      // Gagal (timeout/jaringan): lewati tick ini, jangan retry -- coba lagi otomatis di tick berikutnya.
    });
}

setInterval(liveFetch, 2000);
</script>
</body>
</html>
)HTMLDIAG";
