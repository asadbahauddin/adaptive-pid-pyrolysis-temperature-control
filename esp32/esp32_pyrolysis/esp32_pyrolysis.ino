// ============================================================
// esp32_pyrolysis.ino  (v3)
// Firmware ESP32 untuk sistem pyrolysis: dashboard diagnostik via
// WiFi Access Point + pengiriman data status ke Arduino UNO (LCD)
// lewat Serial2 dalam format JSON setiap 500ms.
//
// v2: log suhu ke Serial Monitor setiap 2 detik
// v3: TAMBAH dua fitur pengujian termal di dashboard:
//   Card 7  STEP TEST  - ukur Dead Time (L), Time Constant (T) dan gain (K)
//                        dari respons open-loop (metode dua titik 28,3%/63,2%)
//       route: /step/start?pct=60&limit=500&min=30 | /step/stop | /step/status
//   Card 8  RELAY TEST - Ziegler-Nichols relay -> Ku, Tu, Kp, Ki, Kd,
//                        lalu tombol "Terapkan ke PID" (disimpan di NVS)
//       route: /autotune/start|stop|status  dan  /pid/apply | /pid/get
//   - SD CARD DIHAPUS. Data diambil laptop lewat WiFi (server/logger
//     Python, lihat laptop/logger.py):
//       /data/live  -> satu paket data (suhu, fan, pompa, tips, mode uji)
//       /step/data  -> seluruh rekaman Step Test (CSV, resolusi 1 detik)
//   - keduanya saling mengunci (tidak bisa jalan bersamaan) dan mengunci
//     /fan/* serta /test/all selama berjalan
//   - pengaman: timeout, overtemp, sensor gagal baca -> fan dimatikan
//   - readTemperature() dibatasi >=250ms antar pembacaan fisik
//     (MAX6675 butuh waktu konversi) dan di-cache di antaranya
//   - suhu di dashboard auto-refresh tiap 5 detik
//   - debounce tipping bucket 100 ms
//   - Water pump = GPIO26, Oil pump = GPIO27 (relay active-LOW)
// ============================================================
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <Preferences.h>
#include <max6675.h>

// ------------------------------------------------------------
// Pin map
// ------------------------------------------------------------
#define PIN_FAN_PWM       25   // Fan DC PWM (MOSFET IRLB8721)
#define PIN_WATER_PUMP    26   // Water Pump Relay (via PC817+BD139), active-LOW
#define PIN_OIL_PUMP      27   // Oil Pump Relay   (via PC817+BD139), active-LOW
#define PIN_TIPPING       34   // Tipping Bucket Hall Sensor

#define PIN_MAX6675_CS     5   // HSPI  <-- strapping pin, aman sebagai CS (boot: pull-up = CS HIGH = chip idle)
#define PIN_MAX6675_SCK   14   // HSPI
#define PIN_MAX6675_MISO   35   // HSPI  <-- strapping pin, AWAS: jangan pakai sebagai MISO di sistem final

#define PIN_SERIAL2_TX    17
#define PIN_SERIAL2_RX    16

#define PWM_CHANNEL        0
#define PWM_FREQ        5000
#define PWM_RESOLUTION     8

#define VOLUME_PER_TIP_ML  1.42
#define DEFAULT_SETPOINT 300.0

// ------------------------------------------------------------
// Interval log Serial Monitor suhu (ms)
// Ubah nilai ini untuk mempercepat/memperlambat update
// ------------------------------------------------------------
#define TEMP_LOG_INTERVAL_MS  2000   // log suhu ke Serial setiap 2 detik

// ------------------------------------------------------------
// WiFi Access Point
// ------------------------------------------------------------
#define WIFI_SSID     "Pyrolysis-Diag"
#define WIFI_PASSWORD "12345678"
#define HTTP_PORT     80

#define SERIAL2_INTERVAL_MS 500

// ------------------------------------------------------------
// Objek global
// ------------------------------------------------------------
static WebServer server(HTTP_PORT);
static MAX6675 thermocouple(PIN_MAX6675_SCK, PIN_MAX6675_CS, PIN_MAX6675_MISO);

static volatile unsigned long tipCount = 0;
static unsigned long lastTipIsrMs = 0;
#define TIP_DEBOUNCE_MS 100

static bool oilState = false;
static bool waterState = false;
static uint8_t fanPct = 0;
static uint8_t fanRawValue = 0;

static float lastGoodTemp = 25.0;
static unsigned long lastSerial2Ms = 0;

// *** NEWs: timer untuk log suhu ke Serial Monitor ***
static unsigned long lastTempLogMs = 0;
static uint32_t tempReadCount = 0;  // jumlah total pembacaan sejak boot

// ------------------------------------------------------------
// ISR Tipping Bucket
// ------------------------------------------------------------
void IRAM_ATTR onTip() {
  unsigned long now = millis();
  if (now - lastTipIsrMs >= TIP_DEBOUNCE_MS) {
    tipCount++;
    lastTipIsrMs = now;
  }
}

// ------------------------------------------------------------
// Baca suhu MAX6675, dengan validasi rentang normal
// ------------------------------------------------------------
// MAX6675 butuh ~170-220ms per konversi: pembacaan lebih rapat dari
// MAX6675_MIN_INTERVAL_MS mengembalikan nilai cache (bukan baca ulang),
// supaya autotune (1s), log Serial (2s) dan request web tidak saling
// menabrak dan menghasilkan data basi/duplikat.
#define MAX6675_MIN_INTERVAL_MS 250

float readTemperature() {
  static unsigned long lastReadMs = 0;
  static float cachedTemp = NAN;
  static bool hasCache = false;

  unsigned long nowMs = millis();
  if (hasCache && (nowMs - lastReadMs) < MAX6675_MIN_INTERVAL_MS) return cachedTemp;

  float t = thermocouple.readCelsius();
  lastReadMs = nowMs;
  cachedTemp = t;
  hasCache = true;

  if (isnan(t)) return NAN;
  if (t < 0 || t > 1000) return t;
  lastGoodTemp = t;
  return t;
}

// ------------------------------------------------------------
// LOG SUHU KE SERIAL MONITOR
// Dipanggil dari loop() setiap TEMP_LOG_INTERVAL_MS
// ------------------------------------------------------------
static void logTempToSerial() {
  float t = readTemperature();
  tempReadCount++;

  Serial.print(F("[TEMP] #"));
  Serial.print(tempReadCount);
  Serial.print(F("  uptime="));
  Serial.print(millis() / 1000);
  Serial.print(F("s  ->  "));

  if (isnan(t)) {
    Serial.println(F("ERROR: NaN - sensor tidak terhubung / thermocouple putus"));
  } else if (t < 0) {
    Serial.print(F("WARN: suhu negatif ("));
    Serial.print(t, 2);
    Serial.println(F(" C) - cek wiring SCK/CS/MISO"));
  } else if (t > 1000) {
    Serial.print(F("WARN: suhu >1000 C ("));
    Serial.print(t, 2);
    Serial.println(F(" C) - cek koneksi thermocouple"));
  } else if (t == 0.0f) {
    // 0.0 persis biasanya artinya SPI tidak nyambung sama sekali
    Serial.println(F("WARN: 0.00 C - kemungkinan SPI tidak nyambung, cek pin CS/SCK/MISO"));
  } else {
    Serial.print(t, 2);
    Serial.println(F(" C  [OK]"));
  }
}

// ============================================================
// AUTOTUNE PID - RELAY METHOD (ZIEGLER-NICHOLS)
// ============================================================
// Satuan hasil: Kp dalam [% PWM fan per degC], Ki dalam [% per degC.s],
// Kd dalam [% per degC/s] -- sama dengan skala PWM 0-100% di simulasi
// MATLAB, jadi hasil bisa langsung dipakai sebagai nilai awal PID.
//
// CATATAN deteksi puncak: sampel per 1 detik pada sistem termal yang
// lambat berubah < 1 degC per sampel, sehingga membandingkan suhu
// dengan SAMPEL SEBELUMNYA tidak akan pernah memicu balik-arah. Karena
// itu yang dilacak adalah EKSTREM berjalan (max saat naik / min saat
// turun); balik-arah terdeteksi ketika suhu menyimpang >= AT_PEAK_BAND
// dari ekstrem itu, dan yang disimpan adalah nilai ekstremnya.
#define AT_PEAK_MAX          20       // ukuran buffer puncak
#define AT_PEAK_BAND         1.0f     // degC, histeresis deteksi puncak
#define AT_SAMPLE_MS         1000UL   // periode proses autotune
#define AT_TIMEOUT_MS        3600000UL // batas 1 jam, lalu batal otomatis
#define AT_OVERTEMP_MARGIN   100.0f   // batal jika suhu > setpoint + margin
#define AT_MAX_BAD_READS     5        // batal jika sensor gagal N kali beruntun

// Ketahanan terhadap noise sensor (hasil uji simulasi: detektor mentah
// dengan band 1 C SALAH TOTAL pada noise >= 0.6 C, karena noise memicu
// "puncak" palsu di dekat setpoint dan menghasilkan Ku/Kp ngawur):
//  - suhu difilter EMA orde-1 sebelum dipakai relay & deteksi puncak.
//    alpha 0.2 pada sampel 1 s ~ konstanta waktu 4.5 s (setara filter 5 s
//    pada simulasi MATLAB). Konsekuensi: gain hasil tuning berlaku untuk
//    loop dengan sensor yang DIFILTER sama -> PID adaptive sebaiknya juga
//    memakai suhu terfilter/rata-rata.
//  - siklus pertama (transien pemanasan awal) dibuang, bukan dipakai hitung.
//  - hasil ditolak bila period antar-siklus tidak konsisten.
#define AT_FILTER_ALPHA        0.2f
#define AT_WARMUP_EXTREMES     2      // ekstrem awal yang dibuang (1 siklus)
#define AT_MAX_PERIOD_SPREAD   0.5f   // (Tmax-Tmin)/Tmean di atas ini = tolak hasil

static bool          atActive       = false;   // sedang autotune atau tidak
static float         atSetpoint     = 200.0;   // setpoint autotune (C)
static float         atHighOut      = 0.8;     // fan output saat ON  (0.0 - 1.0)
static float         atLowOut       = 0.0;     // fan output saat OFF (0.0 - 1.0)
static bool          atRelayState   = false;   // relay ON/OFF saat ini
static float         atPeaks[AT_PEAK_MAX];     // buffer suhu puncak
static unsigned long atPeakTimes[AT_PEAK_MAX]; // timestamp setiap puncak (ms)
static int           atPeakCount    = 0;       // jumlah puncak terdeteksi
static int           atCyclesNeeded = 3;       // minimal siklus sebelum hitung hasil
static float         atLastTemp     = 0;       // suhu sampel terakhir
static bool          atRising       = true;    // arah suhu saat ini

// Pelacak ekstrem berjalan + pengaman
static float         atExtreme      = 0;       // max (naik) / min (turun) sejak balik arah terakhir
static unsigned long atExtremeMs    = 0;       // kapan ekstrem itu terjadi
static bool          atHaveSample   = false;   // sudah ada sampel pertama?
static bool          atSkipFirst    = false;   // mulai dari suhu >= setpoint: "puncak" pertama palsu
static int           atWarmup       = AT_WARMUP_EXTREMES; // sisa ekstrem transien yang dibuang
static float         atFiltTemp     = 0;       // suhu terfilter (EMA) untuk relay & deteksi
static unsigned long atStartMs      = 0;
static uint8_t       atBadReads     = 0;
static const char   *atError        = "";      // alasan batal (kosong = tidak ada)
static char           atErrorBuf[160];          // buffer utk pesan error yang dibentuk dinamis

// Amplitudo osilasi minimal (degC) supaya hasil Ku/Kp/Ki/Kd dipercaya. Di
// bawah ini, amplitudo sudah terlalu dekat dengan histeresis deteksi puncak
// (AT_PEAK_BAND=1C) sehingga Ku = 4d/(pi*a) bisa meledak jadi puluhan kali
// lipat terlalu besar -- PID jadi saturasi hampir selalu (perilaku bang-bang,
// bukan PID halus) walau tampak "berhasil" selesai tanpa error.
#define AT_MIN_AMPLITUDE  3.0f

// Hasil autotune
static float resultKu = 0, resultTu = 0;
static float resultKp = 0, resultKi = 0, resultKd = 0;
static bool  atDone = false;

// Atur fan dalam persen (0-100), sekaligus update status untuk Arduino UNO
static void applyFan(int pct) {
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  int raw = (pct * 255) / 100;
  ledcWrite(PWM_CHANNEL, raw);
  fanRawValue = (uint8_t)raw;
  fanPct = (uint8_t)pct;
}

static int atPctFromFraction(float f) {
  return (int)(f * 100.0f + 0.5f);
}

void autotuneStart(float setpoint, float highOut, float lowOut) {
  atSetpoint     = setpoint;
  atHighOut      = highOut;
  atLowOut       = lowOut;
  atPeakCount    = 0;
  atLastTemp     = 0;
  atRising       = true;
  atHaveSample   = false;
  atSkipFirst    = false;
  atWarmup       = AT_WARMUP_EXTREMES;
  atFiltTemp     = 0;
  atExtreme      = 0;
  atExtremeMs    = 0;
  atBadReads     = 0;
  atError        = "";
  atDone         = false;
  resultKu = resultTu = resultKp = resultKi = resultKd = 0;
  atStartMs      = millis();

  atActive       = true;
  atRelayState   = true;                        // mulai dengan fan ON
  applyFan(atPctFromFraction(atHighOut));

  Serial.printf("[AUTOTUNE] Mulai, setpoint=%.1f C, high=%.0f%%, low=%.0f%%\n",
                atSetpoint, atHighOut * 100.0f, atLowOut * 100.0f);
}

void autotuneStop() {
  atActive = false;
  atRelayState = false;
  applyFan(0);
  Serial.println(F("[AUTOTUNE] Dibatalkan"));
}

// Batal karena kondisi tidak aman / gagal: fan mati, status "error"
static void autotuneAbort(const char *reason) {
  atActive = false;
  atRelayState = false;
  atDone = false;
  atError = reason;
  applyFan(0);
  Serial.print(F("[AUTOTUNE] GAGAL: "));
  Serial.println(reason);
}

void autotuneFinish() {
  // Rata-rata amplitude dari semua pasangan puncak berurutan.
  // Pasangan = (ekstrem ke-2i, ekstrem ke-2i+1); |selisih|/2 tidak
  // bergantung apakah pasangan dimulai dari puncak atau lembah.
  int pairs = atPeakCount / 2;
  float ampSum = 0;
  for (int i = 0; i < pairs; i++) {
    ampSum += fabsf(atPeaks[2 * i] - atPeaks[2 * i + 1]) / 2.0f;
  }

  // Rata-rata period Tu (jarak antar ekstrem sejenis), detik
  float tuSum = 0, tuMin = 1e9f, tuMax = 0;
  int tuCount = 0;
  for (int i = 0; i <= atPeakCount - 3; i += 2) {
    float p = (float)(atPeakTimes[i + 2] - atPeakTimes[i]) / 1000.0f;
    tuSum += p;
    if (p < tuMin) tuMin = p;
    if (p > tuMax) tuMax = p;
    tuCount++;
  }

  if (pairs == 0 || tuCount == 0) {
    autotuneAbort("data puncak tidak cukup untuk menghitung hasil");
    return;
  }
  float a  = ampSum / pairs;      // amplitude osilasi suhu (degC)
  float Tu = tuSum / tuCount;     // period osilasi (detik)

  // Tolak hasil bila period antar-siklus tidak konsisten (mis. puncak
  // palsu/terlewat akibat noise): lebih baik gagal daripada memberi gain ngawur.
  if (tuCount >= 2 && Tu > 0.0f && (tuMax - tuMin) / Tu > AT_MAX_PERIOD_SPREAD) {
    autotuneAbort("osilasi tidak konsisten (sensor terlalu noisy / sistem belum stabil), ulangi");
    return;
  }
  if (a <= 0.0f || Tu <= 0.0f) {
    autotuneAbort("amplitude/period osilasi tidak valid");
    return;
  }
  // Tolak bila amplitudo terlalu kecil -- Ku=4d/(pi*a) akan meledak dan
  // menghasilkan Kp/Ki/Kd yang membuat PID berperilaku bang-bang (saturasi
  // hampir selalu), bukan PID yang halus. Biasanya terjadi kalau setpoint
  // tuning terlalu dekat suhu ambient/osilasi belum sempat terbentuk bersih.
  if (a < AT_MIN_AMPLITUDE) {
    snprintf(atErrorBuf, sizeof(atErrorBuf),
      "osilasi suhu terlalu kecil untuk dipercaya (a=%.2f C, minimal %.1f C) -- "
      "coba setpoint tuning lebih tinggi (mendekati kondisi operasi nyata) "
      "atau naikkan Fan ON (%%)", a, AT_MIN_AMPLITUDE);
    autotuneAbort(atErrorBuf);
    return;
  }

  // d = relay half-amplitude. Dalam skala 0-1: d = (high-low)/2.
  // Rumus spek Ku = 4d / (pi * a/100) dengan d skala 0-1 sama persis
  // dengan Ku = 4*dPct / (pi*a) dengan dPct dalam persen fan (0-100),
  // yang dipakai di bawah -> Ku dalam [% PWM per degC].
  float dPct = (atHighOut - atLowOut) * 100.0f / 2.0f;

  // Ziegler-Nichols relay method (PID klasik)
  resultKu = (4.0f * dPct) / (PI * a);
  resultTu = Tu;
  resultKp = 0.6f * resultKu;
  float Ti = 0.5f   * Tu;
  float Td = 0.125f * Tu;
  resultKi = resultKp / Ti;
  resultKd = resultKp * Td;

  atDone       = true;
  atActive     = false;
  atRelayState = false;
  applyFan(0);

  Serial.println(F("[AUTOTUNE] Selesai!"));
  Serial.printf("  a=%.2f C  Ku=%.4f  Tu=%.2fs\n", a, resultKu, resultTu);
  Serial.printf("  Kp=%.4f  Ki=%.4f  Kd=%.4f\n", resultKp, resultKi, resultKd);
}

// Simpan satu ekstrem (puncak atas / bawah) lalu balik arah pelacakan
static void atRecordExtreme(float currentTemp, unsigned long nowMs) {
  if (atSkipFirst) {
    // Mulai dari suhu >= setpoint: "puncak atas" pertama hanyalah suhu awal, bukan osilasi
    atSkipFirst = false;
  } else if (atWarmup > 0) {
    // Siklus pertama = transien pemanasan dari suhu awal, bukan limit cycle: dibuang
    atWarmup--;
    Serial.printf("[AUTOTUNE] Ekstrem %s %.2f C diabaikan (transien awal)\n",
                  atRising ? "atas" : "bawah", atExtreme);
  } else if (atPeakCount < AT_PEAK_MAX) {
    atPeaks[atPeakCount]     = atExtreme;
    atPeakTimes[atPeakCount] = atExtremeMs;
    atPeakCount++;
    Serial.printf("[AUTOTUNE] Puncak %s #%d: %.2f C\n",
                  atRising ? "atas" : "bawah", atPeakCount, atExtreme);
  }
  atRising    = !atRising;
  atExtreme   = currentTemp;
  atExtremeMs = nowMs;
}

// Dipanggil dari loop() setiap 1 detik saat atActive = true
void autotuneProcess(float rawTemp) {
  unsigned long now = millis();
  atLastTemp = rawTemp;                 // untuk tampilan (suhu mentah)

  // 0. Pengaman (pakai suhu MENTAH supaya tidak tertunda oleh filter)
  if (now - atStartMs > AT_TIMEOUT_MS) {
    autotuneAbort("timeout: osilasi tidak terbentuk dalam 1 jam");
    return;
  }
  if (rawTemp > atSetpoint + AT_OVERTEMP_MARGIN) {
    autotuneAbort("suhu melebihi batas aman (setpoint + 100 C)");
    return;
  }

  // Filter EMA untuk relay & deteksi puncak (lihat catatan AT_FILTER_ALPHA)
  if (!atHaveSample) atFiltTemp = rawTemp;
  else               atFiltTemp += AT_FILTER_ALPHA * (rawTemp - atFiltTemp);
  float currentTemp = atFiltTemp;

  // 1. Relay switching
  atRelayState = (currentTemp < atSetpoint);
  applyFan(atPctFromFraction(atRelayState ? atHighOut : atLowOut));

  // 2. Deteksi puncak (ekstrem berjalan + histeresis AT_PEAK_BAND)
  if (!atHaveSample) {
    atHaveSample = true;
    atExtreme    = currentTemp;
    atExtremeMs  = now;
    atSkipFirst  = (currentTemp >= atSetpoint);
  } else if (atRising) {
    if (currentTemp > atExtreme) {
      atExtreme = currentTemp; atExtremeMs = now;
    } else if (currentTemp < atExtreme - AT_PEAK_BAND) {
      atRecordExtreme(currentTemp, now);            // puncak atas terdeteksi
    }
  } else {
    if (currentTemp < atExtreme) {
      atExtreme = currentTemp; atExtremeMs = now;
    } else if (currentTemp > atExtreme + AT_PEAK_BAND) {
      atRecordExtreme(currentTemp, now);            // puncak bawah terdeteksi
    }
  }

  // 3. Cukup siklus? (1 siklus = 2 puncak). Buffer penuh -> hitung dgn data yang ada.
  if (atPeakCount >= atCyclesNeeded * 2 || atPeakCount >= AT_PEAK_MAX) {
    autotuneFinish();
  }
}

// ============================================================
// PARAMETER PID (hasil "Terapkan ke PID" dari Relay Test)
// ============================================================
// Disimpan di NVS (Preferences) supaya bertahan setelah reboot. Firmware
// diagnostik ini belum punya loop PID; nilai ini dibaca oleh loop PID
// firmware utama (pidKp/pidKi/pidKd). Satuan: % PWM fan per degC.
// Default = DEFAULT_KP/KI/KD di config.h firmware utama (2.0 / 0.1 / 10.0).
static Preferences prefs;
static float   pidKp = 2.0f, pidKi = 0.1f, pidKd = 10.0f;
static uint8_t pidSource = 0;              // 0 = default, 1 = hasil relay test

static void pidLoad() {
  prefs.begin("pid", false);
  pidKp     = prefs.getFloat("kp", 2.0f);
  pidKi     = prefs.getFloat("ki", 0.1f);
  pidKd     = prefs.getFloat("kd", 10.0f);
  pidSource = prefs.getUChar("src", 0);
  prefs.end();
}

static void pidSave() {
  prefs.begin("pid", false);
  prefs.putFloat("kp", pidKp);
  prefs.putFloat("ki", pidKi);
  prefs.putFloat("kd", pidKd);
  prefs.putUChar("src", pidSource);
  prefs.end();
}

// ============================================================
// STEP TEST - ukur Dead Time (L), Time Constant (T), Gain (K)
// ============================================================
// Prosedur: fan 0% selama ST_BASELINE_SEC (ukur suhu dasar T0), lalu fan
// dinaikkan ke stStepPct (step), suhu direkam tiap 1 detik sampai steady
// state (atau timeout). Model FOPDT: G(s) = K e^(-Ls) / (T s + 1).
//
// Metode DUA TITIK (Smith): cari waktu (sejak step) saat kenaikan suhu
// mencapai 28,3% dan 63,2% dari dT = Tss - T0:
//     T = 1,5 (t63 - t28)        L = t63 - T        K = dT / stepPct
// Lebih tahan noise daripada garis singgung (metode tangen) karena
// tidak butuh turunan. Data dihaluskan moving-average 5 sampel TERPUSAT
// (tanpa lag) hanya saat menghitung, jadi L tidak bergeser.
#define ST_MAX_SAMPLES     3600       // 60 menit @ 1 s (int16 x 0,1 C = 7,2 KB)
#define ST_BASELINE_SEC    30         // fase baseline sebelum step
#define ST_MIN_STEP_SEC    150        // minimal durasi step sebelum cek steady
#define ST_MIN_DELTA_T     5.0f       // kenaikan minimal agar hasil bermakna (C)
#define ST_STEADY_ABS      0.3f       // steady: perubahan 60 s < max(ABS, REL*dT)
#define ST_STEADY_REL      0.004f

static bool     stActive     = false;
static bool     stDone       = false;
static const char *stError   = "";
static char     stWarn[96]   = "";
static int16_t  stTemp[ST_MAX_SAMPLES];    // suhu x10 (0,1 C per satuan)
static int      stCount      = 0;
static int      stMaxCount   = 1800;
static int      stStepPct    = 60;
static float    stTempLimit  = 500.0f;
static bool     stStepApplied = false;
static bool     stSteady     = false;
static uint8_t  stBadReads   = 0;
static float    stBaseDrift  = 0;
static float    stT0 = 0, stTss = 0, stDeltaT = 0;
static float    stK = 0, stL = 0, stTau = 0, stT283 = 0, stT632 = 0;

static inline float stRaw(int i) { return stTemp[i] / 10.0f; }

static float stMean(int from, int to) {   // rata-rata inklusif, batas dijepit
  if (from < 0) from = 0;
  if (to >= stCount) to = stCount - 1;
  if (to < from) return 0;
  float s = 0;
  for (int i = from; i <= to; i++) s += stRaw(i);
  return s / (float)(to - from + 1);
}

static float stSmooth(int i) { return stMean(i - 2, i + 2); }

// Waktu (detik dari awal rekaman, interpolasi linear) saat suhu terhaluskan
// pertama kali melewati T0 + thr; -1 jika tidak pernah.
static float stCross(float thr) {
  for (int j = ST_BASELINE_SEC; j < stCount; j++) {
    float y = stSmooth(j) - stT0;
    if (y >= thr) {
      float y0 = stSmooth(j - 1) - stT0;
      float frac = (y > y0) ? (thr - y0) / (y - y0) : 1.0f;
      if (frac < 0) frac = 0;
      if (frac > 1) frac = 1;
      return (float)(j - 1) + frac;
    }
  }
  return -1.0f;
}

static void stAddWarn(const char *msg) {
  size_t len = strlen(stWarn);
  if (len > 0 && len < sizeof(stWarn) - 3) { strcat(stWarn, "; "); len += 2; }
  strncat(stWarn, msg, sizeof(stWarn) - len - 1);
}

void stepStart(int pct, float tempLimit, int maxMin) {
  stStepPct   = pct;
  stTempLimit = tempLimit;
  stMaxCount  = maxMin * 60;
  if (stMaxCount > ST_MAX_SAMPLES) stMaxCount = ST_MAX_SAMPLES;
  stCount = 0; stStepApplied = false; stSteady = false; stBadReads = 0;
  stError = ""; stWarn[0] = '\0'; stDone = false; stBaseDrift = 0;
  stT0 = stTss = stDeltaT = stK = stL = stTau = stT283 = stT632 = 0;

  stActive = true;
  applyFan(0);                                   // fase baseline: fan OFF
  Serial.printf("[STEP] Mulai: baseline %ds @0%%, lalu step ke %d%% (batas %.0f C, maks %d menit)\n",
                ST_BASELINE_SEC, stStepPct, stTempLimit, maxMin);
}

void stepStop() {
  stActive = false;
  applyFan(0);
  Serial.println(F("[STEP] Dibatalkan"));
}

static void stAbort(const char *reason) {
  stActive = false;
  stDone = false;
  stError = reason;
  applyFan(0);
  Serial.print(F("[STEP] GAGAL: "));
  Serial.println(reason);
}

static bool stCompute() {
  int n = stCount;
  if (n < ST_BASELINE_SEC + 30) { stError = "data terlalu sedikit"; return false; }

  int tail = (n >= 30) ? 30 : n;
  stTss    = stMean(n - tail, n - 1);
  stDeltaT = stTss - stT0;
  if (stDeltaT < ST_MIN_DELTA_T) {
    stError = "kenaikan suhu terlalu kecil (<5 C): naikkan Fan STEP atau beri waktu lebih lama";
    return false;
  }

  float t28 = stCross(0.283f * stDeltaT);
  float t63 = stCross(0.632f * stDeltaT);
  if (t28 < 0 || t63 < 0) {
    stError = "respons tidak mencapai 63% dari kenaikan akhir";
    return false;
  }

  stT283 = t28 - ST_BASELINE_SEC;        // relatif terhadap saat step
  stT632 = t63 - ST_BASELINE_SEC;
  stTau  = 1.5f * (stT632 - stT283);
  stL    = stT632 - stTau;
  stK    = stDeltaT / (float)stStepPct;

  if (stTau <= 0) { stError = "time constant tidak valid (kurva tidak eksponensial)"; return false; }
  if (stL < 0)    { stL = 0; stAddWarn("dead time < resolusi 1 detik, dibulatkan ke 0"); }
  if (!stSteady)  stAddWarn("belum steady-state saat batas waktu, hasil perkiraan");
  if (stBaseDrift > 2.0f) stAddWarn("baseline suhu belum stabil, gain bisa bias");
  return true;
}

static void stFinish(bool steady) {
  stSteady = steady;
  applyFan(0);
  bool ok = stCompute();
  stActive = false;
  stDone = ok;
  if (ok) {
    Serial.println(F("[STEP] Selesai!"));
    Serial.printf("  T0=%.1f Tss=%.1f dT=%.1f C  K=%.4f C/%%  L=%.1fs  T=%.1fs\n",
                  stT0, stTss, stDeltaT, stK, stL, stTau);
  } else {
    Serial.print(F("[STEP] GAGAL: "));
    Serial.println(stError);
  }
}

// Dipanggil dari loop() setiap 1 detik saat stActive = true
void stepProcess(float raw) {
  if (raw > stTempLimit) { stAbort("suhu melebihi batas aman yang diset"); return; }
  if (stCount >= stMaxCount) { stFinish(false); return; }

  stTemp[stCount++] = (int16_t)lroundf(raw * 10.0f);

  // Akhir baseline: tetapkan T0 lalu berikan step
  if (!stStepApplied && stCount >= ST_BASELINE_SEC) {
    stT0        = stMean(ST_BASELINE_SEC - 10, ST_BASELINE_SEC - 1);
    stBaseDrift = fabsf(stT0 - stMean(0, 9));
    stStepApplied = true;
    applyFan(stStepPct);
    Serial.printf("[STEP] Baseline T0=%.1f C (drift %.1f C) -> STEP fan %d%%\n",
                  stT0, stBaseDrift, stStepPct);
  }

  // Deteksi steady state: rata-rata 20 s terakhir vs 20 s yang berakhir 60 s sebelumnya
  if (stStepApplied && stCount >= ST_BASELINE_SEC + ST_MIN_STEP_SEC && (stCount % 10) == 0) {
    float a = stMean(stCount - 20, stCount - 1);
    float b = stMean(stCount - 80, stCount - 61);
    float rise = a - stT0;
    float tol = ST_STEADY_REL * rise;
    if (tol < ST_STEADY_ABS) tol = ST_STEADY_ABS;
    if (rise >= ST_MIN_DELTA_T && fabsf(a - b) < tol) stFinish(true);
  }
}

// ============================================================
// UJI METODE - Adaptive PID closed-loop pada satu setpoint
// ============================================================
// Adaptive PID di sini SAMA PERSIS dengan pid_controller.cpp firmware
// utama (bukan varian lambda-adaptation di simulasi MATLAB, yang masih
// punya masalah osilasi belum tuntas):
//   - Gain scheduling 3-zona HANYA pada Kp (Ki/Kd tetap):
//       T < 200 C: Kp x1.3 | 200-400 C: Kp x1.0 | T > 400 C: Kp x0.7
//   - Anti-windup: integral di-clamp SEBELUM dipakai
//   - Derivative dari error mentah (tanpa filter), dt = 1 detik
// Memakai pidKp/pidKi/pidKd yang SEDANG AKTIF (hasil Relay Test, atau
// default jika belum pernah "Terapkan ke PID").
//
// Pompa oli & air dinyalakan otomatis selama uji (kondisi operasi
// nyata, supaya oli benar-benar menetes) dan dimatikan otomatis saat
// uji selesai/dibatalkan/gagal.
//
// Waktu setiap tetesan (elapsed detik sejak uji dimulai) dicatat supaya
// laju produksi (mL/jam) bisa dihitung nanti -- lihat /method/data.
#define MT_MAX_TIPS       200      // buffer waktu tetes (jika lebih, kelebihan tetap terhitung di total)
#define MT_I_MAX          100.0f
#define MT_I_MIN         -100.0f
#define MT_ZONE_COLD_MAX  200.0f
#define MT_ZONE_MID_MAX   400.0f

// Filter EMA (exponential moving average) pada suhu SEBELUM dipakai PID
// (error, gain scheduling, derivative) -- meredam noise/kuantisasi sensor
// MAX6675 (+-0.25 C per langkah) supaya tidak memicu lonjakan derivative
// yang tidak perlu. Alpha sama dengan yang dipakai Relay Test (AT_FILTER_
// ALPHA) supaya konsisten. CATATAN: ini meredam NOISE, bukan osilasi besar
// akibat gain PID yang salah -- itu diatasi lewat validasi amplitudo di
// Relay Test (AT_MIN_AMPLITUDE) dan tombol Reset PID, bukan di sini.
#define MT_FILTER_ALPHA   0.2f

static bool          mtActive       = false;
static bool          mtDone         = false;
static const char   *mtError        = "";
static float         mtSetpoint     = 0;
static unsigned long mtStartMs      = 0;
static unsigned long mtMaxMs        = 0;
static float         mtInteg        = 0;
static float         mtPrevErr      = 0;
static bool          mtFirstSample  = true;
static uint8_t       mtBadReads     = 0;
static float         mtLastTemp     = 0;      // suhu MENTAH (untuk tampilan & pengaman overtemp)
static float         mtFiltTemp     = 0;      // suhu terfilter EMA (dipakai PID)
static uint8_t       mtLastFanPct   = 0;
static float         mtLastKp = 0, mtLastKi = 0, mtLastKd = 0;   // Kp/Ki/Kd AKTIF (setelah gain schedule)
static const char   *mtLastZone     = "-";    // "cold" / "mid" / "hot"
static unsigned long mtTipCountBase = 0;      // tipCount saat uji mulai
static unsigned long mtTipTotal     = 0;      // total tetes baru sejak mulai (tidak dibatasi)
static float         mtTipElapsedS[MT_MAX_TIPS];
static int           mtTipLogCount  = 0;      // jumlah tersimpan di buffer (dibatasi MT_MAX_TIPS)
static double         mtSqErrSum    = 0;
static unsigned long mtSampleN      = 0;

// Satu uji termal/kontrol pada satu waktu; juga dipakai sebagai guard fan manual
static bool thermalBusy() { return atActive || stActive || mtActive; }

static void mtApplyGainSchedule(float measTemp, float &kp, float &ki, float &kd) {
  float mult;
  if (measTemp < MT_ZONE_COLD_MAX)      { mult = 1.3f; mtLastZone = "cold"; }
  else if (measTemp <= MT_ZONE_MID_MAX) { mult = 1.0f; mtLastZone = "mid";  }
  else                                   { mult = 0.7f; mtLastZone = "hot";  }
  kp = pidKp * mult;
  ki = pidKi;   // TIDAK dijadwal (sama seperti pid_controller.cpp)
  kd = pidKd;   // TIDAK dijadwal
  mtLastKp = kp; mtLastKi = ki; mtLastKd = kd;
}

// maxMin <= 0 berarti TANPA BATAS WAKTU (mtMaxMs=0): uji hanya berhenti
// lewat STOP manual atau pengaman (overtemp/sensor gagal). maxMin > 0
// berarti WAKTU TETAP: begitu durasi habis, uji dimatikan otomatis oleh
// mtFinish() -- mau tidak mau, tidak bisa diperpanjang tanpa uji baru.
void methodTestStart(float setpoint, int maxMin) {
  mtSetpoint    = setpoint;
  mtMaxMs       = (maxMin > 0) ? ((unsigned long)maxMin * 60000UL) : 0UL;
  mtInteg       = 0;
  mtPrevErr     = 0;
  mtFirstSample = true;
  mtBadReads    = 0;
  mtError       = "";
  mtDone        = false;
  mtTipLogCount = 0;
  mtTipTotal    = 0;
  mtSqErrSum    = 0;
  mtSampleN     = 0;
  mtLastKp = mtLastKi = mtLastKd = 0;
  mtLastZone    = "-";
  mtFiltTemp    = 0;

  noInterrupts();
  mtTipCountBase = tipCount;
  interrupts();
  mtStartMs = millis();

  // Kondisi operasi nyata: pompa oli & air ON supaya oli benar-benar menetes
  digitalWrite(PIN_OIL_PUMP, LOW);   oilState = true;
  digitalWrite(PIN_WATER_PUMP, LOW); waterState = true;

  mtActive = true;
  Serial.printf("[METODE] Mulai Adaptive PID, setpoint=%.1f C, maks %d menit\n", setpoint, maxMin);
}

static void mtStopActuators() {
  applyFan(0);
  digitalWrite(PIN_OIL_PUMP, HIGH);   oilState = false;
  digitalWrite(PIN_WATER_PUMP, HIGH); waterState = false;
}

void methodTestStop() {
  mtActive = false;
  mtStopActuators();
  Serial.println(F("[METODE] Dibatalkan"));
}

static void mtAbort(const char *reason) {
  mtActive = false;
  mtDone   = false;
  mtError  = reason;
  mtStopActuators();
  Serial.print(F("[METODE] GAGAL: "));
  Serial.println(reason);
}

static void mtFinish() {
  mtActive = false;
  mtDone   = true;
  mtStopActuators();
  Serial.println(F("[METODE] Selesai (durasi maksimum tercapai)"));
}

// Dipanggil dari loop() setiap 1 detik saat mtActive = true
void methodTestProcess(float raw) {
  // Pengaman overtemp pakai suhu MENTAH supaya tidak tertunda filter
  if (raw > mtSetpoint + AT_OVERTEMP_MARGIN) {
    mtAbort("suhu melebihi batas aman (setpoint + 100 C)");
    return;
  }
  unsigned long elapsedMs = millis() - mtStartMs;
  if (mtMaxMs > 0 && elapsedMs >= mtMaxMs) { mtFinish(); return; }   // mtMaxMs=0 -> tanpa batas

  mtLastTemp = raw;

  // Filter EMA sebelum dipakai PID (lihat catatan MT_FILTER_ALPHA)
  if (mtFirstSample) mtFiltTemp = raw;
  else                mtFiltTemp += MT_FILTER_ALPHA * (raw - mtFiltTemp);
  float meas = mtFiltTemp;

  float kp, ki, kd;
  mtApplyGainSchedule(meas, kp, ki, kd);

  float err = mtSetpoint - meas;

  mtInteg += ki * err;                  // dt = 1 detik (AT_SAMPLE_MS), tersirat *dt=1
  if (mtInteg > MT_I_MAX) mtInteg = MT_I_MAX;
  if (mtInteg < MT_I_MIN) mtInteg = MT_I_MIN;

  float deriv = mtFirstSample ? 0 : kd * (err - mtPrevErr);
  mtFirstSample = false;

  float u = kp * err + mtInteg + deriv;
  int pct = (int)lroundf(u);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  applyFan(pct);
  mtLastFanPct = (uint8_t)pct;
  mtPrevErr = err;

  mtSqErrSum += (double)err * (double)err;
  mtSampleN++;

  // Tetes baru sejak sampel terakhir -> catat waktu (detik sejak mulai uji)
  noInterrupts();
  unsigned long tipsNow = tipCount;
  interrupts();
  unsigned long newTotal = tipsNow - mtTipCountBase;
  if (newTotal > mtTipTotal) {
    unsigned long newTips = newTotal - mtTipTotal;
    float elapsedS = elapsedMs / 1000.0f;
    for (unsigned long i = 0; i < newTips && mtTipLogCount < MT_MAX_TIPS; i++) {
      mtTipElapsedS[mtTipLogCount++] = elapsedS;
    }
    mtTipTotal = newTotal;
  }
}

// ------------------------------------------------------------
// Helper HTTP JSON
// ------------------------------------------------------------
static void sendJson(const String &json, int code = 200) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(code, "application/json", json);
}

// ------------------------------------------------------------
// Handler rute HTTP
// ------------------------------------------------------------
static void handleRoot();

static void handlePing() {
  sendJson(F("{\"ok\":true}"));
}

static void handleTestTemp() {
  float t = readTemperature();
  char buf[192];

  // Cetak ke Serial juga saat user pencet "Baca Suhu" di web
  Serial.print(F("[TEMP/WEB] Permintaan dari browser -> "));

  if (isnan(t)) {
    Serial.println(F("ERROR NaN"));
    snprintf(buf, sizeof(buf),
             "{\"type\":\"TEMP\",\"value\":null,\"error\":\"Sensor MAX6675 tidak terhubung / thermocouple putus\"}");
  } else if (t < 0 || t > 1000) {
    Serial.print(F("OUT-OF-RANGE: "));
    Serial.print(t, 2);
    Serial.println(F(" C"));
    snprintf(buf, sizeof(buf),
             "{\"type\":\"TEMP\",\"value\":%.1f,\"error\":\"Suhu di luar rentang normal (0-1000C), cek wiring/sensor\"}", t);
  } else {
    Serial.print(t, 2);
    Serial.println(F(" C  [OK]"));
    snprintf(buf, sizeof(buf), "{\"type\":\"TEMP\",\"value\":%.1f}", t);
  }
  sendJson(buf);
}

static void handleTipGet() {
  noInterrupts();
  unsigned long c = tipCount;
  interrupts();
  char buf[80];
  snprintf(buf, sizeof(buf), "{\"type\":\"TIP\",\"count\":%lu}", c);
  sendJson(buf);
}

static void handleTipReset() {
  noInterrupts();
  tipCount = 0;
  interrupts();
  sendJson(F("{\"type\":\"TIP\",\"count\":0}"));
}

// Relay active-LOW: ON = LOW, OFF = HIGH
static void handleOilOn() {
  if (mtActive) { sendJson(F("{\"type\":\"OIL\",\"error\":\"uji metode mengendalikan pompa otomatis\"}"), 409); return; }
  digitalWrite(PIN_OIL_PUMP, LOW);
  oilState = true;
  Serial.println(F("[OIL] ON"));
  sendJson(F("{\"type\":\"OIL\",\"state\":true}"));
}
static void handleOilOff() {
  if (mtActive) { sendJson(F("{\"type\":\"OIL\",\"error\":\"uji metode mengendalikan pompa otomatis\"}"), 409); return; }
  digitalWrite(PIN_OIL_PUMP, HIGH);
  oilState = false;
  Serial.println(F("[OIL] OFF"));
  sendJson(F("{\"type\":\"OIL\",\"state\":false}"));
}

static void handleWaterOn() {
  if (mtActive) { sendJson(F("{\"type\":\"WATER\",\"error\":\"uji metode mengendalikan pompa otomatis\"}"), 409); return; }
  digitalWrite(PIN_WATER_PUMP, LOW);
  waterState = true;
  Serial.println(F("[WATER] ON"));
  sendJson(F("{\"type\":\"WATER\",\"state\":true}"));
}
static void handleWaterOff() {
  if (mtActive) { sendJson(F("{\"type\":\"WATER\",\"error\":\"uji metode mengendalikan pompa otomatis\"}"), 409); return; }
  digitalWrite(PIN_WATER_PUMP, HIGH);
  waterState = false;
  Serial.println(F("[WATER] OFF"));
  sendJson(F("{\"type\":\"WATER\",\"state\":false}"));
}

static void handleFan(int pct) {
  // Guard: saat Step/Relay Test aktif, fan dikendalikan uji -- tolak perubahan manual
  if (thermalBusy()) {
    sendJson(F("{\"type\":\"FAN\",\"error\":\"uji termal (step/relay) sedang berjalan\"}"), 409);
    return;
  }
  int raw = (pct * 255) / 100;
  ledcWrite(PWM_CHANNEL, raw);
  fanRawValue = (uint8_t)raw;
  fanPct = (uint8_t)pct;
  Serial.print(F("[FAN] "));
  Serial.print(pct);
  Serial.print(F("%  raw="));
  Serial.println(raw);
  char buf[80];
  snprintf(buf, sizeof(buf), "{\"type\":\"FAN\",\"pct\":%d,\"raw\":%d}", pct, raw);
  sendJson(buf);
}

static void handleTestAll() {
  // Guard: test ini menyalakan fan -- tolak saat Step/Relay Test aktif
  if (thermalBusy()) {
    sendJson(F("{\"type\":\"ALL\",\"error\":\"uji termal (step/relay) sedang berjalan\"}"), 409);
    return;
  }
  int fanRaw = (50 * 255) / 100;
  ledcWrite(PWM_CHANNEL, fanRaw);
  fanRawValue = fanRaw; fanPct = 50;
  Serial.println(F("[ALL] Fan 50%..."));
  delay(1000);
  ledcWrite(PWM_CHANNEL, 0);
  fanRawValue = 0; fanPct = 0;

  digitalWrite(PIN_OIL_PUMP, LOW); oilState = true;
  Serial.println(F("[ALL] Oil ON..."));
  delay(1000);
  digitalWrite(PIN_OIL_PUMP, HIGH); oilState = false;

  digitalWrite(PIN_WATER_PUMP, LOW); waterState = true;
  Serial.println(F("[ALL] Water ON..."));
  delay(1000);
  digitalWrite(PIN_WATER_PUMP, HIGH); waterState = false;

  float t = readTemperature();
  bool tempOk = !isnan(t) && t >= 0 && t <= 1000;

  Serial.print(F("[ALL] Suhu: "));
  if (isnan(t)) Serial.println(F("NaN (ERROR)"));
  else { Serial.print(t, 2); Serial.println(F(" C")); }

  noInterrupts();
  unsigned long tips = tipCount;
  interrupts();

  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"type\":\"ALL\",\"tempOk\":%s,\"temp\":%.1f,\"tips\":%lu}",
           tempOk ? "true" : "false", isnan(t) ? -1.0f : t, tips);
  sendJson(buf);
}

// ------------------------------------------------------------
// Handler AUTOTUNE
// (server.on() tanpa method menerima GET maupun POST)
// ------------------------------------------------------------
static void atSendError(const char *msg, int code) {
  char buf[160];
  snprintf(buf, sizeof(buf), "{\"type\":\"AT\",\"status\":\"error\",\"error\":\"%s\"}", msg);
  sendJson(buf, code);
}

// GET /autotune/start?sp=200&high=80&low=0
static void handleAtStart() {
  if (atActive) {
    atSendError("relay test sudah berjalan", 409);
    return;
  }
  if (stActive) {
    atSendError("step test sedang berjalan, tunggu selesai atau tekan STOP", 409);
    return;
  }
  if (mtActive) {
    atSendError("uji metode sedang berjalan, tunggu selesai atau tekan STOP", 409);
    return;
  }

  float sp   = server.hasArg("sp")   ? server.arg("sp").toFloat()   : 200.0f;
  float high = server.hasArg("high") ? server.arg("high").toFloat() : 80.0f;
  float low  = server.hasArg("low")  ? server.arg("low").toFloat()  : 0.0f;

  if (sp < 50.0f || sp > 800.0f)   { atSendError("setpoint harus 50-800 C", 400); return; }
  if (high < 10.0f || high > 100.0f) { atSendError("Fan ON harus 10-100 %", 400); return; }
  if (low < 0.0f || low > 50.0f)   { atSendError("Fan OFF harus 0-50 %", 400); return; }
  if (low >= high)                 { atSendError("Fan OFF harus lebih kecil dari Fan ON", 400); return; }

  // Jangan menyalakan fan HIGH kalau sensor tidak terbaca
  float t = readTemperature();
  if (isnan(t) || t < 0 || t > 1000) {
    atSendError("sensor MAX6675 tidak terbaca, autotune tidak dimulai", 400);
    return;
  }

  autotuneStart(sp, high / 100.0f, low / 100.0f);

  char buf[128];
  snprintf(buf, sizeof(buf),
           "{\"type\":\"AT\",\"status\":\"started\",\"sp\":%.1f,\"high\":%.0f,\"low\":%.0f}",
           sp, high, low);
  sendJson(buf);
}

// GET /autotune/stop
static void handleAtStop() {
  if (atActive) autotuneStop();   // jangan sentuh fan manual bila autotune tidak aktif
  sendJson(F("{\"type\":\"AT\",\"status\":\"stopped\"}"));
}

// GET /autotune/status
static void handleAtStatus() {
  char buf[420];

  if (atActive) {
    int cycles = atPeakCount / 2;
    if (cycles > atCyclesNeeded) cycles = atCyclesNeeded;

    int n = snprintf(buf, sizeof(buf),
      "{\"type\":\"AT\",\"status\":\"running\",\"peaks\":%d,\"temp\":%.1f,\"relay\":%s,"
      "\"progress\":\"%d/%d siklus\",\"cycles\":%d,\"need\":%d,\"sp\":%.1f,\"pk\":[",
      atPeakCount, atLastTemp, atRelayState ? "true" : "false",
      cycles, atCyclesNeeded, cycles, atCyclesNeeded, atSetpoint);

    // maksimal 6 puncak terakhir
    int first = atPeakCount > 6 ? atPeakCount - 6 : 0;
    for (int i = first; i < atPeakCount && n < (int)sizeof(buf) - 24; i++) {
      n += snprintf(buf + n, sizeof(buf) - n, "%s%.1f", (i > first) ? "," : "", atPeaks[i]);
    }
    snprintf(buf + n, sizeof(buf) - n, "]}");
  } else if (atError[0] != '\0') {
    snprintf(buf, sizeof(buf), "{\"type\":\"AT\",\"status\":\"error\",\"error\":\"%s\"}", atError);
  } else if (atDone) {
    snprintf(buf, sizeof(buf),
      "{\"type\":\"AT\",\"status\":\"done\",\"Ku\":%.6f,\"Tu\":%.2f,"
      "\"Kp\":%.6f,\"Ki\":%.6f,\"Kd\":%.6f}",
      resultKu, resultTu, resultKp, resultKi, resultKd);
  } else {
    snprintf(buf, sizeof(buf), "{\"type\":\"AT\",\"status\":\"idle\"}");
  }
  sendJson(buf);
}

// ------------------------------------------------------------
// Auto-refresh suhu di browser (tiap 5 detik): sama dengan /test/temp
// tetapi TANPA cetak ke Serial, supaya log Serial 2 detik tidak berisik.
// ------------------------------------------------------------
static void handleTempAuto() {
  float t = readTemperature();
  char buf[192];
  if (isnan(t)) {
    snprintf(buf, sizeof(buf),
             "{\"type\":\"TEMP\",\"value\":null,\"error\":\"Sensor MAX6675 tidak terhubung / thermocouple putus\"}");
  } else if (t < 0 || t > 1000) {
    snprintf(buf, sizeof(buf),
             "{\"type\":\"TEMP\",\"value\":%.1f,\"error\":\"Suhu di luar rentang normal (0-1000C), cek wiring/sensor\"}", t);
  } else {
    snprintf(buf, sizeof(buf), "{\"type\":\"TEMP\",\"value\":%.1f}", t);
  }
  sendJson(buf);
}

// ------------------------------------------------------------
// Handler STEP TEST
// ------------------------------------------------------------
static void stSendError(const char *msg, int code) {
  char buf[200];
  snprintf(buf, sizeof(buf), "{\"type\":\"ST\",\"status\":\"error\",\"error\":\"%s\"}", msg);
  sendJson(buf, code);
}

// Tambahkan trace suhu (maks ~80 titik, di-downsample) ke buffer JSON
static int stAppendTrace(char *buf, int n, int cap) {
  int stride = (stCount + 79) / 80;
  if (stride < 1) stride = 1;
  n += snprintf(buf + n, cap - n, ",\"dt\":%d,\"ts\":%d,\"tr\":[", stride, ST_BASELINE_SEC);
  for (int i = 0; i < stCount && n < cap - 16; i += stride) {
    n += snprintf(buf + n, cap - n, "%s%.1f", (i > 0) ? "," : "", stRaw(i));
  }
  n += snprintf(buf + n, cap - n, "]");
  return n;
}

// GET /step/start?pct=60&limit=500&min=30
static void handleStStart() {
  if (stActive) { stSendError("step test sudah berjalan", 409); return; }
  if (atActive) { stSendError("relay test sedang berjalan, tunggu selesai atau tekan STOP", 409); return; }
  if (mtActive) { stSendError("uji metode sedang berjalan, tunggu selesai atau tekan STOP", 409); return; }

  float pct   = server.hasArg("pct")   ? server.arg("pct").toFloat()   : 60.0f;
  float limit = server.hasArg("limit") ? server.arg("limit").toFloat() : 500.0f;
  float mins  = server.hasArg("min")   ? server.arg("min").toFloat()   : 30.0f;

  if (pct < 10.0f || pct > 100.0f)     { stSendError("Fan STEP harus 10-100 %", 400); return; }
  if (limit < 50.0f || limit > 800.0f) { stSendError("Batas suhu harus 50-800 C", 400); return; }
  if (mins < 5.0f || mins > 60.0f)     { stSendError("Durasi maksimum harus 5-60 menit", 400); return; }

  float t = readTemperature();
  if (isnan(t) || t < 0 || t > 1000) {
    stSendError("sensor MAX6675 tidak terbaca, step test tidak dimulai", 400);
    return;
  }
  if (t >= limit) {
    stSendError("suhu awal sudah di atas batas suhu aman", 400);
    return;
  }

  stepStart((int)(pct + 0.5f), limit, (int)(mins + 0.5f));

  char buf[128];
  snprintf(buf, sizeof(buf),
           "{\"type\":\"ST\",\"status\":\"started\",\"pct\":%d,\"limit\":%.0f,\"min\":%d}",
           stStepPct, stTempLimit, stMaxCount / 60);
  sendJson(buf);
}

// GET /step/stop
static void handleStStop() {
  if (stActive) stepStop();       // jangan sentuh fan manual bila tidak aktif
  sendJson(F("{\"type\":\"ST\",\"status\":\"stopped\"}"));
}

// GET /step/status
static void handleStStatus() {
  char buf[900];
  int n;

  if (stActive) {
    n = snprintf(buf, sizeof(buf),
      "{\"type\":\"ST\",\"status\":\"running\",\"phase\":\"%s\",\"t\":%d,\"max\":%d,"
      "\"temp\":%.1f,\"T0\":%.1f,\"fan\":%d,\"pct\":%d",
      stStepApplied ? "step" : "baseline", stCount, stMaxCount,
      stCount > 0 ? stRaw(stCount - 1) : 0.0f, stT0, stStepApplied ? stStepPct : 0, stStepPct);
    n = stAppendTrace(buf, n, (int)sizeof(buf) - 4);
    snprintf(buf + n, sizeof(buf) - n, "}");
  } else if (stError[0] != '\0') {
    snprintf(buf, sizeof(buf), "{\"type\":\"ST\",\"status\":\"error\",\"error\":\"%s\"}", stError);
  } else if (stDone) {
    n = snprintf(buf, sizeof(buf),
      "{\"type\":\"ST\",\"status\":\"done\",\"K\":%.5f,\"L\":%.2f,\"T\":%.2f,"
      "\"T0\":%.1f,\"Tss\":%.1f,\"dT\":%.1f,\"t283\":%.1f,\"t632\":%.1f,"
      "\"steady\":%s,\"pct\":%d,\"warn\":\"%s\"",
      stK, stL, stTau, stT0, stTss, stDeltaT, stT283, stT632,
      stSteady ? "true" : "false", stStepPct, stWarn);
    n = stAppendTrace(buf, n, (int)sizeof(buf) - 4);
    snprintf(buf + n, sizeof(buf) - n, "}");
  } else {
    snprintf(buf, sizeof(buf), "{\"type\":\"ST\",\"status\":\"idle\"}");
  }
  sendJson(buf);
}

// ------------------------------------------------------------
// Handler PID: terapkan hasil Relay Test ke parameter PID (disimpan NVS)
// ------------------------------------------------------------
static void sendPidJson(const char *status) {
  char buf[200];
  snprintf(buf, sizeof(buf),
           "{\"type\":\"PID\",\"status\":\"%s\",\"Kp\":%.6f,\"Ki\":%.6f,\"Kd\":%.6f,\"src\":\"%s\"}",
           status, pidKp, pidKi, pidKd, pidSource == 1 ? "relay" : "default");
  sendJson(buf);
}

// GET /pid/apply  -> pakai hasil relay test TERAKHIR yang ada di ESP32
// (bukan angka dari browser, supaya tidak bisa salah ketik/tertukar)
static void handlePidApply() {
  if (atActive || !atDone || !(resultKp > 0.0f)) {
    sendJson(F("{\"type\":\"PID\",\"status\":\"error\",\"error\":\"belum ada hasil relay test yang valid\"}"), 409);
    return;
  }
  pidKp = resultKp; pidKi = resultKi; pidKd = resultKd;
  pidSource = 1;
  pidSave();
  Serial.printf("[PID] Diterapkan & disimpan: Kp=%.4f Ki=%.4f Kd=%.4f\n", pidKp, pidKi, pidKd);
  sendPidJson("applied");
}

// GET /pid/reset -> kembalikan Kp/Ki/Kd ke default pabrik (2.0/0.1/10.0)
static void handlePidReset() {
  if (mtActive) {
    sendJson(F("{\"type\":\"PID\",\"status\":\"error\",\"error\":\"uji metode sedang memakai gain ini, stop dulu\"}"), 409);
    return;
  }
  pidKp = 2.0f; pidKi = 0.1f; pidKd = 10.0f;
  pidSource = 0;
  pidSave();
  Serial.println(F("[PID] Direset ke default: Kp=2.0 Ki=0.1 Kd=10.0"));
  sendPidJson("reset");
}

// GET /pid/get
static void handlePidGet() {
  sendPidJson("ok");
}

// ------------------------------------------------------------
// Jalur data untuk laptop (logger Python, tanpa SD card)
// ------------------------------------------------------------
// GET /data/live -> satu paket data ringan; dipanggil logger tiap ~1 detik.
static void handleDataLive() {
  float t = readTemperature();
  bool tempOk = !isnan(t) && t >= 0 && t <= 1000;
  noInterrupts();
  unsigned long tips = tipCount;
  interrupts();

  const char *mode = atActive ? "relay" : (stActive ? "step" : (mtActive ? "method" : "idle"));
  float sp = atActive ? atSetpoint : (mtActive ? mtSetpoint : 0.0f);
  char tempStr[16];
  if (tempOk) snprintf(tempStr, sizeof(tempStr), "%.2f", t);
  else        snprintf(tempStr, sizeof(tempStr), "null");

  // Kp/Ki/Kd/zona AKTIF hanya berarti saat mode=="method" (Adaptive PID sungguhan
  // berjalan); di luar itu dilaporkan 0/"-" apa adanya, bukan nilai basi dari uji
  // sebelumnya, supaya tidak salah dibaca sebagai kondisi saat ini.
  float mKp = mtActive ? mtLastKp : 0.0f;
  float mKi = mtActive ? mtLastKi : 0.0f;
  float mKd = mtActive ? mtLastKd : 0.0f;
  const char *mZone = mtActive ? mtLastZone : "-";

  char buf[320];
  snprintf(buf, sizeof(buf),
           "{\"ms\":%lu,\"temp\":%s,\"fan\":%d,\"oil\":%d,\"water\":%d,\"tips\":%lu,"
           "\"mode\":\"%s\",\"relay\":%s,\"sp\":%.1f,\"stn\":%d,"
           "\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f,\"zone\":\"%s\"}",
           millis(), tempStr, (int)fanPct, oilState ? 1 : 0, waterState ? 1 : 0, tips,
           mode, (atActive && atRelayState) ? "true" : "false", sp, stCount,
           mKp, mKi, mKd, mZone);
  sendJson(buf);
}

// GET /step/data -> seluruh rekaman Step Test terakhir (CSV, 1 sampel/detik).
// Dikirim bertahap (chunked) supaya tidak butuh buffer besar di RAM.
// Data ada di RAM sampai Step Test berikutnya dimulai atau ESP32 di-reboot.
static void handleStData() {
  if (stCount == 0) {
    sendJson(F("{\"type\":\"ST\",\"status\":\"error\",\"error\":\"belum ada data step test\"}"), 404);
    return;
  }
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("t_s,temp_c,fan_pct\n");

  char chunk[512];
  int n = 0;
  for (int i = 0; i < stCount; i++) {
    int fan = (i < ST_BASELINE_SEC || !stStepApplied) ? 0 : stStepPct;
    n += snprintf(chunk + n, sizeof(chunk) - n, "%d,%.1f,%d\n", i, stRaw(i), fan);
    if (n > (int)sizeof(chunk) - 40) {      // sisakan ruang untuk satu baris lagi
      server.sendContent(chunk, n);
      n = 0;
    }
  }
  if (n > 0) server.sendContent(chunk, n);
  server.sendContent("");                   // akhiri chunked
}

// ------------------------------------------------------------
// Handler UJI METODE
// ------------------------------------------------------------
static void mtSendError(const char *msg, int code) {
  char buf[200];
  snprintf(buf, sizeof(buf), "{\"type\":\"MT\",\"status\":\"error\",\"error\":\"%s\"}", msg);
  sendJson(buf, code);
}

// GET /method/start?sp=300&min=60
static void handleMtStart() {
  if (mtActive) { mtSendError("uji metode sudah berjalan", 409); return; }
  if (atActive) { mtSendError("relay test sedang berjalan, tunggu selesai atau tekan STOP", 409); return; }
  if (stActive) { mtSendError("step test sedang berjalan, tunggu selesai atau tekan STOP", 409); return; }

  float sp   = server.hasArg("sp")  ? server.arg("sp").toFloat()  : 0.0f;
  float mins = server.hasArg("min") ? server.arg("min").toFloat() : 60.0f;

  // min <= 0 => TANPA BATAS WAKTU (berhenti manual lewat STOP). Selain itu
  // (waktu tetap) harus 5-240 menit.
  bool unlimited = (mins <= 0.0f);
  int maxMin = unlimited ? 0 : (int)(mins + 0.5f);

  if (sp < 50.0f || sp > 800.0f) { mtSendError("setpoint harus 50-800 C", 400); return; }
  if (!unlimited && (mins < 5.0f || mins > 240.0f)) {
    mtSendError("Durasi maksimum harus 5-240 menit (atau 0 = tanpa batas waktu)", 400);
    return;
  }

  float t = readTemperature();
  if (isnan(t) || t < 0 || t > 1000) {
    mtSendError("sensor MAX6675 tidak terbaca, uji tidak dimulai", 400);
    return;
  }
  if (t >= sp + AT_OVERTEMP_MARGIN) {
    mtSendError("suhu awal sudah di atas batas suhu aman untuk setpoint ini", 400);
    return;
  }

  methodTestStart(sp, maxMin);

  char buf[128];
  snprintf(buf, sizeof(buf), "{\"type\":\"MT\",\"status\":\"started\",\"sp\":%.1f,\"min\":%d}",
           sp, maxMin);
  sendJson(buf);
}

// GET /method/stop
static void handleMtStop() {
  if (mtActive) methodTestStop();   // jangan sentuh fan/pompa bila uji tidak aktif
  sendJson(F("{\"type\":\"MT\",\"status\":\"stopped\"}"));
}

// GET /method/status
static void handleMtStatus() {
  char buf[600];
  int n;

  if (mtActive) {
    unsigned long elapsedS = (millis() - mtStartMs) / 1000UL;
    unsigned long maxS = mtMaxMs / 1000UL;
    n = snprintf(buf, sizeof(buf),
      "{\"type\":\"MT\",\"status\":\"running\",\"t\":%lu,\"max\":%lu,\"temp\":%.1f,\"sp\":%.1f,"
      "\"fan\":%d,\"tips\":%lu,\"recentTips\":[",
      elapsedS, maxS, mtLastTemp, mtSetpoint, mtLastFanPct, mtTipTotal);
    int first = mtTipLogCount > 6 ? mtTipLogCount - 6 : 0;
    for (int i = first; i < mtTipLogCount && n < (int)sizeof(buf) - 24; i++) {
      n += snprintf(buf + n, sizeof(buf) - n, "%s%.0f", (i > first) ? "," : "", mtTipElapsedS[i]);
    }
    snprintf(buf + n, sizeof(buf) - n, "]}");
  } else if (mtError[0] != '\0') {
    snprintf(buf, sizeof(buf), "{\"type\":\"MT\",\"status\":\"error\",\"error\":\"%s\"}", mtError);
  } else if (mtDone) {
    float rmsErr = mtSampleN > 0 ? sqrtf((float)(mtSqErrSum / mtSampleN)) : 0.0f;
    snprintf(buf, sizeof(buf),
      "{\"type\":\"MT\",\"status\":\"done\",\"sp\":%.1f,\"durS\":%lu,\"tips\":%lu,"
      "\"rmsErr\":%.2f,\"volumeMl\":%.2f}",
      mtSetpoint, mtMaxMs / 1000UL, mtTipTotal, rmsErr, mtTipTotal * VOLUME_PER_TIP_ML);
  } else {
    snprintf(buf, sizeof(buf), "{\"type\":\"MT\",\"status\":\"idle\"}");
  }
  sendJson(buf);
}

// GET /method/data -> waktu tetes (elapsed detik sejak mulai uji), CSV.
static void handleMtData() {
  if (mtTipLogCount == 0) {
    sendJson(F("{\"type\":\"MT\",\"status\":\"error\",\"error\":\"belum ada data tetes pada uji metode terakhir\"}"), 404);
    return;
  }
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("tip_index,elapsed_s\n");

  char chunk[256];
  int n = 0;
  for (int i = 0; i < mtTipLogCount; i++) {
    n += snprintf(chunk + n, sizeof(chunk) - n, "%d,%.0f\n", i + 1, mtTipElapsedS[i]);
    if (n > (int)sizeof(chunk) - 24) { server.sendContent(chunk, n); n = 0; }
  }
  if (n > 0) server.sendContent(chunk, n);
  server.sendContent("");
}

// ------------------------------------------------------------
// Dashboard HTML (PROGMEM) - v3: card 7 (Step Test), 8 (Relay Test) & 9 (Uji Metode)
// ------------------------------------------------------------
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

static void handleRoot() {
  server.send_P(200, "text/html", HTML);
}

// ------------------------------------------------------------
// Kirim status ke Arduino UNO via Serial2
// ------------------------------------------------------------
static void sendStatusToArduino() {
  noInterrupts();
  unsigned long tips = tipCount;
  interrupts();
  float volume = tips * VOLUME_PER_TIP_ML;

  const char *statusStr = (fanRawValue > 0 || oilState || waterState) ? "RUN" : "IDLE";

  char buf[160];
  snprintf(buf, sizeof(buf),
           "{\"t\":%.1f,\"sp\":%.1f,\"pw\":%d,\"v\":%.2f,\"oil\":%d,\"water\":%d,\"s\":\"%s\"}\n",
           lastGoodTemp, atActive ? atSetpoint : (float)DEFAULT_SETPOINT, fanPct, volume,
           oilState ? 1 : 0, waterState ? 1 : 0, statusStr);

  Serial2.print(buf);
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n[esp32] Pyrolysis firmware booting..."));
  Serial.println(F("================================================="));
  Serial.println(F("  Serial Monitor Log Aktif"));
  Serial.println(F("  Format: [TEMP] #<n>  uptime=<s>s  ->  <suhu> C"));
  Serial.println(F("  Interval: setiap 2 detik"));
  Serial.println(F("=================================================\n"));

  // 1. Relay OFF dan Fan PWM 0 PERTAMA
  digitalWrite(PIN_OIL_PUMP, HIGH);
  digitalWrite(PIN_WATER_PUMP, HIGH);
  pinMode(PIN_OIL_PUMP, OUTPUT);
  pinMode(PIN_WATER_PUMP, OUTPUT);
  pinMode(PIN_FAN_PWM, OUTPUT);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PIN_FAN_PWM, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);

  // 2. Init HSPI untuk MAX6675
  pinMode(PIN_MAX6675_CS, OUTPUT);
  digitalWrite(PIN_MAX6675_CS, HIGH);
  delay(250);

  // 4. Attach interrupt tipping bucket
  pinMode(PIN_TIPPING, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_TIPPING), onTip, FALLING);

  // 4b. Muat parameter PID tersimpan (NVS)
  pidLoad();
  Serial.printf("[PID] Aktif: Kp=%.4f Ki=%.4f Kd=%.4f (%s)\n",
                pidKp, pidKi, pidKd, pidSource == 1 ? "hasil relay test" : "default");

  // 5. Start Serial2 ke Arduino UNO
  Serial2.begin(9600, SERIAL_8N1, PIN_SERIAL2_RX, PIN_SERIAL2_TX);

  // 6. Start WiFi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

  // 7. Register semua routes
  server.on("/", handleRoot);
  server.on("/ping", handlePing);
  server.on("/test/temp", handleTestTemp);
  server.on("/tip/get", handleTipGet);
  server.on("/tip/reset", handleTipReset);
  server.on("/oil/on", handleOilOn);
  server.on("/oil/off", handleOilOff);
  server.on("/water/on", handleWaterOn);
  server.on("/water/off", handleWaterOff);
  server.on("/test/all", handleTestAll);
  server.on("/autotune/start", handleAtStart);
  server.on("/autotune/stop", handleAtStop);
  server.on("/autotune/status", handleAtStatus);
  server.on("/step/start", handleStStart);
  server.on("/step/stop", handleStStop);
  server.on("/step/status", handleStStatus);
  server.on("/pid/apply", handlePidApply);
  server.on("/pid/reset", handlePidReset);
  server.on("/pid/get", handlePidGet);
  server.on("/temp/auto", handleTempAuto);
  server.on("/data/live", handleDataLive);
  server.on("/step/data", handleStData);
  server.on("/method/start", handleMtStart);
  server.on("/method/stop", handleMtStop);
  server.on("/method/status", handleMtStatus);
  server.on("/method/data", handleMtData);

  static const int fanPercents[] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
  for (int i = 0; i < 11; i++) {
    int pct = fanPercents[i];
    String path = "/fan/" + String(pct);
    server.on(path, [pct]() { handleFan(pct); });
  }

  // 8. Start web server
  server.begin();

  // 9. Print info ke Serial Monitor
  Serial.print(F("[esp32] AP SSID: "));
  Serial.println(WIFI_SSID);
  Serial.print(F("[esp32] IP address: "));
  Serial.println(WiFi.softAPIP());
  Serial.println(F("[esp32] Serial2 -> Arduino UNO siap (9600 baud)"));
  Serial.println(F("[esp32] Step test : /step/start?pct=60&limit=500&min=30 | /step/stop | /step/status"));
  Serial.println(F("[esp32] Relay test: /autotune/start?sp=200&high=80&low=0 | /autotune/stop | /autotune/status"));
  Serial.println(F("[esp32] PID       : /pid/apply | /pid/get"));
  Serial.println(F("[esp32] Laptop    : /data/live | /step/data (logger Python, lihat laptop/logger.py)"));
  Serial.println(F("[esp32] Ready. Buka IP di atas lewat browser.\n"));

  // Baca suhu pertama kali langsung saat boot
  Serial.println(F("[TEMP] Pembacaan awal saat boot:"));
  logTempToSerial();
}

// ------------------------------------------------------------
// Loop
// ------------------------------------------------------------
void loop() {
  server.handleClient();

  // Autotune: proses relay + deteksi puncak setiap AT_SAMPLE_MS (1 detik)
  static unsigned long lastAtMs = 0;
  if (atActive) {
    unsigned long atNow = millis();
    if (atNow - lastAtMs >= AT_SAMPLE_MS) {
      lastAtMs = atNow;
      float t = readTemperature();
      if (!isnan(t) && t >= 0 && t <= 1000) {
        atBadReads = 0;
        autotuneProcess(t);
      } else if (++atBadReads >= AT_MAX_BAD_READS) {
        // Sensor putus saat fan ON = berbahaya: matikan fan dan batalkan
        autotuneAbort("sensor MAX6675 tidak terbaca (NaN/di luar rentang)");
      }
    }
  }

  // Step test: rekam suhu tiap AT_SAMPLE_MS (1 detik), fase baseline -> step -> steady
  static unsigned long lastStMs = 0;
  if (stActive) {
    unsigned long stNow = millis();
    if (stNow - lastStMs >= AT_SAMPLE_MS) {
      lastStMs = stNow;
      float t = readTemperature();
      if (!isnan(t) && t >= 0 && t <= 1000) {
        stBadReads = 0;
        stepProcess(t);
      } else if (++stBadReads >= AT_MAX_BAD_READS) {
        stAbort("sensor MAX6675 tidak terbaca (NaN/di luar rentang)");
      }
    }
  }

  // Uji metode (Adaptive PID): proses tiap AT_SAMPLE_MS (1 detik)
  static unsigned long lastMtMs = 0;
  if (mtActive) {
    unsigned long mtNow = millis();
    if (mtNow - lastMtMs >= AT_SAMPLE_MS) {
      lastMtMs = mtNow;
      float t = readTemperature();
      if (!isnan(t) && t >= 0 && t <= 1000) {
        mtBadReads = 0;
        methodTestProcess(t);
      } else if (++mtBadReads >= AT_MAX_BAD_READS) {
        mtAbort("sensor MAX6675 tidak terbaca (NaN/di luar rentang)");
      }
    }
  }

  unsigned long now = millis();

  // Kirim JSON ke Arduino setiap 500ms
  if (now - lastSerial2Ms >= SERIAL2_INTERVAL_MS) {
    lastSerial2Ms = now;
    sendStatusToArduino();
  }

  // Log suhu ke Serial Monitor setiap TEMP_LOG_INTERVAL_MS
  if (now - lastTempLogMs >= TEMP_LOG_INTERVAL_MS) {
    lastTempLogMs = now;
    logTempToSerial();
  }
}