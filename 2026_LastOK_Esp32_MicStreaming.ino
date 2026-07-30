#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <driver/i2s.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>

// ===================== Streaming state (shared between cores) =====================
static portMUX_TYPE g_streamMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool g_streamActive = false;
static char g_streamIP[16] = {0}; // "255.255.255.255" max 15 + NUL
static char g_streamCodec[8] = {0}; // "WAV" / "OPUS"
static volatile uint32_t g_streamSR = 0;

// I2S read diagnostics (underruns = short/failed reads)
static volatile uint32_t g_i2sUnderruns = 0;
static volatile uint32_t g_i2sReads = 0;
static volatile uint64_t g_i2sBytesRead = 0;



#include <SPIFFS.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <esp_heap_caps.h>

// Optional direct MySQL client (requires MySQL_Connector_Arduino library)
// Set USE_MYSQL 1 only if library is installed and you really want ESP32->MySQL directly.
#define USE_MYSQL 1

// Optional Opus output (UI + routing). Set to 1 only if you add an Opus encoder library.
#define USE_OPUS 1  // enable esp32_opus (sh123) Ogg/Opus streaming

#if USE_MYSQL
#define ESP32_MYSQL_DEBUG_PORT Serial
#define _ESP32_MYSQL_LOGLEVEL_ 1
#include <ESP32_MySQL.h>
#endif

#include <Preferences.h>

#if USE_OPUS
  #include <opus.h>
#endif
// ===== Runtime Gain Control =====
#include <math.h>

// ---- forward declarations (needed for Arduino compilation order) ----
static String getParam(const String &reqLine, const char *key);
static uint32_t mapOpusRate(uint32_t hz);
static void applySampleRate(uint32_t hz);
// --------------------------------------------------------------------


// --- Forward declarations (needed because Arduino builds are order-sensitive) ---
String htmlHeader();
String htmlFooter();
void sendHtml(WiFiClient &client, const String &html);

volatile int g_gain_db = 3;                 // default +3 dB
volatile int32_t g_gain_q15 = 46341;        // Q15 scale (1.4142 * 32768)
Preferences prefs;



// ===== ESP runtime specs / autotune (RAM/Flash/PSRAM) =====
struct EspSpecs {
  String chipModel;
  uint8_t cores = 0;
  uint16_t revision = 0;
  uint32_t cpuMHz = 0;
  uint32_t flashBytes = 0;
  uint32_t heapFree = 0;
  uint32_t heapMinFree = 0;
  uint32_t heapLargest = 0;
  bool hasPSRAM = false;
  uint32_t psramBytes = 0;
  uint32_t psramFree = 0;
};
static EspSpecs g_specs;

// MySQL fallback storage / watchdog tuning (autotuned)
static bool     g_fsOK = false;
static size_t   g_fallbackMaxBytes = 256 * 1024; // adjusted at runtime
static uint16_t g_flushMaxRows     = 20;         // adjusted at runtime

// MySQL reconnect watchdog/backoff
static uint32_t g_mysqlBackoffMs   = 0;          // 0 = no backoff
static uint32_t g_nextMysqlTryMs   = 0;
static uint16_t g_mysqlFailStreak  = 0;

static const char* MYSQL_FALLBACK_FILE = "/mysql_fallback.csv";

// Status / UI telemetry
static bool     g_db_ok = false;
static uint32_t g_db_last_try_ms = 0;
static uint8_t  g_vu_pct = 0; // 0..100, updated in audio loop

static uint32_t mysqlBacklogBytes() {
  if (!g_fsOK) return 0;
  if (!SPIFFS.exists(MYSQL_FALLBACK_FILE)) return 0;
  File f = SPIFFS.open(MYSQL_FALLBACK_FILE, FILE_READ);
  if (!f) return 0;
  uint32_t sz = (uint32_t)f.size();
  f.close();
  return sz;
}

// --- Time-series logging for charts (SPIFFS) ---
static uint32_t g_last_series_ms = 0;
static const char *ENVLOG_FILE = "/env_series.csv";
// Max size for series file (auto-tuned later); keep modest for 4MB flash.
static size_t g_envlog_max_bytes = 384 * 1024; // 384KB default

static uint32_t getFlashSizeBytes() {
  uint32_t size = 0;
  if (esp_flash_get_size(NULL, &size) == ESP_OK) return size;
  return 0;
}

static void detectEspSpecs() {
  esp_chip_info_t ci;
  esp_chip_info(&ci);
  g_specs.chipModel  = String(ESP.getChipModel());
  g_specs.cores      = ci.cores;
  g_specs.revision   = ci.revision;
  g_specs.cpuMHz     = ESP.getCpuFreqMHz();
  g_specs.flashBytes = getFlashSizeBytes();
  g_specs.heapFree    = ESP.getFreeHeap();
  g_specs.heapMinFree = ESP.getMinFreeHeap();
  g_specs.heapLargest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  g_specs.hasPSRAM = psramFound();
  if (g_specs.hasPSRAM) {
    g_specs.psramBytes = ESP.getPsramSize();
    g_specs.psramFree  = ESP.getFreePsram();
  }

  Serial.println();
  Serial.println(F("=== ESP Runtime Specs ==="));
  Serial.print(F("Chip: ")); Serial.println(g_specs.chipModel);
  Serial.print(F("Cores: ")); Serial.println(g_specs.cores);
  Serial.print(F("Rev: ")); Serial.println(g_specs.revision);
  Serial.print(F("CPU MHz: ")); Serial.println(g_specs.cpuMHz);
  Serial.print(F("Flash bytes: ")); Serial.println(g_specs.flashBytes);
  Serial.print(F("Heap free: ")); Serial.println(g_specs.heapFree);
  Serial.print(F("Heap min free: ")); Serial.println(g_specs.heapMinFree);
  Serial.print(F("Heap largest: ")); Serial.println(g_specs.heapLargest);
  Serial.print(F("PSRAM: ")); Serial.println(g_specs.hasPSRAM ? F("YES") : F("NO"));
  if (g_specs.hasPSRAM) {
    Serial.print(F("PSRAM bytes: ")); Serial.println(g_specs.psramBytes);
    Serial.print(F("PSRAM free: ")); Serial.println(g_specs.psramFree);
  }
  Serial.println(F("========================="));
}

static void autoTuneFromSpecs() {
  // internal flash fallback cap based on total flash size
  // (filesystem size still depends on partition scheme, but this keeps file bounded)
  g_fallbackMaxBytes = 256 * 1024; // safe on 4MB parts if SPIFFS is ~1MB
  if (g_specs.flashBytes >= 8UL * 1024 * 1024) g_fallbackMaxBytes = 1024 * 1024;
  if (g_specs.flashBytes >= 16UL * 1024 * 1024) g_fallbackMaxBytes = 2UL * 1024 * 1024;

  // flush batch size based on contiguous heap (to avoid audio glitches)
  if (g_specs.heapLargest < 50 * 1024) g_flushMaxRows = 10;
  else if (g_specs.heapLargest < 90 * 1024) g_flushMaxRows = 20;
  else g_flushMaxRows = 40;

  if (g_specs.hasPSRAM && g_specs.psramFree > 2UL * 1024 * 1024) {
    // give a bit more headroom if PSRAM is available
    g_flushMaxRows = (uint16_t)min<int>(80, g_flushMaxRows + 30);
  }
}



// ===== Time / Network info =====
static String   g_public_ip = "";
static uint32_t g_last_public_ip_ms = 0;

static void timeInit() {
  // Romania / Bucharest timezone (EET/EEST DST rules)
  configTzTime("EET-2EEST,M3.5.0/3,M10.5.0/4", "pool.ntp.org", "time.nist.gov", "time.google.com");
}

static void refreshPublicIP(bool force=false) {
  if (WiFi.status() != WL_CONNECTED) return;
  uint32_t now = millis();
  if (!force && g_public_ip.length() && (now - g_last_public_ip_ms) < 10UL*60UL*1000UL) return; // 10 min cache
  if (!force && !g_public_ip.length() && (now - g_last_public_ip_ms) < 30UL*1000UL) return;      // retry at most every 30s when empty
  g_last_public_ip_ms = now;

  WiFiClientSecure tls;
  tls.setInsecure();
  HTTPClient http;
  if (http.begin(tls, "https://api.ipify.org")) {
    int code = http.GET();
    if (code == 200) {
      String ip = http.getString();
      ip.trim();
      if (ip.length() <= 64) g_public_ip = ip;
    }
    http.end();
  }
}


// ===== Firmware / build / install info (shown in UI) =====
static String g_fw_file = "";
static String g_fw_build = String(__DATE__) + " " + String(__TIME__);
static String g_fw_install_local = "";

static String _basename(const String &p){
  int s1 = p.lastIndexOf('/');
  int s2 = p.lastIndexOf('\\');
  int s = (s1 > s2) ? s1 : s2;
  if (s >= 0) return p.substring(s+1);
  return p;
}

static bool timeIsValid() {
  time_t now = time(nullptr);
  // 1700000000 ~ 2023-11; anything newer is "real" NTP time
  return (now > (time_t)1700000000);
}

static String formatLocalTime(time_t t){
  struct tm lt; localtime_r(&t, &lt);
  char buf[24];
  snprintf(buf, sizeof(buf), "%02d-%02d-%04d %02d:%02d:%02d",
           lt.tm_mday, lt.tm_mon+1, lt.tm_year+1900,
           lt.tm_hour, lt.tm_min, lt.tm_sec);
  return String(buf);
}

static void fwInit() {
  g_fw_file = _basename(String(__FILE__));
  // If firmware changed since last boot, clear install stamp so it gets re-stamped after NTP sync.
  prefs.begin("fw", true);
  String lastFile  = prefs.getString("file", "");
  String lastBuild = prefs.getString("build", "");
  g_fw_install_local = prefs.getString("inst_local", "");
  prefs.end();

  if (lastFile != g_fw_file || lastBuild != g_fw_build) {
    g_fw_install_local = ""; // force re-stamp
    prefs.begin("fw", false);
    prefs.putString("file", g_fw_file);
    prefs.putString("build", g_fw_build);
    prefs.putString("inst_local", "");
    prefs.putULong64("inst_epoch", 0);
    prefs.end();
  }
}

// Set install time once, AFTER NTP is valid. Stored in NVS (Preferences).
static void fwMaybeStampInstallTime() {
  if (g_fw_install_local.length()) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (!timeIsValid()) return;

  time_t now = time(nullptr);
  g_fw_install_local = formatLocalTime(now);

  prefs.begin("fw", false);
  prefs.putString("inst_local", g_fw_install_local);
  // also store raw epoch for future uses (not currently displayed)
  prefs.putULong64("inst_epoch", (uint64_t)now);
  prefs.end();
}

static String fmt2(int v){ if(v<10) return "0"+String(v); return String(v); }

static void handleTimeJson(WiFiClient &client) {
  time_t now = time(nullptr);
  struct tm lt, ut;
  localtime_r(&now, &lt);
  gmtime_r(&now, &ut);

  String localS = fmt2(lt.tm_mday) + "-" + fmt2(lt.tm_mon+1) + "-" + String(lt.tm_year+1900) + " " +
                  fmt2(lt.tm_hour) + ":" + fmt2(lt.tm_min) + ":" + fmt2(lt.tm_sec);
  String utcS   = fmt2(ut.tm_mday) + "-" + fmt2(ut.tm_mon+1) + "-" + String(ut.tm_year+1900) + " " +
                  fmt2(ut.tm_hour) + ":" + fmt2(ut.tm_min) + ":" + fmt2(ut.tm_sec);

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json; charset=utf-8");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();
  client.print("{\"local\":\""); client.print(localS);
  client.print("\",\"utc\":\""); client.print(utcS);
  client.print("\"}");
}

static void handleNetJson(WiFiClient &client) {
  refreshPublicIP(false);

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json; charset=utf-8");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();

  client.print("{\"ssid\":\""); client.print(WiFi.SSID());
  client.print("\",\"local_ip\":\""); client.print(WiFi.localIP().toString());
  client.print("\",\"public_ip\":\""); client.print(g_public_ip);
  client.print("\"}");
}

void handleStreamJson(WiFiClient &client) {
  bool active;
  char ip[16];
  char codec[8];
  uint32_t sr;

  portENTER_CRITICAL(&g_streamMux);
  active = g_streamActive;
  strncpy(ip, g_streamIP, sizeof(ip)); ip[sizeof(ip)-1] = 0;
  strncpy(codec, g_streamCodec, sizeof(codec)); codec[sizeof(codec)-1] = 0;
  sr = g_streamSR;
  portEXIT_CRITICAL(&g_streamMux);

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json; charset=utf-8");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();

  client.print("{\"active\":");
  client.print(active ? "true" : "false");
  client.print(",\"ip\":\"");
  client.print(active ? ip : "");
  client.print("\",\"codec\":\"");
  client.print(active ? codec : "");
  client.print("\",\"sr\":");
  client.print(active ? sr : 0);
  client.print("}");
}


void handleMetricsJson(WiFiClient &client) {
  // Best-effort metrics endpoint. CPU load is reported as null unless runtime stats are enabled in the core.
  bool s_active;
  char s_ip[16];
  char s_codec[8];
  uint32_t s_sr;
  uint32_t i2s_und;
  uint32_t i2s_reads;
  uint64_t i2s_bytes;

  portENTER_CRITICAL(&g_streamMux);
  s_active = g_streamActive;
  strncpy(s_ip, g_streamIP, sizeof(s_ip)); s_ip[sizeof(s_ip)-1] = 0;
  strncpy(s_codec, g_streamCodec, sizeof(s_codec)); s_codec[sizeof(s_codec)-1] = 0;
  s_sr = g_streamSR;
  i2s_und = g_i2sUnderruns;
  i2s_reads = g_i2sReads;
  i2s_bytes = g_i2sBytesRead;
  portEXIT_CRITICAL(&g_streamMux);

  uint32_t heap_free = ESP.getFreeHeap();
  uint32_t heap_min  = ESP.getMinFreeHeap();
  uint32_t heap_largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

  uint32_t psram_free = 0;
  #if defined(ESP32)
    // On targets without PSRAM this will return 0
    psram_free = ESP.getFreePsram();
  #endif

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json; charset=utf-8");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();

  client.print("{\"uptime_ms\":"); client.print((uint32_t)millis());
  client.print(",\"heap_free\":"); client.print(heap_free);
  client.print(",\"heap_min\":"); client.print(heap_min);
  client.print(",\"heap_largest\":"); client.print(heap_largest);
  client.print(",\"psram_free\":"); client.print(psram_free);

  // CPU load: only available if FreeRTOS runtime stats are enabled; keep null otherwise.
  client.print(",\"cpu_load_pct\":null");

  client.print(",\"stream\":{");
  client.print("\"active\":"); client.print(s_active ? "true" : "false");
  client.print(",\"ip\":\""); client.print(s_active ? s_ip : ""); client.print("\"");
  client.print(",\"codec\":\""); client.print(s_active ? s_codec : ""); client.print("\"");
  client.print(",\"sr\":"); client.print(s_active ? s_sr : 0);
  client.print("}");

  client.print(",\"i2s\":{");
  client.print("\"reads\":"); client.print(i2s_reads);
  client.print(",\"bytes\":"); client.print((unsigned long long)i2s_bytes);
  client.print(",\"underruns\":"); client.print(i2s_und);
  client.print("}");

  client.print("}");
}

static void handleSpecsJson(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json; charset=utf-8");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();

  uint32_t flash_mb = (g_specs.flashBytes + (1024*1024-1)) / (1024*1024);
  uint32_t psram_mb = (g_specs.psramBytes + (1024*1024-1)) / (1024*1024);

  client.print("{\"chip\":\""); client.print(g_specs.chipModel);
  client.print("\",\"cores\":"); client.print(g_specs.cores);
  client.print(",\"rev\":"); client.print(g_specs.revision);
  client.print(",\"mhz\":"); client.print(g_specs.cpuMHz);
  client.print(",\"flash_mb\":"); client.print(flash_mb);
  client.print(",\"heap_free\":"); client.print(g_specs.heapFree);
  client.print(",\"heap_min\":"); client.print(g_specs.heapMinFree);
  client.print(",\"heap_largest\":"); client.print(g_specs.heapLargest);
  client.print(",\"psram\":"); client.print(g_specs.hasPSRAM ? "true" : "false");
  client.print(",\"psram_mb\":"); client.print(psram_mb);
  client.print(",\"psram_free\":"); client.print(g_specs.psramFree);
  client.print(",\"fallback_kb\":"); client.print((uint32_t)(g_fallbackMaxBytes/1024));
  client.print(",\"flush_rows\":"); client.print(g_flushMaxRows);
  client.print(",\"fw_file\":\""); client.print(g_fw_file);
  client.print("\",\"fw_build\":\""); client.print(g_fw_build);
  client.print("\",\"fw_install\":\""); client.print(g_fw_install_local);
  client.print("\"");
  client.print("}");
}
static bool initInternalFS() {
  // Requires a partition scheme that includes SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println(F("[FS] SPIFFS mount FAILED (choose a partition scheme with SPIFFS)."));
    return false;
  }
  Serial.print(F("[FS] SPIFFS total=")); Serial.print(SPIFFS.totalBytes());
  Serial.print(F(" used=")); Serial.println(SPIFFS.usedBytes());
  return true;
}// Return bytes for a SPIFFS file (0 if missing)
static size_t spiffsFileSize(const char* path) {
  if (!g_fsOK) return 0;
  File f = SPIFFS.open(path, FILE_READ);
  if (!f) return 0;
  size_t n = f.size();
  f.close();
  return n;
}

// Keep a CSV file bounded by rotating to .bak when oversized
static void spiffsRotateIfOversize(const char* path, size_t maxBytes) {
  if (!g_fsOK) return;
  File f = SPIFFS.open(path, FILE_READ);
  size_t cur = f ? f.size() : 0;
  if (f) f.close();
  if (cur <= maxBytes) return;
  String bak = String(path) + ".bak";
  SPIFFS.remove(bak);
  SPIFFS.rename(path, bak);
}

// Append one env+VU sample per minute to SPIFFS for charts.
// Format: ts_unix,tempC_corr,vu_pct
static void seriesLogOncePerMinute(float tCorr) {
  if (!g_fsOK) return;
  uint32_t ms = millis();
  if (ms - g_last_series_ms < 60000UL) return;
  g_last_series_ms = ms;

  spiffsRotateIfOversize(ENVLOG_FILE, g_envlog_max_bytes);
File a = SPIFFS.open(ENVLOG_FILE, FILE_APPEND);
  if (!a) return;
  time_t now = time(nullptr);
  a.printf("%ld,%.3f\n", (long)now, tCorr);
a.close();
}

// Parse interval string like "60s", "5m", "1h", "1d". Default: 300s.
static uint32_t parseIntervalSec(const String& s) {
  if (!s.length()) return 300;
  char unit = s[s.length()-1];
  long v = s.substring(0, s.length()-1).toInt();
  if (v <= 0) return 300;
  if (unit=='s') return (uint32_t)v;
  if (unit=='m') return (uint32_t)v * 60UL;
  if (unit=='h') return (uint32_t)v * 3600UL;
  if (unit=='d') return (uint32_t)v * 86400UL;
  return (uint32_t)s.toInt();
}

// Range -> seconds
static uint32_t rangeToSec(const String& r) {
  if (r == "day") return 86400UL;
  if (r == "week") return 7UL*86400UL;
  if (r == "month") return 30UL*86400UL;
  if (r == "year") return 365UL*86400UL;
  return 86400UL;
}

// Aggregated series from ENVLOG_FILE into JSON: [{"t":<unix>,"v":<value>},...]
// metric: "temp" or "vu"
static void handleSeriesJson(WiFiClient &client, const String& reqLine) {
  String url = reqLine;
  int sp = url.indexOf(' ');
  if (sp > 0) url = url.substring(0, sp);

  String metric = getParam(url, "m");
  if (!metric.length()) metric = "temp";
    if (metric == "vu") {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json; charset=utf-8");
    client.println("Cache-Control: no-store");
    client.println("Connection: close");
    client.println();
    client.print("{\"m\":\"vu\",\"points\":[]}");
    return;
  }
String range  = getParam(url, "r");
  if (!range.length()) range = "day";
  String ivS    = getParam(url, "i");

  uint32_t iv = (ivS == "auto") ? 0 : parseIntervalSec(ivS);
  uint32_t span = rangeToSec(range);
  // auto interval heuristic
  if (iv == 0) {
    if (span <= 86400UL) iv = 300;          // day -> 5m
    else if (span <= 7UL*86400UL) iv = 900; // week -> 15m
    else if (span <= 30UL*86400UL) iv = 3600; // month -> 1h
    else iv = 21600;                        // year -> 6h
  }
  if (iv < 10) iv = 10;

  if (!g_fsOK) {
    client.println("HTTP/1.1 500 Internal Server Error");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println();
    client.println("{\"ok\":false,\"msg\":\"SPIFFS not mounted\"}");
    return;
  }

  time_t now = time(nullptr);
  time_t t0 = (now > (time_t)span) ? (now - (time_t)span) : 0;

  File f = SPIFFS.open(ENVLOG_FILE, FILE_READ);
  if (!f) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println();
    client.println("[]");
    return;
  }

  const uint16_t MAX_POINTS = 720;
  static uint32_t bt[MAX_POINTS];
  static float    sum[MAX_POINTS];
  static uint16_t cnt[MAX_POINTS];
  uint16_t n=0;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;
    int p1 = line.indexOf(',');
    if (p1 < 0) continue;

    long tsL = line.substring(0, p1).toInt();
    if (tsL <= 0) continue;
    time_t ts = (time_t)tsL;
    if (ts < t0) continue;

    uint32_t b = ((uint32_t)ts / iv) * iv;

    float v = line.substring(p1 + 1).toFloat();
    if (isnan(v)) continue;

    int found = -1;
    for (uint16_t i = 0; i < n; i++) { if (bt[i] == b) { found = i; break; } }
    if (found < 0) {
      if (n >= MAX_POINTS) continue;
      bt[n] = b; sum[n] = v; cnt[n] = 1; n++;
    } else {
      sum[found] += v; cnt[found]++;
    }
  }

  f.close();

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json; charset=utf-8");
  client.println("Connection: close");
  client.println();
  client.print("[");
  for (uint16_t i=0;i<n;i++){
    if (i) client.print(",");
    float v = cnt[i] ? (sum[i]/(float)cnt[i]) : 0.0f;
    client.print("{\"t\":"); client.print(bt[i]);
    client.print(",\"v\":"); client.print(v, (metric=="vu")?0:3);
    client.print("}");
  }
  client.print("]");
}


static void fallbackRotateIfNeeded() {
  if (!g_fsOK) return;
  if (!SPIFFS.exists(MYSQL_FALLBACK_FILE)) return;
  File f = SPIFFS.open(MYSQL_FALLBACK_FILE, FILE_READ);
  if (!f) return;
  size_t sz = f.size();
  f.close();
  if (sz <= g_fallbackMaxBytes) return;

  SPIFFS.remove("/mysql_fallback.bak");
  SPIFFS.rename(MYSQL_FALLBACK_FILE, "/mysql_fallback.bak");
  Serial.println(F("[FS] Fallback rotated to /mysql_fallback.bak"));
}

static void fallbackAppendCSV(float tC, float p_hPa, float h, const char* sensorType) {
  if (!g_fsOK) return;
  fallbackRotateIfNeeded();

  File a = SPIFFS.open(MYSQL_FALLBACK_FILE, FILE_APPEND);
  if (!a) return;
  // CSV format (no timestamp): sensor,t_c,p_hpa,h_pct_or_NULL
  a.print(sensorType); a.print(',');
  a.print(String(tC, 2)); a.print(',');
  a.print(String(p_hPa, 1)); a.print(',');
  if (isnan(h)) a.print("NULL");
  else a.print(String(h, 1));
  a.print('\n');
  a.close();
}


// ===== Dynamic Noise Reduction (simple noise-gate/expander) =====
volatile bool g_dnr_on = false;
static float g_dnr_noise_est = 500.0f;         // running abs estimate
static float g_dnr_gain = 1.0f;               // smoothed gate gain (0..1)
static const float DNR_NOISE_ALPHA = 0.01f;    // smoothing
static const float DNR_OPEN_MULT   = 3.0f;     // threshold = noise_est * mult
static const int32_t DNR_FLOOR     = 600;      // minimum threshold
static const float DNR_ATTEN       = 0.15f;    // attenuation when below thr

static inline int32_t q15FromDb(int db) {
  if (db < -24) db = -24;
  if (db >  24) db =  24;
  double f = pow(10.0, db / 20.0);
  long q = lround(32768.0 * f);
  if (q < 0) q = 0;
  if (q > 524288) q = 524288; // ~+24 dB
  return (int32_t)q;
}
static inline void setGainDb(int db) {
  if (db < -24) db = -24;
  if (db >  24) db =  24;
  g_gain_db = db;
  g_gain_q15 = q15FromDb(db);
}
// ===================== Wi‑Fi (multi‑AP) =====================
WiFiMulti wifiMulti;

struct WifiAP { const char* ssid; const char* pass; };
static const WifiAP WIFI_APS[] = {
  {"TP-Link_D85C", "23253742"},   
  {"yo4tnv.net",   "yo4tnv54"},  
  {"yo4tnv.tk",    "yo4tnv54"}
  
};
static const int WIFI_AP_COUNT = sizeof(WIFI_APS)/sizeof(WIFI_APS[0]);

static void connectBestAP(uint32_t timeout_ms=30000) {
  for (int i=0;i<WIFI_AP_COUNT;i++) wifiMulti.addAP(WIFI_APS[i].ssid, WIFI_APS[i].pass);
  Serial.print("Connecting Wi‑Fi (multi‑AP)… ");
  uint32_t t0 = millis();
  while (wifiMulti.run() != WL_CONNECTED) {
    if (millis()-t0 > timeout_ms) break;
    delay(100);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWi‑Fi OK: %s  RSSI=%d dBm  IP=%s\n", WiFi.SSID().c_str(), WiFi.RSSI(), WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWi‑Fi connect timed out. Continuing anyway; server will bind when Wi‑Fi becomes available.");
  }
}

// ===================== I2S pins / config =====================
// Adjust to your wiring. Example for INMP441/ICS‑43434 style I2S mics
// NOTE: BCLK/WS must be on valid I2S pins; SD is data from mic
#define I2S_WS   25   // LRCLK / WS
#define I2S_SCK  26   // BCLK / SCK
#define I2S_SD   33   // DOUT -> ESP32 data in

static volatile uint32_t g_sample_rate = 44100;

// Audio output format selection (persisted)
enum AudioFmt : uint8_t { FMT_WAV = 0, FMT_OPUS = 1 };
static AudioFmt g_audio_fmt = FMT_WAV;

// Opus settings (persisted)
static uint32_t g_opus_target_bitrate = 32000; // bits/s (VBR target)
static uint8_t  g_opus_complexity = 5;         // 0..10
static uint8_t  g_opus_channels = 1;          // 1=mono,2=stereo (Opus)
    // default 22.05 kHz
static const uint16_t BITS        = 16;       // 16‑bit samples
static const uint16_t CHANNELS    = 2;        // stereo (duplicate mono into L/R if using single mic)

// +3 dB gain (Q15 fixed-point: ~1.4142)
static const int32_t GAIN_Q15 = 46341; // 1.41421356 * 32768
static const int      PKT_FRAMES  = 256;      // frames per write chunk

WiFiServer server(80);
WiFiServer audioServer(81);

// ===================== BMP280 =====================
Adafruit_BMP280 bmp;              // I2C
Adafruit_BME280 bme;              // I2C (optional)
bool bmpOK = false;
bool bmeOK = false;
bool hasHumidity = false;
float last_T = NAN, last_P = NAN, last_H = NAN;
float g_temp_off_c = 0.0f;   // temperature correction offset in °C (-20..+20), persistent
float g_tcal_step  = 0.10f;  // UI step for temp correction increment (°C), persistent
float g_gain_step  = 1.0f;   // UI step for gain increment (dB), persistent
uint32_t lastReadMs = 0;
const uint32_t READ_PERIOD_MS = 3000;   // ~1.5 Hz is fine


void bmpInit() {
  Wire.begin();
  // Try BME280 first (has humidity)
  if (bme.begin(0x76) || bme.begin(0x77)) {
    bmeOK = true;
    bmpOK = true;
    hasHumidity = true;
    Serial.println("BME280: initialized");
    return;
  }
  // Fallback to BMP280
  if (bmp.begin(0x76)) {
    bmpOK = true;
    hasHumidity = false;
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X4,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("BMP280: initialized @ 0x76");
  } else if (bmp.begin(0x77)) {
    bmpOK = true;
    hasHumidity = false;
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X4,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("BMP280: initialized @ 0x77");
  } else {
    Serial.println("BMP/BME280: NOT found");
  }
}


static inline void bmpPoll() {
  if (!bmpOK) return;
  uint32_t now = millis();
  if (now - lastReadMs < READ_PERIOD_MS) return;
  lastReadMs = now;

  if (bmeOK) {
    last_T = bme.readTemperature();
    last_P = bme.readPressure() / 100.0f; // hPa
    last_H = bme.readHumidity();
  } else {
    last_T = bmp.readTemperature();
    last_P = bmp.readPressure() / 100.0f; // hPa
    last_H = NAN;
  }
}

// --- tiny helpers (avoid std::min/max type issues) ---
static inline int16_t clamp16(int32_t x) {
  if (x >  32767) x =  32767;
  if (x < -32768) x = -32768;
  return (int16_t)x;
}
// ===================== I2S driver =====================
void i2sInit() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = (int)g_sample_rate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S, // OK as-is
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = true,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pins = {
    .bck_io_num = I2S_SCK,
    .ws_io_num  = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = I2S_SD
  };

  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_set_sample_rates(I2S_NUM_0, g_sample_rate);
  i2s_start(I2S_NUM_0);
}

// Build a 44‑byte WAV header with huge sizes to allow endless streaming (VLC‑friendly)
void writeWavHeader(WiFiClient &cli, uint32_t sampleRate, uint16_t bits, uint16_t channels) {
  uint32_t byteRate   = sampleRate * channels * (bits/8);
  uint16_t blockAlign = channels * (bits/8);

  uint8_t hdr[44] = {
    'R','I','F','F',
    0xFF,0xFF,0xFF,0xFF,                  // ChunkSize (placeholder)
    'W','A','V','E',
    'f','m','t',' ',
    16,0,0,0,                              // Subchunk1Size (PCM)
    1,0,                                   // AudioFormat = PCM
    (uint8_t)(channels & 0xFF), (uint8_t)(channels >> 8),
    (uint8_t)(sampleRate & 0xFF), (uint8_t)((sampleRate>>8)&0xFF),
    (uint8_t)((sampleRate>>16)&0xFF), (uint8_t)((sampleRate>>24)&0xFF),
    (uint8_t)(byteRate & 0xFF), (uint8_t)((byteRate>>8)&0xFF),
    (uint8_t)((byteRate>>16)&0xFF), (uint8_t)((byteRate>>24)&0xFF),
    (uint8_t)(blockAlign & 0xFF), (uint8_t)(blockAlign >> 8),
    (uint8_t)(bits & 0xFF), (uint8_t)(bits >> 8),
    'd','a','t','a',
    0xFF,0xFF,0xFF,0xFF                   // Subchunk2Size (placeholder)
  };
  cli.write(hdr, sizeof(hdr));
}

void streamWav(WiFiClient &client) {
  // HTTP response headers + WAV header
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: audio/wav");
  client.println("Connection: close");
  client.println("Cache-Control: no-store");
  client.println();
  writeWavHeader(client, g_sample_rate, BITS, CHANNELS);

  const size_t frameBytes = CHANNELS * (BITS/8);
  const size_t pktBytes   = PKT_FRAMES * frameBytes;
  std::unique_ptr<uint8_t[]> buf(new uint8_t[pktBytes]);

  // we read 32‑bit samples from I2S, downshift to 16‑bit, duplicate to stereo if needed
  const int BYTES_PER_32 = 4;
  const size_t i2sChunk = PKT_FRAMES * CHANNELS * BYTES_PER_32;
  std::unique_ptr<uint8_t[]> i2sRaw(new uint8_t[i2sChunk]);

  while (client.connected()) {
    size_t got = 0;
    size_t bytes_read = 0;
    // Pull a chunk of raw 32‑bit samples
    i2s_read(I2S_NUM_0, i2sRaw.get(), i2sChunk, &bytes_read, portMAX_DELAY);
    portENTER_CRITICAL(&g_streamMux);
    g_i2sReads++;
    g_i2sBytesRead += bytes_read;
    if (bytes_read < i2sChunk) g_i2sUnderruns++;
    portEXIT_CRITICAL(&g_streamMux);
    if (bytes_read < i2sChunk) continue; // short read; try again

    // Convert: take MS 16 bits of each 32‑bit slot
    int16_t *out = (int16_t*)buf.get();
    const int32_t *in32 = (const int32_t*)i2sRaw.get();
    // Update DNR gate once per chunk (cheap + audible)
    if (g_dnr_on) {
      int64_t sumAbs = 0;
      for (size_t k=0;k<PKT_FRAMES*CHANNELS;k++) {
        int32_t s0 = in32[k] >> 14;
        s0 = (int32_t)(((int64_t)s0 * g_gain_q15) >> 15);
        int32_t a0 = s0 < 0 ? -s0 : s0;
        sumAbs += a0;
      }
      float avgAbs = (float)sumAbs / (float)(PKT_FRAMES*CHANNELS);
      // VU meter (0..100) from avg absolute level
      {
        float pct = (avgAbs / 8000.0f) * 100.0f;
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        // Smooth to avoid flicker
        g_vu_pct = (uint8_t)(0.7f * (float)g_vu_pct + 0.3f * pct);
      }
      // Update noise estimate mostly when we're in a quiet region
      if (avgAbs < g_dnr_noise_est * 1.2f) {
        g_dnr_noise_est = (1.0f - DNR_NOISE_ALPHA) * g_dnr_noise_est + DNR_NOISE_ALPHA * avgAbs;
      } else {
        // very slow drift upward to follow environments
        g_dnr_noise_est = (1.0f - (DNR_NOISE_ALPHA*0.1f)) * g_dnr_noise_est + (DNR_NOISE_ALPHA*0.1f) * avgAbs;
      }
      float thr = g_dnr_noise_est * DNR_OPEN_MULT;
      if (thr < (float)DNR_FLOOR) thr = (float)DNR_FLOOR;
      float target = (avgAbs >= thr) ? 1.0f : DNR_ATTEN; // atten hard when below threshold
      // Smooth changes so it doesn't click: fast attack, slower release
      float a = (target > g_dnr_gain) ? 0.25f : 0.06f;
      g_dnr_gain = g_dnr_gain + a * (target - g_dnr_gain);
    } else {
      g_dnr_gain = 1.0f;
    }
    for (size_t i=0;i<PKT_FRAMES*CHANNELS;i++) {
      int32_t s = in32[i] >> 14; // typical scaling for I2S mics (24‑bit left‑justified in 32‑bit)
      s = (int32_t)(((int64_t)s * g_gain_q15) >> 15);
      // DNR: adaptive noise gate/expander (frame-level threshold, smoothed gain)
      if (g_dnr_on) {
        // g_dnr_gain is updated once per chunk below (see before loop)
        s = (int32_t)((float)s * g_dnr_gain);
      }
      out[i] = clamp16(s);
    }

    client.write(buf.get(), pktBytes);
    if (!client.connected()) break;
  }
}



void streamOpus(WiFiClient &client) {
#if USE_OPUS
  // Ogg/Opus live stream (one Opus packet per Ogg page).
  // NOTE: Opus supports encoder sample rates {8000,12000,16000,24000,48000}.
  // We map UI "44/22/11 kHz" to {48/24/12 kHz} for Opus mode.
  const uint32_t uiRate = (uint32_t)g_sample_rate;
  const uint32_t encRate = mapOpusRate(uiRate);
  const uint8_t  encCh = (g_opus_channels == 2) ? 2 : 1;

  // Switch I2S clock for Opus (keeps stereo slot config; we downmix if mono).
  i2s_set_clk(I2S_NUM_0, encRate, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);

  int err = 0;
  OpusEncoder *enc = opus_encoder_create((opus_int32)encRate, (int)encCh, OPUS_APPLICATION_AUDIO, &err);
  if (!enc || err != OPUS_OK) {
    client.println("HTTP/1.1 500 Internal Server Error");
    client.println("Content-Type: text/plain; charset=utf-8");
    client.println("Connection: close");
    client.println();
    client.println("Opus encoder init failed");
    if (enc) opus_encoder_destroy(enc);
    return;
  }

  // Variable bitrate + tuning
  opus_encoder_ctl(enc, OPUS_SET_VBR(1));
  opus_encoder_ctl(enc, OPUS_SET_BITRATE((int)g_opus_target_bitrate));
  opus_encoder_ctl(enc, OPUS_SET_COMPLEXITY((int)g_opus_complexity));
  opus_encoder_ctl(enc, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));

  opus_int32 lookahead = 0;
  opus_encoder_ctl(enc, OPUS_GET_LOOKAHEAD(&lookahead));
  if (lookahead < 0) lookahead = 0;
  uint16_t pre_skip = (uint16_t)lookahead;

  // HTTP headers
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/ogg");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();

  // ---- Ogg helpers ----
  auto ogg_crc_update = [](uint32_t crc, uint8_t b) -> uint32_t {
    crc ^= ((uint32_t)b) << 24;
    for (int i = 0; i < 8; i++) crc = (crc & 0x80000000u) ? ((crc << 1) ^ 0x04C11DB7u) : (crc << 1);
    return crc;
  };

  auto ogg_write_page = [&](const uint8_t *pkt, size_t pktLen, uint8_t headerType, uint64_t granulePos,
                            uint32_t serial, uint32_t seqno) {
    // Segment table
    uint8_t segs[255];
    uint8_t nSeg = 0;
    size_t remaining = pktLen;
    while (remaining > 0 && nSeg < 255) {
      uint8_t s = (remaining >= 255) ? 255 : (uint8_t)remaining;
      segs[nSeg++] = s;
      remaining -= s;
    }
    if (remaining > 0) return; // packet too large for single page (shouldn't happen here)

    const size_t headerLen = 27 + nSeg;
    const size_t pageLen = headerLen + pktLen;

    // Build header in a small buffer
    uint8_t hdr[27 + 255];
    memset(hdr, 0, sizeof(hdr));
    hdr[0] = 'O'; hdr[1] = 'g'; hdr[2] = 'g'; hdr[3] = 'S';
    hdr[4] = 0; // version
    hdr[5] = headerType;
    // granule position (LE)
    for (int i = 0; i < 8; i++) hdr[6 + i] = (uint8_t)((granulePos >> (8 * i)) & 0xFF);
    // serial (LE)
    for (int i = 0; i < 4; i++) hdr[14 + i] = (uint8_t)((serial >> (8 * i)) & 0xFF);
    // seqno (LE)
    for (int i = 0; i < 4; i++) hdr[18 + i] = (uint8_t)((seqno >> (8 * i)) & 0xFF);
    // checksum at [22..25] left zero for now
    hdr[26] = nSeg;
    memcpy(&hdr[27], segs, nSeg);

    // CRC over header+segments + packet
    uint32_t crc = 0;
    for (size_t i = 0; i < headerLen; i++) crc = ogg_crc_update(crc, hdr[i]);
    for (size_t i = 0; i < pktLen; i++)   crc = ogg_crc_update(crc, pkt[i]);

    hdr[22] = (uint8_t)(crc & 0xFF);
    hdr[23] = (uint8_t)((crc >> 8) & 0xFF);
    hdr[24] = (uint8_t)((crc >> 16) & 0xFF);
    hdr[25] = (uint8_t)((crc >> 24) & 0xFF);

    client.write(hdr, headerLen);
    client.write(pkt, pktLen);
  };

  // Ogg serial
  const uint32_t serial = (uint32_t)esp_random();
  uint32_t seq = 0;
  uint64_t granule = 0;

  // OpusHead packet
  uint8_t head[19];
  memcpy(head, "OpusHead", 8);
  head[8]  = 1;           // version
  head[9]  = encCh;       // channels
  head[10] = (uint8_t)(pre_skip & 0xFF);
  head[11] = (uint8_t)((pre_skip >> 8) & 0xFF);
  // original sample rate (LE)
  head[12] = (uint8_t)(encRate & 0xFF);
  head[13] = (uint8_t)((encRate >> 8) & 0xFF);
  head[14] = (uint8_t)((encRate >> 16) & 0xFF);
  head[15] = (uint8_t)((encRate >> 24) & 0xFF);
  head[16] = 0; head[17] = 0; // output gain
  head[18] = 0;              // mapping family 0

  ogg_write_page(head, sizeof(head), 0x02 /*BOS*/, 0, serial, seq++);

  // OpusTags packet (minimal)
  const char *vendor = "ESP32";
  const uint32_t vendorLen = (uint32_t)strlen(vendor);
  const uint32_t userCommentListLen = 0;
  const size_t tagsLen = 8 + 4 + vendorLen + 4;
  uint8_t *tags = (uint8_t*)malloc(tagsLen);
  if (tags) {
    memcpy(tags, "OpusTags", 8);
    tags[8]  = (uint8_t)(vendorLen & 0xFF);
    tags[9]  = (uint8_t)((vendorLen >> 8) & 0xFF);
    tags[10] = (uint8_t)((vendorLen >> 16) & 0xFF);
    tags[11] = (uint8_t)((vendorLen >> 24) & 0xFF);
    memcpy(tags + 12, vendor, vendorLen);
    size_t off = 12 + vendorLen;
    tags[off+0] = (uint8_t)(userCommentListLen & 0xFF);
    tags[off+1] = (uint8_t)((userCommentListLen >> 8) & 0xFF);
    tags[off+2] = (uint8_t)((userCommentListLen >> 16) & 0xFF);
    tags[off+3] = (uint8_t)((userCommentListLen >> 24) & 0xFF);

    ogg_write_page(tags, tagsLen, 0x00, 0, serial, seq++);
    free(tags);
  }

  // 20 ms frames
  const int frameSize = (int)(encRate / 50);
  const int i2sFrames = frameSize; // stereo frames from I2S
  const int i2sWords  = i2sFrames * 2; // L+R 32-bit
  const size_t i2sBytes = i2sWords * sizeof(int32_t);

  int32_t *i2sBuf = (int32_t*)malloc(i2sBytes);
  int16_t *pcm    = (int16_t*)malloc(frameSize * encCh * sizeof(int16_t));
  uint8_t *opusOut= (uint8_t*)malloc(4000); // enough for 20ms @ typical bitrates

  if (!i2sBuf || !pcm || !opusOut) {
    if (i2sBuf) free(i2sBuf);
    if (pcm) free(pcm);
    if (opusOut) free(opusOut);
    opus_encoder_destroy(enc);
    return;
  }

  // rescale for Ogg granulepos (always 48kHz units)
  const uint32_t gpMult = 48000 / encRate; // 1,2,4

  while (client.connected()) {
    size_t got = 0;
    esp_err_t r = i2s_read(I2S_NUM_0, (void*)i2sBuf, i2sBytes, &got, portMAX_DELAY);
    portENTER_CRITICAL(&g_streamMux);
    g_i2sReads++;
    g_i2sBytesRead += got;
    if (r != ESP_OK || got != i2sBytes) g_i2sUnderruns++;
    portEXIT_CRITICAL(&g_streamMux);
    if (r != ESP_OK || got != i2sBytes) continue;

    // Convert + gain + DNR + optional mono downmix
    for (int i = 0; i < frameSize; i++) {
      // Input is stereo 32-bit; use top 16 bits
      int32_t L = (int32_t)(i2sBuf[2*i + 0] >> 16);
      int32_t R = (int32_t)(i2sBuf[2*i + 1] >> 16);

      // Apply gain (Q15)
      L = (L * (int32_t)g_gain_q15) >> 15;
      R = (R * (int32_t)g_gain_q15) >> 15;

      // Simple dynamic noise reduction (same gate as WAV path)
      if (g_dnr_on) {
        int32_t a = (abs(L) + abs(R)) >> 1;
        g_dnr_noise_est = (1.0f - DNR_NOISE_ALPHA) * g_dnr_noise_est + DNR_NOISE_ALPHA * (float)a;
        int32_t thr = (int32_t)(g_dnr_noise_est * DNR_OPEN_MULT);
        if (thr < DNR_FLOOR) thr = DNR_FLOOR;
        if (a < thr) {
          L = (int32_t)((float)L * DNR_ATTEN);
          R = (int32_t)((float)R * DNR_ATTEN);
        }
      }

      int16_t sL = clamp16(L);
      int16_t sR = clamp16(R);

      if (encCh == 2) {
        pcm[2*i + 0] = sL;
        pcm[2*i + 1] = sR;
      } else {
        // downmix
        pcm[i] = (int16_t)(((int32_t)sL + (int32_t)sR) / 2);
      }

      // VU estimate (peak of absolute)
      int32_t pk = max(abs((int32_t)sL), abs((int32_t)sR));
      uint32_t pct = (uint32_t)((pk * 100L) / 32767L);
      if (pct > g_vu_pct) g_vu_pct = pct;
      else g_vu_pct = (uint32_t)((g_vu_pct * 7 + pct) / 8);
    }

    int n = opus_encode(enc, (const opus_int16*)pcm, frameSize, opusOut, 4000);
    if (n > 0) {
      granule += (uint64_t)frameSize * (uint64_t)gpMult;
      ogg_write_page(opusOut, (size_t)n, 0x00, granule, serial, seq++);
    }
    delay(0);
  }

  free(i2sBuf);
  free(pcm);
  free(opusOut);
  opus_encoder_destroy(enc);

  // Restore I2S clock for current UI rate (honor current format)
  applySampleRate(g_sample_rate);
#else
  client.println("HTTP/1.1 501 Not Implemented");
  client.println("Content-Type: text/plain; charset=utf-8");
  client.println("Connection: close");
  client.println();
  client.println("OPUS disabled at compile time (USE_OPUS=0).");
#endif
}

static void handleGainJson(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json; charset=utf-8");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();
  client.print("{\"db\":"); client.print(g_gain_db); client.print(",\"step\":"); client.print(g_gain_step, 3); client.print("}");
}



static void handleTcalJson(WiFiClient &client) {
  // Include offset + UI step + live raw/corrected (if available)
  bmpPoll();
  float tRaw  = last_T;
  float tCorr = isnan(last_T) ? NAN : (last_T + g_temp_off_c);

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json; charset=utf-8");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();

  client.print("{\"off\":");  client.print(g_temp_off_c, 2);
  client.print(",\"step\":"); client.print(g_tcal_step, 3);

  client.print(",\"traw\":");
  if (!isnan(tRaw)) client.print(tRaw, 3); else client.print("null");

  client.print(",\"tcorr\":");
  if (!isnan(tCorr)) client.print(tCorr, 3); else client.print("null");

  client.print("}");
}

static void handleTcalStepSet(WiFiClient &client, const String &reqLine) {
  String stepS = getParam(reqLine, "step");
  if (stepS.length()) {
    float v = stepS.toFloat();
    // Accept typical values; keep within sane bounds.
    if (v < 0.001f) v = 0.01f;
    if (v > 5.0f)   v = 1.0f;
    g_tcal_step = v;
    prefs.begin("env", false);
    prefs.putFloat("t_step", g_tcal_step);
    prefs.end();
  }
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json; charset=utf-8");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();
  client.print("{\"ok\":true,\"step\":"); client.print(g_tcal_step, 3); client.print("}");
}

static void handleGainStepSet(WiFiClient &client, const String &reqLine) {
  String stepS = getParam(reqLine, "step");
  if (stepS.length()) {
    float s = stepS.toFloat();
    if (s < 0.1f) s = 0.1f;
    if (s > 12.0f) s = 12.0f;
    g_gain_step = s;
    prefs.begin("audio", false);
    prefs.putFloat("gain_step", g_gain_step);
    prefs.end();
  }
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json; charset=utf-8");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();
  client.print("{\"ok\":true,\"step\":"); client.print(g_gain_step, 3); client.print("}");
}


static void handleTcalMatch(WiFiClient &client, const String &reqLine) {
  // Set offset so that corrected temperature matches a user-provided reference value.
  // ref = desired corrected temperature. offset = ref - raw.
  bmpPoll();
  String refS = getParam(reqLine, "ref");
  bool ok = false;

  if (refS.length() && !isnan(last_T)) {
    float ref = refS.toFloat();
    float off = ref - last_T;
    if (off < -20.0f) off = -20.0f;
    if (off >  20.0f) off =  20.0f;
    g_temp_off_c = off;
    prefs.begin("env", false);
    prefs.putFloat("t_off", g_temp_off_c);
    prefs.end();
    ok = true;
  }

  float tCorr = isnan(last_T) ? NAN : (last_T + g_temp_off_c);

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json; charset=utf-8");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();
  client.print("{\"ok\":"); client.print(ok ? "true" : "false");
  client.print(",\"off\":"); client.print(g_temp_off_c, 2);
  client.print(",\"traw\":"); if (!isnan(last_T)) client.print(last_T, 3); else client.print("null");
  client.print(",\"tcorr\":"); if (!isnan(tCorr)) client.print(tCorr, 3); else client.print("null");
  client.print("}");
}

static void handleTcalSet(WiFiClient &client, const String &reqLine) {
  String offS = getParam(reqLine, "off");
  if (offS.length()) {
    float v = offS.toFloat();
    if (v < -20.0f) v = -20.0f;
    if (v >  20.0f) v =  20.0f;
    g_temp_off_c = v;
    prefs.begin("env", false);
    prefs.putFloat("t_off", g_temp_off_c);
    prefs.end();
  }
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json; charset=utf-8");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();
  client.print("{\"ok\":true,\"off\":"); client.print(g_temp_off_c, 2); client.print("}");
}


static void handleDnrJson(WiFiClient &client) {
  client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"));
  client.print(F("{\"dnr_on\":"));
  client.print(g_dnr_on ? F("true") : F("false"));
  client.print(F("}\n"));
}


static void handleDnrSet(WiFiClient &client, const String &reqLine) {
  String on = getParam(reqLine, "on");
  if (on.length()) {
    g_dnr_on = (on.toInt() != 0);
    prefs.begin("audio", false);
    prefs.putBool("dnr_on", g_dnr_on);
    prefs.end();
  }
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain; charset=utf-8");
  client.println("Connection: close");
  client.println();
  client.println(g_dnr_on ? "DNR ON" : "DNR OFF");
}

static void handleSrJson(WiFiClient &client) {
  client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"));
  client.print(F("{\"hz\":"));
  client.print((uint32_t)g_sample_rate);
  client.print(F("}\n"));
}


static void handleSrSet(WiFiClient &client, const String &reqLine) {
  String hzS = getParam(reqLine, "hz");
  if (hzS.length()) {
    uint32_t hz = (uint32_t)hzS.toInt();
    applySampleRate(hz);
    prefs.begin("audio", false);
    prefs.putUInt("sr_hz", (uint32_t)g_sample_rate);
    prefs.end();
  }
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain; charset=utf-8");
  client.println("Connection: close");
  client.println();
  client.print("SR "); client.println((uint32_t)g_sample_rate);
}


void handleFmtJson(WiFiClient &client) {
  client.print("{\"fmt\":");
  client.print((uint32_t)g_audio_fmt);
  client.print(",\"opus_br\":");
  client.print((uint32_t)g_opus_target_bitrate);
  client.print(",\"opus_cx\":");
  client.print((uint32_t)g_opus_complexity);
  client.print(",\"opus_ch\":");
  client.print((uint32_t)g_opus_channels);
  client.print("}\n");
}

void handleFmtSet(WiFiClient &client, const String &reqLine) {
  String fmt = getParam(reqLine, "fmt"); // wav|opus|0|1
  String br  = getParam(reqLine, "br");  // bits/s or kbps
  String cx  = getParam(reqLine, "cx");  // 0..10
  String ch  = getParam(reqLine, "ch");  // 1|2

  if (fmt.length()) {
    String f = fmt; f.toLowerCase();
    if (f == "opus" || f == "1") g_audio_fmt = FMT_OPUS;
    else g_audio_fmt = FMT_WAV;
  }
  if (br.length()) {
    uint32_t v = (uint32_t)br.toInt();
    if (v > 0 && v < 1000) v *= 1000; // treat as kbps if small
    if (v < 6000) v = 6000;
    if (v > 256000) v = 256000;
    g_opus_target_bitrate = v;
  }
  if (cx.length()) {
    int v = cx.toInt();
    if (v < 0) v = 0;
    if (v > 10) v = 10;
    g_opus_complexity = (uint8_t)v;
  }
  if (ch.length()) {
    int v = ch.toInt();
    if (v != 2) v = 1;
    g_opus_channels = (uint8_t)v;
  }

  prefs.begin("audio", false);
  prefs.putUChar("fmt", (uint8_t)g_audio_fmt);
  prefs.putUInt("opus_br", g_opus_target_bitrate);
  prefs.putUChar("opus_cx", g_opus_complexity);
  prefs.putUChar("opus_ch", g_opus_channels);
  prefs.end();

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain; charset=utf-8");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();
  client.println("OK");
}


// ===== MySQL config + logging =====
struct MysqlCfg {
  bool enabled = false;
  String host = "";
  uint16_t port = 3306;
  String user = "";
  String pass = "";
  String db = "weather";
  String table = "focsani";
};
static MysqlCfg g_mysql;
static uint32_t g_last_mysql_ms = 0;

static void mysqlLoadPrefs() {
  prefs.begin("mysql", true);
  g_mysql.enabled = prefs.getBool("en", false);
  g_mysql.host = prefs.getString("host", "");
  g_mysql.port = (uint16_t)prefs.getUShort("port", 3306);
  g_mysql.user = prefs.getString("user", "");
  g_mysql.pass = prefs.getString("pass", "");
  g_mysql.db   = prefs.getString("db", "weather");
  g_mysql.table= prefs.getString("tbl", "focsani");
  prefs.end();
}
static void mysqlSavePrefs() {
  prefs.begin("mysql", false);
  prefs.putBool("en", g_mysql.enabled);
  prefs.putString("host", g_mysql.host);
  prefs.putUShort("port", g_mysql.port);
  prefs.putString("user", g_mysql.user);
  prefs.putString("pass", g_mysql.pass);
  prefs.putString("db", g_mysql.db);
  prefs.putString("tbl", g_mysql.table);
  prefs.end();
}

static void handleMysqlJson(WiFiClient &client) {
  client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"));
  client.print(F("{\"enabled\":"));
  client.print(g_mysql.enabled ? F("true") : F("false"));
  client.print(F(",\"host\":\""));  client.print(g_mysql.host);  client.print(F("\""));
  client.print(F(",\"port\":"));     client.print(g_mysql.port);
  client.print(F(",\"user\":\""));   client.print(g_mysql.user);  client.print(F("\""));
  client.print(F(",\"db\":\""));     client.print(g_mysql.db);    client.print(F("\""));
  client.print(F(",\"table\":\""));  client.print(g_mysql.table); client.print(F("\""));
#if USE_MYSQL
  client.print(F(",\"direct\":true"));
#else
  client.print(F(",\"direct\":false"));
#endif
  client.print(F("}\n"));
}


static void handleMysqlSave(WiFiClient &client, const String &url) {
  // GET /mysql/save?en=1&host=...&port=3306&user=...&pass=...&db=weather&table=focsani
  String en   = getParam(url, "en");
  String host = getParam(url, "host");
  String port = getParam(url, "port");
  String user = getParam(url, "user");
  String pass = getParam(url, "pass");
  String db   = getParam(url, "db");
  String tbl  = getParam(url, "table");
  if (!tbl.length()) tbl = getParam(url, "tbl");
if (en.length())   g_mysql.enabled = (en.toInt() != 0);
  if (host.length()) g_mysql.host = host;
  if (port.length()) g_mysql.port = (uint16_t)port.toInt();
  if (user.length()) g_mysql.user = user;
  if (pass.length()) g_mysql.pass = pass;
  if (db.length())   g_mysql.db = db;
  if (tbl.length())  g_mysql.table = tbl;

  if (!g_mysql.db.length()) g_mysql.db = "weather";
  if (!g_mysql.table.length()) g_mysql.table = "focsani";

  mysqlSavePrefs();

  client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"));
  client.print(F("{\"ok\":true}\n"));
}

void handleMysqlTest(WiFiClient &client) {
  client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"));
#if USE_MYSQL
  String msg;
  bool ok = mysqlTest(msg);
  client.print(F("{\"ok\":"));
  client.print(ok ? "true" : "false");
  client.print(F(",\"msg\":\""));
  client.print(msg);
  client.print(F("\"}\n"));
#else
  client.print(F("{\"ok\":false,\"msg\":\"mysql disabled\"}\n"));
#endif
}



#if USE_MYSQL

static bool mysqlConnectOnce(ESP32_MySQL_Connection &conn) {
  if (!g_mysql.enabled) return false;
  if (g_mysql.host.length() == 0 || g_mysql.user.length() == 0) return false;

  char hostbuf[96];
  memset(hostbuf, 0, sizeof(hostbuf));
  strncpy(hostbuf, g_mysql.host.c_str(), sizeof(hostbuf) - 1);

  // ESP32_MySQL expects mutable C strings for user/pass
  char userbuf[48];
  char passbuf[64];
  memset(userbuf, 0, sizeof(userbuf));
  memset(passbuf, 0, sizeof(passbuf));
  strncpy(userbuf, g_mysql.user.c_str(), sizeof(userbuf) - 1);
  strncpy(passbuf, g_mysql.pass.c_str(), sizeof(passbuf) - 1);

  // Non-blocking connect (returns RESULT_FAIL on failure)
  if (conn.connectNonBlocking(hostbuf, g_mysql.port, userbuf, passbuf) == RESULT_FAIL) {
    conn.close();
    return false;
  }
  return true;
}

static void mysqlEnsureSchema(ESP32_MySQL_Query &query) {
  // Create DB + table if needed
  String q1 = "CREATE DATABASE IF NOT EXISTS `" + g_mysql.db + "`";
  query.execute(q1.c_str());

  String q2 = "CREATE TABLE IF NOT EXISTS `" + g_mysql.db + "`.`" + g_mysql.table + "` ("
              "id BIGINT AUTO_INCREMENT PRIMARY KEY,"
              "dt DATETIME NOT NULL,"
              "date_str VARCHAR(10),"
              "time_str VARCHAR(8),"
              "sensor VARCHAR(8),"
              "t_c FLOAT,"
              "p_hpa FLOAT,"
              "h_pct FLOAT)";
  query.execute(q2.c_str());

  // Try to add missing columns safely (ignore failure if already exists)
  query.execute(("ALTER TABLE `" + g_mysql.db + "`.`" + g_mysql.table + "` ADD COLUMN date_str VARCHAR(10) NULL").c_str());
  query.execute(("ALTER TABLE `" + g_mysql.db + "`.`" + g_mysql.table + "` ADD COLUMN time_str VARCHAR(8) NULL").c_str());
  query.execute(("ALTER TABLE `" + g_mysql.db + "`.`" + g_mysql.table + "` ADD COLUMN sensor VARCHAR(8) NULL").c_str());
}

static bool mysqlInsertRow(ESP32_MySQL_Query &query, float tC, float p_hPa, float h, const char *sensor) {
  String q = "INSERT INTO `" + g_mysql.db + "`.`" + g_mysql.table + "` "
             "(dt,date_str,time_str,sensor,t_c,p_hpa,h_pct) VALUES ("
             "NOW(),"
             "DATE_FORMAT(NOW(),'%d-%m-%Y'),"
             "DATE_FORMAT(NOW(),'%H:%i:%s'),'";
  q += sensor;
  q += "',";
  q += String(tC, 2);
  q += ",";
  q += String(p_hPa, 1);
  q += ",";
  if (isnan(h)) q += "NULL"; else q += String(h, 1);
  q += ")";
  return query.execute(q.c_str());
}

bool mysqlLogOnce(float tC, float p_hPa, float h) {
  g_db_last_try_ms = millis();

  if (!g_mysql.enabled) { g_db_ok = false; return false; }
  if (g_mysql.host.length() == 0 || g_mysql.user.length() == 0) { g_db_ok = false; return false; }

  WiFiClient net;
  ESP32_MySQL_Connection conn((Client*)&net);
  if (!mysqlConnectOnce(conn)) { g_db_ok = false; return false; }

  ESP32_MySQL_Query query(&conn);

  mysqlEnsureSchema(query);

  const char *stype = hasHumidity ? "BME280" : "BMP280";
  String q3 = "INSERT INTO `" + g_mysql.db + "`.`" + g_mysql.table + "` "
              "(dt,date_str,time_str,sensor,t_c,p_hpa,h_pct) VALUES ("
              "NOW(),"
              "DATE_FORMAT(NOW(),'%d-%m-%Y'),"
              "DATE_FORMAT(NOW(),'%H:%i:%s'),'";
  q3 += stype;
  q3 += "',";
  q3 += String(tC, 2);
  q3 += ",";
  q3 += String(p_hPa, 1);
  q3 += ",";
  if (isnan(h)) q3 += "NULL"; else q3 += String(h, 1);
  q3 += ")";
  bool ok = query.execute(q3.c_str());

  conn.close();
  g_db_ok = ok;
  return ok;
}

static bool mysqlTest(String &msgOut) {
  WiFiClient net;
  ESP32_MySQL_Connection conn((Client*)&net);
  if (!mysqlConnectOnce(conn)) {
    msgOut = "connect failed";
    return false;
  }
  ESP32_MySQL_Query query(&conn);
  bool ok = query.execute("SELECT 1");
  msgOut = ok ? "ok" : "query failed";
  conn.close();
  return ok;
}



// Flush fallback CSV into MySQL in small batches (audio-safe).
static bool mysqlFlushFallbackOnce(ESP32_MySQL_Query &query) {
  if (!g_fsOK) return true;
  if (!SPIFFS.exists(MYSQL_FALLBACK_FILE)) return true;

  File f = SPIFFS.open(MYSQL_FALLBACK_FILE, FILE_READ);
  if (!f) return false;
  File tmp = SPIFFS.open("/mysql_fallback.tmp", FILE_WRITE);
  if (!tmp) { f.close(); return false; }

  uint16_t sent = 0;
  bool allOk = true;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;

    // Expected CSV: sensor,t_c,p_hpa,h_or_NULL
    int p1 = line.indexOf(',');
    int p2 = (p1 >= 0) ? line.indexOf(',', p1 + 1) : -1;
    int p3 = (p2 >= 0) ? line.indexOf(',', p2 + 1) : -1;
    if (p1 < 0 || p2 < 0 || p3 < 0) {
      // keep malformed line (don’t drop data)
      tmp.println(line);
      continue;
    }

    String sensor = line.substring(0, p1);
    float tC      = line.substring(p1 + 1, p2).toFloat();
    float p_hPa   = line.substring(p2 + 1, p3).toFloat();
    String hs     = line.substring(p3 + 1);
    float h = NAN;
    if (!hs.equalsIgnoreCase("NULL") && hs.length()) h = hs.toFloat();

    bool ok = mysqlInsertRow(query, tC, p_hPa, h, sensor.c_str());
    if (!ok) {
      allOk = false;
      tmp.println(line); // keep for later retry
    } else {
      sent++;
      if (sent >= g_flushMaxRows) {
        // copy the rest unchanged
        while (f.available()) {
          String rest = f.readStringUntil('\n');
          rest.trim();
          if (rest.length()) tmp.println(rest);
        }
        break;
      }
    }
  }

  f.close();
  tmp.close();

  SPIFFS.remove(MYSQL_FALLBACK_FILE);
  SPIFFS.rename("/mysql_fallback.tmp", MYSQL_FALLBACK_FILE);

  return allOk;
}

static void mysqlWatchdogReset() {
  g_mysqlFailStreak = 0;
  g_mysqlBackoffMs = 0;
  g_nextMysqlTryMs = 0;
}

static void mysqlWatchdogOnFail() {
  g_mysqlFailStreak++;
  // exponential backoff: 10s,20s,40s,80s,160s,320s -> cap 300s
  uint32_t step = 10000UL << (g_mysqlFailStreak > 5 ? 5 : g_mysqlFailStreak);
  if (step < 10000UL) step = 10000UL;
  if (step > 300000UL) step = 300000UL;
  g_mysqlBackoffMs = step;
  g_nextMysqlTryMs = millis() + g_mysqlBackoffMs;
}


#else
static bool mysqlLogOnce(float, float, float) {
  // USE_MYSQL==0: stub
  return false;
}
#endif


static void handleGainSet(WiFiClient &client, const String &reqLine) {
  int qPos = reqLine.indexOf(" /gain");
  int dbVal = g_gain_db;
  if (qPos >= 0) {
    int dbIdx = reqLine.indexOf("db=", qPos);
    if (dbIdx >= 0) {
      dbIdx += 3;
      int end = reqLine.indexOf(' ', dbIdx);
      String s = reqLine.substring(dbIdx, end < 0 ? reqLine.length() : end);
      String num = "";
      for (size_t i=0;i<s.length();++i) {
        char c = s[i];
        if ((c=='-' && num.length()==0) || (c>='0' && c<='9')) num += c;
        else break;
      }
      if (num.length()) dbVal = num.toInt();
    }
  }
  setGainDb(dbVal);
  prefs.begin("audio", false);
  prefs.putInt("gain_db", g_gain_db);
  prefs.end();
client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain; charset=utf-8");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();
  client.print("OK ");
  client.print(g_gain_db);
  client.print(" dB");
}

String htmlHeader() {
  // Legacy helper kept for compatibility; current UI uses a raw HTML literal.
  // Keep this function simple (no long/truncated literals) to avoid macro/F() parse issues.
  return String(F("<!doctype html><html><head><meta charset='utf-8'>"
                  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                  "<title>ESP32</title></head><body>"));
}


void handleRoot(WiFiClient &client) {
  // Minimal, Android-friendly single-page UI (polls JSON endpoints).
  // NOTE: /wav is intentionally left unprotected elsewhere; UI endpoints are protected via BasicAuth.
  String s;
  s.reserve(9000);

  s += 
R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Remote Enviroment Surveillance Terminal</title>
<style>
:root{
  --bg:#000;
  --fg:#00ff00;
  --fg2:rgba(0,255,0,.75);
  --line:rgba(0,255,0,.25);
  --line2:rgba(0,255,0,.35);
  --bad:#ff3b3b;
  --ok:#00ff00;
  --cap:#ff0000;
}
*{box-sizing:border-box}
html,body{margin:0;padding:0;background:var(--bg);color:var(--fg);font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif;font-size:20px}
a{color:var(--fg)}
.wrap{max-width:980px;margin:0 auto;padding:14px}
.title{ text-align:center; font-weight:900; letter-spacing:1px; margin:10px 0 14px 0; line-height:1.15; }
.title span{ font-size:28px; }
.title .cap{ color:var(--cap); }
@media(max-width:600px){ .title span{font-size:22px;} }

.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
@media(max-width:820px){ .grid{grid-template-columns:1fr;} }

.card{border:3px solid var(--line);border-radius:12px;padding:12px}
.card h3{margin:0 0 10px 0;font-size:16px;border-bottom:1px solid var(--line);padding-bottom:6px}
.row{display:flex;gap:10px;align-items:center;flex-wrap:wrap}
.kv{font-size:15px;color:var(--fg2);line-height:1.6}
.kv b{color:var(--fg);font-weight:700}
input,select,button{background:#000;color:var(--fg);border:1px solid var(--line2);border-radius:10px;padding:10px 12px;font-size:18px}
input{width:100%}
label{font-size:13px;color:var(--fg2);display:block;margin:10px 0 6px}
button{width:100%;font-weight:700}
button:active{transform:translateY(1px)}
button.btnTiny{width:auto;padding:6px 10px;font-size:13px;border-radius:9px}
.small{font-size:12px;color:var(--fg2)}
hr{border:none;border-top:1px solid var(--line);margin:10px 0}
.inline{display:flex;gap:10px}
.inline > div{flex:1}
.toggle{display:flex;gap:10px;align-items:center}
.toggle input{width:auto}
.pill{display:inline-block;border:1px solid var(--line2);padding:2px 8px;border-radius:999px;font-size:12px}
.bad{color:var(--bad)}
.ok{color:var(--ok)}

.bar{flex:1;height:18px;border:1px solid var(--g);margin:0 10px;}
.barin{height:100%;background:var(--g);}

/* Collapsible frames */
.stack{display:flex;flex-direction:column;gap:12px;width:100%}
.stack > details.card{width:100%;display:block}
details.card{padding:0}
details.card > summary{
  list-style:none;
  cursor:pointer;
  padding:14px 14px;
  font-size:20px;
  font-weight:800;
  border-bottom:2px solid #073;
  outline:none;
}
details.card > summary::-webkit-details-marker{display:none}
details.card[open] > summary{border-bottom:2px solid #0a5}
.section{padding:14px}
.timebar{ text-align:center; margin:6px 0 14px 0; }
.timebar .date{ font-size:20px; font-weight:800; }
.timebar .times{ font-size:16px; opacity:0.95; display:flex; gap:18px; justify-content:center; flex-wrap:wrap; }
.kv .mono{font-variant-numeric: tabular-nums;}
/* Smaller charts */
canvas{width:100%;max-width:100%;height:auto}
@media (max-width: 600px){
  details.card > summary{font-size:18px}
  .timebar .date{font-size:18px}
  .timebar .times{font-size:15px}
}
.pre{white-space:pre-wrap;font-size:14px;line-height:1.4;color:var(--fg2);}
</style>
</head>
<body>
<div class="wrap">
<div class="title"><span class="cap">R</span>emote <span class="cap">E</span>nviroment <span class="cap">S</span>urveillance <span class="cap">T</span>erminal</div>

<div class="timebar">
  <div class="date"><span id="ldate">-</span></div>
  <div class="times">
    <div>Local: <span id="ltime" class="mono">-</span></div>
    <div>UTC: <span id="utime" class="mono">-</span></div>
  </div>
</div>

<div class="stack">

  <details class="card" open>
    <summary>Enviroment</summary>
    <div class="section">
      <div class="kv">
      <div><b>Sensor:</b> <span id="sensor">-</span></div>
      <div><b>Temp:</b> <span id="t">-</span> °C</div>
      <div><b>Press:</b> <span id="p">-</span> hPa</div>
      <div><b>Hum:</b> <span id="h">-</span> %</div>
      <div style="margin-top:12px">
        <label for="tcal" style="margin:6px 0 6px">Temperature correction (°C)</label>
        <div class="row" style="margin-top:6px;gap:8px;align-items:center;flex-wrap:wrap">
          <button id="tcalminus" type="button" style="width:auto;padding:8px 12px;font-size:16px">-</button>
          <input id="tcal" type="number" min="-20" max="20" step="0.01" value="0" style="width:120px">
          <button id="tcalplus" type="button" style="width:auto;padding:8px 12px;font-size:16px">+</button>

          <div style="display:flex;align-items:center;gap:8px">
            <span class="mono" style="opacity:.85">step</span>
            <select id="tcalstep" style="width:auto">
              <option value="0.01">0.01</option>
              <option value="0.1" selected>0.1</option>
              <option value="0.5">0.5</option>
              <option value="1">1.0</option>
            </select>
          </div>

          <div style="flex:1;min-width:260px">
            <b>Raw:</b> <span id="tcalraw">-</span> °C&nbsp;&nbsp;
            <b>Corrected:</b> <span id="tcalcorr">-</span> °C&nbsp;&nbsp;
            <b>Δ:</b> <span id="tcalv">0.00</span> °C
          </div>

          <button id="tcalsave" type="button" style="width:auto;padding:8px 14px;font-size:14px">Save</button>
        </div>

        <div class="row" style="margin-top:10px;gap:8px;align-items:center;flex-wrap:wrap">
          <span class="mono" style="opacity:.85">Match external reference</span>
          <input id="tcalref" type="number" step="0.01" placeholder="ref °C" style="width:120px">
          <button id="tcalmatch" type="button" style="width:auto;padding:8px 14px;font-size:14px">Auto-calc</button>
          <span class="mono" id="tcalmatchmsg" style="opacity:.85"></span>
        </div>
      </div>
      <hr>
      </div>
      <hr>
      <div class="row">
    <div style="flex:1">
      <label>Range</label>
      <select id="crange">
        <option value="day">Day</option>
        <option value="week">Week</option>
        <option value="month">Month</option>
        <option value="year">Year</option>
      </select>
    </div>
    <div style="flex:1">
      <label>Interval</label>
      <select id="cint">
        <option value="auto">Auto</option>
        <option value="60s">1 min</option>
        <option value="5m">5 min</option>
        <option value="15m">15 min</option>
        <option value="1h">1 hour</option>
        <option value="6h">6 hours</option>
        <option value="1d">1 day</option>
      </select>
    </div>
    <div style="flex:1">
      <button id="crefresh" class="btn">Refresh</button>
    </div>
  </div>

  <div class="chartwrap">
    <div class="charttitle">Temperature (°C)</div>
    
  </div>

  
  <div class="small">Data source: local SPIFFS log (1/min). If time is not valid yet, charts stay empty.</div>
      <div class="small" style="margin-top:8px;opacity:0.9">Range/interval apply to temperature chart (SPIFFS history).</div>
      <canvas id="ctemp" width="600" height="130"></canvas>
    </div>
  </details>

  <details class="card" open>
    <summary>Audio control</summary>
    <div class="section">
      <div class="row" style="align-items:center;gap:10px">
        <div style="flex:1">
          <label>Audio level</label>
          <div class="row" style="gap:10px;align-items:center">
            <div style="flex:1">
              <div class="bar"><div id="vuBar" class="barin" style="width:0%"></div></div>
            </div>
</div>
        </div>
      </div>
<hr>
      <label for="gain" style="margin:6px 0 6px">Gain (dB)</label>
    <div class="row" style="margin-top:6px;gap:8px;align-items:center;flex-wrap:wrap">
      <button id="gainminus" type="button" style="width:auto;padding:8px 12px;font-size:16px">-</button>
      <input id="gain" type="number" min="-24" max="24" step="1" value="0" style="width:120px">
      <button id="gainplus" type="button" style="width:auto;padding:8px 12px;font-size:16px">+</button>

      <div style="display:flex;align-items:center;gap:8px">
        <span class="mono" style="opacity:.85">step</span>
        <select id="gainstep" style="width:auto">
          <option value="0.5">0.5</option>
          <option value="1" selected>1</option>
          <option value="2">2</option>
          <option value="3">3</option>
          <option value="6">6</option>
        </select>
      </div>

      <button id="gainsave" type="button" style="width:auto;padding:8px 12px">Apply</button>
    </div>
    <div class="kv"><b>Current:</b> <span id="gainv">0</span> dB</div>

    <div class="toggle" style="margin-top:12px">
      <input id="dnr" type="checkbox">
      <label for="dnr" style="margin:0">Dynamic noise reduction</label>
    </div>

    <label>Sample rate</label>
    <div class="row">
      <label class="toggle"><input type="radio" name="sr" value="44100"> 44.1 kHz</label>
      <label class="toggle"><input type="radio" name="sr" value="22050"> 22.05 kHz</label>
      <label class="toggle"><input type="radio" name="sr" value="11025"> 11.025 kHz</label>
    </div>

    <div class="small" style="margin-top:8px">Changing sample rate requires reconnecting the player.</div>
  <div class="row">
    <div class="lbl">Format</div>
    <div class="ctl">
      <label><input type="radio" name="fmt" id="fmtwav" value="wav"> WAV</label>&nbsp;&nbsp;
      <label><input type="radio" name="fmt" id="fmtopus" value="opus"> OPUS (VBR)</label>
      <div class="hint">Opus requires encoder support; if disabled, /opus returns 501.</div>
      <div class="subrow">
        <label>Target kbps <input id="opusbr" type="number" min="6" max="256" step="1" value="32" style="width:6em"></label>&nbsp;&nbsp;
        <label>Complexity <input id="opuscx" type="number" min="0" max="10" step="1" value="5" style="width:5em"></label>&nbsp;&nbsp;<span class="hint">Channels</span>&nbsp;<label class="toggle" style="display:inline-flex;align-items:center;gap:6px"><input type="radio" name="opusch" value="2"> Stereo</label>&nbsp;<label class="toggle" style="display:inline-flex;align-items:center;gap:6px"><input type="radio" name="opusch" value="1"> Mono</label>
        <button class="btnTiny" id="openStream" type="button" title="Open audio stream in a new tab">Open stream</button>
        <button class="btnTiny" id="fmtsave" type="button">Save</button>
      </div>
    </div>
  </div>
    </div>
  </details>

  <details class="card" open>
    <summary>MySQL</summary>
    <div class="section">
      <div class="row" style="align-items:center;gap:14px;flex-wrap:wrap">
        <div><b>DB:</b> <span id="dbStat">-</span></div>
        <div><b>Backlog:</b> <span id="backlogKB">0</span> KB</div>
      </div>
      <hr>
      <div class="toggle">
      <input id="men" type="checkbox">
      <label for="men" style="margin:0">Enable logging</label>
    </div>

    <div class="inline">
      <div>
        <label for="mhost">Host</label>
        <input id="mhost" placeholder="yo4tnv.go.ro">
      </div>
      <div style="max-width:140px">
        <label for="mport">Port</label>
        <input id="mport" inputmode="numeric" placeholder="3306">
      </div>
    </div>

    <div class="inline">
      <div>
        <label for="muser">User</label>
        <input id="muser" placeholder="bme280">
      </div>
      <div>
        <label for="mpass">Password</label>
        <input id="mpass" type="password" placeholder="••••••">
      </div>
    </div>

    <div class="inline">
      <div>
        <label for="mdb">Database</label>
        <input id="mdb" placeholder="weather">
      </div>
      <div>
        <label for="mtbl">Table</label>
        <input id="mtbl" placeholder="focsani">
      </div>
    </div>

    <div class="row" style="margin-top:10px">
      <div style="flex:1"><button id="msave">Save</button></div>
      <div style="flex:1"><button id="mtest">Test</button></div>
    </div>
    <div class="small" id="mmsg" style="margin-top:8px"></div>
    </div>
  </details>

  <details class="card">
    <summary>Info</summary>
    <div class="section">
      <div class="kv">
        <div><b>SSID:</b> <span id="ssid">-</span></div>
        <div><b>Local IP:</b> <span id="lip">-</span></div>
        <div><b>Public IP:</b> <span id="pip">-</span></div>
        <div><b>Streaming:</b> <span id="stream">-</span></div>
        <div><b>Firmware:</b> <span id="fwfile">-</span></div>
        <div><b>Build:</b> <span id="fwbuild">-</span></div>
        <div><b>Installed:</b> <span id="fwinst">-</span></div>
      </div>
      <div class="small" style="margin-top:10px">Streams: <span class="pill">/wav</span> <span class="pill">/opus</span></div>
      <hr>
      <label>ESP32 specifications</label>
      <div id="specs_pre" class="pre"></div>
    </div>
  </details>

</div>
</div>
<script>
const $ = (id)=>document.getElementById(id);
// Temperature correction: increment (+/-) + numeric input + persistent step + auto-match
window.addEventListener('load', ()=>{
  const tc = $("tcal");
  if(!tc) return;

  const tv = $("tcalv");
  const stepSel = $("tcalstep");
  const btnMinus = $("tcalminus");
  const btnPlus  = $("tcalplus");
  const rawSpan  = $("tcalraw");
  const corSpan  = $("tcalcorr");
  const refIn    = $("tcalref");
  const btnMatch = $("tcalmatch");
  const msg      = $("tcalmatchmsg");

  const clamp = (v, lo, hi)=> Math.min(hi, Math.max(lo, v));
  const getStep = ()=> {
    const s = stepSel ? Number(stepSel.value) : NaN;
    return (Number.isFinite(s) && s>0) ? s : 0.1;
  };

  const showOffset = ()=>{
    editingTcal = true;
    if(tcalEditTimer) clearTimeout(tcalEditTimer);
    const v = Number(tc.value || 0);
    if(tv) tv.textContent = v.toFixed(2);
    tcalEditTimer = setTimeout(()=>{ editingTcal = false; }, 900);
  };

  const nudge = (dir)=>{
    const s = getStep();
    const lo = Number(tc.min ?? -10), hi = Number(tc.max ?? 10);
    const v0 = Number(tc.value || 0);
    tc.value = String(clamp(v0 + dir*s, lo, hi));
    showOffset();
  };

  // Long-press / hold-to-repeat (+/-)
  const bindHold = (btn, dir)=>{
    if(!btn) return;
    let timer=null, interval=null;
    const stop=()=>{
      if(timer){ clearTimeout(timer); timer=null; }
      if(interval){ clearInterval(interval); interval=null; }
    };
    const start=()=>{
      stop();
      nudge(dir); // immediate
      timer=setTimeout(()=>{
        interval=setInterval(()=>nudge(dir), 140);
      }, 320);
    };
    btn.addEventListener('pointerdown', (ev)=>{ ev.preventDefault(); start(); });
    btn.addEventListener('pointerup', stop);
    btn.addEventListener('pointerleave', stop);
    btn.addEventListener('pointercancel', stop);
  };
  bindHold(btnMinus, -1);
  bindHold(btnPlus,  +1);

  // Also allow single click (desktop)
  if(btnMinus) btnMinus.addEventListener('click', ()=> nudge(-1));
  if(btnPlus)  btnPlus.addEventListener('click', ()=> nudge(+1));

  // Save offset
  const btnSave = $("tcalsave");
  if(btnSave){
    btnSave.addEventListener('click', ()=>{
      const v = String(tc.value ?? "0");
      editingTcal = false;
      fetch(`/tcal?off=${encodeURIComponent(v)}`).catch(()=>{});
    });
  }

  // Persist step in NVS
  if(stepSel){
    stepSel.addEventListener('change', ()=>{
      const s = String(stepSel.value);
      fetch(`/tcal.step?step=${encodeURIComponent(s)}`).catch(()=>{});
    });
  }

  // Keyboard nudges + enter-to-save
  tc.addEventListener('input', showOffset);
  tc.addEventListener('change', showOffset);
  tc.addEventListener('keydown', (ev)=>{
    if(ev.key === "Enter"){
      if(btnSave) btnSave.click();
    } else if(ev.key === "ArrowUp"){
      ev.preventDefault(); nudge(+1);
    } else if(ev.key === "ArrowDown"){
      ev.preventDefault(); nudge(-1);
    }
  });

  // Auto-calc from external reference temperature
  if(btnMatch && refIn){
    btnMatch.addEventListener('click', async ()=>{
      const ref = refIn.value;
      if(msg) msg.textContent = "…";
      try{
        const r = await fetch(`/tcal.match?ref=${encodeURIComponent(ref)}`, {cache:"no-store"});
        const j = await r.json();
        if(j && j.ok){
          if(tc) tc.value = Number(j.off).toFixed(2);
          showOffset();
          if(msg) msg.textContent = "OK";
        } else {
          if(msg) msg.textContent = "No raw temp";
        }
      }catch(_){
        if(msg) msg.textContent = "ERR";
      } finally {
        setTimeout(()=>{ if(msg) msg.textContent=""; }, 1200);
      }
    });
  }

  // Init from device (offset + step + live raw/corr if available)
  (async ()=>{
    try{
      const r = await fetch("/tcal.json", {cache:"no-store"});
      const j = await r.json();
      if(j){
        if(!editingTcal && typeof j.off === "number") {
          tc.value = j.off.toFixed(2);
          showOffset();
        }
        if(stepSel && typeof j.step === "number"){
          const s = String(j.step);
          // pick closest option
          const opts=[...stepSel.options].map(o=>Number(o.value));
          let best=opts[0], bd=1e9;
          for(const v of opts){ const d=Math.abs(v-j.step); if(d<bd){bd=d; best=v;} }
          stepSel.value = String(best);
        }
        if(rawSpan && j.traw!=null)  rawSpan.textContent = Number(j.traw).toFixed(2);
        if(corSpan && j.tcorr!=null) corSpan.textContent = Number(j.tcorr).toFixed(2);
      }
    }catch(_){}
    showOffset();
  })();
});

function markMysqlEditing(){ editingMysql=true; }
["mhost","mport","muser","mpass","mdb","mtbl","men"].forEach(id=>{
  const el=$(id);
  el.addEventListener('focus', markMysqlEditing);
  el.addEventListener('input', markMysqlEditing);
  el.addEventListener('change', markMysqlEditing);
});


// Gain (dB) numeric adjuster (same style as temperature correction)
$("gain").addEventListener('focus', ()=>{editingGain=true;});
$("gain").addEventListener('blur', ()=>{setTimeout(()=>editingGain=false,300);});

function clampGain(v){
  if (v < -24) v = -24;
  if (v >  24) v =  24;
  return v;
}
function gainStep(){
  const sel = $("gainstep");
  if(!sel) return 1;
  const fv = parseFloat(sel.value);
  return isNaN(fv) ? 1 : fv;
}
function showGain(){
  const v = parseFloat($("gain").value);
  $("gainv").textContent = isNaN(v) ? "0" : String(v);
}
async function sendGain(){
  let v = clampGain(parseFloat($("gain").value) || 0);
  if (Math.abs(gainStep()-0.5) < 1e-6) v = Math.round(v*2)/2;
  else v = Math.round(v);
  $("gain").value = v;
  showGain();
  try{ await fetch(`/gain?db=${encodeURIComponent(String(Math.round(v)))}`,{cache:"no-store"}); }catch(_){}
}
function gainNudge(dir){
  let cur = parseFloat($("gain").value) || 0;
  let v = cur + dir*gainStep();
  v = clampGain(v);
  if (Math.abs(gainStep()-0.5) < 1e-6) v = Math.round(v*2)/2;
  else v = Math.round(v);
  $("gain").value = v;
  showGain();
  sendGain();
}
function addHold(btnId, dir){
  const btn = $(btnId);
  if(!btn) return;
  let t=null, i=null;
  const start=()=>{
    editingGain=true;
    gainNudge(dir);
    t=setTimeout(()=>{ i=setInterval(()=>gainNudge(dir), 120); }, 350);
  };
  const stop=()=>{
    if(t){clearTimeout(t); t=null;}
    if(i){clearInterval(i); i=null;}
    setTimeout(()=>editingGain=false,300);
  };
  btn.addEventListener('pointerdown', (ev)=>{ev.preventDefault(); start();});
  btn.addEventListener('pointerup', stop);
  btn.addEventListener('pointercancel', stop);
  btn.addEventListener('pointerleave', stop);
}
addHold("gainminus", -1);
addHold("gainplus", +1);

$("gain").addEventListener('input', showGain);
$("gain").addEventListener('keydown', (ev)=>{
  if(ev.key === "Enter"){
    const b=$("gainsave"); if(b) b.click();
  } else if(ev.key === "ArrowUp"){
    ev.preventDefault(); gainNudge(+1);
  } else if(ev.key === "ArrowDown"){
    ev.preventDefault(); gainNudge(-1);
  }
});

const gsave = $("gainsave");
if(gsave) gsave.addEventListener('click', ()=>sendGain());

const gstep = $("gainstep");
if(gstep){
  gstep.addEventListener('change', ()=>{
    const s = String(gstep.value);
    fetch(`/gain.step?step=${encodeURIComponent(s)}`).catch(()=>{});
  });
}

showGain();

async function jget(url){
  const r=await fetch(url,{cache:"no-store"});
  return await r.json();
}

async function refreshEnv(){
  try{
    const e=await jget("/env.json");
    $("sensor").textContent = e.sensor ?? "-";
    $("t").textContent = (e.t==null) ? "-" : e.t.toFixed(2);
    $("p").textContent = (e.p==null) ? "-" : e.p.toFixed(1);
    $("h").textContent = (e.h==null) ? "-" : e.h.toFixed(1);

    // Live raw vs corrected
    const rs=$("tcalraw");  if(rs) rs.textContent = (e.traw==null) ? "-" : e.traw.toFixed(2);
    const cs=$("tcalcorr"); if(cs) cs.textContent = (e.tcorr==null) ? "-" : e.tcorr.toFixed(2);

    if(e.toff!=null){
      // Δ display always reflects current offset unless user is actively editing
      if(!editingTcal){
        const tc=$("tcal"); if(tc) tc.value = Number(e.toff).toFixed(2);
        const tv=$("tcalv"); if(tv) tv.textContent = Number(e.toff).toFixed(2);
      }
    }
  }catch(_){}
}

async function refreshTime(){
  try{
    const t=await jget("/time.json");
    const ls = (t.local ?? "-");
    const us = (t.utc ?? "-");
    const ldate = (ls.length>=10) ? ls.substring(0,10) : ls;
    const ltime = (ls.length>=19) ? ls.substring(11,19) : (ls.length>11 ? ls.substring(11) : "-");
    const utime = (us.length>=19) ? us.substring(11,19) : (us.length>11 ? us.substring(11) : "-");
    $("ldate").textContent = ldate;
    $("ltime").textContent = ltime;
    $("utime").textContent = utime;
  }catch(_){}
}

async function refreshNet(){
  try{
    const n=await jget("/net.json");
    $("ssid").textContent = n.ssid ?? "-";
    $("lip").textContent = n.local_ip ?? "-";
    $("pip").textContent = n.public_ip ?? "-";
  }catch(_){}
}

async function refreshStream(){
  try{
    const s=await jget("/stream.json");
    if(s && s.active){
      const c = (s.codec && s.codec.length) ? s.codec : "?";
      const sr = (s.sr && s.sr>0) ? (s.sr + " Hz") : "";
      $("stream").textContent = "connected: " + (s.ip ?? "-") + " (" + c + (sr?(" " + sr):"") + ")";
    }else{
      $("stream").textContent = "no";
    }
  }catch(_){}
}


async function refreshAudio(){
  try{
    const g=await jget("/gain.json");
    if(!editingGain){
      const db = (typeof g.db==="number") ? g.db : parseInt(g.db);
      if(!isNaN(db)) {
        $("gain").value = db;
        $("gainv").textContent = db;
        if(g.step !== undefined && $("gainstep")) {
          const st = String(g.step);
          if($("gainstep").value !== st) $("gainstep").value = st;
        }
      }
    }
  }catch(_){}
  try{
    const d=await jget("/dnr.json");
    $("dnr").checked = !!d.dnr_on;
  }catch(_){}
  try{
    const s=await jget("/sr.json");
    const hz = s.hz;
    document.querySelectorAll('input[name="sr"]').forEach(r=>{ r.checked = (parseInt(r.value)==hz); });
  }catch(_){}
  if(!editingFmt){
    try{
      const f=await jget("/fmt.json");
      const fmt = (f.fmt==1) ? "opus" : "wav";
      const wav = document.querySelector('input[name="fmt"][value="wav"]');
      const op  = document.querySelector('input[name="fmt"][value="opus"]');
      if(wav) wav.checked = (fmt=="wav");
      if(op)  op.checked  = (fmt=="opus");
      if($("opusbr")) $("opusbr").value = Math.max(6, Math.min(256, Math.round((f.opus_br||32000)/1000)));
      if($("opuscx")) $("opuscx").value = (f.opus_cx ?? 5);
      const ch = (f.opus_ch==2) ? 2 : 1;
      document.querySelectorAll('input[name="opusch"]').forEach(r=>{ r.checked = (parseInt(r.value)==ch); });
    }catch(_){}
  }
}

async function refreshMysql(){
  if(editingMysql) return;
  try{
    const m=await jget("/mysql.json");
    $("men").checked = !!m.enabled;
    $("mhost").value = m.host ?? "";
    $("mport").value = m.port ?? "";
    $("muser").value = m.user ?? "";
    $("mpass").value = ""; // never auto-fill password
    $("mdb").value = m.db ?? "";
    $("mtbl").value = m.table ?? "";
  }catch(_){}
}

async function refreshSpecs(){
  try{
    const s=await jget("/specs.json");
    $("specs_pre").innerHTML =
      `<div><b>Chip:</b> ${s.chip}</div>`+
      `<div><b>Cores:</b> ${s.cores} &nbsp; <b>Rev:</b> ${s.rev} &nbsp; <b>CPU:</b> ${s.mhz} MHz</div>`+
      `<div><b>Flash:</b> ${s.flash_mb} MB</div>`+
      `<div><b>Heap free:</b> ${s.heap_free} &nbsp; <b>Min:</b> ${s.heap_min} &nbsp; <b>Largest:</b> ${s.heap_largest}</div>`+
      `<div><b>PSRAM:</b> ${s.psram ? "YES" : "NO"} ${s.psram ? ("("+s.psram_mb+" MB, free "+s.psram_free+")") : ""}</div>`+
      `<div><b>Fallback max:</b> ${s.fallback_kb} KB &nbsp; <b>Flush batch:</b> ${s.flush_rows} rows</div>`;
    // Firmware info
    if($("fwfile")) $("fwfile").textContent = s.fw_file ?? "-";
    if($("fwbuild")) $("fwbuild").textContent = s.fw_build ?? "-";
    if($("fwinst")) $("fwinst").textContent = (s.fw_install && String(s.fw_install).length) ? s.fw_install : "-";

  }catch(_){}
}

$("gain").addEventListener('change', ()=>{ sendGain(); });

$("dnr").addEventListener('change', async ()=>{
  try{ await fetch(`/dnr?on=${$("dnr").checked?1:0}`,{cache:"no-store"}); }catch(_){}
});

document.querySelectorAll('input[name="sr"]').forEach(r=>{
  r.addEventListener('change', async ()=>{
    try{ await fetch(`/sr?hz=${encodeURIComponent(r.value)}`,{cache:"no-store"}); }catch(_){}
  });
});

if($("openStream")){
  $("openStream").addEventListener('click', ()=>{
    const fmtEl = document.querySelector('input[name="fmt"]:checked');
    const fmt = fmtEl ? fmtEl.value : "wav";
    const url = `${location.protocol}//${location.hostname}:81/${fmt}`;
    window.open(url, "_blank");
  });
}

$("msave").addEventListener('click', async ()=>{
  const en=$("men").checked?1:0;
  const host=$("mhost").value.trim();
  const port=$("mport").value.trim();
  const user=$("muser").value.trim();
  const pass=$("mpass").value; // may be empty => keep old if empty (server side)
  const db=$("mdb").value.trim();
  const tbl=$("mtbl").value.trim();
  const url = `/mysql.save?en=${en}&host=${encodeURIComponent(host)}&port=${encodeURIComponent(port)}&user=${encodeURIComponent(user)}&pass=${encodeURIComponent(pass)}&db=${encodeURIComponent(db)}&tbl=${encodeURIComponent(tbl)}`;
  $("mmsg").textContent="saving...";
  try{
    const r=await fetch(url,{cache:"no-store"});
    const j=await r.json();
    $("mmsg").innerHTML = j.ok ? `<span class="ok">saved</span>` : `<span class="bad">save failed</span>`;
    editingMysql=false;
  }catch(e){
    $("mmsg").innerHTML = `<span class="bad">save failed</span>`;
  }
});

$("mtest").addEventListener('click', async ()=>{
  $("mmsg").textContent="testing...";
  try{
    const r=await fetch("/mysql.test",{cache:"no-store"});
    const j=await r.json();
    $("mmsg").innerHTML = j.ok ? `<span class="ok">${j.msg}</span>` : `<span class="bad">${j.msg}</span>`;
  }catch(e){
    $("mmsg").innerHTML = `<span class="bad">test failed</span>`;
  }
});

async function tick(){
  await refreshEnv();
  await refreshTime();
  await refreshNet();
  await refreshStream();
  await refreshAudio();
  await refreshMysql();
}
tick();
refreshSpecs();
setInterval(tick, 2000);
setInterval(refreshSpecs, 10000);
setInterval(refreshNet, 15000);

async function refreshStatus(){
  try{
    const s=await jget("/status.json");
    if(s && typeof s.vu==="number"){
      const vu=Math.max(0,Math.min(100,Math.round(s.vu)));
      $("vuBar").style.width = vu + "%";
      }
    if(s && typeof s.db_ok==="boolean"){
      $("dbStat").textContent = s.db_ok ? "OK" : "DOWN";
      $("dbStat").style.color = s.db_ok ? "#00ff00" : "#ff0000";
    }
    if(s && typeof s.backlog_bytes==="number"){
      const kb = Math.round(s.backlog_bytes/1024);
      $("bkSize").textContent = kb + " KB";
    }
  }catch(_){}
}

setOtaLink();

// ---- Charts (simple canvas plot, green on black) ----
function autoIntervalFor(range){
  if(range==="day") return "auto";
  if(range==="week") return "auto";
  if(range==="month") return "auto";
  return "auto";
}

function drawSeries(canvasId, pts, yLabel, isInt){
  const c = document.getElementById(canvasId);
  if(!c) return;
  const ctx = c.getContext('2d');
  const W = c.width, H = c.height;
  ctx.clearRect(0,0,W,H);

  // background grid
  ctx.strokeStyle = "#003300";
  ctx.lineWidth = 1;
  for(let i=0;i<=4;i++){
    const y = Math.round(i*(H/4));
    ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke();
  }

  if(!pts || pts.length<2){
    ctx.fillStyle = "#00ff00";
    ctx.font = "16px monospace";
    ctx.fillText("No data", 10, 24);
    return;
  }

  let ymin=Number.POSITIVE_INFINITY, ymax=Number.NEGATIVE_INFINITY;
  for(const p of pts){
    if(p.v<ymin) ymin=p.v;
    if(p.v>ymax) ymax=p.v;
  }
  if(ymin===ymax){ ymin-=1; ymax+=1; }
  const pad=10;
  const x0=pad, x1=W-pad;
  const y0=H-pad, y1=pad;

  // axis labels (min/max)
  ctx.fillStyle="#00ff00";
  ctx.font="14px monospace";
  ctx.fillText((isInt?Math.round(ymax):ymax.toFixed(1))+" "+yLabel, 10, 16);
  ctx.fillText((isInt?Math.round(ymin):ymin.toFixed(1))+" "+yLabel, 10, H-4);

  ctx.strokeStyle="#00ff00";
  ctx.lineWidth=2;
  ctx.beginPath();
  for(let i=0;i<pts.length;i++){
    const x = x0 + (i*(x1-x0))/(pts.length-1);
    const y = y0 - ((pts[i].v - ymin)*(y0-y1))/(ymax-ymin);
    if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.stroke();
}

async function fetchSeries(metric){
  const r = document.getElementById("crange")?.value || "day";
  const i = document.getElementById("cint")?.value || "auto";
  const res = await fetch(`/series.json?m=${encodeURIComponent(metric)}&r=${encodeURIComponent(r)}&i=${encodeURIComponent(i)}`);
  return await res.json();
}

async function refreshCharts(){
  try{
    const tpts = await fetchSeries("temp");
    drawSeries("ctemp", tpts, "°C", false);
}catch(e){}
}

document.addEventListener("DOMContentLoaded", ()=>{
  const b=document.getElementById("crefresh");
  if(b) b.addEventListener("click", refreshCharts);
  refreshCharts();
  setInterval(refreshCharts, 60000); // 1/min
});

</script>
</body>
</html>
)HTML";


  sendHtml(client, s);
}




void handleEnvJson(WiFiClient &client) {
  bmpPoll();
  float tRaw  = last_T;
  float tCorr = isnan(last_T) ? NAN : (last_T + g_temp_off_c);
  client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"));
  client.print(F("{\"t\":"));
  // Backward compatible: "t" is corrected temperature
  if (!isnan(tCorr)) client.print(tCorr, 3); else client.print(F("null"));
  client.print(F(",\"traw\":"));
  if (!isnan(tRaw)) client.print(tRaw, 3); else client.print(F("null"));
  client.print(F(",\"tcorr\":"));
  if (!isnan(tCorr)) client.print(tCorr, 3); else client.print(F("null"));
  client.print(F(",\"p\":"));
  if (!isnan(last_P)) client.print(last_P, 3); else client.print(F("null"));
  client.print(F(",\"h\":"));
  if (hasHumidity && !isnan(last_H)) client.print(last_H, 3); else client.print(F("null"));
  client.print(F(",\"sensor\":\""));
  client.print(hasHumidity ? F("BME280") : F("BMP280"));
  client.print(F("\",\"toff\":"));
  client.print(g_temp_off_c, 2);
  client.print(F("}\n"));
}





// --- HTML helpers (added for UI stability) ---
String htmlFooter() {
  return F("</body></html>\n");
}

void sendHtml(WiFiClient &client, const String &html) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/html; charset=utf-8"));
  client.println(F("Cache-Control: no-store, no-cache, must-revalidate, max-age=0"));
  client.println(F("Pragma: no-cache"));
  client.println(F("Connection: close"));
  client.println();
  client.print(html);
}


static void streamSetState(bool active, const IPAddress &ip) {
  portENTER_CRITICAL(&g_streamMux);
  g_streamActive = active;
  if (active) {
    snprintf(g_streamIP, sizeof(g_streamIP), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  } else {
    g_streamIP[0] = 0;
    g_streamCodec[0] = 0;
    g_streamSR = 0;
  }
  portEXIT_CRITICAL(&g_streamMux);
}


static void audioServerTask(void *pv) {
  (void)pv;
  Serial.printf("Audio server task running on core %d (port 81)\n", xPortGetCoreID());
  for (;;) {
    WiFiClient client = audioServer.available();
    if (!client) { vTaskDelay(1); continue; }

    // If already streaming, reject quickly
    bool busy;
    portENTER_CRITICAL(&g_streamMux);
    busy = g_streamActive;
    portEXIT_CRITICAL(&g_streamMux);

    String reqLine = client.readStringUntil('\n');
    reqLine.trim();

    // drain headers
    while (client.connected()) {
      String h = client.readStringUntil('\n');
      if (h == "\r" || h.length() == 0) break;
    }

    bool isWav  = reqLine.startsWith("GET /wav");
    bool isOpus = reqLine.startsWith("GET /opus");

    if (!isWav && !isOpus) {
      client.println("HTTP/1.1 404 Not Found");
      client.println("Connection: close");
      client.println();
      client.stop();
      continue;
    }

    if (busy) {
      client.println("HTTP/1.1 503 Service Unavailable");
      client.println("Content-Type: text/plain; charset=utf-8");
      client.println("Connection: close");
      client.println();
      client.println("stream busy");
      client.stop();
      continue;
    }

    streamSetState(true, client.remoteIP());


    // Update stream meta + reset I2S diagnostics
    portENTER_CRITICAL(&g_streamMux);
    strncpy(g_streamCodec, isWav ? "WAV" : "OPUS", sizeof(g_streamCodec));
    g_streamCodec[sizeof(g_streamCodec)-1] = 0;
    g_streamSR = (uint32_t)g_sample_rate;
    g_i2sUnderruns = 0;
    g_i2sReads = 0;
    g_i2sBytesRead = 0;
    portEXIT_CRITICAL(&g_streamMux);
    if (isWav) {
      streamWav(client);
    } else {
      streamOpus(client);
    }

    streamSetState(false, IPAddress(0,0,0,0));
    client.stop();
    vTaskDelay(1);
  }
}
void setup() {
  Serial.begin(115200);

  delay(200);
  detectEspSpecs();
  g_fsOK = initInternalFS();
  
// Tune chart log capacity based on actual SPIFFS size (keep <= ~33% of FS)
  if (g_fsOK) {
    size_t fsTot = SPIFFS.totalBytes();
    size_t cap = fsTot / 3;
    if (cap < 128*1024) cap = 128*1024;
    if (cap > 1024*1024) cap = 1024*1024;
    g_envlog_max_bytes = cap;
  }
autoTuneFromSpecs();
  delay(200);
  Serial.println();
  Serial.println("=== ESP32 Stereo WAV over HTTP + Gain UI + DNR + SR + Env + MySQL cfg ===");

  // Load persisted audio settings
  prefs.begin("audio", false);
  int storedDb = prefs.getInt("gain_db", g_gain_db);
  float storedGainStep = prefs.getFloat("gain_step", g_gain_step);
  uint32_t storedSr = (uint32_t)prefs.getUInt("sr_hz", g_sample_rate);
  bool storedDnr = prefs.getBool("dnr_on", g_dnr_on);
  uint8_t storedFmt = (uint8_t)prefs.getUChar("fmt", (uint8_t)FMT_WAV);
  g_gain_step = storedGainStep;
  uint32_t storedOpBr = (uint32_t)prefs.getUInt("opus_br", 32000);
  uint8_t storedOpCx = (uint8_t)prefs.getUChar("opus_cx", 5);
  uint8_t storedOpCh = (uint8_t)prefs.getUChar("opus_ch", 1);
  prefs.end();
  // Load persisted environment settings
  prefs.begin("env", false);
  g_temp_off_c = prefs.getFloat("t_off", 0.0f);
  g_tcal_step  = prefs.getFloat("t_step", 0.10f);
  // sanitize step
  if (g_tcal_step < 0.001f) g_tcal_step = 0.01f;
  if (g_tcal_step > 5.0f)   g_tcal_step = 1.0f;
  prefs.end();

  g_audio_fmt = (AudioFmt)storedFmt;
  g_opus_target_bitrate = storedOpBr;
  g_opus_complexity = storedOpCx;
  g_opus_channels = (storedOpCh==2)?2:1;
  setGainDb(storedDb);
  g_sample_rate = storedSr;
  g_dnr_on = storedDnr;

  // Load persisted MySQL config (even if USE_MYSQL==0, UI uses it)
  mysqlLoadPrefs();

  // Firmware info (file/build/install time)
  fwInit();

  connectBestAP();
  timeInit();
  refreshPublicIP(true);
  // Try to stamp first-install time shortly after NTP sync (will also retry in loop)
  for (int i=0;i<20;i++){ fwMaybeStampInstallTime(); if(g_fw_install_local.length()) break; delay(50); }
  i2sInit();
  applySampleRate(g_sample_rate); // ensure runtime variable applied
  bmpInit();

  server.begin();
  Serial.println("HTTP server on :80 (WebUI)");

  audioServer.begin();
  Serial.println("Audio server on :81 (WAV/OPUS) ");
  xTaskCreatePinnedToCore(audioServerTask, "audioSrv", 8192, nullptr, 2, nullptr, 0);

  Serial.println("WAV : http://<ESP32-IP>:81/wav  (or /wav on :80 redirects)");
  Serial.println("UI : http://<ESP32-IP>/  (auth: vic / yo4tnv)");
}



// ===================== Small helpers =====================
static String getParam(const String &reqLine, const char *key) {
  int q = reqLine.indexOf('?');
  if (q < 0) return "";
  int sp = reqLine.indexOf(' ', q);
  String qs = reqLine.substring(q + 1, sp < 0 ? reqLine.length() : sp);
  String k = String(key) + "=";
  int p = qs.indexOf(k);
  if (p < 0) return "";
  p += k.length();
  int amp = qs.indexOf('&', p);
  String v = qs.substring(p, amp < 0 ? qs.length() : amp);
  v.replace("%2E", "."); v.replace("%3A", ":"); v.replace("%2F","/");
  v.replace("%40","@"); v.replace("+"," ");
  return v;
}

static void send401(WiFiClient &client) {
  client.println("HTTP/1.1 401 Unauthorized");
  client.println("WWW-Authenticate: Basic realm=\"ESP32\"");
  client.println("Content-Type: text/plain; charset=utf-8");
  client.println("Connection: close");
  client.println();
  client.println("Auth required");
}

static bool authOk(const String &authHeader) {
  // "vic:yo4tnv" => base64 => dmljOnBhcmF6aXRpaQ==
  const char *EXPECTED = "Basic dmljOnBhcmF6aXRpaQ==";
  return authHeader.indexOf(EXPECTED) >= 0;
}

static uint32_t mapOpusRate(uint32_t hz){
  if (hz==44100) return 48000;
  if (hz==22050) return 24000;
  if (hz==11025) return 12000;
  return hz;
}

static void applySampleRate(uint32_t hz) {
  if (hz != 44100 && hz != 22050 && hz != 11025) return;
  g_sample_rate = hz;
  // Keep I2S in 32-bit slot mode; we output 16-bit WAV (top bits used)
  uint32_t hw = (g_audio_fmt==FMT_OPUS) ? mapOpusRate(g_sample_rate) : g_sample_rate;
  i2s_set_clk(I2S_NUM_0, hw, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);
}



static void handleStatusJson(WiFiClient &client){
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();
  client.print("{\"vu\":");
  client.print((uint32_t)g_vu_pct);
  client.print(",\"db_ok\":");
  client.print(g_db_ok ? "true":"false");
  client.print(",\"backlog_bytes\":");
  client.print((uint32_t)mysqlBacklogBytes());
  client.print("}");
}

void loop() {
  bmpPoll();
  fwMaybeStampInstallTime();

  // Periodic MySQL logging (if enabled). 1/minute.
  uint32_t ms = millis();
  if (ms - g_last_mysql_ms >= 60000UL) {
    g_last_mysql_ms = ms;
    if (bmpOK && g_mysql.enabled) {
      float tCorr = isnan(last_T) ? NAN : (last_T + g_temp_off_c);
      mysqlLogOnce(tCorr, last_P, (hasHumidity ? last_H : NAN));
    }
  }


// Local chart logging (1/min) regardless of MySQL availability
if (bmpOK) {
  float tCorr2 = isnan(last_T) ? NAN : (last_T + g_temp_off_c);
  if (!isnan(tCorr2)) seriesLogOncePerMinute(tCorr2);
}

  WiFiClient client = server.available();
  if (!client) { delay(1); return; }

  // Request line
  String reqLine = client.readStringUntil('\n');
  reqLine.trim();

  // Read headers; capture Authorization
  String auth = "";
  String host = "";
  while (client.connected()) {
    String h = client.readStringUntil('\n');
    if (h == "\r" || h.length() == 0) break;
    h.trim();
    if (h.startsWith("Authorization:")) auth = h;
  }

  // Protect everything except /wav (so VLC can connect without auth)
  bool isWav  = reqLine.startsWith("GET /wav ");
  bool isOpus = reqLine.startsWith("GET /opus ");
  if (!isWav && !isOpus && !authOk(auth)) {
    send401(client);
    client.stop();
    return;
  }

  // Routing
  if (isWav || isOpus) {
    // Redirect streaming requests to dedicated audio server (:81) so WebUI stays responsive.
    String h = host;
    h.trim();
    if (h.length() == 0) h = WiFi.localIP().toString();
    // strip any existing port
    int c = h.indexOf(':');
    if (c >= 0) h = h.substring(0, c);
    String loc = String("http://") + h + ":81" + (isWav ? "/wav" : "/opus");
    client.println("HTTP/1.1 302 Found");
    client.print("Location: "); client.println(loc);
    client.println("Cache-Control: no-store");
    client.println("Connection: close");
    client.println();
  } else if (reqLine.startsWith("GET /env.json ")) {
    handleEnvJson(client);
  } else if (reqLine.startsWith("GET /time.json ")) {
    handleTimeJson(client);
  } else if (reqLine.startsWith("GET /net.json ")) {
    handleNetJson(client);
  } else if (reqLine.startsWith("GET /stream.json ")) {
    handleStreamJson(client);
  } else if (reqLine.startsWith("GET /metrics.json ")) {
    handleMetricsJson(client);
  } else if (reqLine.startsWith("GET /specs.json ")) {
    handleSpecsJson(client);
  } else if (reqLine.startsWith("GET /series.json")) {
    handleSeriesJson(client, reqLine);
  } else if (reqLine.startsWith("GET /gain.json ")) {
    handleGainJson(client);
  } else if (reqLine.startsWith("GET /gain.step")) {
    handleGainStepSet(client, reqLine);
  } else if (reqLine.startsWith("GET /tcal.json ")) {
    handleTcalJson(client);
  } else if (reqLine.startsWith("GET /tcal.step")) {
    handleTcalStepSet(client, reqLine);
  } else if (reqLine.startsWith("GET /tcal.match")) {
    handleTcalMatch(client, reqLine);
  } else if (reqLine.startsWith("GET /tcal")) {
    handleTcalSet(client, reqLine);
  } else if (reqLine.startsWith("GET /gain")) {
    handleGainSet(client, reqLine);
  } else if (reqLine.startsWith("GET /dnr.json ")) {
    handleDnrJson(client);
  } else if (reqLine.startsWith("GET /dnr")) {
    handleDnrSet(client, reqLine);
  } else if (reqLine.startsWith("GET /sr.json ")) {
    handleSrJson(client);
  } else if (reqLine.startsWith("GET /sr")) {
    handleSrSet(client, reqLine);
  } else if (reqLine.startsWith("GET /fmt.json ")) {
    handleFmtJson(client);
  } else if (reqLine.startsWith("GET /fmt")) {
    handleFmtSet(client, reqLine);
  } else if (reqLine.startsWith("GET /mysql.json ")) {
    handleMysqlJson(client);
  } else if (reqLine.startsWith("GET /mysql.test")) {
    handleMysqlTest(client);
  } else if (reqLine.startsWith("GET /mysql.save")) {
    handleMysqlSave(client, reqLine);
  } else {
    handleRoot(client);
  }


  client.flush();
  client.stop();
}
  
