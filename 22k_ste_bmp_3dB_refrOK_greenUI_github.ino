#include <WiFi.h>
#include <WiFiMulti.h>
#include <driver/i2s.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// ===================== Wi‑Fi (multi‑AP) =====================
WiFiMulti wifiMulti;
struct WifiAP { const char* ssid; const char* pass; };
static const WifiAP WIFI_APS[] = {
  {"SSID1", "passwd1"},   
  {"SSID2",   "passwd2"},  
  {"SSID3",    "passwd3"}
  
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

static const uint32_t SAMPLE_RATE = 22050;    // 22.05 kHz
static const uint16_t BITS        = 16;       // 16‑bit samples
static const uint16_t CHANNELS    = 2;        // stereo (duplicate mono into L/R if using single mic)

// +3 dB gain (Q15 fixed-point: ~1.4142)
static const int32_t GAIN_Q15 = 46341; // 1.41421356 * 32768
static const int      PKT_FRAMES  = 256;      // frames per write chunk

WiFiServer server(80);

// ===================== BMP280 =====================
Adafruit_BMP280 bmp;              // I2C
bool bmpOK = false;
float last_T = NAN, last_P = NAN;
uint32_t lastReadMs = 0;
const uint32_t READ_PERIOD_MS = 3000;   // ~1.5 Hz is fine

void bmpInit() {
  Wire.begin(); // SDA=21, SCL=22 by default on ESP32
  // Lock to address 0x76 as requested
  if (bmp.begin(0x76)) { bmpOK = true; }
  else { bmpOK = false; }
  if (bmpOK) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,    // Temp oversampling
                    Adafruit_BMP280::SAMPLING_X16,   // Pressure oversampling
                    Adafruit_BMP280::FILTER_X4,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("BMP280: initialized @ 0x76");
  } else {
    Serial.println("BMP280: NOT found at 0x76");
  }
}

static inline void bmpPoll() {
  if (!bmpOK) return;
  uint32_t now = millis();
  if (now - lastReadMs < READ_PERIOD_MS) return;
  lastReadMs = now;
  last_T = bmp.readTemperature();             // °C
  last_P = bmp.readPressure() / 100.0f;       // hPa
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
    .sample_rate = SAMPLE_RATE,
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
  i2s_set_sample_rates(I2S_NUM_0, SAMPLE_RATE);
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
  writeWavHeader(client, SAMPLE_RATE, BITS, CHANNELS);

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
    if (bytes_read < i2sChunk) continue; // short read; try again

    // Convert: take MS 16 bits of each 32‑bit slot
    int16_t *out = (int16_t*)buf.get();
    const int32_t *in32 = (const int32_t*)i2sRaw.get();
    for (size_t i=0;i<PKT_FRAMES*CHANNELS;i++) {
      int32_t s = in32[i] >> 14; // typical scaling for I2S mics (24‑bit left‑justified in 32‑bit)
      s = (s * GAIN_Q15) >> 15; // +3 dB
      out[i] = clamp16(s);
    }

    client.write(buf.get(), pktBytes);
    if (!client.connected()) break;
  }
}

String htmlHeader() {
  String s;
  s += F("<!doctype html><html><head><meta charset='utf-8'>");
  s += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  s += F("<title>ESP32 WAV + BMP280</title>");  s += F("<style>html,body{background:#000;color:#39ff14;font-family:system-ui,Segoe UI,Roboto,Arial;margin:16px}a,a:visited{color:#39ff14}.card{background:#000;border:1px solid #39ff1422;border-radius:12px;padding:12px;margin:12px 0}code{background:#000;border:1px solid #39ff1422;padding:2px 6px;border-radius:6px;color:#39ff14}input,select,button{background:#000;color:#39ff14;border:1px solid #39ff14;border-radius:8px;padding:6px 10px}hr{border-color:#39ff14}</style>");s += F("</head><body>");
  s += F("<h2>ESP32 Stereo WAV (22.05 kHz) + BMP280</h2>");
  return s;
}

void handleRoot(WiFiClient &client) {
  String s = htmlHeader();
  s += F("<div class='card'><h3>Streaming</h3><p>Open <a href='/wav'>/wav</a> in a player (e.g., VLC).<br>Direct URL: <code>http://");
  s += WiFi.localIP().toString();
  s += F("/wav</code></p></div>");

  // BMP card
  bmpPoll();
  s += F("<div class='card'><h3>Environment (BMP280)</h3>");
  s += F("<p><b>T</b>: <span id='t'>--</span> &deg;C &nbsp; <b>P</b>: <span id='p'>--</span> hPa</p>");
  s += F("<small>Last update: <span id='ts'>--</span></small>");
  s += F("<script>"
          "async function pollEnv(){"
            "try{"
              "const r=await fetch('/env.json',{cache:'no-store'});"
              "const j=await r.json();"
              "if(j&&typeof j.t==='number'){document.getElementById('t').textContent=j.t.toFixed(2);}else{document.getElementById('t').textContent='--';}"
              "if(j&&typeof j.p==='number'){document.getElementById('p').textContent=j.p.toFixed(1);}else{document.getElementById('p').textContent='--';}"
              "document.getElementById('ts').textContent=new Date().toLocaleTimeString();"
            "}catch(e){}"
          "}"
          "pollEnv();"
          "setInterval(pollEnv,3000);"
        "</script>");
  s += F("</div>");

  // Wi‑Fi info
  s += F("<div class='card'><h3>Wi‑Fi</h3><p>SSID: <b>"); s += WiFi.SSID(); s += F("</b><br>IP: <b>"); s += WiFi.localIP().toString(); s += F("</b></p></div>");

  s += F("<script>setInterval(async()=>{try{let r=await fetch('/env.json');if(!r.ok)return;let j=await r.json();let card=document.querySelectorAll('.card')[1];card.innerHTML='<h3>Environment (BMP280)</h3>'+(j.ok?`<p><b>T</b>: ${j.t.toFixed(2)} °C &nbsp; <b>P</b>: ${j.p.toFixed(1)} hPa &nbsp; <b>H</b>: ${j.h.toFixed(1)} %RH</p>`:'<p><i>BMP280 not available</i></p>');}catch(e){}}, 2000);</script>");
  s += F("</body></html>");

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html; charset=utf-8");
  client.println("Connection: close");
  client.println();
  client.print(s);
}

void handleEnvJson(WiFiClient &client) {
  bmpPoll();
  String s = F("{\"ok\":"); s += (bmpOK? "true":"false");
  s += F(",\"t\":"); s += isnan(last_T)?String("null"):String(last_T, 3);
  s += F(",\"p\":"); s += isnan(last_P)?String("null"):String(last_P, 3);
  s += F("}\n");

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json; charset=utf-8");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();
  client.print(s);
}


void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=== ESP32 Stereo WAV over HTTP (22 kHz) + BMP280 ===");

  connectBestAP();
  i2sInit();
  bmpInit();
  server.begin();
  Serial.println("HTTP server on :80");
  Serial.println("Open VLC → Media → Open Network Stream → http://<ESP32-IP>/wav");
}

void loop() {
  bmpPoll();
  WiFiClient client = server.available();
  if (!client) { delay(1); return; }

  // Wait for HTTP request line
  String reqLine = client.readStringUntil('\n');
  reqLine.trim();
  // Consume the rest of the headers quickly
  while (client.connected()) {
    String h = client.readStringUntil('\n');
    if (h == "\r" || h.length() == 0) break;
  }

  if (reqLine.startsWith("GET /wav ")) {
    streamWav(client);
  } else if (reqLine.startsWith("GET /env.json ")) {
    handleEnvJson(client);
  } else {
    handleRoot(client);
  }

  client.flush();
  client.stop();
}
