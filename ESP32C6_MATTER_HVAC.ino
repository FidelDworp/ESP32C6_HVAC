/* ESP32C6_MATTER_HVAC.ino = Centrale HVAC controller voor kelder (ESP32-C6) op basis van particle sketch voor Flobecq
Transition from Photon based to ESP32 based Home automation system. Developed together with ChatGPT & Grok in januari '26.
Thuis bereikbaar op static IP http://192.168.0.70  (mDNS verwijderd — conflict met Matter-stack)

Compileer met "partitions_16mb.csv" in de sketchfolder (app0 + app1 elk 6MB).

17mar26       Version v 1.18: ECO JSON keys aangepast aan nieuwe ECO sketch structuur:
              temp_top ETopH→b, temp_bottom EBotL→g, temp_avg EAv→h, qtot EQtot→i.
              Filter-doc bijgewerkt.
17mar26       Version v 1.17: Room JSON keys aangepast aan nieuwe room sketch structuur:
              heat_request y→b, vent_request z→g, setpoint aa→c,
              room_temp h→e, home_status af→v. Filter-doc bijgewerkt.
13mar26       Version v 1.16: Sliding window duty% per circuit (12 slots × 20 min = 4u rollend).
              duty_4h = representatief gemiddelde over laatste 4u → naar JSON/Sheets (keys i-o).
              duty_cycle blijft instantaan (lopend slot) voor live UI. Struct uitgebreid met
              dc_window[12], dc_slot, dc_slots_filled, dc_slot_start, dc_last_poll,
              dc_slot_on, duty_4h. Delta-tijdmeting per poll (dc_last_poll) voorkomt
              cumulatieve fout; clamp op 1200s per slot; noemer correct bij lege ring.
13mar26       Version v 1.15: TSTAT hardware pins (10/11/12) snelcheck elke 100ms in loop(),
              flankdetectie via tstat_last_state[], relay onmiddellijk bij TSTAT-wijziging,
              override heeft voorrang. Ventilatie max()-logica consistent doorgevoerd in
              set_vent handler, check_vent_override() en update_matter_sensors(). CSS fix:
              header-row td.value achtergrond blauw (specificiteitsconflict opgelost).
              NVS-keys via snprintf char buf i.p.v. String(i) (heap-allocs bij boot weg).
13mar26       Version v 1.14: Circuit override handlers schakelen relay onmiddellijk
              (mcp.digitalWrite + matter sync), niet meer wachten op pollcyclus.
              Cancel override herstelt relay naar auto-staat direct.
              Ventilatie effective = max(vent_rooms, vent_hvac_minimum) — pin en JSON altijd identiek.
12mar26       Version v 1.13: ArduinoJson v7 heap-fix: StaticJsonDocument volledig weg,
              globale JsonDocuments met clear() hergebruik voor pollRooms/pollEcoBoiler,
              buildLogJson() = pure snprintf static char buf (nul heap-alloc),
              http.getString() → http.getStream() + filter (elimineert ~1KB String/poll/circuit),
              /json_ui + /json_settings + /scan streamen direct via AsyncResponseStream.
12mar26       Version v 1.12: matter_boiler_bot + matter_alles_auto verwijderd (~7KB heap),
              /json gesplitst: compact a/b/c-keys voor Sheets, /json_ui circuits voor webpagina,
              KW* / ECO-velden uit JSON weg, SCH/WON pompstaat correct (relay+auto+manual),
              RSSI/heap/heap_block toegevoegd aan JSON, crash-log String→char[].
12mar26       Version v 1.11: Heap-optimalisaties: globale String→char[] (room_id/wifi/eco_ip/sensor_nicks),
              getPumpStatusMessage() verwijderd (status-banner weg), /matter chunked streaming (~3KB),
              getTrend/getFormattedDateTime→const char* (0 heap-allocs), StaticJson<4096→2048>,
              strcmp fix savePumpEvent, Refresh-knop weg.
12mar26       Version v 1.10: Nuclear Matter reset (nvs_flash_erase+restore, 100% betrouwbaar, settings intact),
              /settings hybrid (static shell + circuits via /json_settings + JS), heap-diagnose log bij boot.
12mar26       Version v 1.9: uptime toegevoegd aan /json, Circuit struct String→char[] (48 heap-allocs weg),
              circuits[16]→circuits[7], hybrid hoofdpagina (statische shell + circuits-tabel via JS/fetch /json).
12mar26       Version v 1.8: Matter 14→11 endpoints (boiler_mid + 2 fake-sensoren weg, ~10KB heap),
              getMainPage() + /settings chunked streaming via AsyncResponseStream (2× reserve(10000) weg),
              getLogData DynamicJson→StaticJson.
11mar26       Version v 1.7: #define Serial Serial0 (C6 fix), CONVERT_ALL DS18B20 (WDT crash fix: 4.5s→750ms),
              interval 2s→60s, DynamicJson→StaticJson in pollRooms/pollEcoBoiler/getWifi,
              crash-logging NVS (heap-bewaking + /settings weergave), largest_block UI kleurcode,
              mDNS verwijderd (Matter-conflict), goto statements weggewerkt.
3mar26        Version v 1.6: /matter pagina toegevoegd, serial R-reset gefixed, boot-tekst opgeruimd
2mar26 16:54  Version v 1.4: Matter integrated, nvs correcties
1mar26 16:54  Version v 1.3: Matter integrated
26feb26 17:30 Version v 1.2: Static IP setting geactiveerd: 192.168.0.70
10jan26 08:30 Version v 1.1: UI & JSON Improvements
*/

// v1.7 FIX 1: Verplicht voor ESP32-C6 (RISC-V) in Arduino IDE — zonder dit werkt Serial niet correct
#define Serial Serial0


// ============== DEEL 1/5: HEADERS, STRUCTS & HELPER FUNCTIES ==============

#include <WiFi.h>
#include <WiFiClientSecure.h>   // v1.13: HTTPS voor Google Apps Script push
// ESPmDNS verwijderd (v1.7 FIX 5) — veroorzaakte mdns_service_remove_for_host errors samen met Matter-stack
#include <DNSServer.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <Preferences.h>
#include <OneWireNg_CurrentPlatform.h>
#include <Adafruit_MCP23X17.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <Matter.h>
#include <MatterEndPoints/MatterTemperatureSensor.h>
#include <MatterEndPoints/MatterOnOffPlugin.h>
#include <MatterEndPoints/MatterFan.h>

Preferences preferences;

#define ONE_WIRE_PIN   3
#define I2C_SDA       13
#define I2C_SCL       11
#define VENT_FAN_PIN  20

OneWireNg_CurrentPlatform ow(ONE_WIRE_PIN, false);
Adafruit_MCP23X17 mcp;
AsyncWebServer server(80);
DNSServer dnsServer;
const byte DNS_PORT = 53;

// NVS keys
const char* NVS_ROOM_ID = "room_id";
const char* NVS_WIFI_SSID = "wifi_ssid";
const char* NVS_WIFI_PASS = "wifi_password";
const char* NVS_STATIC_IP = "static_ip";
const char* NVS_CIRCUITS_NUM = "circuits_num";
const char* NVS_SENSOR_NICK_BASE = "sensor_nick_";
const char* NVS_ECO_THRESHOLD = "eco_thresh";
const char* NVS_ECO_HYSTERESIS = "eco_hyst";
const char* NVS_POLL_INTERVAL = "poll_interval";
const char* NVS_ECO_IP = "eco_ip";
const char* NVS_ECO_MDNS = "eco_mdns";
const char* NVS_ECO_MIN_TEMP = "eco_min_temp";
const char* NVS_ECO_MAX_TEMP = "eco_max_temp";
const char* NVS_BOILER_REF_TEMP = "boil_ref_t";
const char* NVS_BOILER_VOLUME = "boil_vol";
const char* NVS_LAST_SCH_PUMP = "last_sch_pump";
const char* NVS_LAST_WON_PUMP = "last_won_pump";
const char* NVS_LAST_SCH_KWH = "last_sch_kwh";
const char* NVS_LAST_WON_KWH = "last_won_kwh";
const char* NVS_TOTAL_SCH_KWH = "tot_sch_kwh";
const char* NVS_TOTAL_WON_KWH = "tot_won_kwh";
const char* NVS_GAS_URL       = "gas_url";   // v1.13: Google Apps Script web app URL

// Structs
struct Circuit {
  // v1.9: char[] i.p.v. String — elimineer 3 heap-allocs per circuit (was 48 totaal voor circuits[16])
  char name[32];
  char ip[20];
  char mdns[32];
  float power_kw;
  bool has_tstat;
  int tstat_pin;
  bool online;
  unsigned long last_seen;
  bool heating_on;
  int vent_request;
  unsigned long on_time;
  unsigned long off_time;
  unsigned long last_change;
  float duty_cycle;   // instantaan (lopend slot) — voor live UI
  int setpoint;
  float room_temp;
  bool heat_request;
  int home_status;
  bool override_active;
  bool override_state;
  unsigned long override_start;
  // v1.16: sliding window duty% over 4u (12 slots × 20 min)
  uint16_t dc_window[12];      // on-seconden per afgesloten slot (max 1200s → past in uint16)
  uint8_t  dc_slot;            // huidig actief slot (0..11)
  uint8_t  dc_slots_filled;    // aantal afgesloten slots (0..12), voor correcte noemer
  uint32_t dc_slot_start;      // millis() bij start van huidig slot
  uint32_t dc_last_poll;       // millis() bij vorige pollcyclus, voor delta-berekening
  uint16_t dc_slot_on;         // on-seconden opgebouwd in huidig slot
  float    duty_4h;            // rollend gemiddelde over gevulde slots → naar JSON/Sheets
};

struct EcoBoilerData {
  bool online;
  float temp_avg;
  float qtot;
  float temp_top;
  float temp_bottom;
  unsigned long last_seen;
};

struct PumpEvent {
  unsigned long timestamp;
  float kwh_pumped;
};

// Global variables
// v1.11: String→char[] — permanente heap-allocs weg, BSS/data segment
char room_id[32]         = "HVAC";
char wifi_ssid[64]       = "";
char wifi_pass[64]       = "";
char static_ip_str[20]   = "";
IPAddress static_ip;
int circuits_num = 7;
Circuit circuits[7];  // v1.9: was [16] — 9 lege slots = 27 onnodige heap-allocs
char sensor_nicknames[6][32];
float eco_threshold = 15.0;  // V53.5: Updated default
float eco_hysteresis = 5.0;  // V53.5: Updated default
int poll_interval = 10;
char eco_controller_ip[20]   = "";
char eco_controller_mdns[32] = "eco";
char gas_url[256]            = "";    // v1.13: Google Apps Script web app URL (leeg = uitgeschakeld)
unsigned long last_gas_push  = 0;     // v1.13: tijdstempel laatste GAS push
float eco_min_temp = 80.0;  // V53.5: Stop temp (Tmin)
float eco_max_temp = 90.0;  // V53.5: Start temp (Tmax)
float boiler_ref_temp = 20.0;
float boiler_layer_volume = 50.0;

#define RELAY_PUMP_SCH 8
#define RELAY_PUMP_WON 9

int vent_percent = 0;
float total_power = 0.0;  // Matter: globaal i.p.v. lokaal in pollRooms()

// Ventilatie override (Matter)
int           vent_override_percent  = 0;
bool          vent_override_active   = false;
unsigned long vent_override_start    = 0;
const unsigned long VENT_OVERRIDE_DURATION = 180UL * 60UL * 1000UL;  // 3 uur

// Matter flags
bool ignore_callbacks       = false;
// v1.12: alles_auto_requested verwijderd (matter_alles_auto endpoint weg)
bool matter_nuclear_reset_requested = false;
unsigned long last_matter_update = 0;

// =============================================================================
// Matter endpoints (identiek aan ESP32-C6_HVAC_SIM)
// =============================================================================
// v1.8: 14→11 endpoints (ref.doc max=12, veiliger op 11)
// Verwijderd: matter_boiler_mid (sch_temps[2]), matter_sch_qtot (FAKE kWh), matter_total_power (FAKE kW)
// Besparing: ~3 × 3.5 KB = ~10 KB heap
MatterTemperatureSensor matter_boiler_top;   // sch_temps[0]  — bovenste laag
// v1.12: matter_boiler_bot verwijderd (~3.5KB) — bovenste laag is voldoende indicator
// v1.12: matter_alles_auto verwijderd (~3.5KB) — overrides via webUI resetten

MatterOnOffPlugin       matter_circuit[7];   // Kringen 1–7, bidirectioneel

MatterFan               matter_vent;         // Ventilatie: snelheid + aan/uit

float sch_temps[6] = {-127,-127,-127,-127,-127,-127};
float eco_temps[6] = {-127,-127,-127,-127,-127,-127};
bool sensor_ok[6] = {false};
float sch_qtot = 0.0;
float eco_qtot = 0.0;

EcoBoilerData eco_boiler = {false, 0.0, 0.0, 0.0, 0.0, 0};

// Last pump events
PumpEvent last_sch_pump = {0, 0.0};
PumpEvent last_won_pump = {0, 0.0};

// Cumulatieve totalen (persistent in NVS)
float total_sch_kwh = 0.0;
float total_won_kwh = 0.0;

// V53.5: NIEUWE pump state machine met afzonderlijke wait states
enum EcoPumpState { ECO_IDLE, ECO_PUMP_SCH, ECO_WAIT_SCH, ECO_PUMP_WON, ECO_WAIT_WON };
EcoPumpState eco_pump_state = ECO_IDLE;
unsigned long eco_pump_timer = 0;
bool last_pump_was_sch = false;

// V53.5: Nieuwe tijdconstanten (1 min cycles!)
const unsigned long ECO_PUMP_DURATION = 1 * 60 * 1000UL;     // 1 min pompen
const unsigned long ECO_WAIT_SCH_DURATION = 1 * 60 * 1000UL; // 1 min wacht na SCH
const unsigned long ECO_WAIT_WON_DURATION = 2 * 60 * 1000UL; // 2 min wacht na WON

bool sch_pump_manual = false;
bool won_pump_manual = false;
unsigned long sch_pump_manual_start = 0;
unsigned long won_pump_manual_start = 0;
const unsigned long MANUAL_PUMP_DURATION = 60000UL;
bool sch_pump_manual_on = true;  // true = ON override, false = OFF override
bool won_pump_manual_on = true;

// V53.5: Trend tracking voor ECO data
float prev_eco_temp_top = 0.0;
float prev_eco_qtot = 0.0;

unsigned long last_poll = 0;
unsigned long last_temp_read = 0;
unsigned long uptime_sec = 0;
unsigned long last_slow = 0;
bool ap_mode_active = false;
bool mcp_available = false;

// v1.15: TSTAT snelcheck — vorige staat bijhouden voor edge-detectie
// Geïnitialiseerd op true (HIGH = open = uit) zodat LOW=GND onmiddellijk detecteerbaar is
bool tstat_last_state[7] = {true, true, true, true, true, true, true};
// Globaal + clear() = eenmalige allocatie, geen fragmentatie per poll-cyclus.
JsonDocument room_poll_doc;
JsonDocument eco_poll_doc;
JsonDocument room_filter_doc;
JsonDocument eco_filter_doc;
bool poll_filters_initialized = false;

OneWireNg::Id sensor_addresses[6] = {
  {0x28,0xDB,0xB5,0x03,0x00,0x00,0x80,0xBB},
  {0x28,0x7C,0xF0,0x03,0x00,0x00,0x80,0x59},
  {0x28,0x72,0xDB,0x03,0x00,0x00,0x80,0xC2},
  {0x28,0xAA,0xFB,0x03,0x00,0x00,0x80,0x5F},
  {0x28,0x49,0xDD,0x03,0x00,0x00,0x80,0x4B},
  {0x28,0xC3,0xD6,0x03,0x00,0x00,0x80,0x1E}
};

// Helper functies
// v1.11: static char buf — geen heap-alloc bij aanroep
const char* getFormattedDateTime() {
  static char buf[32];
  time_t now;
  time(&now);
  if (now < 1700000000) return "tijd niet gesync";
  struct tm tm;
  localtime_r(&now, &tm);
  strftime(buf, sizeof(buf), "%d-%m-%Y %H:%M:%S", &tm);
  return buf;
}

float calculateQtot(float temps[6]) {
  const float Cp = 1.16;
  float total_energy = 0.0;
  
  float T_layer0 = (boiler_ref_temp + temps[0]) / 2.0;
  if (T_layer0 > boiler_ref_temp) {
    total_energy += (T_layer0 - boiler_ref_temp) * boiler_layer_volume * Cp;
  }
  
  for (int i = 1; i < 6; i++) {
    float T_layer = (temps[i-1] + temps[i]) / 2.0;
    if (T_layer > boiler_ref_temp) {
      total_energy += (T_layer - boiler_ref_temp) * boiler_layer_volume * Cp;
    }
  }
  
  float T_layer6 = temps[5];
  if (T_layer6 > boiler_ref_temp) {
    total_energy += (T_layer6 - boiler_ref_temp) * boiler_layer_volume * Cp;
  }
  
  return total_energy / 1000.0;
}

// v1.11: const char* i.p.v. String — geen heap-alloc
const char* getTrend(float current, float previous, float threshold = 0.1) {
  if (abs(current - previous) < threshold) return "\xe2\x86\x92";  // →
  return (current > previous) ? "\xe2\x86\x91" : "\xe2\x86\x93";  // ↑ ↓
}

void readBoilerTemps() {
  // v1.7 FIX 2: Interval 2s→60s. Boilertemperatuur verandert traag; 2s was zinloos en belastend.
  if (millis() - last_temp_read < 60000) return;
  last_temp_read = millis();

  // v1.7 FIX 2: CONVERT_ALL broadcast (0xCC + 0x44) — triggert alle 6 sensoren tegelijk met 1x delay(750).
  // Oud patroon: 6× individueel MATCH ROM + CONVERT + delay(750) = 4500ms blocking per aanroep elke 2s → WDT crash.
  // Nieuw: 750ms één keer, daarna per-sensor READ SCRATCHPAD. WDT-safe in loop().
  ow.reset();
  ow.writeByte(0xCC);  // SKIP ROM — alle sensoren op de bus
  ow.writeByte(0x44);  // CONVERT T — start conversie op alle sensoren tegelijk
  delay(750);          // Eén wacht voor alle 6 sensoren samen

  // Per-sensor: MATCH ROM + READ SCRATCHPAD (geen extra delay nodig)
  for (int i = 0; i < 6; i++) {
    ow.reset();
    ow.writeByte(0x55);  // MATCH ROM
    for (int j = 0; j < 8; j++) ow.writeByte(sensor_addresses[i][j]);
    ow.writeByte(0xBE);  // READ SCRATCHPAD

    uint8_t data[9];
    for (int j = 0; j < 9; j++) data[j] = ow.readByte();

    // CRC-validatie (bestaande bit-by-bit implementatie behouden — werkt correct)
    uint8_t crc = 0;
    for (int j = 0; j < 8; j++) {
      uint8_t inbyte = data[j];
      for (int k = 0; k < 8; k++) {
        uint8_t mix = (crc ^ inbyte) & 0x01;
        crc >>= 1;
        if (mix) crc ^= 0x8C;
        inbyte >>= 1;
      }
    }

    if (crc == data[8]) {
      int16_t raw = (data[1] << 8) | data[0];
      sch_temps[i] = raw / 16.0;
      sensor_ok[i] = true;
    } else {
      Serial.printf("[DS18B20] CRC fout sensor %d — vorige waarde bewaard\n", i);
      sch_temps[i] = -127.0;
      sensor_ok[i] = false;
    }
  }

  sch_qtot = calculateQtot(sch_temps);
}

void checkPumpFeedback(float total_power) {
  if (!mcp_available) return;
  bool pump_should_be_on = (total_power > 0.01);
  bool pump_is_on = (mcp.digitalRead(7) == LOW);
  if (pump_should_be_on && !pump_is_on) {
    Serial.println("ALERT: Pump should be ON but is OFF!");
  } else if (!pump_should_be_on && pump_is_on) {
    Serial.println("ALERT: Pump should be OFF but is ON!");
  }
}

void pollEcoBoiler() {
  static unsigned long last_eco_poll = 0;
  if (millis() - last_eco_poll < (unsigned long)poll_interval * 1000) return;
  last_eco_poll = millis();
  
  if (strlen(eco_controller_ip) == 0 && strlen(eco_controller_mdns) == 0) {
    eco_boiler.online = false;
    return;
  }
  
  Serial.println("\n=== POLLING ECO BOILER ===");
  
  WiFiClient client;
  HTTPClient http;
  String url;
  
  if (strlen(eco_controller_ip) > 0) {
    url = "http://" + String(eco_controller_ip) + "/json";  // V53.5: Changed to /json
  } else {
    Serial.printf("Resolving %s.local ... ", eco_controller_mdns);
    IPAddress resolvedIP;
    if (WiFi.hostByName((String(eco_controller_mdns) + ".local").c_str(), resolvedIP)) {
      if (resolvedIP.toString() == "0.0.0.0" || resolvedIP[0] == 0) {
        Serial.printf("FAILED\n");
        eco_boiler.online = false;
        return;
      }
      Serial.printf("OK -> %s\n", resolvedIP.toString().c_str());
      url = "http://" + resolvedIP.toString() + "/json";  // V53.5: Changed to /json
    } else {
      Serial.printf("FAILED\n");
      eco_boiler.online = false;
      return;
    }
  }
  
  Serial.printf("ECO: Polling %s\n    ", url.c_str());
  
  http.begin(client, url);
  http.setTimeout(2000);
  http.setConnectTimeout(1000);
  http.setReuse(false);
  
  int httpCode = http.GET();
  Serial.printf("Result: %d", httpCode);
  
  if (httpCode == 200) {
    // v1.13: stream direct in JsonDocument — geen http.getString() String-allocatie
    eco_poll_doc.clear();
    DeserializationError error = deserializeJson(
      eco_poll_doc, http.getStream(),
      DeserializationOption::Filter(eco_filter_doc));
    Serial.printf(" ✓\n");

    eco_boiler.online = true;
    eco_boiler.last_seen = millis();

    if (!error) {
      eco_boiler.temp_avg    = eco_poll_doc["h"]  | 0.0;
      eco_boiler.qtot        = eco_poll_doc["i"]  | 0.0;
      eco_boiler.temp_top    = eco_poll_doc["b"]  | 0.0;
      eco_boiler.temp_bottom = eco_poll_doc["g"]  | 0.0;
      eco_qtot = eco_boiler.qtot;
      Serial.printf("    ETopH=%.1f°C EQtot=%.2f kWh EBotL=%.1f°C\n",
        eco_boiler.temp_top, eco_boiler.qtot, eco_boiler.temp_bottom);
      
      // V53.5: Update trend data (skip eerste poll)
      static int eco_poll_count = 0;
      eco_poll_count++;
      if (eco_poll_count > 1) {
        // Trends worden nu bijgehouden voor UI
      }
      prev_eco_temp_top = eco_boiler.temp_top;
      prev_eco_qtot = eco_boiler.qtot;
      
    } else {
      Serial.printf("    JSON parse error: %s\n", error.c_str());
    }
  } else {
    Serial.printf(" FAILED\n");
    eco_boiler.online = false;
  }
  
  http.end();
  client.stop();
}

void savePumpEvent(const char* pump_type, float kwh) {
  unsigned long timestamp = millis() / 1000;
  
  if (strcmp(pump_type, "SCH") == 0) {
    last_sch_pump.timestamp = timestamp;
    last_sch_pump.kwh_pumped = kwh;
    total_sch_kwh += kwh;
    preferences.putULong(NVS_LAST_SCH_PUMP, timestamp);
    preferences.putFloat(NVS_LAST_SCH_KWH, kwh);
    preferences.putFloat(NVS_TOTAL_SCH_KWH, total_sch_kwh);
    Serial.printf("Saved SCH pump event: %.2f kWh (totaal: %.2f kWh)\n", kwh, total_sch_kwh);
  } else if (strcmp(pump_type, "WON") == 0) {
    last_won_pump.timestamp = timestamp;
    last_won_pump.kwh_pumped = kwh;
    total_won_kwh += kwh;
    preferences.putULong(NVS_LAST_WON_PUMP, timestamp);
    preferences.putFloat(NVS_LAST_WON_KWH, kwh);
    preferences.putFloat(NVS_TOTAL_WON_KWH, total_won_kwh);
    Serial.printf("Saved WON pump event: %.2f kWh (totaal: %.2f kWh)\n", kwh, total_won_kwh);
  }
}

// v1.11: getPumpStatusMessage() verwijderd — status-banner weggevallen







// ============== DEEL 2/5: ECO PUMPS & POLLING FUNCTIES ==============

// V53.5: handleEcoPumps() met nieuwe state machine (WAIT_SCH en WAIT_WON)
void handleEcoPumps() {
  if (!mcp_available) return;
  
  // Check timeouts en RESET manual flags
  bool sch_manual_active = sch_pump_manual && (millis() - sch_pump_manual_start < MANUAL_PUMP_DURATION);
  bool won_manual_active = won_pump_manual && (millis() - won_pump_manual_start < MANUAL_PUMP_DURATION);
  
  if (sch_pump_manual && !sch_manual_active) {
    Serial.println("\n=== MANUAL PUMP TIMEOUT ===");
    Serial.println("SCH pump manual mode expired (60s)");
    sch_pump_manual = false;
  }
  if (won_pump_manual && !won_manual_active) {
    Serial.println("\n=== MANUAL PUMP TIMEOUT ===");
    Serial.println("WON pump manual mode expired (60s)");
    won_pump_manual = false;
  }
  
  // Manual mode heeft prioriteit (ON of OFF!)
  if (sch_manual_active || won_manual_active) {
    mcp.digitalWrite(RELAY_PUMP_SCH, sch_manual_active ? (sch_pump_manual_on ? LOW : HIGH) : HIGH);
    mcp.digitalWrite(RELAY_PUMP_WON, won_manual_active ? (won_pump_manual_on ? LOW : HIGH) : HIGH);
    return;
  }
  
  // START conditie (OR logica)
  bool should_start = eco_boiler.online 
                   && ((eco_boiler.temp_top > eco_max_temp) || (eco_boiler.qtot > eco_threshold));
  
  // STOP conditie (OR logica)
  bool should_stop = !eco_boiler.online
                  || ((eco_boiler.temp_top < eco_min_temp) || (eco_boiler.qtot < (eco_threshold - eco_hysteresis)));
  
  // V53.5: State machine met 5 states
  switch (eco_pump_state) {
    case ECO_IDLE:
      // Beide pompen uit
      mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
      mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
      
      // Check of we moeten starten
      if (should_start) {
        // Wissel tussen WON en SCH voor fair share
        if (last_pump_was_sch) {
          eco_pump_state = ECO_PUMP_WON;
          Serial.println("\n=== AUTO PUMP START ===");
          Serial.println("Pump: WON");
          Serial.println("Duration: 1 minute");
          Serial.printf("Reason: temp_top=%.1f°C>%.0f OR qtot=%.1f>%.1f kWh\n",
            eco_boiler.temp_top, eco_max_temp, eco_boiler.qtot, eco_threshold);
        } else {
          eco_pump_state = ECO_PUMP_SCH;
          Serial.println("\n=== AUTO PUMP START ===");
          Serial.println("Pump: SCH");
          Serial.println("Duration: 1 minute");
          Serial.printf("Reason: temp_top=%.1f°C>%.0f OR qtot=%.1f>%.1f kWh\n",
            eco_boiler.temp_top, eco_max_temp, eco_boiler.qtot, eco_threshold);
        }
        eco_pump_timer = millis();
      }
      break;
      
    case ECO_PUMP_SCH:
      // SCH pompt, WON uit
      mcp.digitalWrite(RELAY_PUMP_SCH, LOW);
      mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
      
      // Check stop conditie
      if (should_stop) {
        Serial.println("\n=== AUTO PUMP STOP ===");
        Serial.println("Pump: SCH");
        Serial.printf("Reason: temp_top=%.1f<%.0f OR qtot=%.1f<%.1f kWh\n",
          eco_boiler.temp_top, eco_min_temp, eco_boiler.qtot, eco_threshold - eco_hysteresis);
        eco_pump_state = ECO_IDLE;
        mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
        
        // Save event
        float kwh_transferred = 0.5;  // TODO: Echte berekening
        savePumpEvent("SCH", kwh_transferred);
        last_pump_was_sch = true;
        break;
      }
      
      // Check timeout (1 min)
      if (millis() - eco_pump_timer >= ECO_PUMP_DURATION) {
        Serial.println("\n=== AUTO PUMP FINISHED ===");
        Serial.println("Pump: SCH");
        Serial.println("Duration: 1 minute completed");
        mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
        
        // Save event
        float kwh_transferred = 0.5;  // TODO: Echte berekening
        savePumpEvent("SCH", kwh_transferred);
        last_pump_was_sch = true;
        
        // V53.5: Ga naar WAIT_SCH
        eco_pump_state = ECO_WAIT_SCH;
        eco_pump_timer = millis();
        Serial.println("Entering wait phase after SCH: 1 minute");
      }
      break;
      
    // V53.5: NIEUWE state - Wait na SCH (1 min)
    case ECO_WAIT_SCH:
      // Beide pompen uit
      mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
      mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
      
      // Check stop conditie
      if (should_stop) {
        Serial.println("\n=== CYCLE STOPPED ===");
        Serial.println("Reason: Stop conditie tijdens wacht fase");
        eco_pump_state = ECO_IDLE;
        break;
      }
      
      // Check timeout (1 min)
      if (millis() - eco_pump_timer >= ECO_WAIT_SCH_DURATION) {
        Serial.println("\n=== WAIT AFTER SCH FINISHED ===");
        
        if (should_start) {
          eco_pump_state = ECO_PUMP_WON;
          eco_pump_timer = millis();
          Serial.println("Starting WON pump (1 min)");
        } else {
          eco_pump_state = ECO_IDLE;
          Serial.println("No more pumping needed -> IDLE");
        }
      }
      break;
      
    case ECO_PUMP_WON:
      // WON pompt, SCH uit
      mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
      mcp.digitalWrite(RELAY_PUMP_WON, LOW);
      
      // Check stop conditie
      if (should_stop) {
        Serial.println("\n=== AUTO PUMP STOP ===");
        Serial.println("Pump: WON");
        Serial.printf("Reason: temp_top=%.1f<%.0f OR qtot=%.1f<%.1f kWh\n",
          eco_boiler.temp_top, eco_min_temp, eco_boiler.qtot, eco_threshold - eco_hysteresis);
        eco_pump_state = ECO_IDLE;
        mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
        
        // Save event
        float kwh_transferred = 0.5;  // TODO: Echte berekening
        savePumpEvent("WON", kwh_transferred);
        last_pump_was_sch = false;
        break;
      }
      
      // Check timeout (1 min)
      if (millis() - eco_pump_timer >= ECO_PUMP_DURATION) {
        Serial.println("\n=== AUTO PUMP FINISHED ===");
        Serial.println("Pump: WON");
        Serial.println("Duration: 1 minute completed");
        mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
        
        // Save event
        float kwh_transferred = 0.5;  // TODO: Echte berekening
        savePumpEvent("WON", kwh_transferred);
        last_pump_was_sch = false;
        
        // V53.5: Ga naar WAIT_WON
        eco_pump_state = ECO_WAIT_WON;
        eco_pump_timer = millis();
        Serial.println("Entering wait phase after WON: 2 minutes");
      }
      break;
      
    // V53.5: NIEUWE state - Wait na WON (2 min)
    case ECO_WAIT_WON:
      // Beide pompen uit
      mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
      mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
      
      // Check stop conditie
      if (should_stop) {
        Serial.println("\n=== CYCLE STOPPED ===");
        Serial.println("Reason: Stop conditie tijdens wacht fase");
        eco_pump_state = ECO_IDLE;
        break;
      }
      
      // Check timeout (2 min)
      if (millis() - eco_pump_timer >= ECO_WAIT_WON_DURATION) {
        Serial.println("\n=== WAIT AFTER WON FINISHED ===");
        
        if (should_start) {
          eco_pump_state = ECO_PUMP_SCH;
          eco_pump_timer = millis();
          Serial.println("Starting SCH pump (1 min)");
        } else {
          eco_pump_state = ECO_IDLE;
          Serial.println("No more pumping needed -> IDLE");
        }
      }
      break;
  }
}

void pollRooms() {
  if (millis() - last_poll < (unsigned long)poll_interval * 1000) return;
  last_poll = millis();

  Serial.println("\n=== POLLING ROOMS ===");
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WARNING: WiFi disconnected — reconnecting...");
    WiFi.reconnect();
    // Geen delay() — pollRooms() keert terug, loop() herprobeert bij volgende poll-cyclus
    return;
  } else {
    Serial.printf("WiFi OK - IP: %s, RSSI: %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  }

  vent_percent = 0;
  total_power = 0.0;  // Globale variabele (Matter)
  const unsigned long OVERRIDE_TIMEOUT = 600000UL;

  for (int i = 0; i < circuits_num; i++) {
    bool heating_demand = false;
    bool tstat_demand   = false;
    bool http_demand    = false;
    int  home_status    = 0;

    // ── Override check ───────────────────────────────────────────────────────
    // v1.7 FIX 7: goto apply_relay vervangen door skip_decision vlag
    bool skip_decision = false;
    if (circuits[i].override_active) {
      unsigned long elapsed = millis() - circuits[i].override_start;
      if (elapsed < OVERRIDE_TIMEOUT) {
        heating_demand = circuits[i].override_state;
        unsigned long remaining = (OVERRIDE_TIMEOUT - elapsed) / 1000;
        Serial.printf("c%d: ⚠️ OVERRIDE %s (%lu:%02lu remaining)\n",
          i, circuits[i].override_state ? "FORCE ON" : "FORCE OFF",
          remaining / 60, remaining % 60);
        skip_decision = true;
      } else {
        circuits[i].override_active = false;
        Serial.printf("c%d: Override timeout - terug naar AUTO\n", i);
      }
    }

    if (!skip_decision) {
      // ── TSTAT check ─────────────────────────────────────────────────────────
      if (circuits[i].has_tstat && mcp_available && circuits[i].tstat_pin < 13) {
        bool tstat_on = (mcp.digitalRead(circuits[i].tstat_pin) == LOW);
        Serial.printf("c%d: TSTAT pin %d = %s\n", i, circuits[i].tstat_pin, tstat_on ? "ON" : "OFF");
        tstat_demand = tstat_on;
      }

      // ── HTTP polling ─────────────────────────────────────────────────────────
      if (strlen(circuits[i].ip) > 0 || strlen(circuits[i].mdns) > 0) {
        WiFiClient client;
        HTTPClient http;
        String url;
        bool url_ok = false;

        if (strlen(circuits[i].ip) > 0) {
          url = "http://" + String(circuits[i].ip) + "/json";
          Serial.printf("c%d: Polling %s\n    ", i, url.c_str());
          url_ok = true;
        } else {
          // v1.7 FIX 7: goto decision_logic vervangen door url_ok vlag
          Serial.printf("c%d: Resolving %s.local ... ", i, circuits[i].mdns);
          IPAddress resolvedIP;
          if (WiFi.hostByName((String(circuits[i].mdns) + ".local").c_str(), resolvedIP)) {
            if (resolvedIP.toString() == "0.0.0.0" || resolvedIP[0] == 0) {
              Serial.printf("FAILED (host not found)\n");
              circuits[i].online = false;
              circuits[i].vent_request = 0;
            } else {
              Serial.printf("OK -> %s\n", resolvedIP.toString().c_str());
              url = "http://" + resolvedIP.toString() + "/json";
              Serial.printf("c%d: Polling %s\n    ", i, url.c_str());
              url_ok = true;
            }
          } else {
            Serial.printf("FAILED (DNS error)\n");
            circuits[i].online = false;
            circuits[i].vent_request = 0;
          }
        }

        if (url_ok) {
          http.begin(client, url);
          http.setTimeout(4000);
          http.setConnectTimeout(1500);
          http.setReuse(false);

          int httpCode = http.GET();
          Serial.printf("Result: %d", httpCode);

          if (httpCode == 200) {
            // v1.13: stream direct in JsonDocument — geen http.getString() String-allocatie
            // filter voorkomt dat ongebruikte velden geheugen innemen
            room_poll_doc.clear();
            DeserializationError error = deserializeJson(
              room_poll_doc, http.getStream(),
              DeserializationOption::Filter(room_filter_doc));
            Serial.printf(" ✓\n");

            circuits[i].online = true;
            circuits[i].last_seen = millis();

            if (!error) {
              int   y_val  = room_poll_doc["b"]  | 0;
              int   z_val  = room_poll_doc["g"]  | 0;
              int   aa_val = room_poll_doc["c"]  | 0;
              float h_val  = room_poll_doc["e"]  | 0.0f;
              int   af_val = room_poll_doc["v"]  | 0;

              circuits[i].heat_request = (y_val == 1);
              circuits[i].vent_request = z_val;
              circuits[i].setpoint     = aa_val;
              circuits[i].room_temp    = h_val;
              circuits[i].home_status  = af_val;

              http_demand  = circuits[i].heat_request;
              home_status  = circuits[i].home_status;

              Serial.printf("    y=%d z=%d aa=%d h=%.1f af=%d\n",
                y_val, z_val, aa_val, h_val, af_val);

              if (z_val > vent_percent) vent_percent = z_val;
            } else {
              Serial.printf("    JSON parse error: %s\n", error.c_str());
            }
          } else if (httpCode == 404) {
            Serial.printf(" - 404 Not Found\n");
            circuits[i].online = false;
            circuits[i].vent_request = 0;
          } else if (httpCode < 0) {
            Serial.printf(" - Timeout/Unreachable\n");
            circuits[i].online = false;
            circuits[i].vent_request = 0;
          } else {
            Serial.printf(" - HTTP Error\n");
            circuits[i].online = false;
            circuits[i].vent_request = 0;
          }

          http.end();
          client.stop();
          delay(100);
        }
      }

      // ── Beslissingslogica (was: goto decision_logic label) ───────────────────
      if (!circuits[i].online) {
        heating_demand = tstat_demand;
        Serial.printf("c%d: OFFLINE → TSTAT only = %s\n", i, heating_demand ? "ON" : "OFF");
      } else {
        if (home_status == 1) {
          heating_demand = tstat_demand || http_demand;
          Serial.printf("c%d: HOME → TSTAT(%d) OR HTTP(%d) = %s\n",
            i, tstat_demand, http_demand, heating_demand ? "ON" : "OFF");
        } else {
          heating_demand = http_demand;
          Serial.printf("c%d: AWAY → HTTP only = %s\n", i, heating_demand ? "ON" : "OFF");
        }
      }
    }  // end if (!skip_decision)

    // ── Relay aansturen (was: goto apply_relay label) ────────────────────────
    if (heating_demand != circuits[i].heating_on) {
      Serial.printf("c%d: Relay %s -> %s\n", i,
        circuits[i].heating_on ? "ON" : "OFF", heating_demand ? "ON" : "OFF");
      if (mcp_available && i < 7) {
        mcp.digitalWrite(i, heating_demand ? LOW : HIGH);
      }
      if (heating_demand) {
        circuits[i].off_time += millis() - circuits[i].last_change;
      } else {
        circuits[i].on_time  += millis() - circuits[i].last_change;
      }
      circuits[i].last_change = millis();
      circuits[i].heating_on  = heating_demand;
    }

    unsigned long total = circuits[i].on_time + circuits[i].off_time;
    if (total > 0) {
      circuits[i].duty_cycle = 100.0 * circuits[i].on_time / total;
    }

    // v1.16: sliding window — on-seconden bijhouden in huidig slot
    // Elke 20 minuten slot afsluiten en duty_4h herberekenen over alle gevulde slots
    {
      const uint32_t SLOT_MS  = 20UL * 60UL * 1000UL;   // 20 minuten per slot
      const uint8_t  N_SLOTS  = 12;                       // 12 slots = 4 uur

      uint32_t now_ms   = (uint32_t)millis();
      uint32_t elapsed  = now_ms - circuits[i].dc_slot_start;

      // Delta sinds vorige poll (niet since slot-start) — voorkomt cumulatieve optelling
      uint32_t poll_delta_ms = now_ms - circuits[i].dc_last_poll;
      circuits[i].dc_last_poll = now_ms;
      uint16_t on_sec_this_poll = circuits[i].heating_on
                                  ? (uint16_t)(poll_delta_ms / 1000UL)
                                  : 0;
      circuits[i].dc_slot_on += on_sec_this_poll;

      // Slot verlopen? → afsluiten, volgende openen, duty_4h herberekenen
      if (elapsed >= SLOT_MS) {
        // Clamp op max slotduur (bij lange pollpauze)
        uint16_t stored = min(circuits[i].dc_slot_on, (uint16_t)(SLOT_MS / 1000UL));
        circuits[i].dc_window[circuits[i].dc_slot] = stored;
        circuits[i].dc_slot       = (circuits[i].dc_slot + 1) % N_SLOTS;
        circuits[i].dc_slot_start = now_ms;
        circuits[i].dc_slot_on    = 0;

        // Herbereken duty_4h: gemiddelde over alle gevulde slots
        // dc_slots_filled telt afgesloten slots, geplafonneerd op N_SLOTS (ring vol)
        if (circuits[i].dc_slots_filled < N_SLOTS) circuits[i].dc_slots_filled++;
        uint32_t sum_on = 0;
        for (uint8_t s = 0; s < N_SLOTS; s++) sum_on += circuits[i].dc_window[s];
        // Noemer: dc_slots_filled × 1200s — correct ook als ring nog niet vol is
        circuits[i].duty_4h = 100.0f * sum_on
                              / ((float)circuits[i].dc_slots_filled * (SLOT_MS / 1000UL));
      }
    }

    if (circuits[i].heating_on) total_power += circuits[i].power_kw;
  }
  
  Serial.printf("Total power: %.2f kW, Vent rooms: %d%% HVAC: %d%%\n", total_power, vent_percent, vent_override_percent);
  // v1.13: effectieve ventilatie = max(rooms, HVAC-minimum) — hogere waarde wint altijd
  int effective_vent = max(vent_percent, vent_override_active ? vent_override_percent : 0);
  int pwm_value = map(effective_vent, 0, 100, 0, 255);
  ledcWrite(VENT_FAN_PIN, pwm_value);
  Serial.printf("Vent PWM: %d/255 (%d%% effectief, override=%s)\n", pwm_value, effective_vent, vent_override_active ? "JA" : "NEE");
  checkPumpFeedback(total_power);
}

// v1.13: Google Apps Script push — HTTP POST met buildLogJson() payload
// Wordt elke 5 minuten vanuit loop() aangeroepen als gas_url ingesteld is.
// Volgt redirects (GAS stuurt altijd 302 → 200).
void pushToGoogleSheets() {
  if (strlen(gas_url) == 0) return;
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure client;
  client.setInsecure();  // GAS gebruikt HTTPS — certificaat niet gevalideerd
  HTTPClient http;
  http.begin(client, gas_url);
  http.addHeader("Content-Type", "application/json");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(8000);

  const char* payload = buildLogJson();
  int code = http.POST((uint8_t*)payload, strlen(payload));
  Serial.printf("[GAS] POST → %d (%d bytes)\n", code, (int)strlen(payload));
  http.end();
  client.stop();
}

// v1.13: getWifiScanJson verwijderd — /scan streamt direct (zie setupWebServer)

// v1.13: Pure snprintf — geen JsonDocument, geen String, nul heap-allocatie
// Caller ontvangt pointer naar statische buffer (overschreven bij volgende aanroep)
const char* buildLogJson() {
  static char buf[520];

  float KSAv = 0; int valid_count = 0;
  for (int i = 0; i < 6; i++) {
    if (sensor_ok[i] && sch_temps[i] > -100) { KSAv += sch_temps[i]; valid_count++; }
  }
  if (valid_count > 0) KSAv /= valid_count;

  float total_power = 0.0;
  for (int i = 0; i < circuits_num; i++)
    if (circuits[i].heating_on) total_power += circuits[i].power_kw;

  bool sch_on = (mcp_available && mcp.digitalRead(RELAY_PUMP_SCH) == LOW)
             || (eco_pump_state == ECO_PUMP_SCH)
             || (sch_pump_manual && sch_pump_manual_on
                 && millis() - sch_pump_manual_start < MANUAL_PUMP_DURATION);
  bool won_on = (mcp_available && mcp.digitalRead(RELAY_PUMP_WON) == LOW)
             || (eco_pump_state == ECO_PUMP_WON)
             || (won_pump_manual && won_pump_manual_on
                 && millis() - won_pump_manual_start < MANUAL_PUMP_DURATION);

  snprintf(buf, sizeof(buf),
    "{\"a\":%lu"
    ",\"b\":%.4f,\"c\":%.4f,\"d\":%.4f,\"e\":%.4f,\"f\":%.4f,\"g\":%.4f"
    ",\"h\":%.4f"
    ",\"i\":%d,\"j\":%d,\"k\":%d,\"l\":%d,\"m\":%d,\"n\":%d,\"o\":%d"
    ",\"p\":%d,\"q\":%d,\"r\":%d,\"s\":%d,\"t\":%d,\"u\":%d,\"v\":%d"
    ",\"w\":%.2f,\"x\":%d"
    ",\"y\":%d,\"z\":%.4f"
    ",\"aa\":%d,\"ab\":%.4f"
    ",\"ac\":%d,\"ad\":%lu,\"ae\":%lu}",
    uptime_sec,
    sch_temps[0], sch_temps[1], sch_temps[2],
    sch_temps[3], sch_temps[4], sch_temps[5],
    KSAv,
    (int)circuits[0].duty_4h, (int)circuits[1].duty_4h,
    (int)circuits[2].duty_4h, (int)circuits[3].duty_4h,
    (int)circuits[4].duty_4h, (int)circuits[5].duty_4h,
    (int)circuits[6].duty_4h,
    circuits[0].heating_on?1:0, circuits[1].heating_on?1:0,
    circuits[2].heating_on?1:0, circuits[3].heating_on?1:0,
    circuits[4].heating_on?1:0, circuits[5].heating_on?1:0,
    circuits[6].heating_on?1:0,
    total_power, max(vent_percent, vent_override_active ? vent_override_percent : 0),
    sch_on?1:0, last_sch_pump.kwh_pumped,
    won_on?1:0, last_won_pump.kwh_pumped,
    (int)WiFi.RSSI(),
    (unsigned long)((ESP.getFreeHeap() * 100UL) / ESP.getHeapSize()),
    (unsigned long)(ESP.getMaxAllocHeap() / 1024)
  );
  return buf;
}












// ============== DEEL 3/5: MAIN PAGE UI ==============

// v1.8: getMainPage() vervangen door streamMainPage() — chunked streaming via AsyncResponseStream.
// Principe: HTML wordt in ~10 kleine stukken via p->print() naar de client gestuurd.
// Nooit meer dan ~1-2 KB tijdelijk in RAM — geen html.reserve(10000) meer nodig.
// AsyncWebServer beheert de TCP-buffering intern; wij geven alleen kleine stukjes.
void streamMainPage(AsyncWebServerRequest* request) {
  // Pre-compute variabelen (kleine Strings op stack — geen grote buffer)
  float total_power_local = 0.0;
  for (int i = 0; i < circuits_num; i++) {
    if (circuits[i].heating_on) total_power_local += circuits[i].power_kw;
  }

  const char* trend_char_t = getTrend(eco_boiler.temp_top, prev_eco_temp_top, 0.5);
  const char* trend_char_q = getTrend(eco_boiler.qtot,     prev_eco_qtot,     0.1);

  const char* temp_color  = eco_boiler.temp_top >= 90 ? "#c00"
                          : eco_boiler.temp_top >= 80 ? "#fa0"
                          : eco_boiler.temp_top >= 60 ? "#0a0" : "#0af";
  const char* energy_color = eco_boiler.qtot >= 20 ? "#c00"
                           : eco_boiler.qtot >= 15 ? "#fa0"
                           : eco_boiler.qtot >= 10 ? "#0a0" : "#999";

  uint32_t lb = ESP.getMaxAllocHeap();
  const char* lb_color = lb >= 35000 ? "#0a0" : lb >= 25000 ? "#f80" : "#c00";
  const char* lb_label = lb >= 35000 ? "OK"   : lb >= 25000 ? "LAAG" : "KRITIEK";

  // Chunk 1 — DOCTYPE, head, CSS (~1.8 KB)
  AsyncResponseStream* p = request->beginResponseStream("text/html; charset=utf-8");
  p->print(F("<!DOCTYPE html><html><head>"
    "<meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>"));
  p->print(room_id);
  p->print(F(" Status</title><style>"
    "body{font-family:Arial,sans-serif;background:#fff;margin:0;padding:0;}"
    ".header{display:flex;background:#ffcc00;color:#000;padding:10px 15px;font-size:18px;font-weight:bold;align-items:center;}"
    ".header-left{flex:1;}.header-right{flex:1;text-align:right;font-size:15px;}"
    ".slider{width:150px;height:28px;vertical-align:middle;}"
    ".container{display:flex;min-height:calc(100vh - 60px);}"
    ".sidebar{width:80px;padding:10px 5px;background:#fff;border-right:3px solid #c00;}"
    ".sidebar a{display:block;background:#369;color:#fff;padding:8px;text-decoration:none;font-weight:bold;font-size:12px;border-radius:6px;text-align:center;width:60px;margin:8px auto;}"
    ".sidebar a:hover{background:#036;}.sidebar a.active{background:#c00;}"
    ".main{flex:1;padding:15px;overflow-y:auto;}"
    ".group-title{font-size:17px;font-style:italic;font-weight:bold;color:#fff;background:#336699;padding:8px 12px;margin:20px 0 8px 0;border-radius:4px;}"
    ".section-divider{border-top:1px solid #eee;margin:10px 0;}"
    "table{width:100%;border-collapse:collapse;margin-bottom:15px;}"
    "td.label{color:#369;font-size:13px;padding:8px 5px;border-bottom:1px solid #ddd;text-align:left;}"
    "td.value{background:#e6f0ff;font-size:13px;padding:8px 5px;border-bottom:1px solid #ddd;text-align:center;}"
    "tr.header-row td,tr.header-row td.label,tr.header-row td.value{background:#336699;color:#fff;font-weight:bold;padding:10px 5px;font-size:12px;}"
    ".status-ok{color:#0a0;font-weight:bold;}.status-na{color:#c00;font-weight:bold;}.status-away{color:#f80;font-weight:bold;}"
    ".eco-status-badge{display:inline-block;padding:4px 12px;border-radius:4px;font-weight:bold;font-size:12px;margin-left:10px;}"
    ".eco-online{background:#0a0;color:#fff;}.eco-offline{background:#c00;color:#fff;}"
    ".pump-info{font-size:11px;color:#666;font-style:italic;margin-top:5px;text-align:center;}"
    ".pump-total{font-size:12px;color:#369;font-weight:bold;margin-top:3px;text-align:center;}"
    ".circuits-table-wrapper{overflow-x:auto;-webkit-overflow-scrolling:touch;margin:15px 0;}"
    "table.circuits-table{min-width:1400px;}"
    ".btn-override{padding:4px 8px;margin:2px;font-size:11px;cursor:pointer;border:none;border-radius:4px;background:#369;color:#fff;}"
    ".btn-override:hover{background:#036;}.btn-override-cancel{background:#c00;}.btn-override-cancel:hover{background:#900;}"
    ".override-badge{background:#c00;color:#fff;padding:4px 8px;border-radius:4px;font-size:11px;font-weight:bold;}"
    ".override-badge-off{background:#666;color:#fff;}"
    "@media(max-width:600px){.container{flex-direction:column;}.sidebar{width:100%;border-right:none;border-bottom:3px solid #c00;display:flex;justify-content:center;}.sidebar a{width:60px;margin:0 3px;}.main{padding:8px;}table.circuits-table{min-width:850px;}}"
    "</style></head><body>"));

  // Chunk 2 — header, status banner, sidebar (~0.4 KB)
  p->print(F("<div class='header'><div class='header-left'>"));
  p->print(room_id);
  p->print(F("</div><div class='header-right'>"));
  p->print(uptime_sec);
  p->print(F(" s &nbsp;&nbsp; "));
  p->print(getFormattedDateTime());
  p->print(F("</div></div>"
    "<div class='container'>"
    "<div class='sidebar'>"
    "<a href='/' class='active'>Status</a>"
    "<a href='/matter'>Matter</a>"
    "<a href='/update'>OTA</a>"
    "<a href='/json'>JSON</a>"
    "<a href='/settings'>Settings</a>"
    "</div><div class='main'>"));

  // Chunk 3 — STATUS tabel is verplaatst naar onderaan de pagina

  // Chunk 4 — SCH boiler sensoren (~0.5 KB)
  p->print(F("<div class='group-title'>SCH BOILER (Warmtepomp schuur)</div>"
    "<table><tr class='header-row'><td class='label'>Boiler sensors</td>"
    "<td class='value'>Temperatuur</td><td class='value'>Status</td></tr>"));
  for (int i = 0; i < 6; i++) {
    p->print(F("<tr><td class='label'>"));
    p->print(sensor_nicknames[i]);
    p->print(F("</td><td class='value'>"));
    if (sensor_ok[i]) { p->print(sch_temps[i], 1); p->print(F(" &deg;C")); }
    else p->print(F("--"));
    p->print(F("</td><td class='value'>"));
    p->print(sensor_ok[i] ? F("OK") : F("Error"));
    p->print(F("</td></tr>"));
  }
  p->print(F("</table><table><tr><td class='label'>Energieinhoud (QTot)</td>"
    "<td class='value'><b>"));
  p->print(sch_qtot, 2);
  p->print(F(" kWh</b></td></tr></table>"));

  // Chunk 5 — ECO boiler (~0.5 KB)
  p->print(F("<div class='group-title'>ECO BOILER (Solar + Haarden) <span class='eco-status-badge "));
  p->print(eco_boiler.online ? F("eco-online'>ONLINE") : F("eco-offline'>OFFLINE"));
  p->print(F("</span></div><table>"
    "<tr><td class='label'>ETop (Hoogste temp)</td><td class='value'>"));
  if (eco_boiler.online) {
    p->print(F("<b style='color:")); p->print(temp_color); p->print(F("'>"));
    p->print(eco_boiler.temp_top, 1); p->print(F(" &deg;C</b> ")); p->print(trend_char_t);
  } else p->print(F("<span class='status-na'>NA</span>"));
  p->print(F("</td></tr><tr><td class='label'>EQtot (Energie)</td><td class='value'>"));
  if (eco_boiler.online) {
    p->print(F("<b style='color:")); p->print(energy_color); p->print(F("'>"));
    p->print(eco_boiler.qtot, 2); p->print(F(" kWh</b> ")); p->print(trend_char_q);
  } else p->print(F("<span class='status-na'>NA</span>"));
  p->print(F("</td></tr>"
    "<tr><td class='label'>Tmax pomp (Start)</td><td class='value'>"));
  p->print(eco_max_temp, 1); p->print(F(" &deg;C</td></tr>"
    "<tr><td class='label'>Tmin pomp (Stop)</td><td class='value'>"));
  p->print(eco_min_temp, 1); p->print(F(" &deg;C</td></tr>"));

  // Chunk 6 — SCH pomp (~0.4 KB)
  p->print(F("<tr><td class='label'>SCH Pomp</td><td class='value'>"));
  if (sch_pump_manual && (millis() - sch_pump_manual_start < MANUAL_PUMP_DURATION)) {
    unsigned long rem = (MANUAL_PUMP_DURATION - (millis() - sch_pump_manual_start)) / 1000;
    p->print(F("<span class='override-badge timer' data-remaining='"));
    p->print(rem); p->print(F("'>"));
    p->print(sch_pump_manual_on ? F("ON ") : F("OFF "));
    p->print(rem / 60); p->print(F(":")); p->print(rem % 60);
    p->print(F("</span> <button class='btn-override btn-override-cancel' onclick=\"cancelPump('sch')\">&#215;</button>"));
  } else {
    p->print(F("<button class='btn-override' onclick=\"setPump('sch',true)\">ON</button> "
               "<button class='btn-override' onclick=\"setPump('sch',false)\">OFF</button>"));
  }
  if (last_sch_pump.timestamp > 0) {
    time_t et = last_sch_pump.timestamp; struct tm tm; localtime_r(&et, &tm);
    char tb[32]; strftime(tb, sizeof(tb), "%d-%m-%Y %H:%M", &tm);
    p->print(F("<div class='pump-info'>Laatste: ")); p->print(tb);
    p->print(F(" - ")); p->print(last_sch_pump.kwh_pumped, 2); p->print(F(" kWh</div>"));
  } else p->print(F("<div class='pump-info'>Laatste: NA</div>"));
  p->print(F("<div class='pump-total'>TOTAAL: ")); p->print(total_sch_kwh, 1); p->print(F(" kWh</div></td></tr>"));

  // Chunk 7 — WON pomp (~0.4 KB)
  p->print(F("<tr><td class='label'>WON Pomp</td><td class='value'>"));
  if (won_pump_manual && (millis() - won_pump_manual_start < MANUAL_PUMP_DURATION)) {
    unsigned long rem = (MANUAL_PUMP_DURATION - (millis() - won_pump_manual_start)) / 1000;
    p->print(F("<span class='override-badge timer' data-remaining='"));
    p->print(rem); p->print(F("'>"));
    p->print(won_pump_manual_on ? F("ON ") : F("OFF "));
    p->print(rem / 60); p->print(F(":")); p->print(rem % 60);
    p->print(F("</span> <button class='btn-override btn-override-cancel' onclick=\"cancelPump('won')\">&#215;</button>"));
  } else {
    p->print(F("<button class='btn-override' onclick=\"setPump('won',true)\">ON</button> "
               "<button class='btn-override' onclick=\"setPump('won',false)\">OFF</button>"));
  }
  if (last_won_pump.timestamp > 0) {
    time_t et = last_won_pump.timestamp; struct tm tm; localtime_r(&et, &tm);
    char tb[32]; strftime(tb, sizeof(tb), "%d-%m-%Y %H:%M", &tm);
    p->print(F("<div class='pump-info'>Laatste: ")); p->print(tb);
    p->print(F(" - ")); p->print(last_won_pump.kwh_pumped, 2); p->print(F(" kWh</div>"));
  } else p->print(F("<div class='pump-info'>Laatste: NA</div>"));
  p->print(F("<div class='pump-total'>TOTAAL: ")); p->print(total_won_kwh, 1);
  p->print(F(" kWh</div></td></tr></table>"
    "<div class='group-title'>VENTILATIE</div><table>"
    "<tr><td class='label'>Auto (rooms)</td><td class='value'>"));
  p->print(vent_percent);
  p->print(F(" %</td><td class='value'></td></tr>"
    "<tr><td class='label'>Override</td><td class='value' id='vent-val'>"));
  p->print(vent_override_active ? vent_override_percent : 0);
  p->print(F(" %</td><td class='value'>"
    "<input type='range' class='slider' id='vent-slider' min='0' max='100' value='"));
  p->print(vent_override_active ? vent_override_percent : 0);
  p->print(F("' oninput=\"document.getElementById('vent-val').textContent=this.value+' %;'\""
    " onchange=\"fetch('/set_vent?vent='+this.value).then(()=>setTimeout(()=>location.reload(),500));\">"
    "</td></tr><tr><td class='label' colspan='3' style='font-size:11px;color:#999;'>"));
  p->print(vent_override_active ? F("Override actief — vervalt na 3u") : F("Slider op 0 = auto"));
  p->print(F("</td></tr></table>"));

  // Chunk 9 — Circuits-tabel via JS fetch('/json') — nooit meer onvolledig door heap-druk
  // De statische HTML-shell (~2KB) laadt altijd volledig. JS vult de tabel na onload.
  // Override-timers worden ook client-side bijgehouden via setInterval.
  p->print(F("<div class='group-title'>CIRCUITS</div>"
    "<div class='circuits-table-wrapper'>"
    "<table class='circuits-table'>"
    "<tr class='header-row'><td>#</td><td>Naam</td><td>IP</td><td>mDNS</td>"
    "<td>Set</td><td>Temp</td><td>Heat</td><td>Home</td>"
    "<td>TSTAT</td><td>Pomp</td><td>P</td><td>Duty</td><td>Vent</td><td>Override</td></tr>"
    "<tbody id='ct'><tr><td colspan='14' style='text-align:center;padding:12px;color:#999;'>"
    "Laden...</td></tr></tbody>"
    "</table></div>"
    "<div class='group-title'>STATUS</div>"));
  p->print(F("<table>"
    "<tr><td class='label'>MCP23017</td><td class='value'>"));
  p->print(mcp_available ? F("Verbonden") : F("Niet gevonden"));
  p->print(F("</td></tr><tr><td class='label'>WiFi</td><td class='value'>"));
  p->print(WiFi.localIP().toString());
  p->print(F("</td></tr><tr><td class='label'>WiFi RSSI</td><td class='value'>"));
  p->print(WiFi.RSSI());
  p->print(F(" dBm</td></tr><tr><td class='label'>Free heap</td><td class='value'>"));
  p->print((ESP.getFreeHeap() * 100) / ESP.getHeapSize());
  p->print(F(" %</td></tr><tr><td class='label'>Heap largest block</td><td class='value'><b style='color:"));
  p->print(lb_color);
  p->print(F("'>"));
  p->print(lb / 1024);
  p->print(F(" KB — "));
  p->print(lb_label);
  p->print(F("</b></td></tr></table>"
    "</div></div>"
    "<script>"
    "function setPump(t,s){fetch(t==='sch'?(s?'/pump_sch_on':'/pump_sch_off'):(s?'/pump_won_on':'/pump_won_off')).then(()=>setTimeout(()=>location.reload(),500));}"
    "function cancelPump(t){fetch(t==='sch'?'/pump_sch_cancel':'/pump_won_cancel').then(()=>setTimeout(()=>location.reload(),500));}"
    "function setOverride(c,s){fetch((s?'/circuit_override_on':'/circuit_override_off')+'?circuit='+c).then(()=>setTimeout(()=>location.reload(),500));}"
    "function cancelOverride(c){fetch('/circuit_override_cancel?circuit='+c).then(()=>setTimeout(()=>location.reload(),500));}"
    // Timer countdown
    "setInterval(()=>{"
      "document.querySelectorAll('.timer').forEach(b=>{"
        "let r=parseInt(b.dataset.remaining);"
        "if(r>0){r--;b.dataset.remaining=r;const s=b.textContent.split(' ')[0];"
        "b.textContent=s+' '+Math.floor(r/60)+':'+(r%60).toString().padStart(2,'0');}"
        "else if(r===0)setTimeout(()=>location.reload(),1000);"
      "});"
    "},1000);"
    // v1.12: circuits via /json_ui — gescheiden van /json (Google Sheets)
    "function loadCircuits(){"
      "fetch('/json_ui').then(r=>r.json()).then(circ=>{"
        "if(!circ||!circ.length){document.getElementById('ct').innerHTML="
          "'<tr><td colspan=14 style=\"text-align:center;color:#c00;\">Geen data</td></tr>';return;}"
        "let h='';"
        "let tot_p=0;"
        "circ.forEach((c,i)=>{"
          "const oc=c.online?'status-ok':'status-na';"
          "const rc=c.override_active?'override-active':'';"
          "h+=`<tr class='${rc}'>`;"
          "h+=`<td class='label'>${i+1}</td>`;"
          "h+=`<td class='value'>${c.name}</td>`;"
          "h+=`<td class='value ${oc}'>${c.ip?(c.online?'&#10003;':'&#10007;'):'-'}</td>`;"
          "h+=`<td class='value ${oc}'>${c.mdns?(c.online?'&#10003;':'&#10007;'):'-'}</td>`;"
          "h+=`<td class='value'>${c.online&&c.setpoint>0?c.setpoint+'&deg;C':'--'}</td>`;"
          "h+=`<td class='value'>${c.online&&c.room_temp>0?c.room_temp.toFixed(1)+'&deg;C':'--'}</td>`;"
          "h+=`<td class='value'>${c.online?(c.heat_request?'ON':'OFF'):'--'}</td>`;"
          "h+=`<td class='value ${c.online?(c.home_status===1?'status-ok':'status-away'):'status-na'}'>`;"
          "h+=`${c.online?(c.home_status===1?'Thuis':'Away'):'NA'}</td>`;"
          "h+=`<td class='value'>${c.tstat??'-'}</td>`;"
          "h+=`<td class='value'>${c.heating_on?'<b>AAN</b>':'UIT'}</td>`;"
          "h+=`<td class='value'>${c.heating_on?c.power_kw.toFixed(1)+' kW':'0 kW'}</td>`;"
          "h+=`<td class='value'>${c.duty_cycle.toFixed(1)}%</td>`;"
          "h+=`<td class='value'>${c.vent_request}%</td>`;"
          "h+='<td class=\"value\">';"
          "if(c.override_active){"
            "const rem=c.override_remaining??0;"
            "h+=`<span class='override-badge timer' data-remaining='${rem}'>`;"
            "h+=`${c.override_state?'ON':'OFF'} ${Math.floor(rem/60)}:${String(rem%60).padStart(2,'0')}</span> `;"
            "h+=`<button class='btn-override btn-override-cancel' onclick='cancelOverride(${i})'>&#215;</button>`;"
          "}else{"
            "h+=`<button class='btn-override' onclick='setOverride(${i},true)'>ON</button> `;"
            "h+=`<button class='btn-override' onclick='setOverride(${i},false)'>OFF</button>`;"
          "}"
          "h+='</td></tr>';"
          "if(c.heating_on)tot_p+=c.power_kw;"
        "});"
        "h+=`<tr style='border-top:2px solid #369;'>`;"
        "h+=`<td colspan='9' class='label'><b>TOTAAL</b></td>`;"
        "h+=`<td class='value'></td>`;"
        "h+=`<td class='value'><b>${tot_p.toFixed(1)} kW</b></td>`;"
        "h+=`<td colspan='3' class='value'><b id='vent-tot'>... %</b></td></tr>`;"
        "document.getElementById('ct').innerHTML=h;"
        // Haal vent% op via /json (compact endpoint)
        "fetch('/json').then(r=>r.json()).then(j=>{"
          "const el=document.getElementById('vent-tot');"
          "if(el)el.textContent=(j.x??0)+' %';"
        "}).catch(()=>{});"
      "}).catch(()=>{"
        "document.getElementById('ct').innerHTML="
          "'<tr><td colspan=14 style=\"text-align:center;color:#c00;\">Fout bij laden</td></tr>';"
      "});"
    "}"
    "loadCircuits();"
    "</script></body></html>"));

  request->send(p);
}
  















// ============== DEEL 4/5: SETTINGS PAGE & ENDPOINTS ==============

void setupWebServer() {
  // Captive portal - redirect all requests naar settings in AP mode
  server.onNotFound([](AsyncWebServerRequest *request){
    if (ap_mode_active) {
      request->redirect("/settings");
    } else {
      request->send(404, "text/plain", "Not found");
    }
  });

  // v1.8: streamMainPage() — chunked streaming, geen html.reserve(10000) meer
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    streamMainPage(request);
  });

  // v1.13: buildLogJson() = static char buf, geen heap-allocatie
  server.on("/json", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", buildLogJson());
  });

  server.on("/json_ui", HTTP_GET, [](AsyncWebServerRequest *request){
    // v1.13: serializeJson direct naar stream — geen tussenstap String
    AsyncResponseStream* p = request->beginResponseStream("application/json");
    JsonDocument doc;
    JsonArray jc = doc.to<JsonArray>();
    for (int i = 0; i < circuits_num; i++) {
      JsonObject c = jc.createNestedObject();
      c["name"]    = circuits[i].name;
      c["ip"]      = strlen(circuits[i].ip)   > 0 ? circuits[i].ip   : nullptr;
      c["mdns"]    = strlen(circuits[i].mdns) > 0 ? circuits[i].mdns : nullptr;
      c["online"]  = circuits[i].online;
      c["setpoint"]      = circuits[i].setpoint;
      c["room_temp"]     = circuits[i].room_temp;
      c["heat_request"]  = circuits[i].heat_request;
      c["home_status"]   = circuits[i].home_status;
      c["heating_on"]    = circuits[i].heating_on;
      c["power_kw"]      = circuits[i].power_kw;
      c["duty_cycle"]    = circuits[i].duty_cycle;
      c["vent_request"]  = circuits[i].vent_request;
      c["override_active"] = circuits[i].override_active;
      c["override_state"]  = circuits[i].override_state;
      if (circuits[i].override_active) {
        unsigned long elapsed = millis() - circuits[i].override_start;
        c["override_remaining"] = elapsed < 600000UL ? (int)((600000UL - elapsed) / 1000) : 0;
      } else {
        c["override_remaining"] = 0;
      }
      if (circuits[i].has_tstat && mcp_available && circuits[i].tstat_pin < 13)
        c["tstat"] = (mcp.digitalRead(circuits[i].tstat_pin) == LOW) ? "ON" : "OFF";
      else
        c["tstat"] = nullptr;
    }
    serializeJson(doc, *p);
    request->send(p);
  });

  // v1.10: /json_settings — circuit-config als JSON voor hybrid /settings pagina
  server.on("/json_settings", HTTP_GET, [](AsyncWebServerRequest *request){
    // v1.13: serializeJson direct naar stream — geen tussenstap String
    AsyncResponseStream* p = request->beginResponseStream("application/json");
    JsonDocument doc;
    doc["room_id"]        = room_id;
    doc["wifi_ssid"]      = wifi_ssid;
    doc["wifi_pass"]      = wifi_pass;
    doc["static_ip"]      = static_ip_str;
    doc["circuits_num"]   = circuits_num;
    doc["poll_interval"]  = poll_interval;
    doc["eco_ip"]         = eco_controller_ip;
    doc["eco_mdns"]       = eco_controller_mdns;
    doc["eco_threshold"]  = eco_threshold;
    doc["eco_hysteresis"] = eco_hysteresis;
    doc["eco_min_temp"]   = eco_min_temp;
    doc["eco_max_temp"]   = eco_max_temp;
    doc["boiler_ref_temp"]    = boiler_ref_temp;
    doc["boiler_layer_volume"] = boiler_layer_volume;
    doc["gas_url"]            = gas_url;
    JsonArray nicks = doc["sensor_nicknames"].to<JsonArray>();
    for (int i = 0; i < 6; i++) nicks.add(sensor_nicknames[i]);
    JsonArray circs = doc["circuits"].to<JsonArray>();
    for (int i = 0; i < circuits_num; i++) {
      JsonObject c = circs.createNestedObject();
      c["name"]     = circuits[i].name;
      c["ip"]       = circuits[i].ip;
      c["mdns"]     = circuits[i].mdns;
      c["power_kw"] = circuits[i].power_kw;
      c["has_tstat"]= circuits[i].has_tstat;
      c["tstat_pin"]= circuits[i].tstat_pin;
    }
    serializeJson(doc, *p);
    request->send(p);
  });

  server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request){
    // v1.13: inline stream, geen getWifiScanJson() String meer
    AsyncResponseStream* p = request->beginResponseStream("application/json");
    int n = WiFi.scanNetworks();
    JsonDocument doc;
    JsonArray networks = doc["networks"].to<JsonArray>();
    for (int i = 0; i < n && i < 10; i++) {
      JsonObject net = networks.createNestedObject();
      net["ssid"] = WiFi.SSID(i);
      net["rssi"] = WiFi.RSSI(i);
    }
    serializeJson(doc, *p);
    request->send(p);
  });

  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncResponseStream* p = request->beginResponseStream("text/html; charset=utf-8");
    p->print(F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>"));
    p->print(room_id);
    p->print(F(" - OTA</title><style>"
      "body{font-family:Arial,sans-serif;background:#fff;margin:0;padding:20px;text-align:center;}"
      "h1{color:#369;}"
      ".button{background:#369;color:#fff;padding:12px 24px;border:none;border-radius:8px;cursor:pointer;font-size:16px;margin:10px;}"
      ".button:hover{background:#036;}"
      ".reboot{background:#c00;}.reboot:hover{background:#900;}"
      "</style></head><body>"
      "<h1>OTA Firmware Update</h1>"
      "<form method='POST' action='/update' enctype='multipart/form-data'>"
      "<input type='file' name='update' accept='.bin'><br><br>"
      "<button class='button' type='submit'>Upload Firmware</button>"
      "</form><br>"
      "<button class='button reboot' onclick=\"if(confirm('Reboot?'))location.href='/reboot'\">Reboot</button>"
      "<br><br><a href='/'>&#8592; Terug</a>"
      "</body></html>"));
    request->send(p);
  });

  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request) {
    bool success = !Update.hasError();
    request->send(200, "text/html", success 
      ? "<h2 style='color:#0f0'>Update OK!</h2><p>Rebooting...</p>" 
      : "<h2 style='color:#f00'>Update FAILED!</h2>");
    if (success) { delay(1000); ESP.restart(); }
  }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
    if (!index) {
      Serial.println("\n=== OTA UPDATE ===");
      Update.begin(UPDATE_SIZE_UNKNOWN);
    }
    Update.write(data, len);
    if (final && Update.end(true)) Serial.println("OTA OK");
  });

  server.on("/reboot", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", "<h2>Rebooting...</h2>");
    delay(500);
    ESP.restart();
  });

  server.on("/matter", HTTP_GET, [](AsyncWebServerRequest *request) {
    // v1.11: chunked streaming i.p.v. String html.reserve(3000) — ~3KB heap bespaard
    AsyncResponseStream* p = request->beginResponseStream("text/html; charset=utf-8");
    p->print(F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>Matter</title><style>"
      "body{font-family:Arial,sans-serif;background:#fff;margin:0;padding:0;}"
      ".header{display:flex;background:#ffcc00;color:#000;padding:10px 15px;font-size:18px;font-weight:bold;}"
      ".header-left{flex:1;}.header-right{flex:1;text-align:right;font-size:15px;}"
      ".sidebar{width:80px;padding:10px 5px;background:#fff;border-right:3px solid #c00;box-sizing:border-box;flex-shrink:0;}"
      ".sidebar a{display:block;background:#369;color:#fff;padding:8px;margin:8px auto;text-decoration:none;font-weight:bold;font-size:12px;border-radius:6px;text-align:center;width:60px;}"
      ".sidebar a:hover{background:#036;}.sidebar a.active{background:#c00;}"
      ".container{display:flex;min-height:calc(100vh - 60px);}"
      ".main{flex:1;padding:30px;}"
      ".card{background:#e6f0ff;border:2px solid #369;border-radius:10px;padding:25px;max-width:500px;margin:20px 0;}"
      ".code{font-family:monospace;font-size:28px;font-weight:bold;color:#003366;background:#fff;padding:12px 20px;border-radius:6px;border:2px solid #369;display:inline-block;letter-spacing:2px;margin:12px 0;}"
      ".ok{color:#060;font-size:22px;font-weight:bold;}"
      ".btn-reset{background:#c00;color:#fff;padding:10px 24px;border:none;border-radius:6px;font-size:15px;cursor:pointer;margin-top:20px;}"
      ".btn-reset:hover{background:#900;}"
      ".hint{font-size:13px;color:#666;margin-top:8px;}"
      "@media(max-width:600px){.container{flex-direction:column;}.sidebar{width:100%;border-right:none;border-bottom:3px solid #c00;display:flex;justify-content:center;}.sidebar a{margin:0 3px;}}"
      "</style></head><body>"
      "<div class='header'><div class='header-left'>"));
    p->print(room_id);
    p->print(F("</div><div class='header-right'>Matter / HomeKit</div></div>"
      "<div class='container'><div class='sidebar'>"
      "<a href='/'>Status</a><a href='/matter' class='active'>Matter</a>"
      "<a href='/update'>OTA</a><a href='/json'>JSON</a>"
      "<a href='/settings'>Settings</a>"
      "</div><div class='main'><div class='card'>"));
    if (Matter.isDeviceCommissioned()) {
      p->print(F("<div class='ok'>&#x2705; Matter gepaard</div>"
        "<p>Deze controller is verbonden met Apple Home.</p>"));
    } else {
      String pairingCode = Matter.getManualPairingCode();
      p->print(F("<h2 style='color:#369;margin-top:0;'>Matter koppelen</h2>"
        "<p><b>1.</b> Open de <b>Apple Home</b> app</p>"
        "<p><b>2.</b> Tik op <b>+</b> &rarr; <b>Accessoire toevoegen</b> &rarr; <b>Meer opties</b></p>"
        "<p><b>3.</b> Voer de onderstaande code in:</p>"
        "<div class='code'>"));
      p->print(pairingCode);
      p->print(F("</div><p class='hint'>Of scan de QR-code via de Apple Home app.</p>"));
    }
    p->print(F("<br><button class='btn-reset' onclick=\"if(confirm('Matter pairing wissen?')) location.href='/matter_reset';\">"
      "Matter reset (pairing wissen)</button>"
      "<p class='hint'>Matter reset wist alleen de HomeKit koppeling. HVAC instellingen blijven intact.</p>"
      "</div></div></div></body></html>"));
    request->send(p);
  });

  // v1.10: Nuclear reset via flag — handler zet alleen flag, main loop voert uit
  // Reden: nvs_flash_erase() vanuit async-task kan racen met Matter-stack NVS-writes
  server.on("/matter_reset", HTTP_GET, [](AsyncWebServerRequest *request) {
    matter_nuclear_reset_requested = true;
    request->send(200, "text/html",
      "<h2 style='text-align:center;padding:40px;color:#c00;'>"
      "Matter nuclear reset gestart...<br>"
      "<small style='font-size:16px;color:#666;'>Settings worden bewaard. Rebooting in 1 sec.</small>"
      "</h2>");
    Serial.println("\n[WEB] Matter nuclear reset aangevraagd via /matter_reset");
  });

  // v1.8: /settings ook chunked — was html.reserve(10000) op heap
  // v1.10: /settings hybrid — statische form-shell + circuits via JS/fetch(/json_settings)
  // Shell is ~1.5 KB → laadt altijd volledig ongeacht heap.
  // JS bouwt circuit-blokken na onload, save via /save_settings ongewijzigd.
  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncResponseStream* p = request->beginResponseStream("text/html; charset=utf-8");
    p->print(F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>"));
    p->print(room_id);
    p->print(F(" - Settings</title><style>"
      "body{font-family:Arial,sans-serif;background:#fff;margin:0;padding:0;}"
      ".header{display:flex;background:#ffcc00;color:#000;padding:10px 15px;font-size:18px;font-weight:bold;}"
      ".header-left{flex:1;}.header-right{flex:1;text-align:right;font-size:15px;}"
      ".container{display:flex;min-height:calc(100vh - 60px);}"
      ".sidebar{width:80px;padding:10px 5px;background:#fff;border-right:3px solid #c00;}"
      ".sidebar a{display:block;background:#369;color:#fff;padding:8px;margin:8px auto;text-decoration:none;font-weight:bold;font-size:12px;border-radius:6px;text-align:center;width:60px;}"
      ".sidebar a:hover{background:#036;}.sidebar a.active{background:#c00;}"
      ".main{flex:1;padding:20px;overflow-y:auto;}"
      "table{width:100%;margin:8px 0;}td{padding:8px;}"
      "input,select{padding:6px;border:1px solid #ccc;border-radius:4px;box-sizing:border-box;}"
      ".btn{background:#369;color:#fff;padding:10px 24px;border:none;border-radius:6px;font-size:15px;cursor:pointer;margin:8px 6px;}"
      ".btn:hover{background:#036;}.btn-red{background:#c00;}.btn-red:hover{background:#900;}"
      ".cgroup{background:#369;color:#fff;padding:3px 8px;border-radius:4px;margin:12px 0 4px 0;font-weight:bold;font-size:13px;}"
      "#circuits-area{min-height:40px;}"
      ".loading{color:#999;font-style:italic;padding:8px;}"
      "@media(max-width:600px){.container{flex-direction:column;}.sidebar{width:100%;border-right:none;"
      "border-bottom:3px solid #c00;display:flex;justify-content:center;}.sidebar a{width:60px;margin:0 3px;}"
      ".main{padding:8px;}}"
      "</style></head><body>"
      "<div class='header'><div class='header-left'>"));
    p->print(room_id);
    p->print(F("</div><div class='header-right'>Instellingen</div></div>"
      "<div class='container'><div class='sidebar'>"
      "<a href='/'>Status</a><a href='/matter'>Matter</a>"
      "<a href='/update'>OTA</a><a href='/json'>JSON</a>"
      "<a href='/settings' class='active'>Settings</a>"
      "</div><div class='main'><form id='sf' action='/save_settings' method='get'>"));

    // Crash-log sectie
    {
      Preferences crashPrefs;
      crashPrefs.begin("crash-log", true);
      uint32_t crashCnt    = crashPrefs.getUInt("count", 0);
      String   crashReason = crashPrefs.getString("reason", "geen");
      crashPrefs.end();
      p->print(F("<table><tr><td style='width:35%;'>Crashteller</td><td><b style='color:"));
      p->print(crashCnt > 0 ? F("#c00") : F("#0a0"));
      p->print(F("'>"));
      p->print(crashCnt);
      p->print(F("</b>"));
      if (crashCnt > 0) p->print(F(" &nbsp;<a href='/clear_crash_log' style='font-size:12px;color:#369;'"
        " onclick='return confirm(&quot;Crash-log wissen?&quot;);'>Wissen</a>"));
      p->print(F("</td></tr><tr><td>Laatste crash</td><td><code style='font-size:12px;'>"));
      p->print(crashReason);
      p->print(F("</code></td></tr></table><hr style='border:1px solid #ccc;margin:10px 0;'>"));
    }

    // Statische velden — WiFi, IP, systeem, ECO (~1 KB, altijd werkend)
    p->print(F("<table id='base-table'>"));
    p->print(F("<tr><td style='width:35%;'>WiFi SSID</td>"
      "<td><input type='text' name='wifi_ssid' id='f_ssid' style='width:100%;'></td></tr>"));
    p->print(F("<tr><td>WiFi Password</td>"
      "<td><input type='password' name='wifi_pass' id='f_pass' style='width:100%;'></td></tr>"));
    p->print(F("<tr><td>Static IP</td>"
      "<td><input type='text' name='static_ip' id='f_ip' placeholder='leeg = DHCP' style='width:100%;'></td></tr>"));
    p->print(F("<tr><td>Room naam</td>"
      "<td><input type='text' name='room_id' id='f_rid' required style='width:100%;'></td></tr>"));
    p->print(F("<tr><td>Aantal circuits</td>"
      "<td><input type='number' name='circuits_num' id='f_cnum' min='1' max='7' style='width:100%;'></td></tr>"));
    p->print(F("<tr><td>Poll interval (sec)</td>"
      "<td><input type='number' min='5' name='poll_interval' id='f_poll' style='width:100%;'></td></tr>"));
    p->print(F("<tr><td>ECO IP adres</td>"
      "<td><input type='text' name='eco_ip' id='f_eip' style='width:100%;'></td></tr>"));
    p->print(F("<tr><td>ECO mDNS naam</td>"
      "<td><input type='text' name='eco_mdns' id='f_emdns' style='width:100%;'></td></tr>"));
    p->print(F("<tr><td>ECO Threshold (kWh)</td>"
      "<td><input type='number' step='0.1' name='eco_thresh' id='f_ethr' style='width:100%;'></td></tr>"));
    p->print(F("<tr><td>ECO Hysteresis (kWh)</td>"
      "<td><input type='number' step='0.1' name='eco_hyst' id='f_ehys' style='width:100%;'></td></tr>"));
    p->print(F("<tr><td>ECO Tmin - Stop (&deg;C)</td>"
      "<td><input type='number' step='0.1' name='eco_min_temp' id='f_emin' style='width:100%;'></td></tr>"));
    p->print(F("<tr><td>ECO Tmax - Start (&deg;C)</td>"
      "<td><input type='number' step='0.1' name='eco_max_temp' id='f_emax' style='width:100%;'></td></tr>"));
    p->print(F("<tr><td>Boiler ref temp (&deg;C)</td>"
      "<td><input type='number' step='0.1' name='boiler_ref_temp' id='f_bref' style='width:100%;'></td></tr>"));
    p->print(F("<tr><td>Boiler vol/laag (L)</td>"
      "<td><input type='number' step='1' name='boiler_volume' id='f_bvol' style='width:100%;'></td></tr>"));
    p->print(F("<tr><td>Google Script URL</td>"
      "<td><input type='text' name='gas_url' id='f_gas' placeholder='https://script.google.com/...' style='width:100%;'></td></tr>"));
    p->print(F("</table>"));

    // Sensor nicknames placeholder
    p->print(F("<div id='nicks-area'><span class='loading'>Sensor namen laden...</span></div>"));

    // Circuits placeholder
    p->print(F("<div id='circuits-area'><span class='loading'>Circuits laden...</span></div>"));

    // Submit knoppen + JS
    p->print(F("<div style='text-align:center;margin-top:16px;'>"
      "<button type='submit' class='btn'>Opslaan &amp; Reboot</button>"
      "<a href='/' class='btn btn-red' style='display:inline-block;text-decoration:none;'>Annuleren</a>"
      "</div></form>"
      "<script>"
      "fetch('/json_settings').then(r=>r.json()).then(d=>{"
        // Vul basis-velden
        "document.getElementById('f_ssid').value=d.wifi_ssid||'';"
        "document.getElementById('f_pass').value=d.wifi_pass||'';"
        "document.getElementById('f_ip').value=d.static_ip||'';"
        "document.getElementById('f_rid').value=d.room_id||'';"
        "document.getElementById('f_cnum').value=d.circuits_num||7;"
        "document.getElementById('f_poll').value=d.poll_interval||10;"
        "document.getElementById('f_eip').value=d.eco_ip||'';"
        "document.getElementById('f_emdns').value=d.eco_mdns||'';"
        "document.getElementById('f_ethr').value=d.eco_threshold||15;"
        "document.getElementById('f_ehys').value=d.eco_hysteresis||5;"
        "document.getElementById('f_emin').value=d.eco_min_temp||80;"
        "document.getElementById('f_emax').value=d.eco_max_temp||90;"
        "document.getElementById('f_bref').value=d.boiler_ref_temp||20;"
        "document.getElementById('f_bvol').value=d.boiler_layer_volume||50;"
        "document.getElementById('f_gas').value=d.gas_url||'';"
        // Sensor nicknames
        "let nh='';"
        "if(d.sensor_nicknames)d.sensor_nicknames.forEach((n,i)=>{"
          "nh+=`<label style='display:block;margin:4px 0;'>S${i+1}: "
          "<input type='text' name='sensor_nick_${i}' value='${n}' style='width:220px;'></label>`;"
        "});"
        "document.getElementById('nicks-area').innerHTML=nh;"
        // Circuits
        "let ch='';"
        "if(d.circuits)d.circuits.forEach((c,i)=>{"
          "const pins=[255,10,11,12];"
          "const pnames=['Geen','10','11','12'];"
          "ch+=`<div class='cgroup'>Circuit ${i+1}</div>`;"
          "ch+=`<table style='width:100%;margin:0 0 8px 0;'>`;"
          "ch+=`<tr><td style='width:35%'>Naam</td><td><input type='text' name='circuit_name_${i}' value='${c.name}' style='width:100%'></td></tr>`;"
          "ch+=`<tr><td>IP</td><td><input type='text' name='circuit_ip_${i}' value='${c.ip||''}' style='width:100%'></td></tr>`;"
          "ch+=`<tr><td>mDNS</td><td><input type='text' name='circuit_mdns_${i}' value='${c.mdns||''}' style='width:100%' placeholder='ZONDER .local'></td></tr>`;"
          "ch+=`<tr><td>Vermogen (kW)</td><td><input type='number' step='0.001' name='circuit_power_${i}' value='${c.power_kw.toFixed(3)}' style='width:100%'></td></tr>`;"
          "ch+=`<tr><td>TSTAT</td><td><input type='checkbox' name='circuit_tstat_${i}' value='1'${c.has_tstat?' checked':''}> Pin: <select name='circuit_tstat_pin_${i}'>`;"
          "pins.forEach((p,j)=>{"
            "ch+=`<option value='${p}'${c.tstat_pin===p?' selected':''}>${pnames[j]}</option>`;"
          "});"
          "ch+=`</select></td></tr></table>`;"
        "});"
        "document.getElementById('circuits-area').innerHTML=ch;"
      "}).catch(e=>{"
        "document.getElementById('circuits-area').innerHTML='<p style=color:#c00>Fout bij laden circuits</p>';"
      "});"
      "</script></div></div></body></html>"));
    request->send(p);
  });
  server.on("/save_settings", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("\n=== SAVE SETTINGS ===");
    
    if (request->hasArg("wifi_ssid")) preferences.putString(NVS_WIFI_SSID, request->arg("wifi_ssid"));
    if (request->hasArg("wifi_pass")) preferences.putString(NVS_WIFI_PASS, request->arg("wifi_pass"));
    if (request->hasArg("static_ip")) preferences.putString(NVS_STATIC_IP, request->arg("static_ip"));
    if (request->hasArg("room_id")) preferences.putString(NVS_ROOM_ID, request->arg("room_id"));
    if (request->hasArg("circuits_num")) preferences.putInt(NVS_CIRCUITS_NUM, request->arg("circuits_num").toInt());
    if (request->hasArg("poll_interval")) preferences.putInt(NVS_POLL_INTERVAL, request->arg("poll_interval").toInt());
    if (request->hasArg("eco_ip")) preferences.putString(NVS_ECO_IP, request->arg("eco_ip"));
    if (request->hasArg("eco_mdns")) preferences.putString(NVS_ECO_MDNS, request->arg("eco_mdns"));
    if (request->hasArg("eco_thresh")) preferences.putFloat(NVS_ECO_THRESHOLD, request->arg("eco_thresh").toFloat());
    if (request->hasArg("eco_hyst")) preferences.putFloat(NVS_ECO_HYSTERESIS, request->arg("eco_hyst").toFloat());
    if (request->hasArg("eco_min_temp")) preferences.putFloat(NVS_ECO_MIN_TEMP, request->arg("eco_min_temp").toFloat());
    if (request->hasArg("eco_max_temp")) preferences.putFloat(NVS_ECO_MAX_TEMP, request->arg("eco_max_temp").toFloat());
    if (request->hasArg("boiler_ref_temp")) preferences.putFloat(NVS_BOILER_REF_TEMP, request->arg("boiler_ref_temp").toFloat());
    if (request->hasArg("boiler_volume")) preferences.putFloat(NVS_BOILER_VOLUME, request->arg("boiler_volume").toFloat());
    if (request->hasArg("gas_url"))       preferences.putString(NVS_GAS_URL,      request->arg("gas_url"));
    
    // v1.15: snprintf i.p.v. String(i) voor NVS-keys en form-args
    char sKey[24]; char sArg[32];
    for (int i = 0; i < 6; i++) {
      snprintf(sArg, sizeof(sArg), "sensor_nick_%d", i);
      if (request->hasArg(sArg)) {
        String nick = request->arg(sArg); nick.trim();
        if (nick.length() == 0) { snprintf(sArg, sizeof(sArg), "Sensor %d", i + 1); nick = sArg; }
        snprintf(sKey, sizeof(sKey), "%s%d", NVS_SENSOR_NICK_BASE, i);
        preferences.putString(sKey, nick);
      }
    }

    int save_count = request->arg("circuits_num").toInt();
    if (save_count < 1) save_count = 1;
    if (save_count > 7) save_count = 7;

    for (int i = 0; i < save_count; i++) {
      snprintf(sArg, sizeof(sArg), "circuit_name_%d", i);
      String name_val = request->arg(sArg);
      if (name_val.length() == 0) { snprintf(sArg, sizeof(sArg), "Circuit %d", i+1); name_val = sArg; }
      snprintf(sKey, sizeof(sKey), "c%d_name", i); preferences.putString(sKey, name_val);

      snprintf(sArg, sizeof(sArg), "circuit_ip_%d", i);
      snprintf(sKey, sizeof(sKey), "c%d_ip", i);   preferences.putString(sKey, request->arg(sArg));

      snprintf(sArg, sizeof(sArg), "circuit_mdns_%d", i);
      String mdns_val = request->arg(sArg); mdns_val.replace(".local", ""); mdns_val.trim();
      snprintf(sKey, sizeof(sKey), "c%d_mdns", i); preferences.putString(sKey, mdns_val);

      snprintf(sArg, sizeof(sArg), "circuit_power_%d", i);
      snprintf(sKey, sizeof(sKey), "c%d_power", i); preferences.putFloat(sKey, request->arg(sArg).toFloat());

      snprintf(sArg, sizeof(sArg), "circuit_tstat_%d", i);
      snprintf(sKey, sizeof(sKey), "c%d_tstat", i); preferences.putBool(sKey, request->hasArg(sArg));

      int pin_val = 255;
      snprintf(sArg, sizeof(sArg), "circuit_tstat_pin_%d", i);
      if (request->hasArg(sArg)) {
        pin_val = request->arg(sArg).toInt();
        if (pin_val != 10 && pin_val != 11 && pin_val != 12) pin_val = 255;
      }
      snprintf(sKey, sizeof(sKey), "c%d_pin", i); preferences.putInt(sKey, pin_val);
    }

    Serial.println("Settings saved!");
    request->send(200, "text/html", "<h2 style='text-align:center;color:#369;'>Opgeslagen! Rebooting...</h2>");
    delay(2000);
    ESP.restart();
  });

  server.on("/circuit_override_on", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasArg("circuit")) {
      int idx = request->arg("circuit").toInt();
      if (idx >= 0 && idx < circuits_num) {
        circuits[idx].override_active = true;
        circuits[idx].override_state  = true;
        circuits[idx].override_start  = millis();
        circuits[idx].heating_on      = true;
        // v1.14: relay onmiddellijk schakelen — niet wachten op pollcyclus
        if (mcp_available) mcp.digitalWrite(idx, LOW);
        ignore_callbacks = true;
        matter_circuit[idx].setOnOff(true);
        ignore_callbacks = false;
        Serial.printf("[OVERRIDE] c%d → ON (relay onmiddellijk)\n", idx + 1);
      }
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/circuit_override_off", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasArg("circuit")) {
      int idx = request->arg("circuit").toInt();
      if (idx >= 0 && idx < circuits_num) {
        circuits[idx].override_active = true;
        circuits[idx].override_state  = false;
        circuits[idx].override_start  = millis();
        circuits[idx].heating_on      = false;
        // v1.14: relay onmiddellijk schakelen — niet wachten op pollcyclus
        if (mcp_available) mcp.digitalWrite(idx, HIGH);
        ignore_callbacks = true;
        matter_circuit[idx].setOnOff(false);
        ignore_callbacks = false;
        Serial.printf("[OVERRIDE] c%d → OFF (relay onmiddellijk)\n", idx + 1);
      }
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/circuit_override_cancel", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasArg("circuit")) {
      int idx = request->arg("circuit").toInt();
      if (idx >= 0 && idx < circuits_num) {
        circuits[idx].override_active = false;
        // v1.14: relay onmiddellijk terugzetten naar auto-staat
        if (mcp_available) mcp.digitalWrite(idx, circuits[idx].heating_on ? LOW : HIGH);
        ignore_callbacks = true;
        matter_circuit[idx].setOnOff(circuits[idx].heating_on);
        ignore_callbacks = false;
        Serial.printf("[OVERRIDE] c%d → AUTO (relay onmiddellijk)\n", idx + 1);
      }
    }
    request->send(200, "text/plain", "OK");
  });

  // Pump endpoints met ON/OFF override support
  server.on("/pump_sch_on", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("\n=== MANUAL PUMP CONTROL ===");
    Serial.println("Type: SCH");
    Serial.println("Action: ON (override)");
    Serial.println("Duration: 60 seconds");
    Serial.printf("Relay %d: LOW (pump ON)\n", RELAY_PUMP_SCH);
    
    sch_pump_manual = true;
    sch_pump_manual_on = true;
    sch_pump_manual_start = millis();
    
    if (mcp_available) mcp.digitalWrite(RELAY_PUMP_SCH, LOW);
    request->send(200, "text/plain", "OK");
  });

  server.on("/pump_sch_off", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("\n=== MANUAL PUMP CONTROL ===");
    Serial.println("Type: SCH");
    Serial.println("Action: OFF (override)");
    Serial.println("Duration: 60 seconds");
    Serial.printf("Relay %d: HIGH (pump OFF)\n", RELAY_PUMP_SCH);
    
    sch_pump_manual = true;
    sch_pump_manual_on = false;
    sch_pump_manual_start = millis();
    
    if (mcp_available) mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
    request->send(200, "text/plain", "OK");
  });

  server.on("/pump_sch_cancel", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("\n=== MANUAL PUMP CANCEL ===");
    Serial.println("Type: SCH");
    Serial.println("Manual mode cancelled");
    
    sch_pump_manual = false;
    request->send(200, "text/plain", "OK");
  });

  server.on("/pump_won_on", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("\n=== MANUAL PUMP CONTROL ===");
    Serial.println("Type: WON");
    Serial.println("Action: ON (override)");
    Serial.println("Duration: 60 seconds");
    Serial.printf("Relay %d: LOW (pump ON)\n", RELAY_PUMP_WON);
    
    won_pump_manual = true;
    won_pump_manual_on = true;
    won_pump_manual_start = millis();
    
    if (mcp_available) mcp.digitalWrite(RELAY_PUMP_WON, LOW);
    request->send(200, "text/plain", "OK");
  });

  server.on("/pump_won_off", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("\n=== MANUAL PUMP CONTROL ===");
    Serial.println("Type: WON");
    Serial.println("Action: OFF (override)");
    Serial.println("Duration: 60 seconds");
    Serial.printf("Relay %d: HIGH (pump OFF)\n", RELAY_PUMP_WON);
    
    won_pump_manual = true;
    won_pump_manual_on = false;
    won_pump_manual_start = millis();
    
    if (mcp_available) mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
    request->send(200, "text/plain", "OK");
  });

  server.on("/pump_won_cancel", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("\n=== MANUAL PUMP CANCEL ===");
    Serial.println("Type: WON");
    Serial.println("Manual mode cancelled");
    
    won_pump_manual = false;
    request->send(200, "text/plain", "OK");
  });

  // Ventilatie manuele override via UI slider
  server.on("/set_vent", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("vent")) {
      int pct = constrain(request->getParam("vent")->value().toInt(), 0, 100);
      if (pct == 0) {
        vent_override_active  = false;
        vent_override_percent = 0;
        Serial.println("[UI] Vent override uitgeschakeld → auto");
      } else {
        vent_override_start   = millis();
        vent_override_active  = true;
        vent_override_percent = pct;
        Serial.printf("[UI] Vent override → %d%%\n", pct);
      }
      // v1.15: altijd max(rooms, override) schrijven — consistent met pollRooms en JSON
      int eff = max(vent_percent, vent_override_active ? vent_override_percent : 0);
      ledcWrite(VENT_FAN_PIN, map(eff, 0, 100, 0, 255));
    }
    request->send(200, "text/plain", "OK");
  });

  // v1.7 FIX 4: Crash-log wissen via webUI
  server.on("/clear_crash_log", HTTP_GET, [](AsyncWebServerRequest *request) {
    Preferences crashPrefs;
    crashPrefs.begin("crash-log", false);
    crashPrefs.putUInt("count", 0);
    crashPrefs.putString("reason", "geen");
    crashPrefs.end();
    Serial.println("[CRASH-LOG] Gewist via webUI");
    request->redirect("/settings");
  });

  server.begin();
}

// =============================================================================
// Matter: ventilatie override timeout bewaken
// (circuit-overrides worden al beheerd in pollRooms() via OVERRIDE_TIMEOUT)
// =============================================================================
void check_vent_override() {
  if (vent_override_active &&
      millis() - vent_override_start > VENT_OVERRIDE_DURATION) {
    vent_override_active  = false;
    vent_override_percent = 0;
    // v1.15: terugval naar rooms-waarde, niet naar 0
    ledcWrite(VENT_FAN_PIN, map(vent_percent, 0, 100, 0, 255));
    Serial.printf(F("[OVERRIDE] Ventilatie — vervallen na 3u, terug naar rooms (%d%%)\n"), vent_percent);
  }
}


// =============================================================================
// Matter: sensor feedback en circuit-states pushen naar HomeKit
// =============================================================================
void update_matter_sensors() {
  matter_boiler_top.setTemperature(sch_temps[0]);
  // v1.12: matter_boiler_bot verwijderd

  // Fan: snelheidsfeedback → HomeKit — v1.15: zelfde max() formule als pin en JSON
  ignore_callbacks = true;
  int effective_vent = max(vent_percent, vent_override_active ? vent_override_percent : 0);
  matter_vent.setSpeedPercent((uint8_t)effective_vent);
  if (!vent_override_active) {
    matter_vent.setMode(effective_vent > 0
                        ? MatterFan::FAN_MODE_HIGH
                        : MatterFan::FAN_MODE_OFF);
  }
  ignore_callbacks = false;

  // Circuit-switches: alleen spiegelen als geen override actief
  ignore_callbacks = true;
  for (int i = 0; i < 7; i++) {
    if (!circuits[i].override_active) {
      matter_circuit[i].setOnOff(circuits[i].heating_on);
    }
  }
  ignore_callbacks = false;
}


void factoryResetNVS() {
  Serial.println("\n=== FACTORY RESET NVS ===");
  preferences.begin("hvac-config", false);
  preferences.clear();
  preferences.end();
  Serial.println("NVS cleared! Reboot...");
  delay(1000);
  ESP.restart();
}

// v1.10: Nuclear Matter reset — 100% betrouwbare methode
// Probleem met oude methode: nvs_erase_all() per namespace vanuit async-task
//   → task-conflict met Matter-stack, namespace-resten, onvolledige wis
// Deze methode:
//   1) Laad ALLE hvac-config naar RAM
//   2) nvs_flash_erase() — wist de VOLLEDIGE NVS-partitie atomair
//   3) nvs_flash_init()  — herinitialiseer NVS
//   4) Schrijf hvac-config terug vanuit RAM
//   5) Reboot → Matter.isDeviceCommissioned() = false gegarandeerd
void matterNuclearReset() {
  Serial.println("\n=== MATTER NUCLEAR RESET ===");
  Serial.println("Stap 1: Settings laden naar RAM...");

  // ── Laad alle hvac-config naar lokale RAM-variabelen ─────────────────────
  preferences.begin("hvac-config", true);  // read-only

  String  bk_room_id          = preferences.getString("room_id",          room_id);
  String  bk_wifi_ssid         = preferences.getString("wifi_ssid",        wifi_ssid);
  String  bk_wifi_pass         = preferences.getString("wifi_pass",        wifi_pass);
  String  bk_static_ip         = preferences.getString("static_ip",        static_ip_str);
  int     bk_circuits_num      = preferences.getInt   ("circuits_num",     circuits_num);
  int     bk_poll_interval     = preferences.getInt   ("poll_interval",    poll_interval);
  String  bk_eco_ip            = preferences.getString("eco_ip",           eco_controller_ip);
  String  bk_eco_mdns          = preferences.getString("eco_mdns",         eco_controller_mdns);
  float   bk_eco_threshold     = preferences.getFloat ("eco_threshold",    eco_threshold);
  float   bk_eco_hysteresis    = preferences.getFloat ("eco_hysteresis",   eco_hysteresis);
  float   bk_eco_min_temp      = preferences.getFloat ("eco_min_temp",     eco_min_temp);
  float   bk_eco_max_temp      = preferences.getFloat ("eco_max_temp",     eco_max_temp);
  float   bk_boiler_ref_temp   = preferences.getFloat ("boiler_ref_temp",  boiler_ref_temp);
  float   bk_boiler_volume     = preferences.getFloat ("boiler_volume",    boiler_layer_volume);
  String  bk_gas_url           = preferences.getString(NVS_GAS_URL,        gas_url);
  float   bk_tot_sch_kwh       = preferences.getFloat (NVS_TOTAL_SCH_KWH, 0.0);
  float   bk_tot_won_kwh       = preferences.getFloat (NVS_TOTAL_WON_KWH, 0.0);

  String bk_nick[6];
  for (int i = 0; i < 6; i++) {
    char bNickKey[24]; char bNickDef[16];
    snprintf(bNickKey, sizeof(bNickKey), "sensor_nick_%d", i);
    snprintf(bNickDef, sizeof(bNickDef), "Sensor %d", i+1);
    bk_nick[i] = preferences.getString(bNickKey, bNickDef);
  }

  // Circuit backup
  struct CircuitBackup {
    char name[32]; char ip[20]; char mdns[32];
    float power_kw; bool has_tstat; int tstat_pin;
  } bk_circ[7];
  int bk_num = min(bk_circuits_num, 7);
  char bKey[24];
  for (int i = 0; i < bk_num; i++) {
    String tmp;
    snprintf(bKey, sizeof(bKey), "c%d_name", i);
    char defName[16]; snprintf(defName, sizeof(defName), "Circuit %d", i+1);
    tmp = preferences.getString(bKey, defName); strlcpy(bk_circ[i].name, tmp.c_str(), 32);
    snprintf(bKey, sizeof(bKey), "c%d_ip", i);
    tmp = preferences.getString(bKey, "");       strlcpy(bk_circ[i].ip,   tmp.c_str(), 20);
    snprintf(bKey, sizeof(bKey), "c%d_mdns", i);
    tmp = preferences.getString(bKey, "");       strlcpy(bk_circ[i].mdns, tmp.c_str(), 32);
    snprintf(bKey, sizeof(bKey), "c%d_power", i);  bk_circ[i].power_kw  = preferences.getFloat(bKey, 0.0);
    snprintf(bKey, sizeof(bKey), "c%d_tstat", i);  bk_circ[i].has_tstat = preferences.getBool (bKey, false);
    snprintf(bKey, sizeof(bKey), "c%d_pin",   i);  bk_circ[i].tstat_pin = preferences.getInt  (bKey, 255);
  }
  preferences.end();
  Serial.println("  Settings in RAM geladen.");

  // ── Stap 2: Volledige NVS-partitie wissen ────────────────────────────────
  Serial.println("Stap 2: nvs_flash_erase() — volledige NVS partitie...");
  esp_err_t err = nvs_flash_erase();
  Serial.printf("  nvs_flash_erase: %s\n", esp_err_to_name(err));

  // ── Stap 3: NVS herinitialiseren ─────────────────────────────────────────
  Serial.println("Stap 3: nvs_flash_init()...");
  err = nvs_flash_init();
  Serial.printf("  nvs_flash_init: %s\n", esp_err_to_name(err));

  // ── Stap 4: hvac-config terugschrijven ───────────────────────────────────
  Serial.println("Stap 4: Settings terugschrijven...");
  preferences.begin("hvac-config", false);
  preferences.putString("room_id",       bk_room_id);
  preferences.putString("wifi_ssid",     bk_wifi_ssid);
  preferences.putString("wifi_pass",     bk_wifi_pass);
  preferences.putString("static_ip",     bk_static_ip);
  preferences.putInt   ("circuits_num",  bk_circuits_num);
  preferences.putInt   ("poll_interval", bk_poll_interval);
  preferences.putString("eco_ip",        bk_eco_ip);
  preferences.putString("eco_mdns",      bk_eco_mdns);
  preferences.putFloat ("eco_threshold", bk_eco_threshold);
  preferences.putFloat ("eco_hysteresis",bk_eco_hysteresis);
  preferences.putFloat ("eco_min_temp",  bk_eco_min_temp);
  preferences.putFloat ("eco_max_temp",  bk_eco_max_temp);
  preferences.putFloat ("boiler_ref_temp", bk_boiler_ref_temp);
  preferences.putFloat ("boiler_volume", bk_boiler_volume);
  preferences.putString(NVS_GAS_URL,     bk_gas_url);
  preferences.putFloat (NVS_TOTAL_SCH_KWH, bk_tot_sch_kwh);
  preferences.putFloat (NVS_TOTAL_WON_KWH, bk_tot_won_kwh);
  for (int i = 0; i < 6; i++) {
    snprintf(bKey, sizeof(bKey), "sensor_nick_%d", i);
    preferences.putString(bKey, bk_nick[i]);
  }
  for (int i = 0; i < bk_num; i++) {
    snprintf(bKey, sizeof(bKey), "c%d_name",  i); preferences.putString(bKey, bk_circ[i].name);
    snprintf(bKey, sizeof(bKey), "c%d_ip",    i); preferences.putString(bKey, bk_circ[i].ip);
    snprintf(bKey, sizeof(bKey), "c%d_mdns",  i); preferences.putString(bKey, bk_circ[i].mdns);
    snprintf(bKey, sizeof(bKey), "c%d_power", i); preferences.putFloat (bKey, bk_circ[i].power_kw);
    snprintf(bKey, sizeof(bKey), "c%d_tstat", i); preferences.putBool  (bKey, bk_circ[i].has_tstat);
    snprintf(bKey, sizeof(bKey), "c%d_pin",   i); preferences.putInt   (bKey, bk_circ[i].tstat_pin);
  }
  preferences.end();
  Serial.println("  Settings teruggeschreven.");

  // ── Stap 5: Reboot ───────────────────────────────────────────────────────
  Serial.println("Stap 5: Rebooting... Matter zal ongepaard opstarten.");
  delay(500);
  ESP.restart();
}
















// ============== DEEL 5/5: SETUP & LOOP ==============

void setup() {
  Serial.begin(115200);
  delay(1500);  // Wacht tot Serial Monitor klaar is
  while (Serial.available()) Serial.read();  // Flush eventuele rommel
  Serial.println("\n\n=== HVAC Controller v1.10 ===");
  Serial.println("Commando's: 'R' = NVS reset, 'reset-matter', 'reset-all', 'status'");

  // v1.7 FIX 4: Crash-log lezen bij boot — toont of vorige run gecrasht is
  {
    Preferences crashPrefs;
    crashPrefs.begin("crash-log", true);
    uint32_t crashCnt    = crashPrefs.getUInt("count", 0);
    String   crashReason = crashPrefs.getString("reason", "geen");
    crashPrefs.end();
    if (crashCnt > 0) {
      Serial.printf("[BOOT] ⚠️  Vorige crash (#%u): %s\n", crashCnt, crashReason.c_str());
    } else {
      Serial.println("[BOOT] Geen crashes geregistreerd.");
    }
  }

  Serial.println("\nType 'R' binnen 5 sec voor volledige NVS reset...");
  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'R' || c == 'r') {
        Serial.println("-> NVS reset gestart!");
        factoryResetNVS();
      }
    }
    delay(10);
  }
  Serial.println("(geen reset)");

  // Ventilatie PWM output (ESP32-C6: ledcAttach ipv analogWrite)
  ledcAttach(VENT_FAN_PIN, 1000, 8);  // pin, 1kHz, 8-bit (0-255)
  ledcWrite(VENT_FAN_PIN, 0);          // Start op 0%

  // I2C & MCP23017
  Wire.begin(I2C_SDA, I2C_SCL);
  if (mcp.begin_I2C(0x20)) {
    Serial.println("MCP23017 OK!");
    mcp_available = true;
    
    for (int i = 0; i < 7; i++) {
      mcp.pinMode(i, OUTPUT);
      mcp.digitalWrite(i, HIGH);
    }
    mcp.pinMode(7, INPUT_PULLUP);
    mcp.pinMode(8, OUTPUT);
    mcp.digitalWrite(8, HIGH);
    mcp.pinMode(9, OUTPUT);
    mcp.digitalWrite(9, HIGH);
    mcp.pinMode(10, INPUT_PULLUP);
    mcp.pinMode(11, INPUT_PULLUP);
    mcp.pinMode(12, INPUT_PULLUP);
    mcp.pinMode(13, INPUT_PULLUP);
    mcp.pinMode(14, INPUT_PULLUP);
    mcp.pinMode(15, INPUT_PULLUP);
  } else {
    Serial.println("MCP23017 not found!");
    mcp_available = false;
  }

  // Load NVS
  preferences.begin("hvac-config", false);

  // v1.11: getString→strlcpy naar char[] — geen permanente heap-alloc
  strlcpy(room_id,           preferences.getString(NVS_ROOM_ID,   "HVAC").c_str(), sizeof(room_id));
  strlcpy(wifi_ssid,         preferences.getString(NVS_WIFI_SSID, "").c_str(),     sizeof(wifi_ssid));
  strlcpy(wifi_pass,         preferences.getString(NVS_WIFI_PASS, "").c_str(),     sizeof(wifi_pass));
  strlcpy(static_ip_str,     preferences.getString(NVS_STATIC_IP, "").c_str(),     sizeof(static_ip_str));
  circuits_num = preferences.getInt(NVS_CIRCUITS_NUM, 7);
  circuits_num = constrain(circuits_num, 1, 16);
  
  eco_threshold = preferences.getFloat(NVS_ECO_THRESHOLD, 15.0);  // V53.5: Updated
  eco_hysteresis = preferences.getFloat(NVS_ECO_HYSTERESIS, 5.0);  // V53.5: Updated
  poll_interval = preferences.getInt(NVS_POLL_INTERVAL, 10);
  strlcpy(eco_controller_ip,   preferences.getString(NVS_ECO_IP,   "").c_str(),  sizeof(eco_controller_ip));
  strlcpy(eco_controller_mdns, preferences.getString(NVS_ECO_MDNS, "eco").c_str(), sizeof(eco_controller_mdns));
  strlcpy(gas_url,             preferences.getString(NVS_GAS_URL,  "").c_str(),  sizeof(gas_url));
  eco_min_temp = preferences.getFloat(NVS_ECO_MIN_TEMP, 80.0);  // V53.5: Stop temp (was 60.0)
  eco_max_temp = preferences.getFloat(NVS_ECO_MAX_TEMP, 90.0);  // V53.5: Start temp (was 80.0)
  boiler_ref_temp = preferences.getFloat(NVS_BOILER_REF_TEMP, 20.0);
  boiler_layer_volume = preferences.getFloat(NVS_BOILER_VOLUME, 50.0);
  
  // Load last pump events
  last_sch_pump.timestamp = preferences.getULong(NVS_LAST_SCH_PUMP, 0);
  last_sch_pump.kwh_pumped = preferences.getFloat(NVS_LAST_SCH_KWH, 0.0);
  last_won_pump.timestamp = preferences.getULong(NVS_LAST_WON_PUMP, 0);
  last_won_pump.kwh_pumped = preferences.getFloat(NVS_LAST_WON_KWH, 0.0);
  
  // Load totalen
  total_sch_kwh = preferences.getFloat(NVS_TOTAL_SCH_KWH, 0.0);
  total_won_kwh = preferences.getFloat(NVS_TOTAL_WON_KWH, 0.0);
  
  Serial.printf("\nBoiler Qtot settings:\n");
  Serial.printf("  Reference temp: %.1f °C\n", boiler_ref_temp);
  Serial.printf("  Volume per laag: %.0f L\n", boiler_layer_volume);
  
  Serial.printf("\nECO Boiler settings:\n");
  Serial.printf("  Tmin (Stop): %.1f °C\n", eco_min_temp);
  Serial.printf("  Tmax (Start): %.1f °C\n", eco_max_temp);
  Serial.printf("  Threshold: %.1f kWh\n", eco_threshold);
  Serial.printf("  Hysteresis: %.1f kWh\n", eco_hysteresis);
  
  Serial.printf("\nECO Pomp totalen:\n");
  Serial.printf("  SCH totaal: %.1f kWh\n", total_sch_kwh);
  Serial.printf("  WON totaal: %.1f kWh\n", total_won_kwh);
  
  if (last_sch_pump.timestamp > 0) {
    Serial.printf("\nLast SCH pump event: %.2f kWh\n", last_sch_pump.kwh_pumped);
  }
  if (last_won_pump.timestamp > 0) {
    Serial.printf("Last WON pump event: %.2f kWh\n", last_won_pump.kwh_pumped);
  }

  // v1.15: snprintf i.p.v. String(i) — geen heap-alloc voor NVS-keys
  char nvsKey[24];
  for (int i = 0; i < 6; i++) {
    snprintf(nvsKey, sizeof(nvsKey), "%s%d", NVS_SENSOR_NICK_BASE, i);
    char defNick[16]; snprintf(defNick, sizeof(defNick), "Sensor %d", i + 1);
    String tmp = preferences.getString(nvsKey, defNick);
    strlcpy(sensor_nicknames[i], tmp.c_str(), 32);
  }

  // v1.9: loop tot circuits_num (7) i.p.v. 16
  for (int i = 0; i < circuits_num; i++) {
    char defName[16]; snprintf(defName, sizeof(defName), "Circuit %d", i + 1);
    snprintf(nvsKey, sizeof(nvsKey), "c%d_name", i);
    String tmp = preferences.getString(nvsKey, defName);
    strlcpy(circuits[i].name, tmp.c_str(), sizeof(circuits[i].name));
    snprintf(nvsKey, sizeof(nvsKey), "c%d_ip", i);
    tmp = preferences.getString(nvsKey, "");
    strlcpy(circuits[i].ip, tmp.c_str(), sizeof(circuits[i].ip));
    snprintf(nvsKey, sizeof(nvsKey), "c%d_mdns", i);
    tmp = preferences.getString(nvsKey, "");
    strlcpy(circuits[i].mdns, tmp.c_str(), sizeof(circuits[i].mdns));
    snprintf(nvsKey, sizeof(nvsKey), "c%d_power", i);
    circuits[i].power_kw  = preferences.getFloat(nvsKey, 0.0);
    snprintf(nvsKey, sizeof(nvsKey), "c%d_tstat", i);
    circuits[i].has_tstat = preferences.getBool(nvsKey, false);
    snprintf(nvsKey, sizeof(nvsKey), "c%d_pin", i);
    circuits[i].tstat_pin = preferences.getInt(nvsKey, 255);

    circuits[i].online = false;
    circuits[i].heating_on = false;
    circuits[i].vent_request = 0;
    circuits[i].on_time = 1;
    circuits[i].off_time = 100;
    circuits[i].last_change = millis();
    circuits[i].duty_cycle = 0.0;
    circuits[i].override_active = false;
    // v1.16: sliding window initialiseren
    memset(circuits[i].dc_window, 0, sizeof(circuits[i].dc_window));
    circuits[i].dc_slot         = 0;
    circuits[i].dc_slots_filled = 0;
    circuits[i].dc_slot_start   = millis();
    circuits[i].dc_last_poll    = millis();
    circuits[i].dc_slot_on      = 0;
    circuits[i].duty_4h         = 0.0;
  }

  Serial.println("\n=== Circuit Config ===");
  for (int i = 0; i < circuits_num; i++) {
    Serial.printf("c%d: %s", i + 1, circuits[i].name);
    if (circuits[i].has_tstat) Serial.printf(" [TSTAT pin %d]", circuits[i].tstat_pin);
    if (strlen(circuits[i].ip)   > 0) Serial.printf(" [IP: %s]",   circuits[i].ip);
    if (strlen(circuits[i].mdns) > 0) Serial.printf(" [mDNS: %s]", circuits[i].mdns);
    Serial.printf(" [%.3f kW]\n", circuits[i].power_kw);
  }

  // v1.13: Filter-documenten eenmalig initialiseren — kleine permanente allocatie
  // voorkomt dat elke pollRooms/pollEcoBoiler een nieuw JsonDocument aanmaakt
  room_filter_doc["b"]  = true;  // heat_request (Heating_on)
  room_filter_doc["g"]  = true;  // vent_request (Vent_percent)
  room_filter_doc["c"]  = true;  // setpoint (Heating_setpoint)
  room_filter_doc["e"]  = true;  // room_temp (Temp1 DHT22)
  room_filter_doc["v"]  = true;  // home_status (Home switch)
  eco_filter_doc["b"]  = true;  // temp_top    (ETopH)
  eco_filter_doc["g"]  = true;  // temp_bottom (EBotL)
  eco_filter_doc["h"]  = true;  // temp_avg    (EAv)
  eco_filter_doc["i"]  = true;  // qtot        (EQtot)
  poll_filters_initialized = true;
  Serial.println(F("[v1.13] Poll-filters geïnitialiseerd"));

  // WiFi verbinding met 20s timeout per poging
  WiFi.mode(WIFI_STA);


  if (strlen(static_ip_str) > 0 && static_ip.fromString(static_ip_str)) {
    IPAddress gateway(static_ip[0], static_ip[1], static_ip[2], 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.config(static_ip, gateway, subnet, gateway);
    Serial.printf("Static IP: %s  Gateway (auto): %s\n",
                  static_ip_str, gateway.toString().c_str());
  }

  if (strlen(wifi_ssid) > 0) {
    Serial.printf("\nConnecting to '%s'...\n", wifi_ssid);
    
    int retry_count = 0;
    const int MAX_RETRIES = 5;
    bool connected = false;
    
    while (!connected && retry_count < MAX_RETRIES) {
      WiFi.begin(wifi_ssid, wifi_pass);
      
      unsigned long start_attempt = millis();
      while (WiFi.status() != WL_CONNECTED && (millis() - start_attempt) < 20000) {
        delay(500);
        Serial.print(".");
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        connected = true;
        Serial.println("\n✓ WiFi connected!");
      } else {
        retry_count++;
        Serial.printf("\n✗ Attempt %d/%d failed\n", retry_count, MAX_RETRIES);
        
        if (retry_count < MAX_RETRIES) {
          Serial.println("Disconnecting and retrying...");
          WiFi.disconnect();
          delay(2000);
        }
      }
    }
    
    if (!connected) {
      Serial.println("\n✗ All WiFi attempts failed -> AP mode");
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi failed -> AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("HVAC-Setup");
    ap_mode_active = true;
    
    Serial.println("\n=== CAPTIVE PORTAL ACTIVE ===");
    Serial.println("AP SSID: HVAC-Setup");
    Serial.println("AP IP: " + WiFi.softAPIP().toString());
    Serial.println("DNS: Wildcard redirect to settings page");
    Serial.println("→ Connect and settings will auto-open!");
    
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  } else {
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
    Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
    
    // NTP SYNC
    Serial.println("\nSyncing NTP time...");
    configTime(3600, 3600, "pool.ntp.org", "time.nist.gov");
    setenv("TZ", "CET-1CEST,M3.5.0/02,M10.5.0/03", 1);
    tzset();
    
    int retry = 0;
    time_t now = 0;
    struct tm timeinfo;
    while (now < 1700000000 && retry < 20) {
      time(&now);
      localtime_r(&now, &timeinfo);
      delay(500);
      Serial.print(".");
      retry++;
    }
    
    if (now >= 1700000000) {
      Serial.println(" OK!");
      Serial.print("Time: "); Serial.println(getFormattedDateTime());
    } else {
      Serial.println(" TIMEOUT");
      Serial.println("Tijd wordt niet getoond tot NTP sync lukt");
    }
  }

  // mDNS verwijderd (v1.7 FIX 5) — veroorzaakte mdns_service_remove_for_host fouten samen met Matter-stack.
  // Gebruik static IP 192.168.0.70 of de hostnaam in router-DHCP-tabel voor toegang.

  // ── Matter initialisatie (alleen als WiFi verbonden — niet in AP mode) ────
  if (!ap_mode_active) {
    Serial.println(F("\n── Matter initialisatie ────────────────────────────────"));

    // v1.8: enkel top — bot verwijderd in v1.12
    matter_boiler_top.begin();

    // Circuit-plugins: begin + callback
    for (int i = 0; i < 7; i++) {
      matter_circuit[i].begin();
      matter_circuit[i].setOnOff(circuits[i].heating_on);

      matter_circuit[i].onChangeOnOff([i](bool on_off) -> bool {
        if (ignore_callbacks) return true;
        circuits[i].override_start  = millis();
        circuits[i].override_active = true;
        circuits[i].override_state  = on_off;
        Serial.printf("[HomeKit] Kring %d '%s' → override %s (10m)\n",
                      i + 1, circuits[i].name, on_off ? "AAN" : "UIT");
        return true;
      });
    }

    // v1.12: matter_alles_auto verwijderd — alles_auto_requested flag niet meer gebruikt

    // MatterFan: ventilatiebediening
    matter_vent.begin(0, MatterFan::FAN_MODE_OFF, MatterFan::FAN_MODE_SEQ_OFF_HIGH);

    matter_vent.onChangeSpeedPercent([](uint8_t new_pct) -> bool {
      if (ignore_callbacks) return true;
      if (new_pct == 0) {
        vent_override_active  = false;
        vent_override_percent = 0;
        ledcWrite(VENT_FAN_PIN, 0);  // Direct naar pin
        Serial.println(F("[HomeKit] Vent speed = 0% → terug naar auto"));
      } else {
        vent_override_start   = millis();
        vent_override_active  = true;
        vent_override_percent = new_pct;
        ledcWrite(VENT_FAN_PIN, map(new_pct, 0, 100, 0, 255));  // Direct naar pin
        Serial.printf("[HomeKit] Vent speed override → %d%%\n", new_pct);
      }
      return true;
    });

    matter_vent.onChangeMode([](uint8_t new_mode) -> bool {
      if (ignore_callbacks) return true;
      if (new_mode == MatterFan::FAN_MODE_OFF) {
        vent_override_active  = false;
        vent_override_percent = 0;
        ignore_callbacks = true;
        matter_vent.setSpeedPercent(0);
        ignore_callbacks = false;
        Serial.println(F("[HomeKit] Vent mode OFF → terug naar auto, speed=0"));
      } else {
        if (!vent_override_active) {
          vent_override_start   = millis();  // EERST
          vent_override_active  = true;      // DAN
          vent_override_percent = 30;
          ignore_callbacks = true;
          matter_vent.setSpeedPercent(30);
          ignore_callbacks = false;
          Serial.println(F("[HomeKit] Vent mode AAN → override 30%"));
        }
      }
      return true;
    });

    // v1.10: Heap-diagnose voor Matter.begin() — geeft inzicht in stack-kost
    uint32_t heap_pre = ESP.getFreeHeap();
    uint32_t lb_pre   = ESP.getMaxAllocHeap();
    Serial.printf("[HEAP pre-Matter]  free=%u  largest=%u  min_ever=%u\n",
      heap_pre, lb_pre, ESP.getMinFreeHeap());

    // Matter starten — detecteer fout via isDeviceCommissioned na begin
    Matter.begin();

    // Check of Matter correct geinitialiseerd is
    bool matter_ok = true;
    // Korte delay om stack te laten starten
    delay(200);

    // v1.10: Heap-diagnose na Matter.begin()
    uint32_t heap_post = ESP.getFreeHeap();
    uint32_t lb_post   = ESP.getMaxAllocHeap();
    Serial.printf("[HEAP post-Matter] free=%u  largest=%u  min_ever=%u\n",
      heap_post, lb_post, ESP.getMinFreeHeap());
    Serial.printf("[HEAP Matter kost] free:-%d  largest:-%d\n",
      (int)(heap_pre - heap_post), (int)(lb_pre - lb_post));

    // Als getManualPairingCode() leeg is EN niet commissioned → init gefaald
    if (!Matter.isDeviceCommissioned() && Matter.getManualPairingCode().length() < 5) {
      matter_ok = false;
    }

    // Pairing check
    Serial.println(F("\n══════════════════════════════════════════"));
    if (!matter_ok) {
      Serial.println(F("MATTER: Initialisatie MISLUKT (NVS corrupt?)"));
      Serial.println(F("-> Nuclear reset: NVS volledig wissen, settings bewaren, reboot..."));
      matterNuclearReset();  // v1.10: nuclear reset i.p.v. per-namespace wis
    } else if (!Matter.isDeviceCommissioned()) {
      Serial.println(F("MATTER: Nog niet gepaard."));
      Serial.println(F("► Manuele code:"));
      Serial.println("    " + Matter.getManualPairingCode());
      Serial.println("► http://" + WiFi.localIP().toString() + "/matter");
      Serial.println(F("Wacht op commissioning... (timeout 5 min, daarna doorgaan)"));
      unsigned long pair_start = millis();
      while (!Matter.isDeviceCommissioned() && (millis() - pair_start < 300000)) {
        delay(500); Serial.print(".");
      }
      if (Matter.isDeviceCommissioned()) {
        Serial.println(F("\nGEPAARD!"));
      } else {
        Serial.println(F("\nTimeout — doorgaan zonder pairing. Ga naar /matter om te koppelen."));
      }
    } else {
      Serial.println(F("MATTER: Al gepaard. Ga naar /matter voor reset."));
    }
    Serial.println(F("══════════════════════════════════════════\n"));
  }
  // ── Einde Matter initialisatie ────────────────────────────────────────────

  setupWebServer();
  Serial.println("\nWeb server started!");
  Serial.println("Ready!\n");
}

// v1.15: TSTAT snelcheck — aangeroepen elke 100ms vanuit loop()
// Leest MCP-pins 10/11/12 direct, schakelt relay onmiddellijk bij flankwijziging.
// Override-staat heeft voorrang: als override actief is, negeer TSTAT.
void checkTstatPins() {
  if (!mcp_available) return;
  for (int i = 0; i < circuits_num; i++) {
    if (!circuits[i].has_tstat) continue;
    int pin = circuits[i].tstat_pin;
    if (pin < 10 || pin > 12) continue;          // alleen hardware TSTAT-pins
    if (circuits[i].override_active) continue;   // override heeft voorrang

    bool tstat_on = (mcp.digitalRead(pin) == LOW);  // LOW = GND = thermostaat vraagt warmte
    if (tstat_on == tstat_last_state[i]) continue;  // geen wijziging
    tstat_last_state[i] = tstat_on;

    // Flankwijziging gedetecteerd — relay onmiddellijk schakelen
    circuits[i].heating_on = tstat_on;
    mcp.digitalWrite(i, tstat_on ? LOW : HIGH);
    ignore_callbacks = true;
    matter_circuit[i].setOnOff(tstat_on);
    ignore_callbacks = false;
    Serial.printf("[TSTAT] c%d pin%d → %s (onmiddellijk)\n",
                  i + 1, pin, tstat_on ? "AAN" : "UIT");
  }
}

void loop() {
  if (ap_mode_active) dnsServer.processNextRequest();

  uptime_sec = millis() / 1000;

  // v1.10: Nuclear reset flag — wordt gezet door /matter_reset handler (async-task)
  // Uitvoering vanuit main loop: geen task-conflict met Matter-stack NVS-writes
  if (matter_nuclear_reset_requested) {
    matter_nuclear_reset_requested = false;
    delay(200);  // Geef async response tijd om te verzenden
    matterNuclearReset();
  }

  // ── Matter serial commando's ─────────────────────────────────────────────
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("reset-matter")) {
      // v1.10: nuclear reset via serial — zelfde methode als web-knop
      Serial.println(F("Matter nuclear reset (instellingen blijven intact)..."));
      matterNuclearReset();
    }
    if (cmd.equalsIgnoreCase("reset-all")) {
      Serial.println(F("Alles wissen (instellingen + Matter) + reboot..."));
      preferences.begin("hvac-config", false);
      preferences.clear();
      preferences.end();
      nvs_flash_erase();
      delay(300);
      ESP.restart();
    }
    if (cmd.equalsIgnoreCase("status")) {
      Serial.printf("\n=== HVAC Status | Uptime: %lu s ===\n", uptime_sec);
      for (int i = 0; i < circuits_num; i++) {
        bool eff = circuits[i].override_active ? circuits[i].override_state : circuits[i].heating_on;
        Serial.printf("c%d %-12s %.3fkW  %s [%s]\n",
          i + 1, circuits[i].name, circuits[i].power_kw,
          eff ? "AAN" : "UIT", circuits[i].override_active ? "OVR" : "AUT");
      }
      Serial.printf("Vent: %d%%  Totaal: %.3f kW\n", vent_percent, total_power);
    }
  }

  // v1.12: alles_auto_requested + matter_alles_auto verwijderd

  // ── Ventilatie override timeout ──────────────────────────────────────────
  check_vent_override();

  // v1.7 FIX 4: 60s heap-bewaking — schrijf naar NVS crash-log als largest block < 25 KB
  if (millis() - last_slow >= 60000) {
    last_slow = millis();
    uint32_t lb = ESP.getMaxAllocHeap();
    if (lb < 25000) {
      Preferences crashPrefs;
      crashPrefs.begin("crash-log", false);
      uint32_t cnt = crashPrefs.getUInt("count", 0) + 1;
      crashPrefs.putUInt("count", cnt);
      char reason[40];
      snprintf(reason, sizeof(reason), "heap %uKB @ %lus", lb / 1024, uptime_sec);
      crashPrefs.putString("reason", reason);
      crashPrefs.end();
      Serial.printf("[HEAP] ⚠️  Largest block %u KB — crash-log geschreven (#%u)\n", lb / 1024, cnt);
    }
  }

  readBoilerTemps();
  pollRooms();
  pollEcoBoiler();
  handleEcoPumps();

  // v1.13: Google Sheets push elke 5 minuten
  if (strlen(gas_url) > 0 && millis() - last_gas_push >= 300000UL) {
    last_gas_push = millis();
    pushToGoogleSheets();
  }

  // ── Matter update elke 5s ────────────────────────────────────────────────
  if (!ap_mode_active && millis() - last_matter_update > 5000) {
    last_matter_update = millis();
    update_matter_sensors();
  }

  // v1.15: TSTAT snelcheck elke 100ms — onmiddellijke relay-reactie op thermostaatwijziging
  checkTstatPins();

  delay(100);
}
