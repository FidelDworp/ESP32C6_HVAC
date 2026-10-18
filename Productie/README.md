# ESP32C6 HVAC Controller

**Intelligent centrale verwarmings- en ventilatiesturing voor een modern verwarmingssysteem met meerdere energiebronnen**

---

## 📋 Inhoudsopgave

1. [High-Level Overzicht](#high-level-overzicht) *(Voor nieuwkomers)*
2. [Technische Documentatie](#technische-documentatie) *(Voor technici)*
3. [Versiegeschiedenis](#versiegeschiedenis)
4. [Hardware Vereisten](#hardware-vereisten)
5. [Software Architectuur](#software-architectuur)
6. [API Documentatie](#api-documentatie)
7. [Installatie & Setup](#installatie--setup)

---

# High-Level Overzicht
*Voor nieuwkomers zonder technische achtergrond*

## Wat doet dit systeem?

Stel je voor: je hebt een huis met meerdere kamers, elk met zijn eigen verwarming. Je hebt ook verschillende warmtebronnen:
- Een **warmtepomp** in de schuur (je "hoofdbron")
- **Zonnepanelen** met een boiler
- **Haarden** in huis

Dit systeem is de "dirigent" die alles slim coördineert, zodat je huis comfortabel blijft tegen de laagste kosten.

## Hoe werkt het?

### 🏠 Kamerbeheer
Het systeem bewaakt 7 verschillende ruimtes in je huis:
- **Badkamer**
- **Woonkamer**
- **Berging**
- **Zolderkamer**
- **Eerste verdieping**
- **Keuken**
- **Inkomhal**

Voor elke kamer weet het systeem:
- 🌡️ Wat is de gewenste temperatuur?
- 🏡 Is er iemand thuis?
- 📊 Hoeveel energie gebruikt deze kamer?
- 💨 Hoeveel ventilatie is nodig?

### ⚡ Slimme Energieverdeling

**Het probleem:**
Je warmtepomp maakt warm water, maar soms hebben je zonnepanelen of haarden EXTRA warmte over. Zonde om dat te verspillen!

**De oplossing:**
Het systeem heeft **twee pompen** die automatisch extra warmte van je zonneboiler naar je hoofdboiler kunnen pompen wanneer:
- De zonneboiler te warm wordt (> 80°C)
- Er veel energie over is (> 12 kWh)

De pompen werken **afwisselend** voor eerlijke verdeling, en het systeem houdt bij hoeveel energie er is overgedragen.

### 🌬️ Slimme Ventilatie

Elke kamer kan aangeven hoeveel ventilatie nodig is (0-100%). Het systeem stuurt de centrale ventilator aan op basis van de kamer met de hoogste vraag.

### 📱 Bediening

Je kunt het systeem bedienen via:
- **Webbrowser** (computer, tablet, smartphone)
- **Thermostaten** in de kamers
- **Automatische modus** (laat het systeem zelf beslissen)

## Wat zie je op het scherm?

### Hoofdpagina
- **Status per kamer**: Temperatuur, setpoint, of de verwarming aan is
- **Energie overzicht**: Hoeveel vermogen gebruikt elke kamer?
- **Boiler status**: Temperaturen van beide boilers
- **Pomp controle**: Handmatig pompen aan/uit zetten (met timer)
- **Ventilatie**: Actuele ventilatie percentage

### Instellingen
- **WiFi configuratie**: Verbind met je netwerk
- **Kamer instellingen**: IP adressen, vermogen per circuit
- **Boiler instellingen**: Wanneer moeten pompen starten/stoppen?
- **Sensoren**: Geef je sensoren eigen namen

## Waarom is dit handig?

1. ⚡ **Energiebesparing**: Gebruikt gratis zonne-energie optimaal
2. 🎯 **Comfort**: Elke kamer op de perfecte temperatuur
3. 📊 **Inzicht**: Zie precies welke kamer hoeveel energie gebruikt
4. 🔒 **Betrouwbaar**: Blijft werken, ook als internet uitvalt
5. 🛠️ **Flexibel**: Alle instellingen eenvoudig aan te passen

---

# Technische Documentatie
*Voor technici met hardware en software kennis*

## Systeemarchitectuur

### Hardware Platform
- **Microcontroller**: ESP32-C6 (WiFi 6, Bluetooth 5, RISC-V)
- **I/O Expander**: MCP23017 (16-bit I2C GPIO)
- **Sensors**: DS18B20 digitale temperatuursensoren (1-Wire)
- **Actuators**: 
  - 7× Relay modules (verwarmingscircuits)
  - 2× Relay modules (energie transfer pompen)
  - 1× PWM output (IO20 → AnaBoX → 0-10V-ingang ventilatie-unit Begetube 350 R)
- **Communication**: WiFi 2.4GHz (Station + AP modes)

### Software Stack
- **Framework**: Arduino-ESP32 (ESP-IDF underneath)
- **Web Server**: ESPAsyncWebServer
- **Storage**: NVS (Non-Volatile Storage) voor persistentie
- **Protocols**: 
  - HTTP/JSON (room controller polling)
  - mDNS (service discovery)
  - OTA (Over-The-Air updates)
  - DNS (Captive portal in AP mode)

## Functionele Modules

### 1. Room Controller Polling Engine

**Functie**: Periodiek uitlezen van 7 verwarmingscircuits via WiFi

**Data per circuit**:
```json
{
  "y": 1,          // Heat request (0/1)
  "z": 75,         // Ventilation request (0-100%)
  "aa": 22,        // Setpoint (°C)
  "h": 21.5,       // Room temperature (°C)
  "af": 1          // Home status (0=away, 1=home)
}
```

**Beslissingslogica**:
```
IF circuit.online:
  IF home_status == HOME:
    heating_demand = TSTAT OR HTTP
  ELSE:
    heating_demand = HTTP only
ELSE:
  heating_demand = TSTAT only
```

**Fallback mechanisme**:
- IP polling (primair)
- mDNS resolution (fallback als IP verandert)
- Thermostat input (fallback als offline)

**Features**:
- 10 seconden poll interval (configureerbaar)
- 4s HTTP timeout met 1.5s connect timeout
- Retry met exponentiële backoff
- WiFi reconnect op disconnect detectie

### 2. ECO Pump Controller

**Doel**: Automatische energie transfer van ECO boiler (solar/haarden) naar SCH boiler (warmtepomp)

**Hardware**:
- 2× Pompen (SCH en WON circuits)
- MCP23017 relay control (pin 8 & 9)

**Automatische Modus - State Machine**:

```
States:
  IDLE → Start conditions met → PUMP_SCH of PUMP_WON
  PUMP_X → 30 min → WAIT
  WAIT → 30 min → Check conditions → PUMP_Y (alternating)
```

**Start Condities (OR logic)**:
```cpp
START = eco_boiler.online AND 
        (temp_top > eco_max_temp OR qtot > eco_threshold)
```

**Stop Condities (OR logic)**:
```cpp
STOP = NOT eco_boiler.online OR
       (temp_top < eco_min_temp OR qtot < threshold - hysteresis)
```

**Pump Cycle Parameters**:
- **Duration**: 30 minuten per pomp
- **Wait**: 30 minuten tussen cycles
- **Alternating**: SCH → WON → SCH → WON (50/50 verdeling)
- **No maximum**: Blijft pompen zolang condities voldaan

**Manual Override**:
- **Mode**: ON override (60s) of OFF override (60s)
- **Timer**: Countdown visible in UI met auto-refresh
- **Cancel**: Mid-cycle annulering mogelijk
- **State tracking**: Separate boolean voor ON/OFF state

**Telemetrie**:
```cpp
struct PumpEvent {
  time_t timestamp;        // Unix timestamp van event
  float kwh_pumped;        // kWh overgedragen
};

// NVS Storage:
- Last event per pomp (timestamp + kWh)
- Cumulative totals (total_sch_kwh, total_won_kwh)
```

### 3. Boiler Temperature Monitoring

**SCH Boiler (Warmtepomp schuur)**:
- **Sensors**: 6× DS18B20 in gestratificeerde lagen
- **Plaatsing**: TopH, TopL, MidH, MidL, BotH, BotL
- **Protocol**: 1-Wire (Dallas/Maxim)
- **Resolution**: 12-bit (0.0625°C)
- **Timing**: 750ms per conversie

**Qtot Berekening (Energieinhoud)**:
```cpp
// Per laag: dT × volume × specifieke warmte
float layer_kwh = (temp - ref_temp) × layer_volume × 4.186 / 3600;

// Totaal: som van alle lagen
Qtot = Σ(layer_kwh)
```

**Parameters**:
- Reference temp: 20°C (configureerbaar)
- Volume per laag: 50L (configureerbaar)
- Specifieke warmte water: 4.186 kJ/kg·K

**ECO Boiler (Solar + Haarden)**:
- **Polling**: Via dedicated ECO controller (sketch #3)
- **Endpoint**: HTTP GET /status.json
- **Data**: temp_top, temp_avg, qtot
- **Update rate**: Poll interval (10s default)

### 4. Ventilation Control

**Input**: Max ventilation request van alle circuits
```cpp
vent_percent = max(circuit[0..6].vent_request)
```

**Effectieve ventilatie** (sinds v1.13): de hoogste waarde wint — kamervraag of HVAC-minimum (override via slider/Matter, vervalt na 3 u)
```cpp
effective_vent = max(vent_percent, vent_override_active ? vent_override_percent : 0);
```

**Output**: PWM op IO20 (0-255)
```cpp
pwm_value = map(effective_vent, 0, 100, 0, 255);
ledcWrite(VENT_FAN_PIN, pwm_value);   // ESP32-C6: ledcWrite i.p.v. analogWrite
```

**Ventilatie-unit en PWM→0-10V-omzetting**: zie [Ventilation PWM](#ventilation-pwm) onder Hardware Interfacing.

**Safety**: Pump feedback monitoring
```cpp
// Check of circulatiepomp draait bij hoge vraag
if (total_power > threshold && !pump_feedback) {
  // Warning/alarm
}
```

### 5. Web Interface

**Technology Stack**:
- **Backend**: ESPAsyncWebServer
- **Frontend**: Vanilla HTML5/CSS3/JavaScript
- **Style**: Custom CSS (geen frameworks voor laag geheugengebruik)
- **Updates**: Server-Sent Events (SSE) of polling

**Endpoints**:

**Read Endpoints**:
```
GET  /              → Main status page (HTML)
GET  /settings      → Configuration page (HTML)
GET  /json          → System status (JSON)
GET  /scan          → WiFi networks (JSON)
GET  /update        → OTA update page (HTML)
```

**Write Endpoints**:
```
GET  /save_settings           → Save configuration
GET  /circuit_override_on     → Force circuit ON (10 min)
GET  /circuit_override_off    → Force circuit OFF (10 min)
GET  /circuit_override_cancel → Cancel override
GET  /pump_sch_on            → SCH pump ON (60s)
GET  /pump_sch_off           → SCH pump OFF (60s)
GET  /pump_sch_cancel        → Cancel SCH override
GET  /pump_won_on            → WON pump ON (60s)
GET  /pump_won_off           → WON pump OFF (60s)
GET  /pump_won_cancel        → Cancel WON override
GET  /reboot                 → System reboot
POST /update                 → OTA firmware upload
```

**JSON Status Format**:
```json
{
  "eco_online": 1,
  "KSTopH": 45.2, "KSTopL": 44.8,
  "KSMidH": 42.1, "KSMidL": 41.7,
  "KSBotH": 38.5, "KSBotL": 38.2,
  "KSAv": 41.8, "KSQtot": 5.23,
  "EAv": 65.4, "EQtot": 12.5,
  "ET": 80.0, "EB": 60.0,
  "BB": 45, "WP": 67, "BK": 12, ...  // Duty cycles
  "R1": 1, "R2": 1, ...              // Relay states
  "R9": 0, "R10": 0,                 // Pump states
  "HeatDem": 4.2, "Vent": 75
}
```

**UI Features**:
- **Responsive**: Mobile-first design
- **Real-time**: Timer countdowns (JavaScript)
- **Auto-refresh**: Bij timer expiry (prevents overflow)
- **Server-side rendering**: Stateless HTML generation
- **Graceful degradation**: Works zonder JavaScript (basic functionaliteit)

### 6. Configuration Management (NVS)

**Namespace**: `hvac-config`

**Stored Parameters**:

**WiFi**:
```cpp
NVS_WIFI_SSID       // String
NVS_WIFI_PASS       // String
NVS_STATIC_IP       // String (optional)
```

**System**:
```cpp
NVS_ROOM_ID         // String (hostname)
NVS_CIRCUITS_NUM    // Int (1-16)
NVS_POLL_INTERVAL   // Int (seconds)
```

**ECO Settings**:
```cpp
NVS_ECO_IP          // String
NVS_ECO_MDNS        // String
NVS_ECO_THRESHOLD   // Float (kWh)
NVS_ECO_HYSTERESIS  // Float (kWh)
NVS_ECO_MIN_TEMP    // Float (°C)
NVS_ECO_MAX_TEMP    // Float (°C)
```

**Boiler**:
```cpp
NVS_BOILER_REF_TEMP // Float (°C)
NVS_BOILER_VOLUME   // Float (L)
```

**Sensors** (6×):
```cpp
NVS_SENSOR_NICK_0..5 // String (nicknames)
```

**Circuits** (per circuit, 0-15):
```cpp
c{N}_name           // String
c{N}_ip             // String
c{N}_mdns           // String
c{N}_power          // Float (kW)
c{N}_tstat          // Bool
c{N}_pin            // Int (10/11/12/255)
```

**Pump Telemetry**:
```cpp
NVS_LAST_SCH_PUMP   // ULong (timestamp)
NVS_LAST_SCH_KWH    // Float
NVS_LAST_WON_PUMP   // ULong (timestamp)
NVS_LAST_WON_KWH    // Float
NVS_TOTAL_SCH_KWH   // Float (cumulative)
NVS_TOTAL_WON_KWH   // Float (cumulative)
```

**Factory Reset**: Type 'R' binnen 3s na boot

### 7. Network Architecture

**WiFi Modes**:

**Station Mode (Normal Operation)**:
```cpp
WiFi.mode(WIFI_STA);
WiFi.begin(ssid, password);

// Retry strategy:
// - 20s timeout per attempt (iPhone hotspot compatibility)
// - 2s delay between attempts
// - Max 5 attempts (~100s total)
// - Exponential backoff on failures
```

**Access Point Mode (Fallback)**:
```cpp
WiFi.mode(WIFI_AP);
WiFi.softAP("HVAC-Setup");  // No password

// IP: 192.168.4.1
// DHCP: Enabled
// DNS: Captive portal (all requests → /settings)
```

**Captive Portal**:
```cpp
DNSServer dnsServer;
dnsServer.start(53, "*", WiFi.softAPIP());

// Redirects ALL domains to 192.168.4.1
// Auto-opens settings on iOS/macOS
```

**mDNS**:
```cpp
MDNS.begin(room_id);  // hostname.local
// Allows discovery without IP knowledge
```

**Time Sync**:
```cpp
configTime(3600, 3600, "pool.ntp.org", "time.nist.gov");
setenv("TZ", "CET-1CEST,M3.5.0/02,M10.5.0/03", 1);

// Timezone: Europe/Brussels (CET/CEST)
// Used for: Pump event timestamps
```

## Hardware Interfacing

### MCP23017 GPIO Expander

**I2C Configuration**:
```cpp
Wire.begin(SDA_PIN, SCL_PIN);
mcp.begin_I2C(0x20);  // Address: 0x20
```

**Pin Mapping**:
```
Pin 0-6:  Relay outputs (circuits)    → OUTPUT, HIGH (off)
Pin 7:    Pump feedback sensor        → INPUT_PULLUP
Pin 8:    SCH pump relay              → OUTPUT, HIGH (off)
Pin 9:    WON pump relay              → OUTPUT, HIGH (off)
Pin 10-12: Thermostat inputs         → INPUT_PULLUP (active LOW)
Pin 13-15: Reserved/unused            → INPUT_PULLUP
```

**Relay Control**:
```cpp
// Active LOW logic (common for relay modules)
mcp.digitalWrite(pin, LOW);   // Relay ON
mcp.digitalWrite(pin, HIGH);  // Relay OFF
```

### DS18B20 Temperature Sensors

**1-Wire Protocol**:
```cpp
#include <OneWire.h>
#include <DallasTemperature.h>

OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);

// Addressing: By index (0-5)
// Resolution: 12-bit (0.0625°C)
// Conversion time: 750ms
```

**Error Handling**:
```cpp
if (temp == DEVICE_DISCONNECTED_C || temp < -100) {
  sensor_ok[i] = false;
  // Use last valid value or default
}
```

### Ventilation PWM

**Output (huidige sketch v1.19)**:
```cpp
#define VENT_FAN_PIN  20
ledcAttach(VENT_FAN_PIN, 1000, 8);    // 1 kHz, 8-bit (0-255)
ledcWrite(VENT_FAN_PIN, pwm_value);
```
De ESP32-C6 heeft **geen DAC**: IO20 levert enkel 3,3V-PWM. De omzetting naar 0-10V gebeurt extern (AnaBoX, zie hieronder).

> ⚠️ **Aandachtspunt PWM-frequentie:** met het RC-filter van de AnaBoX v2.0 (τ = 10 ms) geeft 1 kHz ~0,25 V rimpel bij 50%. Aanbevolen: **≥ 5-10 kHz, liefst ~20 kHz** (rimpel dan ~12 mV). Nog niet aangepast in de sketch — openstaand punt.

#### Ventilatie-unit: Begetube 350 R (profi-air)

Aansluiting op de **groene 8-polige connector** van de sturing (bron: gebruikershandleiding + mail technische dienst). Alle contacten potentiaalvrij.

| Klemmen | Functie | Opmerking |
|---|---|---|
| 1-4 | Calamiteitscontact (brandsensor) | Standaard brug — **nooit verwijderen**, anders werkt de unit niet |
| 1-2 | Stand 2 (P–T1) | Driestandenschakelaar — niet combineren met 0-10V-sturing |
| 1-3 | Stand 3 (P–T2) | idem |
| **5-6** | **PIR/timer-contact** | Gesloten = unit volgt 0-10V; open = unit schakelt uit. Standaard brug |
| **7-8** | **0-10V-ingang** (CO2 / vocht / domotica) | Tekening: rood = 7 (+), blauw = 8 (−) — polariteit nameten vóór aansluiten |

**Huidige toestand:** de unit wordt via klem 7-8 aangestuurd door een **Particle Photon met de oude AnaBoX** (op-amp-buffer). Doel: overschakelen naar de HVAC-controller (IO20) met de **AnaBoX v2.0**.

**Mogelijke uitbreiding klem 5-6:** brug vervangen door een relaiscontact (bv. vrije MCP23017-pin 13-15) voor een harde aan/uit, los van de 0-10V. Kies bewust de rusttoestand: bij voorkeur zo dat de unit **blijft ventileren** bij een crash/stroomuitval (NC-contact of brug laten).

**Nog onbekend (technische dienst kon niet antwoorden):** ingangsweerstand van klem 7-8, gedrag bij 0 V (stop of minimumtoerental), eventuele interne pull-up.

#### AnaBoX v2.0 (iTroniX) — PWM → 0-10V

Eigen ontwerp, PCB gemaakt in China. Twee 2N7002-trappen (Q1+Q2, samen niet-inverterend) zetten 3,3V-PWM om naar 0-VCC-PWM (PWM-OUT); RC-filter R4 (10k) + C1 (1 µF) maakt daar een gelijkspanning van (ANA-OUT).

- **Aansluiting:** PWM-IN ← IO20 + GND · ANA-OUT pin 1 → klem 7, GND → klem 8
- **Fail-safe:** geen PWM-signaal → Q2 geleidt → ANA-OUT = 0 V
- **Aandachtspunten:**
  1. **VCC = exact 10 V** en stabiel (uitgangsamplitude = VCC; rimpel op VCC komt in het stuursignaal)
  2. **Uitgangsimpedantie** is ongelijk: ~20 kΩ naar boven (R2+R4), ~10 kΩ naar beneden (Q2+R4). Bij een laagohmige unit-ingang zakt de spanning en wordt het verloop niet-lineair. Eerste remedie: **R3 (1 kΩ) plaatsen** (staat op de print maar wordt in productie niet bestukt)
  3. **Pulldown (~100 kΩ) op de gate van Q1** — voorkomt kort aanlopen van de ventilator bij boot/reset (zwevende pin)
  4. C1 min. 25 V, X7R · gemeenschappelijke GND tussen ESP32 en unit
  5. Verschil met de oude AnaBoX: die had een **op-amp-buffer** (lage uitgangsimpedantie), v2.0 niet

#### Meetprocedure ingangsweerstand klem 7-8 (met unit in bedrijf)

1. **Open spanning:** draad van klem 7 los, unit aan, spanning over 7-8 meten. Spanning aanwezig → interne pull-up.
2. **Serieweerstand-methode** (met oude AnaBoX op 100%):
   - Spanning op 7-8 meten → **V1**
   - 10 kΩ in serie tussen AnaBoX en klem 7, opnieuw meten → **V2**
   - **R<sub>in</sub> = 10k × V2 / (V1 − V2)**
3. **Vergelijking oude vs. nieuwe AnaBoX** bij 25 / 50 / 100%, met de unit aangesloten.

**Interpretatie:**

| R<sub>in</sub> | Gevolg voor AnaBoX v2.0 |
|---|---|
| ≥ 100 kΩ | Werkt zoals hij is (enkele % afwijking); R3 plaatsen maakt het beter |
| 10-50 kΩ | R3 (1k) zeker plaatsen; R4 → 1k en C1 → 10 µF |
| < 10 kΩ | Buffer (op-amp) nodig, zoals de oude AnaBoX |

**Meetresultaten** *(in te vullen)*:

| Meting | Waarde |
|---|---|
| Open spanning 7-8 | |
| V1 (oude AnaBoX, 100%) | |
| V2 (met 10 kΩ in serie) | |
| R<sub>in</sub> berekend | |
| Oude AnaBoX 25 / 50 / 100% | |
| AnaBoX v2.0 25 / 50 / 100% | |

## Performance Characteristics

### Timing Budget
```
Main Loop Iteration:    ~100ms
Room Polling Cycle:     10s (configurable)
ECO Polling Cycle:      10s (configurable)
DS18B20 Read Cycle:     5s
Pump State Machine:     100ms
Web Request Handling:   <50ms (async)
```

### Memory Usage
```
Flash (Program):        ~1.2 MB (of 4MB)
SRAM (Runtime):         ~180 KB (of 512KB)
NVS (Storage):          ~4 KB (configurable)
Web Page (HTML):        ~40 KB (gzip: ~12 KB)
```

### Network Load
```
HTTP Requests/min:      42 (7 circuits × 6/min)
JSON Size:              ~400 bytes per circuit
DNS Queries:            1 per circuit per poll (if mDNS)
mDNS Packets:           Periodic announcements
NTP Queries:            1 per boot + periodic resync
```

## Safety & Reliability

### Watchdog
```cpp
// ESP32 hardware watchdog (default enabled)
// Triggers reset if main loop hangs
```

### Brownout Detection
```cpp
// ESP32 brownout detector
// Threshold: ~2.8V (configurable via menuconfig)
```

### Data Integrity
```cpp
// NVS uses CRC32 for data validation
// Automatic bad block handling
// Wear leveling across flash sectors
```

### Thermal Protection
- DS18B20: -55°C to +125°C operating range
- ESP32-C6: -40°C to +85°C operating range
- Relay modules: Usually 0°C to +70°C

### Failsafe Modes

**No WiFi**:
```
→ Fall back to thermostat inputs
→ Continue pump operations (if eco_boiler was online)
→ Disable room polling
```

**No MCP23017**:
```
→ Disable all relay control
→ Continue monitoring
→ Show warning in UI
```

**No Sensors**:
```
→ Use last valid readings
→ Mark sensor as error
→ Continue other operations
```

## Security Considerations

**Current Implementation**:
- ⚠️ No authentication on web interface
- ⚠️ No HTTPS (plain HTTP)
- ⚠️ No password protection on settings
- ⚠️ AP mode without password

**Mitigations**:
- Private WiFi network (no internet exposure)
- Physical access control (local network only)
- OTA requires physical access (upload via web)

**Future Enhancements**:
- Basic auth for web interface
- Encrypted NVS storage
- HTTPS with self-signed cert
- AP mode with WPA2 password

## Debugging & Diagnostics

### Serial Monitor Output

**Boot Sequence**:
```
=== HVAC Controller V53.4 ===
WIJZIGINGEN t.o.v. V53.3:
1. Server-side timer check: Voorkomt badge bij verlopen timer
2. Auto-refresh bij timer=0: Pagina ververst automatisch
3. Timer overflow DEFINITIEF opgelost!

[Factory reset check: 3s]
MCP23017 OK!
[Circuit config output]
Connecting to 'SSID'...
✓ WiFi connected!
WiFi OK - IP: 192.168.1.100, RSSI: -45 dBm
Syncing NTP time... OK!
Time: 11-01-2026 14:23:45
mDNS: http://hvac.local
Web server started!
Ready!
```

**Runtime Logging**:
```
=== POLLING ROOMS ===
WiFi OK - IP: 192.168.1.100, RSSI: -45 dBm
c0: Polling http://192.168.1.101/status.json
    Result: 200 (245 bytes) ✓
    y=1 z=75 aa=22 h=21.5 af=1
c0: HOME → TSTAT(1) OR HTTP(1) = ON
c0: Relay ON -> ON
Total power: 4.2 kW, Vent: 75%

=== MANUAL PUMP CONTROL ===
Type: SCH
Action: ON (override)
Duration: 60 seconds
Relay 8: LOW (pump ON)

=== MANUAL PUMP TIMEOUT ===
SCH pump manual mode expired (60s)
```

### Web Interface Debugging

**Browser Console**:
```javascript
// Timer countdown logging
console.log('Timer reached 0 - auto-refreshing page');

// Error detection
console.error('Failed to fetch: ', error);
```

**Network Tab**:
- Monitor HTTP requests
- Check response times
- Validate JSON structure

## Known Limitations

1. **Single Core**: ESP32-C6 is single-core RISC-V (vs dual-core Xtensa in ESP32)
   - Impact: Slight performance vs ESP32, but adequate for this application

2. **WiFi Stability**: Occasional disconnects mogelijk
   - Mitigation: Automatic reconnect logic

3. **mDNS Reliability**: Not 100% reliable on all networks
   - Mitigation: IP address fallback

4. **No RTOS Priority**: Cooperative multitasking
   - Impact: Long blocking operations can delay other tasks

5. **Flash Wear**: NVS write cycles limited (~100,000)
   - Mitigation: Only write on config changes (not runtime data)

6. **Pump Telemetry**: kWh calculation placeholder (TODO)
   - Impact: Totals are approximations until refined

## Development Tools

### Arduino IDE Setup
```
Board: ESP32C6 Dev Module
Flash Size: 4MB
Partition Scheme: Default 4MB with spiffs
Upload Speed: 921600
CPU Frequency: 160MHz
```

### Required Libraries
```cpp
// Core
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>
#include <DNSServer.h>

// I/O
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Data
#include <ArduinoJson.h>
#include <Preferences.h>

// Network
#include <HTTPClient.h>
#include <Update.h>
```

### OTA Update Process
```
1. Navigate to http://hvac.local/update
2. Select firmware.bin file
3. Click "Upload Firmware"
4. Wait for upload progress (1-2 minutes)
5. Automatic reboot
6. Verify version in Serial monitor
```

---

# Versiegeschiedenis

## V53.4 (11 januari 2026) - **CURRENT STABLE**
**Timer Overflow Definitief Opgelost**

### Fixed
- **CRITICAL**: Server-side timer validation prevents overflow bij refresh na timeout
- **CRITICAL**: Auto-refresh bij timer=0 voorkomt badge blijven staan
- Dubbele bescherming: Backend reset flag + Frontend refresh

### Changed
- JavaScript timer countdown met auto-reload functionaliteit
- Server genereert geen badge meer als `elapsed >= MANUAL_PUMP_DURATION`

### Verified
- ✅ Refresh tijdens timer: blijft correct (TEST 2)
- ✅ Refresh na timeout: geen "ON 71582:21" overflow meer (TEST 3, TEST 8)
- ✅ Auto-refresh bij 0:00 werkt (TEST 1, TEST 4)
- ✅ Circuit buttons onafhankelijk (TEST 10)

---

## V53.3 (11 januari 2026)
**Timer Reset & OFF Override**

### Added
- OFF override mode (60s force OFF met timer badge)
- Timer state tracking: `sch_pump_manual_on` / `won_pump_manual_on` booleans
- OFF badge styling (grijze achtergrond vs rode voor ON)

### Fixed
- Manual flag reset na timeout in `handleEcoPumps()`
- Consistent gedrag met circuit override buttons

### Issues
- ⚠️ Timer overflow nog steeds mogelijk bij refresh na timeout (opgelost in V53.4)

---

## V53.2 (11 januari 2026)
**Circuit-Style Buttons & Captive Portal**

### Added
- **Captive Portal**: Auto-open settings page in AP mode (iOS/macOS)
- DNS wildcard redirect naar 192.168.4.1/settings
- 60 second manual override timer (zoals circuit buttons)
- Cancel button (×) tijdens override
- Timer countdown badge in UI

### Changed
- Pump buttons: Kleine ON/OFF buttons (consistent met circuits)
- WiFi timeout: 10s → 20s per attempt (iPhone hotspot compatibility)
- Serial output: Uitgebreidere debug info bij pomp acties

### Fixed
- Labels: "Tmax pomp (Start)" / "Tmin pomp (Stop)" (duidelijker)

### Issues
- ⚠️ Timer blijft op 0:00 staan, overflow bij refresh (opgelost in V53.3)

---

## V53.1 (11 januari 2026)
**ECO Logica & UI Fixes**

### Added
- **ECO Pump OR Logic**: Start bij `(temp_top > Tmax) OR (qtot > threshold)`
- **ECO Pump OR Logic**: Stop bij `(temp_top < Tmin) OR (qtot < threshold-hyst)`
- 30 minuten pomp cycles (was 30 seconden!)
- 30 minuten wait tussen cycles
- Alternating pump pattern: WON → SCH → WON → SCH
- Cumulative totals: `total_sch_kwh` en `total_won_kwh` in NVS
- Last pump event: timestamp + kWh per pomp

### Changed
- ECO UI: Toon ETop (temp_top) ipv EAv (temp_avg)
- ECO UI: Altijd 4 velden tonen (ook als offline → "NA")
- ECO badge: Alleen "ONLINE"/"OFFLINE" (geen dubbele text)
- Settings: eco_max_temp en eco_min_temp velden toegevoegd
- Pump buttons: ON/OFF knoppen (geen toggle meer)

### Fixed
- JSON format: ET/EB tonen settings (niet boiler temps)
- WiFi timeout verhoogd voor iPhone Personal Hotspot
- Button spacing op brede/smalle schermen

---

## V53.0 (10 januari 2026)
**Migratie van Particle Photon naar ESP32-C6**

### Added
- ESP32-C6 platform support
- MCP23017 I2C GPIO expander integration
- 7 verwarmingscircuits met relay control
- 6 DS18B20 sensoren voor SCH boiler
- Boiler Qtot berekening (energieinhoud)
- Room controller polling (HTTP/JSON)
- mDNS fallback voor IP discovery
- Thermostat input (MCP23017 GPIO 10-12)
- ECO boiler polling (placeholder voor sketch #3)
- Basic pump controls (manual toggle)
- NVS configuration management
- Web interface (status + settings)
- OTA firmware updates
- Ventilation PWM control (0-100%)
- Duty cycle tracking per circuit
- WiFi retry mechanism
- Factory reset functie

### Technical Debt
- Pump telemetry: kWh calculation not implemented (placeholder: 0.5 kWh)
- ECO controller: Polling endpoint exists maar controller nog niet gebouwd
- Security: No authentication/encryption

---

# Hardware Vereisten

## Microcontroller
- **ESP32-C6 Development Board**
  - Flash: 4MB minimum
  - RAM: 512KB
  - WiFi: 2.4GHz 802.11b/g/n
  - I2C: 1× interface
  - 1-Wire: 1× GPIO pin
  - PWM: 1× output voor ventilator
  - USB: Type-C voor programmeren

## GPIO Expander
- **MCP23017-E/SP** (I2C, 16-bit, DIP-28 of SOIC-28)
  - I2C Address: 0x20 (A0/A1/A2 tied to GND)
  - Pull-ups: 4.7kΩ op SDA/SCL
  - Decoupling: 100nF keramisch + 10µF elektrolytisch

## Relay Modules
- **9× Relay modules** (5V coil, 10A contacts)
  - 7× voor verwarmingscircuits
  - 2× voor energie transfer pompen
  - Optische isolatie aangeraden
  - Flyback diodes (meestal ingebouwd)

## Temperature Sensors
- **6× DS18B20 digitale temperatuursensoren**
  - Package: TO-92 of waterproof probe
  - Accuracy: ±0.5°C (-10 tot +85°C)
  - Pull-up: 4.7kΩ op data lijn
  - Power: Parasitic of dedicated VDD

## Power Supply
- **5V/3A DC adapter** (voor ESP32 + relays)
- **Optioneel**: Separate 5V/2A voor relay modules (als >7 relays)

## Enclosure
- DIN-rail mounting voor industriële omgeving
- IP rating: IP40 minimum (stofbescherming)
- Cooling: Passive (ventilatie sleuven)

## Wiring
- **I2C**: Twisted pair, max 1 meter (zonder extender)
- **1-Wire**: Twisted pair, max 30 meter (met geschikte pull-up)
- **Relays**: 0.5-1.5mm² draad, geschikt voor 10A
- **Power**: 1.5mm² voor hoofd power, 0.5mm² voor logic

---

# Software Architectuur

## Class Diagram (Simplified)

```
┌─────────────────────┐
│   ESP32_HVAC.ino    │
│  (Main Application) │
└──────────┬──────────┘
           │
     ┌─────┴─────┬──────────────┬─────────────┬──────────────┐
     │           │              │             │              │
┌────▼────┐ ┌────▼────┐   ┌────▼────┐  ┌─────▼─────┐ ┌──────▼──────┐
│ Polling │ │  Pump   │   │ Boiler  │  │    Web    │ │   Config    │
│ Engine  │ │Controller│   │ Monitor │  │  Server   │ │   Manager   │
└─────────┘ └─────────┘   └─────────┘  └───────────┘ └─────────────┘
     │           │              │             │              │
     ▼           ▼              ▼             ▼              ▼
┌─────────┐ ┌─────────┐   ┌─────────┐  ┌─────────┐   ┌──────────┐
│  HTTP   │ │  MCP    │   │ 1-Wire  │  │  HTML   │   │   NVS    │
│ Client  │ │ GPIO    │   │ Dallas  │   │  Gen    │   │ Storage  │
└─────────┘ └─────────┘   └─────────┘  └─────────┘   └──────────┘
```

## Data Structures

### Circuit
```cpp
struct Circuit {
  String name;              // User-defined naam
  String ip;                // Static IP (optioneel)
  String mdns;              // mDNS naam zonder .local
  float power_kw;           // Nominaal vermogen
  bool has_tstat;           // Thermostaat aanwezig?
  int tstat_pin;            // MCP23017 pin (10-12)
  
  // Runtime state
  bool online;              // HTTP bereikbaar?
  bool heating_on;          // Verwarming actief?
  int heat_request;         // Vraag van room controller
  int vent_request;         // Ventilatie vraag (0-100%)
  int setpoint;             // Gewenste temp (°C)
  float room_temp;          // Actuele temp (°C)
  int home_status;          // 0=away, 1=home
  
  // Override
  bool override_active;     // Manual override?
  bool override_state;      // Forced ON/OFF
  unsigned long override_start;  // Start tijd
  
  // Telemetry
  unsigned long on_time;    // Milliseconds ON
  unsigned long off_time;   // Milliseconds OFF
  unsigned long last_change;
  float duty_cycle;         // Percentage (0-100)
};
```

### Eco Boiler
```cpp
struct EcoBoiler {
  bool online;              // Polling succesvol?
  float temp_top;           // Hoogste temp (°C)
  float temp_avg;           // Gemiddelde temp (°C)
  float qtot;               // Energieinhoud (kWh)
  unsigned long last_seen;  // Laatste succesvolle poll
};
```

### Pump Event
```cpp
struct PumpEvent {
  time_t timestamp;         // Unix timestamp
  float kwh_pumped;         // kWh overgedragen
};
```

## State Machines

### Pump Controller
```
     ┌────────┐
     │  IDLE  │◄────────────────────┐
     └───┬────┘                     │
         │ start_conditions          │
         │                          │
    ┌────▼────────┐            ┌────┴───────┐
    │  PUMP_SCH   │            │  PUMP_WON  │
    │  (30 min)   │            │  (30 min)  │
    └────┬────────┘            └────┬───────┘
         │ timeout                  │ timeout
         │                          │
         └────────►┌──────┐◄────────┘
                   │ WAIT │
                   │(30min)│
                   └───┬──┘
                       │ timeout + start_conditions
                       │ (alternates pump)
                       └──────────────────┐
                                          │
                   ┌──────────────────────▼─┐
                   │   Back to opposite     │
                   │   pump (SCH ↔ WON)    │
                   └────────────────────────┘
```

### WiFi Connection
```
     ┌────────┐
     │  BOOT  │
     └───┬────┘
         │
         ▼
   ┌─────────────┐
   │ Credentials │      NO
   │  Available? ├────────►┌──────────┐
   └──────┬──────┘         │ AP Mode  │
          │ YES             │(Fallback)│
          ▼                 └──────────┘
   ┌──────────────┐
   │ Try Connect  │
   │ (20s × 5)    │
   └──────┬───────┘
          │
    ┌─────┴──────┐
    │ Success?   │
    ├─YES────────┼─NO──►┌──────────┐
    │            │      │ AP Mode  │
    ▼            │      └──────────┘
┌────────────┐   │
│ STA Mode   │   │
│ (Normal)   │   │
└────────────┘   │
                 ▼
          ┌────────────┐
          │ Reconnect  │
          │  on Drop   │
          └────────────┘
```

---

# API Documentatie

## JSON Status Endpoint

**Request**:
```http
GET /json HTTP/1.1
Host: hvac.local
```

**Response**:
```json
{
  "eco_online": 1,
  
  "KSTopH": 45.2,
  "KSTopL": 44.8,
  "KSMidH": 42.1,
  "KSMidL": 41.7,
  "KSBotH": 38.5,
  "KSBotL": 38.2,
  "KSAv": 41.8,
  "KSQtot": 5.23,
  
  "KWTopH": 0.0,
  "KWTopL": 0.0,
  "KWMidH": 0.0,
  "KWMidL": 0.0,
  "KWBotH": 0.0,
  "KWBotL": 0.0,
  "KWAv": 0.0,
  "KWQtot": 0.0,
  
  "EAv": 65.4,
  "EQtot": 12.5,
  "ET": 80.0,
  "EB": 60.0,
  
  "BB": 45,
  "WP": 67,
  "BK": 12,
  "ZP": 89,
  "EP": 34,
  "KK": 56,
  "IK": 23,
  
  "R1": 1,
  "R2": 1,
  "R3": 0,
  "R4": 1,
  "R5": 0,
  "R6": 1,
  "R7": 0,
  "R9": 0,
  "R10": 0,
  
  "HeatDem": 4.2,
  "Vent": 75
}
```

**Field Reference**:
| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `eco_online` | int | bool | ECO boiler bereikbaar? (0/1) |
| `KS*` | float | °C | SCH boiler temps (K=Ketel, S=Schuur) |
| `KSAv` | float | °C | SCH gemiddelde temp |
| `KSQtot` | float | kWh | SCH energieinhoud |
| `KW*` | float | °C | WON boiler temps (reserved, not used) |
| `EAv` | float | °C | ECO gemiddelde temp |
| `EQtot` | float | kWh | ECO energieinhoud |
| `ET` | float | °C | ECO Tmax (start conditie) |
| `EB` | float | °C | ECO Tmin (stop conditie) |
| `BB` | int | % | Badkamer duty cycle |
| `WP` | int | % | Woonkamer duty cycle |
| `BK` | int | % | Berging duty cycle |
| `ZP` | int | % | Zolderkamer duty cycle |
| `EP` | int | % | Eerste verdieping duty cycle |
| `KK` | int | % | Keuken duty cycle |
| `IK` | int | % | Inkomhal duty cycle |
| `R1-R7` | int | bool | Relay states circuits (0/1) |
| `R9` | int | bool | SCH pump relay (0/1) |
| `R10` | int | bool | WON pump relay (0/1) |
| `HeatDem` | float | kW | Totaal vermogen vraag |
| `Vent` | int | % | Max ventilatie request |

---

# Installatie & Setup

## Hardware Setup

1. **Power wiring**:
   - Connect 5V power supply to ESP32-C6
   - Connect relay modules to same 5V supply (of separate indien >7 relays)
   
2. **I2C wiring** (MCP23017):
   ```
   ESP32 SDA (GPIO X) → MCP23017 SDA (pin 13)
   ESP32 SCL (GPIO Y) → MCP23017 SCL (pin 12)
   GND               → GND (pin 9)
   3.3V              → VDD (pin 18)
   4.7kΩ pull-up     → SDA + SCL naar 3.3V
   ```

3. **1-Wire wiring** (DS18B20):
   ```
   ESP32 GPIO Z → DS18B20 DQ (middle pin)
   4.7kΩ pull-up → DQ naar 3.3V
   3.3V         → VDD
   GND          → GND
   ```

4. **Relay connections** (MCP23017):
   ```
   Pin 0-6 → Circuit relays (verwarmingszones)
   Pin 8   → SCH pump relay
   Pin 9   → WON pump relay
   ```

5. **Input connections** (MCP23017):
   ```
   Pin 7   → Pump feedback sensor (optional)
   Pin 10-12 → Thermostaat inputs (active LOW)
   ```

6. **PWM output**:
   ```
   ESP32 GPIO W → Ventilator PWM input (0-5V)
   ```

## Software Installation

### Prerequisites
```
- Arduino IDE 2.x
- ESP32 board support: https://espressif.github.io/arduino-esp32/package_esp32_index.json
- Libraries: ESPAsyncWebServer, AsyncTCP, Adafruit_MCP23017, OneWire, DallasTemperature, ArduinoJson
```

### Board Configuration
```
Tools → Board → esp32 → ESP32C6 Dev Module
Tools → Flash Size → 4MB (32Mb)
Tools → Partition Scheme → Default 4MB with spiffs
Tools → Upload Speed → 921600
Tools → CPU Frequency → 160MHz
```

### Upload Firmware
```
1. Connect ESP32-C6 via USB
2. Select correct COM port
3. Click Upload (Ctrl+U)
4. Wait for "Hard resetting via RTS pin..."
5. Open Serial Monitor (115200 baud)
6. Verify boot messages
```

### Initial Configuration

**Option A: Via AP mode** (als geen WiFi geconfigureerd):
```
1. Power on ESP32
2. Connect to WiFi network "HVAC-Setup"
3. Settings pagina opent automatisch (captive portal)
4. Voer WiFi credentials in
5. Configureer circuits en boiler settings
6. Klik "Opslaan & Reboot"
```

**Option B: Via Serial** (factory reset en herconfigureren):
```
1. Open Serial Monitor (115200 baud)
2. Reset ESP32
3. Type 'R' binnen 3 seconden
4. NVS wordt gewist
5. Volg "Option A" procedure
```

### Verification Checklist
```
□ MCP23017 detected: "MCP23017 OK!"
□ WiFi connected: IP address shown
□ NTP sync: Timestamp displayed
□ mDNS working: http://hvac.local reachable
□ Sensors OK: Temperatures shown in UI
□ Relays working: Test manual pump controls
□ Polling working: Room controllers reachable
□ Ventilation: PWM output adjusts with demand
```

---

# Troubleshooting

## Common Issues

### MCP23017 Not Detected
```
Symptom: "MCP23017 not found!" in Serial
Causes:
  - Loose I2C wiring
  - Wrong I2C address (check A0/A1/A2 pins)
  - Missing pull-up resistors (4.7kΩ)
  - Defective MCP23017
Fix:
  - Check continuity SDA/SCL
  - Verify 3.3V on MCP VDD
  - Use I2C scanner sketch to detect address
```

### WiFi Connection Fails
```
Symptom: "✗ All WiFi attempts failed -> AP mode"
Causes:
  - Wrong SSID/password
  - Out of range
  - 5GHz network (ESP32 only supports 2.4GHz)
  - Router MAC filtering
Fix:
  - Verify credentials in /settings
  - Move closer to router during setup
  - Check router logs for connection attempts
  - Add ESP32 MAC to whitelist
```

### Sensors Show -127°C or Error
```
Symptom: Temperatures show -127°C or "Error"
Causes:
  - Loose 1-Wire connection
  - Missing pull-up resistor (4.7kΩ)
  - Sensor addressing issue
  - Defective DS18B20
Fix:
  - Check continuity of DQ line
  - Verify 4.7kΩ pull-up to 3.3V
  - Try "discovery" mode (by index 0-5)
  - Replace suspect sensor
```

### Room Controllers Not Polling
```
Symptom: Circuits show "✗" or "NA" in UI
Causes:
  - Wrong IP address
  - Room controller offline
  - Firewall blocking HTTP
  - mDNS not working on network
Fix:
  - Ping IP from another device
  - Check room controller Serial output
  - Test /status.json endpoint in browser
  - Use IP instead of mDNS
```

### Pumps Don't Activate
```
Symptom: Manual pump buttons don't work
Causes:
  - MCP23017 not detected
  - Relay wiring reversed (NO vs NC)
  - Relay module 5V supply issue
  - Software override state stuck
Fix:
  - Check MCP23017 detection
  - Swap NO/NC terminals on relay
  - Verify 5V to relay module (VCC/JD-VCC)
  - Reboot ESP32 (clears override state)
```

### Web Interface Slow or Hangs
```
Symptom: Page takes >5s to load or times out
Causes:
  - WiFi signal weak
  - Many concurrent requests
  - Browser cache issues
  - Memory fragmentation
Fix:
  - Move ESP32 closer to router
  - Close other browser tabs
  - Hard refresh (Ctrl+Shift+R)
  - Reboot ESP32 if uptime >7 days
```

### Timer Shows Overflow (V53.3 or earlier)
```
Symptom: "ON 71582:21" after page refresh
Cause: Bug in V53.3 and earlier
Fix: Update to V53.4 or later (CRITICAL FIX)
```

---

# Contributing

## Code Style
- Indent: 2 spaces (no tabs)
- Line length: 120 characters max
- Comments: Dutch or English, be descriptive
- Naming: snake_case voor variabelen, camelCase voor functies

## Git Workflow
```
main          → Stable releases only
develop       → Active development
feature/*     → New features
bugfix/*      → Bug fixes
hotfix/*      → Critical production fixes
```

## Pull Request Process
1. Create feature branch from `develop`
2. Implement changes with descriptive commits
3. Test thoroughly (all 10 test cases in checklist)
4. Update version number in code + README
5. Submit PR with detailed description
6. Wait for review and CI checks

## Testing Checklist
- [ ] Compiles zonder errors/warnings
- [ ] Boot sequence succesvol (Serial check)
- [ ] MCP23017 detection OK
- [ ] WiFi connection stable
- [ ] All 7 circuits reachable
- [ ] ECO polling werkt (als ECO controller actief)
- [ ] Pump manual controls werken (ON/OFF/Cancel)
- [ ] Timer countdown correct (geen overflow!)
- [ ] Web interface laadt binnen 2s
- [ ] Settings save/load correct
- [ ] OTA update succesvol

---

# License

**Proprietary / All Rights Reserved**

Dit project is eigendom van Fidel Dworp en mag niet worden gedistribueerd, gekopieerd of aangepast zonder expliciete toestemming.

Voor licentievragen: contact via GitHub repository.

---

# Contact & Support

**Repository**: https://github.com/FidelDworp/ESP32C6_HVAC  
**Issues**: https://github.com/FidelDworp/ESP32C6_HVAC/issues  
**Discussions**: https://github.com/FidelDworp/ESP32C6_HVAC/discussions

**Related Projects**:
- ESP32C6_ROOMS: https://github.com/FidelDworp/ESP32C6_ROOMS (room controllers)
- ESP32C6_ECO: TBD (ECO boiler controller - in development)

---

# Acknowledgments

- **Espressif Systems**: Voor ESP32-C6 platform
- **Adafruit**: Voor MCP23017 library
- **Dallas Semiconductor**: Voor DS18B20 sensor
- **Arduino Community**: Voor talloze voorbeelden en support
- **Me-No-Dev**: Voor ESPAsyncWebServer library

---

**Last Updated**: 11 januari 2026  
**Version**: V53.4  
**Status**: ✅ Production Ready



------------------- OLD VERSION README.md --------------------

# ESP32-HVAC = Centrale HVAC Controller (Vervanging van Particle sketch: HVAC_Photon.cpp)

**FiDel, 8 januari 2026**

Hallo allemaal,

Dit is de open-source sketch voor mijn centrale HVAC-controller in de kelder, die de oude Particle Photon vervangt.  
Het systeem is volledig herwerkt naar een gedistribueerd model: de intelligentie zit nu in de individuele kamercontrollers (universele ESP32-roomsketch, gebaseerd op mijn Testroom-project), en deze centrale ESP32 fungeert alleen als "domme" executor en monitor.

### Inleiding

Na jaren trouwe dienst met een Particle Photon als centrale HVAC-unit, migreer ik naar ESP32.  
De redenen:
- Lokale verwerking (geen cloud-afhankelijkheid meer voor basisfunctionaliteit)
- Betere integratie met mijn bestaande ESP32-roomcontrollers
- Matter-ondersteuning in de toekomst voor Apple Home
- Open-source en volledig zelf te onderhouden

Deze sketch draait op een ESP32-C6 en bestuurt:
- 1-16 verwarmingskleppen via MCP23017 I2C expander (Instelbaar)
- 1 Ventilatie fan (0-10V signaal via PWM + level shifter)
- 2 ECO-overpomp pompen (naar kelderboiler)
- Monitort 12 DS18B20 sensoren (6 in kelderboiler SCH, 6 in ECO-boiler)

De controller pollt periodiek de kamercontrollers (/json endpoint) voor:
- heating_on (0/1) → stuurt bijbehorende klep
- vent_percent (0-100) → max waarde bepaalt fan speed

### Hoe het werkt

1. **Kamercontrollers** (max 10, configureerbaar):
   - Elke kamer bepaalt zelf verwarming en ventilatiebehoefte.
   - Hosten JSON met `y` (heating) en `z` (vent %).

2. **Verwarmingscircuits**:
   - Configureerbaar 1-16 kleppen (zoals pixels in Testroom).
   - Elke klep heeft hernoembare naam (default "Circuit 1" etc.).
   - Relais LOW = AAN.
  
   Vermogen van de circuits in Zarlardinge Schuur:
   BandB = 1254 W
   WASPL = 1096 W
   INKOM = 1025 W
   KEUK = 1018 W
   EETPL = 916 W
   ZITPL = 460 W
   BADK = 832 W

4. **Duty-cycle**:
   - Per circuit Ton/Toff bijgehouden → % berekend en getoond.

5. **Boiler monitoring**:
   - 12 hardcoded DS18B20 addresses.
   - Qtot berekening voor kelder boiler (placeholder-formule, later kalibreren).

6. **ECO-overpompen**:
   - Automatisch als ECO Qtot > threshold (default 12 kWh) en er is verwarmingsvraag.
   - Hysteresis en max 5 min pomptijd.
   - Prioriteit SCH.

7. **Ventilatie**:
   - Max vent_percent van alle kamers → PWM op GPIO20.

8. **Webinterface**:
   - Hoofdpagina: status boilers, ventilatie, circuits (state + duty-cycle)
   - /settings: alle configuratie (namen, IP's rooms, thresholds, etc.)
     → persistent in NVS
   - /logdata: JSON voor Google Sheets push of pull (elke 10 min)

9. **Toekomst**:
   - Matter integratie (switches voor kleppen/pompen, fan voor ventilatie)

### Hardware

- ESP32-C6 devboard
- MCP23017 (adres 0x20) voor relays + Thermostats
  => 3 circuits met hardwired TSTAT:
     Pin 10 = Zitplaats, Pin 11 = Eetplaats, Pin 12 = Keuken.
- OneWire bus op GPIO3 (12 DS18B20)
- PWM ventilatie op GPIO20 → externe 0-10V converter
- I2C op GPIO13 (SDA) / GPIO11 (SCL)

### Installatie

1. Flash de sketch.
2. Eerste boot → verbind met AP "HVAC-Setup" → configureer WiFi (RSSI, PW).
3. Ga naar http://hvac.local/settings → vul room IP's/mDNS, circuitnamen, etc.
4. Save → reboot → klaar!
 
FiDel
Zarlardinge, België

-------------------------------------------------------
GitHub repository links: ESP32C6_HVAC => Deze kan Claude Sonnet niet lezen!

Readme: https://github.com/FidelDworp/ESP32C6_HVAC/blob/main/README.md

ESP32 sketch: https://github.com/FidelDworp/ESP32C6_HVAC/blob/main/ESP32_HVAC.ino

Photon sketch: https://github.com/FidelDworp/ESP32C6_HVAC/blob/main/HVAC_Photon.cpp

MAAR: Exacte URLs: kan hij wél lezen!

Readme: https://raw.githubusercontent.com/FidelDworp/ESP32C6_HVAC/main/README.md

ESP32_HVAC.ino sketch: https://raw.githubusercontent.com/FidelDworp/ESP32C6_HVAC/main/ESP32_HVAC.ino

HVAC_Photon.cpp sketch: https://raw.githubusercontent.com/FidelDworp/ESP32C6_HVAC/main/HVAC_Photon.cpp

-------------------------------------------------------

08jan26 10:00 Version 17: Wat nu werkt:

1. Circuit Struct Uitgebreid

Room controller data: setpoint, room_temp, heat_request, home_status
Override systeem: override_active, override_state, override_start

2. Intelligente Beslissings Logica (in pollRooms())
Prioriteit volgorde:
PRIO 0: Manual Override (1 uur) → FORCE ON/OFF
PRIO 1: Hardwired TSTAT → Input
PRIO 2: HTTP Poll → Parse aa, h, y, af, z
PRIO 3: Beslissing:
  - OFFLINE → TSTAT only
  - ONLINE + HOME (af=1) → TSTAT OR HTTP
  - ONLINE + AWAY (af=0) → HTTP only

3. Complete Dashboard
Nieuwe tabel met 14 kolommen:

INPUT: #, Naam, IP, mDNS, Set, Temp, Heat, Home
OUTPUT: TSTAT, Pomp, P, Duty, Vent
CONTROL: Override (ON/OFF knoppen + countdown timer)

Features:

✅ Horizontaal scrollbaar op mobile
✅ Override badge met live countdown (45:23)
✅ Rode rand bij actieve override
✅ ⚠️ waarschuwing in Pomp kolom
✅ Kleurcodering: Groen (Thuis), Oranje (Away), Rood (NA/offline)

4. Override Endpoints

/circuit_override_on?circuit=N → Force ON
/circuit_override_off?circuit=N → Force OFF
/circuit_override_cancel?circuit=N → Annuleren

5. JSON API Uitgebreid
/json endpoint bevat nu ook:

setpoint, room_temp, heat_request, home_status
override_active, override_state, override_remaining

------------------------

To do DRINGEND:
- 

To do Later:
- HTTP polling stability kan nog verbeterd worden
- mDNS werkt niet 100% betrouwbaar op ESP32-C6 (maar IP adressen werken prima)
- Overweeg poll_interval te verhogen als er connection issues zijn
*/

