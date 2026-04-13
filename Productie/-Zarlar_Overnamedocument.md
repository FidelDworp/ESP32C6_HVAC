# Zarlar Thuisautomatisering — Master Overnamedocument
**ESP32-C6 · Arduino IDE · Matter · Google Sheets**
*Filip Delannoy — Zarlardinge (BE) — bijgewerkt 13 april 2026*

---

## 1. Systeemoverzicht

### 1.1 Wat is Zarlar?

Een volledig zelfgebouwd thuisautomatiseringssysteem op basis van drie ESP32-C6 controllers, elk met een eigen AsyncWebServer en Matter-integratie via WiFi. Een vierde apparaat — het **Zarlar Dashboard** op 192.168.0.60 — fungeert als centrale dataverzamelaar: het pollt de JSON-endpoints van alle controllers en POST de data naar Google Sheets via Google Apps Script. Het Dashboard heeft ook één Matter-endpoint: een HOME/UIT schakelaar (`MatterOnOffPlugin`) die alle room-controllers aanstuurt.

```
[ROOM 192.168.0.80] ──┐
[HVAC 192.168.0.70] ──┼──→ Zarlar Dashboard 192.168.0.60 ──→ Google Sheets
[ECO  192.168.0.71] ──┘
         │
         └──→ Apple Home (via Matter/WiFi)
                    ↕
              Dashboard Matter
              HOME/UIT toggle
```

**Belangrijk leermoment:** de HVAC-controller deed vroeger zelf de HTTPS POST naar Google Sheets. Dit mislukte structureel (te lang, heap-druk). De POST is nu volledig gedelegeerd aan het Zarlar Dashboard. Elke controller publiceert alleen zijn `/json` endpoint — het dashboard doet de rest.

### 1.2 Controllers — huidige staat

| Controller | Naam | IP | MAC | Board | Versie | Status |
|---|---|---|---|---|---|---|
| **HVAC** | ESP32_HVAC.local | 192.168.0.70 | 58:8C:81:32:2B:90 | 32-pin clone (experimenteerbord) | v1.19 | ✅ Productie, stabiel |
| **ECO Boiler** | ESP32_ECO Boiler | 192.168.0.71 | 58:8C:81:32:2B:D4 | 32-pin clone (blote controller) | v1.23 | ✅ Productie, stabiel |
| **ROOM / Eetplaats** | ESP32_EETPLAATS | 192.168.0.80 | 58:8C:81:32:2F:48 | 32-pin clone | **v2.21** | ✅ Matter + heap stabiel |
| **Testroom** | ESP32_EETPLAATS | 192.168.0.80 | 58:8C:81:32:29:54 | 32-pin clone (experimenteerbord, kromme pinnetjes) | v2.21 | 🔄 Zelfde IP als EETPL — actief als R-EETPL (ctrl idx 11) |
| **Zarlar Dashboard** | ESP32_ZARLAR.local | 192.168.0.60 | A8:42:E3:4B:FA:BC | 30-pin clone (blote controller) | **v5.0** | ✅ Matter + Matrix 16×16 |

⚠️ **MAC-wissel HVAC:** het experimenteerbord (MAC `58:8C:81:32:29:54`) is eerder ook als HVAC-controller gebruikt. De huidige productie-HVAC draait op `58:8C:81:32:2B:90`. Bij twijfel: check het MAC-adres in de serial output bij boot.

### 1.3 Particle Photon controllers — transitiefase

Tijdens de migratie van Particle Photon naar ESP32 draaien de Photon-controllers nog in productie. Het Dashboard pollt hun data via een **Cloudflare Worker** die de Particle Cloud API afschermt. De matrix toont automatisch Photon-data als fallback zolang de ESP32-versie nog niet actief is.

| Photon | Naam | Device ID (kort) | Status | Equivalent ESP32 |
|--------|------|-----------------|--------|-----------------|
| P-BandB | R1-BandB | 30002c... | ⚫ Offline | R-BandB (idx 6) |
| P-Badkamer | R2-BADK | 560042... | ✅ Online | R-BADK (idx 7) |
| P-Inkom | R3-INKOM | 420035... | ✅ Online | R-INKOM (idx 8) |
| P-Keuken | R4-KEUK | 310017... | ✅ Online | R-KEUKEN (idx 9) |
| P-Waspl | R5-WASPL | 33004f... | ✅ Online | R-WASPL (idx 10) |
| P-Eetpl | R6-EETPL | 210042... | ✅ Online | R-EETPL (idx 11) ← **ESP32 actief** |
| P-Zitpl | R7-ZITPL | 410038... | ✅ Online | R-ZITPL (idx 12) |

**Cloudflare Worker:** `https://controllers-diagnose.filip-delannoy.workers.dev`

| Endpoint | Gebruik |
|----------|---------|
| `/` | Status + last_seen alle Photon devices |
| `/sensor?id={deviceId}` | Volledige sensordata van één Photon via Particle Cloud |
| `/token` | Particle access token (legacy, wordt niet meer gebruikt) |

⚠️ **Particle token** zit veilig in de Worker — niet in de browser en niet in de ESP32 sketch.

**Geplande controllers** (sketches nog niet geflasht):

| # | Naam | IP | MAC | Functie |
|---|---|---|---|---|
| 3 | S-OUTSIDE | 192.168.0.72 | 58:8C:81:32:2F:B0 | Weerstation |
| 6 | R-BandB | 192.168.0.75 | 58:8C:81:32:28:8C | Room |
| 7 | R-BADK | 192.168.0.76 | 58:8C:81:32:2B:80 | Room |
| 8 | R-INKOM | 192.168.0.77 | 58:8C:81:32:29:3C | Room |
| 9 | R-KEUKEN | 192.168.0.78 | 58:8C:81:32:2B:AC | Room |
| 10 | R-WASPL | 192.168.0.79 | 58:8C:81:32:2F:9C | Room |
| 12 | R-ZITPL | 192.168.0.81 | 58:8C:81:32:2D:14 | Room |
| 13 | S-ACCESS | 192.168.0.82 | 58:8C:81:32:26:9C | Buitenverlichting/toegang |

### 1.4 Partitietabel (identiek voor ALLE vier controllers)

Bestand: `partitions_16mb.csv` — plaatsen naast het `.ino` bestand in de schetsmap.

| Naam | Type | Offset | Grootte |
|------|------|--------|---------|
| nvs | data/nvs | 0x9000 | 20 KB |
| otadata | data/ota | 0xe000 | 8 KB |
| app0 | app/ota_0 | 0x10000 | 6 MB |
| app1 | app/ota_1 | 0x610000 | 6 MB |
| spiffs | data/spiffs | 0xC10000 | ~4 MB |

⚠️ **Nooit `huge_app` gebruiken** — had maar één app-slot en brak OTA. `app0` + `app1` elk 6 MB → OTA volledig werkend.

⚠️ **Nooit een 4MB controller gebruiken voor het Dashboard** — de `partitions_16mb.csv` past niet op 4MB flash. Dit veroorzaakte een bootloop (`partition size 0x600000 exceeds flash chip size 0x400000`). Het Dashboard gebruikt nu een 32-pin 16MB controller, identiek aan de andere drie.

### 1.5 ESP32-C6 hardware — modules en boards

#### Controller-module

Alle Zarlar-controllers draaien op de **ESP32-C6-WROOM-1N16** module (Espressif), met:
- Ingebouwde PCB-antenne (standaard)
- 16 MB flash
- Wi-Fi 6 (2.4 GHz), BLE 5, Zigbee/Thread (IEEE 802.15.4)
- 3.3V IO (niet 5V-tolerant)

Voor locaties met zwakke WiFi-verbinding: **ESP32-C6-WROOM-1U** (identieke module, maar met U.FL/IPEX-connector voor externe antenne). Bereik interne antenne: ~20–50m indoors, ~80–200m buiten. Externe antenne: 2–3× beter indoors.

#### Dev boards — twee formfactors in gebruik

| Type | Pins | Prijs | Aantallen | Opmerking |
|---|---|---|---|---|
| 32-pin clone | 32 | €2.52/stuk (AliExpress, 25 dec 2025) | 10 stuks, €65 | **Huidige productie** — clone, niet officieel Espressif |
| 30-pin clone | 30 | €9/stuk (3 stuks, €27) | 3 stuks | Dashboard + reserve |

⚠️ **Clone-boards:** de 10 stuks AliExpress-boards (€65, 25 dec 2025) zijn clones — niet officieel Espressif. Ze werken identiek maar hebben soms afwijkende pinlabels t.o.v. de officiële Espressif DevKitC-1. Gebruik altijd de pinout uit §3.1 / §5.1 van dit document, niet de opdruk op het board.

#### Strapping pins — nooit als input gebruiken

| Pin | Reden |
|---|---|
| IO8 | Strapping pin — LEEG LATEN |
| IO9 | Strapping pin — LEEG LATEN |
| IO0 | Boot pin — alleen als output of met sterke pull-up |
| IO15 | Alleen als output (geen input) |

⚠️ **IO14 bestaat niet** op het 32-pin devboard — staat wel in de ESP32-C6 SoC datasheet maar is niet uitgebroken op deze modules.

#### Voeding

| Situatie | Voeding |
|---|---|
| **Testopstelling** | 5V via USB-C connector van het devboard |
| **Productie (Zarlar shield)** | 5V via VIN-pin van het devboard, beveiligd met PTC-zekering (500 mA) |

De module heeft een ingebouwde 3.3V LDO. Er is geen batterij-ingang (VBAT) zoals bij de Particle Photon.

### 1.6 Pinout-overzicht alle controllers

Gedetailleerde pinout per controller staat in §3.1 (HVAC), §4.1 (ECO), §5.1 (ROOM) en §6.1 (Dashboard). Hieronder een snel referentieoverzicht.

#### HVAC (192.168.0.70)

| Pin | Functie |
|---|---|
| IO3 | DS18B20 OneWire (6× SCH boiler) |
| IO11 | I2C SCL → MCP23017 |
| IO13 | I2C SDA → MCP23017 |
| IO20 | PWM ventilator (0–100%) |
| MCP 0–6 | Relay circuits 1–7 (actief-laag) |
| MCP 7 | Pomp feedback |
| MCP 8–9 | Distributiepompen SCH/WON |
| MCP 10–12 | TSTAT inputs (INPUT_PULLUP) |

#### ECO Boiler (192.168.0.71)

| Pin | Functie | `#define` |
|---|---|---|
| IO1 | Pomprelais (aan/uit) | `RELAY_PIN 1` |
| IO3 | DS18B20 OneWire (6× boiler Top/Mid/Bot H/L) | `ONEWIRE_PIN 3` |
| IO5 | PWM circulatiepomp (0–255) | `PWM_PIN 5` |
| IO20 | SPI CS → MAX31865 (PT1000 collector) | `SPI_CS 20` |
| IO21 | SPI MOSI → MAX31865 | `SPI_MOSI 21` |
| IO22 | SPI MISO → MAX31865 | `SPI_MISO 22` |
| IO23 | SPI SCK → MAX31865 | `SPI_SCK 23` |

#### ROOM (192.168.0.80)

| Pin | Functie |
|---|---|
| IO1 | LDR1 analog (⚠️ 10k pull-up naar 3V3!) |
| IO2 | LDR2 analog (beam) |
| IO3 | DS18B20 OneWire |
| IO4 | NeoPixels data |
| IO5 | MOV1 PIR |
| IO6 | DHT22 data |
| IO7 | Sharp dust analog (RX) |
| IO10 | TSTAT switch (GND = AAN) |
| IO11 | I2C SCL → TSL2561 |
| IO12 | Sharp dust LED (TX) |
| IO13 | I2C SDA → TSL2561 |
| IO18 | CO2 PWM input (MH-Z19, 5V voeding!) |
| IO19 | MOV2 PIR |

#### Dashboard (192.168.0.60)

IO4 → NeoPixel data van 16×16 matrix. Overige IO-pinnen niet in gebruik.

### 1.7 Shield — connectoroverzicht

Overzicht van alle aansluitingen die het Zarlar-shield moet voorzien per controller. Aansluitingen gemarkeerd met ✅ zijn vereist, 〇 zijn optioneel, — niet van toepassing.

| Connector | Pins | Voeding | ROOM | HVAC | ECO | Dashboard | Opmerking |
|---|---|---|---|---|---|---|---|
| **Roomsense** (RJ45) | 8 | 5V + 3V3 + GND | ✅ | — | — | — | DHT22, MOV1 PIR, Sharp dust (LED+analog), LDR1 |
| **OPTION** (RJ45) | 8 | 5V + 3V3 + GND | ✅ | — | — | — | MOV2 PIR, CO2 PWM, TSTAT, LDR2/beam |
| **T-BUS** (3-pin) | 3 | 3V3 + GND | ✅ | ✅ | ✅ | — | DS18B20 OneWire — één bus, meerdere sensoren |
| **Pixel-line** (3-pin) | 3 | 5V + GND | ✅ | — | — | 〇 | NeoPixel data + 5V voeding |
| **I2C** (4-pin) | 4 | 3V3 + GND | ✅ | ✅ | — | — | SDA + SCL, 4.7k pull-ups naar 3.3V |
| **SPI** (6-pin) | 6 | 3V3 + GND | — | — | ✅ | — | MAX31865 PT1000: CS, MOSI, MISO, SCK |
| **Relay OUT** (2-pin) | 2 | — | — | — | ✅ | — | IO1: pomprelais ECO (actief-laag) |
| **PWM OUT** (2-pin) | 2 | — | — | ✅ | ✅ | — | HVAC: ventilator IO20 / ECO: pomp IO5 |
| **UART** (3-pin) | 3 | 3V3 + GND | 〇 | — | — | — | Optie voor serieel randapparaat |

#### Voltage-specificaties per connector

| Connector | Voedings-pin | Signaal-niveau | Opmerking |
|---|---|---|---|
| Roomsense | 5V (VIN) + 3V3 | 3.3V | **AM312 PIR** draait op 3.3V — beweging = HIGH (push-pull, geen pull-up nodig). DHT22 op 3.3V met pull-up. |
| OPTION | 5V (VIN) + 3V3 | 3.3V | MH-Z19 CO2 heeft **5V voedingspin** nodig; PWM-signaal is 3.3V. |
| T-BUS | 3V3 | 3.3V | DS18B20 werkt op 3.3V met 4.7k pull-up naar 3.3V. |
| Pixel-line | **5V (VIN)** | 3.3V | NeoPixels (WS2812) vereisen 5V voeding; 3.3V data-signaal werkt. |
| I2C | 3V3 | 3.3V | 4.7k pull-ups naar **3.3V** (niet 5V zoals op de oude Photon-shield). |
| SPI | 3V3 of 5V | 3.3V | MAX31865 module heeft ingebouwde 3.3V LDO + level shifter — werkt op 3.3V én 5V voeding. SPI-signalen van ESP32-C6 (3.3V) zijn direct compatibel. |
| Relay OUT | — | 3.3V drive | IO1 actief-laag: LOW = relais AAN. Vrijloop-diode op relay coil! |
| PWM OUT | — | 3.3V | `ledcAttach()` — 1 kHz, 8-bit (0–255). Externe driver nodig voor motor. |

### 1.8 Arduino IDE instellingen

| Instelling | Waarde |
|---|---|
| Board | ESP32C6 Dev Module |
| Flash Size | **16 MB** |
| Partition Scheme | Custom → `partitions_16mb.csv` uit schetsmap |
| USB CDC On Boot | **Enabled** (verplicht voor Serial over USB-C) |
| Upload (eerste keer) | USB |
| Upload (daarna) | OTA via Arduino IDE → Sketch → Upload via OTA |

---

## 2. Gemeenschappelijke regels — van toepassing op ALLE controllers

### 2.1 Verplichte sketch-header

Elke sketch begint met:
```cpp
// ⚠️ Verplicht voor ESP32-C6 (RISC-V) — vóór alle #include statements
#define Serial Serial0
```
**Positie is kritiek:** staat hij ná de `#include` statements → 100+ cascade-compilatiefouten.

⚠️ **`#define Serial Serial0` mag NIET aanwezig zijn zonder Matter.** Zonder Matter stuurt deze define de output naar UART0 (fysieke pins) in plaats van USB CDC — Serial monitor blijft leeg. Alleen toevoegen als Matter effectief geïntegreerd is.

De **versieheader** als blokcommentaar bevat datum, versienummer en beschrijving per wijziging. Let op: schrijf **`* /`** (met spatie) als je `*/` bedoelt in de tekst — anders breekt de blokcommentaar.

### 2.2 Heap — basisregels

De ESP32-C6 heeft 512 KB SRAM. Matter + WiFi reserveren ~130–140 KB. De 16 MB flash is **niet** beschikbaar als heap.

**Drempelwaarden:**

| Largest free block | Status | Actie |
|---|---|---|
| > 35 KB | 🟢 Comfortabel | Geen actie |
| 25–35 KB | 🟡 Werkbaar | Opvolgen bij volgende endpoint-toevoeging |
| < 25 KB | 🔴 Instabiel | STOP — evalueer endpoint-schrapping |

**Regels:**
- Alle webpagina's via **chunked streaming** (`AsyncResponseStream`) — nooit meer `html.reserve(N)` of grote String-opbouw
- `String` globals → **`char[]` + `strlcpy`** — elimineer permanente heap-fragmentatie
- `String(i)` voor NVS-keys → **`snprintf` naar `char buf[]`** op de stack
- **ArduinoJson v7**: `StaticJsonDocument<N>` alloceert ALTIJD op heap (niet stack zoals v6). Gebruik globale `JsonDocument` met `clear()` hergebruik.
- `http.getString()` → **`http.getStream()` + `DeserializationOption::Filter`** — elimineert ~1 KB String-allocatie per poll
- `buildJson()` / log-JSON → **pure `snprintf` naar static `char buf[]`** — nul heap-alloc

**`WebServer` vs `AsyncWebServer`:**
Het Dashboard gebruikt `WebServer` (blocking) — dit is bewust gekozen. Voor een dashboard zonder zware gelijktijdige belasting is `WebServer` beter: minder heap (~10KB verschil), minder complex, bewezen stabiel. `AsyncWebServer` heeft hier geen voordeel.

### 2.3 DS18B20 / OneWireNg

| Regel | Detail |
|---|---|
| `CONVERT_ALL` broadcast | Stuur conversie eenmalig naar alle sensoren (SKIP ROM 0xCC + 0x44). Daarna individueel uitlezen. |
| `delay(750)` na conversie | De enige geautoriseerde lange delay — WDT-safe via FreeRTOS `vTaskDelay` |
| Leesfrequentie ≥ 60s | Elke 2s lezen → 30× meer WDT-exposure. 60s is ruim voldoende. |
| Nooit per sensor in loop | Stapelt interrupt-blocking op → `Interrupt WDT timeout on CPU0` |

### 2.4 Matter — gemeenschappelijke regels

| Regel | Detail |
|---|---|
| `#define Serial Serial0` vóór `#include` | Verplicht, zie §2.1 |
| `Matter.begin()` returns `void` | Geen `if(Matter.begin())` — geeft compilatiefout |
| mDNS verwijderen | Matter start eigen interne mDNS-stack. `MDNS.begin()` → conflicten. Volledig verwijderen, toegang via statisch IP. |
| Max 12 endpoints | Praktische limiet op ESP32-C6 met volledige webUI |
| Updates via `loop()` | Centrale `update_matter()` functie, interval 5–10s via `millis()`. Nooit blocking `while(!commissioned)`. |
| `nvs_flash_erase()` niet vanuit async handler | Gebruik vlag: `matter_nuclear_reset_requested = true` → uitvoeren in `loop()` |
| Matter reset bij endpoint-wijziging | Type of volgorde wijzigen → pairing wissen vóór herpairing |
| `MatterOnOffLight` vs `MatterOnOffPlugin` | Lichten: `OnOffLight`. Logische schakelaars: `OnOffPlugin`. |
| Pairing code niet alleen via Serial | Serial is onbetrouwbaar op ESP32-C6 via USB. Toon pairing code altijd ook in de webUI (`/settings`). |

**Auto-recovery bij corrupt NVS** (toepassen bij Matter-initialisatie):
```cpp
Matter.begin();
delay(200);
if (!Matter.isDeviceCommissioned() && Matter.getManualPairingCode().length() < 5) {
  nvs_flash_erase();
  nvs_flash_init();
  ESP.restart();
}
```

**matterNuclearReset() patroon** (settings bewaard, Matter-NVS gewist):
1. Laad alle config-keys naar lokale RAM-variabelen
2. `nvs_flash_erase()` — wist volledige NVS partitie
3. `nvs_flash_init()` — herinitialiseer NVS
4. Schrijf config-keys terug naar eigen namespace
5. `ESP.restart()`

### 2.5 AsyncWebServer handlers

- Alle handlers: **nooit blocking I/O** — sensordata lezen in `loop()`, niet in handler
- Grote pagina's: **chunked streaming** via `AsyncResponseStream`
- `/json` endpoint: **pure `snprintf`** naar `char buf[]`, direct naar response

### 2.6 IO-pinnen — onmiddellijke reactie

Elke IO-actie via de webUI of Matter moet **direct** de pin aansturen — nooit wachten op een pollcyclus. Patroon:
```cpp
server.on("/actie", HTTP_GET, [](AsyncWebServerRequest *request) {
  // 1. State bijwerken
  circuits[idx].override_active = true;
  // 2. Pin ONMIDDELLIJK schakelen
  mcp.digitalWrite(idx, LOW);
  // 3. Matter synchroon bijwerken
  ignore_callbacks = true;
  matter_circuit[idx].setOnOff(true);
  ignore_callbacks = false;
  request->send(200, "text/plain", "OK");
});
```

**Bewuste code duplicatie — drie plaatsen, één patroon:**

In de HVAC sketch staat `mcp.digitalWrite(idx, on_off ? LOW : HIGH)` op drie afzonderlijke plaatsen:

| Plaats | Trigger | Toegevoegd in |
|---|---|---|
| `/circuit_override_on` + `/circuit_override_off` handler | Bediening via webUI | v1.14 |
| `onChangeOnOff` Matter callback | Bediening via Apple Home / HomeKit | v1.19 |
| `/circuit_override_cancel` handler | Override annuleren via webUI | v1.14 |

Dit is **bewuste duplicatie** — niet refactoren naar een centrale `applyRelay()` functie tenzij er een vierde pad bijkomt.

### 2.7 JSON key synchronisatie — kritiek leermoment

Wanneer een JSON-structuur in een controller hernoemd wordt (bijv. van lange namen naar compacte a/b/c-keys), **falen alle consumers (HVAC, Zarlar, Google Script) stil** — JSON-keys retourneren gewoon 0 als ze niet gevonden worden, zonder foutmelding.

**Regel:** bij elke JSON-structuurwijziging meteen alle consumers nalopen:
1. HVAC sketch (poll-code + filter-doc)
2. Zarlar Dashboard
3. Google Apps Script

### 2.8 Crash-logging (in alle drie sketches aanwezig)

```cpp
// setup(): vorige crash lezen
preferences.begin("crash-log", false);
// loop(): heap bewaken
if (heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) < 25000) {
  // opslaan in NVS
}
```
Crash-info tonen in `/settings`: laatste reden + teller + resetknop.

⚠️ **NVS crashlog feedback loop (geleerd 12 april 2026) — opgelost in v2.20:** de crashlog schrijft elke 60s naar NVS zolang heap_block < 25 KB. Elke `begin()/end()` alloceert tijdelijk een NVS-buffer — dat maakt de toch al krappe heap nog krapper. Fix: schrijf crashlog **maximaal één keer per low-heap episode** via een statische vlag:
```cpp
static bool crash_logged_this_episode = false;
if (lb < 25000 && !crash_logged_this_episode) {
    crash_logged_this_episode = true;
    // ... crashPrefs.begin() etc.
} else if (lb >= 25000) {
    crash_logged_this_episode = false;
}
```
✅ **Geïmplementeerd in v2.20.**

### 2.9 Serial commando's (alle vier sketches)

| Commando | Effect |
|---|---|
| `reset-matter` | Wist alleen Matter/HomeKit koppeling — instellingen blijven intact |
| `reset-all` | Wist alles: instellingen (NVS) + Matter-koppeling |
| `status` | Uitgebreid statusrapport in Serial Monitor |

### 2.10 NVS namespaces

| Namespace | Eigenaar | Mag aanraken? |
|---|---|---|
| `zarlar` | Dashboard sketch | ✅ |
| `room-config` | ROOM sketch | ✅ |
| `hvac-config` | HVAC sketch | ✅ |
| `eco-config` | ECO sketch | ✅ |
| `crash-log` | Alle sketches | ✅ |
| `chip-factory` | Matter intern | ❌ Niet aanraken |
| `chip-config` | Matter intern | ❌ Niet aanraken |
| `chip-counters` | Matter intern | ❌ Niet aanraken |

### 2.11 Serial monitor — bekende valkuilen ESP32-C6

- **Serial blijft leeg na boot:** `USB CDC On Boot` staat op `Disabled` in Arduino IDE → zetten op `Enabled`
- **Serial blijft leeg na boot (2):** `#define Serial Serial0` aanwezig zonder Matter → verwijderen
- **Serial mist boot-berichten:** sketch print te snel na reset, monitor opent te laat → `delay(3000)` na `Serial.begin(115200)` in `setup()`
- **Captive portal werkt niet op Mac Safari:** `onNotFound` alleen volstaat niet — expliciete handlers nodig voor Apple/Android/Windows detectie-URLs (zie §6.5)

### 2.12 WiFi scan — lessen

- `WiFi.channel(k)` kan `0` teruggeven voor bepaalde netwerken → `ch < 1` filter verwijdert geldige netwerken
- ESP32-C6 is **2.4GHz-only** → geen channel-filter nodig
- TCP-ping naar gateway: gebruik **port 53 (DNS)** — altijd open. Port 80 (HTTP) is vaak gesloten op routers
- RSSI-waarden zijn negatief → in JavaScript is `c.r` falsy als `r === 0` maar truthy als `r === -62`. Gebruik `c.r !== 0` als conditie, niet `c.r`
- `Math.abs(c.r)` voor weergave — minteken weglaten vermijdt layout-problemen
- `font-size` < 11px wordt onderdrukt door iOS Safari — gebruik minimum 11px. Voeg `-webkit-text-size-adjust:none` toe aan body

### 2.13 Statusmatrix — lessen en valkuilen

- **WS2812B 5V voeding apart:** bij volle helderheid kan een 16×16 matrix >3A trekken. Nooit via shield PTC (max 500 mA). Aparte 5V rail verplicht.
- **Serpentine adressering:** `matPxIdx()` converteert logische (rij, kolom) naar fysiek pixel-adres. Bij incorrecte richting: `#define MATRIX_FLIP_H true`. Testen via `matrix-test` Serial commando.
- **NeoPixel buffer heap:** 256 pixels × 3 bytes = 768 bytes permanent in heap. Klein maar tel mee bij heap-budgettering.
- **Largest free block is de echte metric**, niet total free heap. Matrix + Matter: largest block daalt naar ~32–36KB. Crashdrempel blijft 25KB.
- **MROW-volgorde moet exact overeenkomen met de SVG labelsheet.** Een off-by-one verschuiving zorgt ervoor dat alle data op de verkeerde rij staat. Verificatie via `status` Serial commando na flash.
- **`sed -i` op `*/`:** een globale sed-vervanging van `*/` naar `* /` corrupteert de afsluitende `*/` van het versieheader-blokcommentaar → compilatiefout "unterminated comment". Nooit globaal `*/` vervangen.
- **Photon data via Cloudflare Worker:** de ESP32 kan niet rechtstreeks de Particle Cloud aanroepen (HTTPS heap-druk, token-beheer). Een Worker als proxy is de elegante oplossing: token veilig in de Worker, ESP32 roept gewone HTTPS GET aan.
- **Automatische fallback-logica:** gebruik `MatrixRowDef { esp_idx, photon_idx, sys_idx }` per rij. `updateMatrix()` kiest dynamisch ESP32 → Photon → zwart. Geen reflash nodig bij transitie.
- **Controller-index verificatie:** de controller-indices in MROW zijn niet hardcoded op volgorde in de sketch maar afhankelijk van de dashboard `/settings` configuratie. Altijd verifiëren via `status` commando na flash — nooit aannemen.

### 2.14 WebUI JavaScript — lessen (geleerd 12–13 april 2026)

- **Nooit de volledige JS-block in één `str_replace` vervangen.** Één fout in quote-escaping breekt het complete script onzichtbaar — alle functies stoppen. Gebruik altijd kleine, chirurgische ingrepen op specifieke regels.
- **`DOMContentLoaded` betrouwbaarder dan `window.addEventListener('load')`** voor inline scripts in gestreamde HTML-pagina's. Bij pagina's zonder externe resources kan `load` al gevuurd zijn vóór het inline script volledig geparsed is → `setInterval` wordt nooit geregistreerd → auto-refresh stopt. Gebruik:
  ```js
  document.addEventListener('DOMContentLoaded', function() {
    updateValues();
    setInterval(updateValues, 3000);
    setInterval(updateClock, 1000);
  });
  ```
- **Klok onafhankelijk van JSON-fetch:** gebruik een aparte `updateClock()` met `setInterval(updateClock, 1000)`. Sla de uptime op als globale `lastUptime` en update die in de fetch-callback. Zo tikt de klok door ook als de JSON-fetch tijdelijk faalt.
- **Slider DOM-waarde uitlezen voor value-cel update** (geen JSON-key nodig):
  ```js
  else if(lbl.includes('Dim snelheid')){
    var sl=document.querySelector('input[name=duration]');
    if(sl) td.textContent=sl.value+' s';
  }
  ```
- **Slider niet overschrijven terwijl de gebruiker hem versleept** — gebruik `document.activeElement` check:
  ```js
  if(sl && sl !== document.activeElement) sl.value = data.g;
  ```
- **PIR triggers direct herberekenen na `pushEvent()`**, niet wachten op de 60s-gate:
  ```cpp
  pushEvent(mov1Times, MOV_BUF_SIZE);
  mov1_triggers = countRecent(mov1Times, MOV_BUF_SIZE);  // onmiddellijk!
  ```
- **Dot-cirkels voor state die geen JSON-key heeft** (bv. AUTO/MANUEEL modes): dot updatet niet live na togglen — dit is verwarrend. Oplossing: elimineer de state (KISS), of voeg een JSON-key toe. Nooit een dot maken voor iets dat niet live kan updaten.

---

## 3. HVAC Controller — specifiek

### 3.1 Hardware

| Component | Detail |
|---|---|
| Board | ESP32-C6 32-pin clone (experimenteerbord, MAC `58:8C:81:32:2B:90`) |
| Voeding | Test: 5V USB-C / Productie: 5V via VIN (Zarlar shield, PTC 500 mA) |
| Static IP | 192.168.0.70 |
| I/O expander | MCP23017 op I2C (SDA=IO13, SCL=IO11) |
| Temperatuursensoren | 6× DS18B20 op OneWire (IO3) — SCH boiler |
| Ventilator | PWM op IO20 via `ledcWrite` (0–255 → 0–100%) |

#### Pinout HVAC

| ESP32-C6 Pin | `#define` | Functie | Opmerking |
|---|---|---|---|
| IO3 | `ONE_WIRE_PIN 3` | DS18B20 OneWire | 6× SCH boiler |
| IO11 | `I2C_SCL 11` | I2C SCL → MCP23017 | `Wire.begin(13, 11)` — 4.7k pull-up naar 5V |
| IO13 | `I2C_SDA 13` | I2C SDA → MCP23017 | 4.7k pull-up naar 5V |
| IO20 | `VENT_FAN_PIN 20` | PWM ventilator | `ledcAttach(pin, 1000, 8)` — 1 kHz, 8-bit (0–255) |

#### MCP23017 poortindeling

| MCP pin | pinMode | Functie |
|---|---|---|
| 0–6 | OUTPUT | Relay circuits 1–7 (actief-laag: LOW = AAN) |
| 7 | INPUT_PULLUP | Pomp feedback |
| 8 | OUTPUT (`RELAY_PUMP_SCH`) | Distributiepomp SCH |
| 9 | OUTPUT (`RELAY_PUMP_WON`) | Distributiepomp WON |
| 10–12 | INPUT_PULLUP | TSTAT inputs circuits (LOW = warmtevraag) |
| 13–15 | INPUT_PULLUP | Reserve TSTAT-slots (toekomstige circuits) |

**Circuitnamen:** instelbaar per circuit via `/settings`. Default: `Circuit 1` … `Circuit 7`. In productie ingesteld op BB, WP, BK, ZP, EP, KK, IK.

### 3.2 Libraries (HVAC)

| Library | Gebruik |
|---|---|
| `OneWireNg_CurrentPlatform` | DS18B20 — C6-compatibel (vervangt OneWire + DallasTemperature) |
| `Adafruit_MCP23X17` | I/O expander — relais + TSTAT |
| `AsyncTCP` + `ESPAsyncWebServer` | Webserver |
| `ArduinoJson` | JSON polling van room controllers + ECO |
| `Preferences` | NVS opslag |

### 3.3 Heap-baseline (v1.18)

```
Setup:   free=~180KB  largest=~55KB
Runtime: largest_block stabiel >35KB  ✅
```

### 3.4 Matter endpoints (v1.18)

| # | Type | Variabele | Opmerking |
|---|---|---|---|
| EP1 | MatterTemperatureSensor | `sch_temps[0]` | Boiler top |
| EP2–EP8 | MatterOnOffPlugin | `circuits[0..6]` | Kringen 1–7 |
| EP9 | MatterFan | `vent_percent` | Ventilatie % |

### 3.5 Versiehistorie HVAC (recente wijzigingen)

| Versie | Wijziging |
|---|---|
| v1.19 | Matter `onChangeOnOff` callback: `mcp.digitalWrite()` onmiddellijk toegevoegd — relais reageren nu direct vanuit Apple Home |
| v1.18 | ECO JSON keys: ETopH→b, EBotL→g, EAv→h, EQtot→i |

### 3.6 HVAC /json output (keys a..ae → naar Zarlar → Google Sheets)

| Key | Sheet | Label | Eenheid | Opmerking |
|-----|-------|-------|---------|-----------|
| `a` | B | uptime_sec | s | |
| `b` | C | KST1 (sch_temps[0]) | °C | Boiler top |
| `c` | D | KST2 (sch_temps[1]) | °C | |
| `d` | E | KST3 (sch_temps[2]) | °C | |
| `e` | F | KST4 (sch_temps[3]) | °C | |
| `f` | G | KST5 (sch_temps[4]) | °C | |
| `g` | H | KST6 (sch_temps[5]) | °C | Boiler bodem |
| `h` | I | KSAv (gemiddelde boiler) | °C | |
| `i` | J | duty_4h C1 (BB) | int | Duty-cyclus circuit 1 |
| `j` | K | duty_4h C2 (WP) | int | |
| `k` | L | duty_4h C3 (BK) | int | |
| `l` | M | duty_4h C4 (ZP) | int | |
| `m` | N | duty_4h C5 (EP) | int | |
| `n` | O | duty_4h C6 (KK) | int | |
| `o` | P | duty_4h C7 (IK) | int | |
| `p` | Q | heating_on C1 (BB) | 0/1 | Relais actief |
| `q` | R | heating_on C2 (WP) | 0/1 | |
| `r` | S | heating_on C3 (BK) | 0/1 | |
| `s` | T | heating_on C4 (ZP) | 0/1 | |
| `t` | U | heating_on C5 (EP) | 0/1 | |
| `u` | V | heating_on C6 (KK) | 0/1 | |
| `v` | W | heating_on C7 (IK) | 0/1 | |
| `w` | X | total_power | kW | Totaal vermogen alle actieve kringen |
| `x` | Y | vent_percent | % | Incl. vent_override indien actief |
| `y` | Z | sch_on | 0/1 | Distributiepomp SCH actief |
| `z` | AA | last_sch_pump.kwh_pumped | kWh | |
| `aa` | AB | won_on | 0/1 | Distributiepomp WON actief |
| `ab` | AC | last_won_pump.kwh_pumped | kWh | |
| `ac` | AD | RSSI | dBm | |
| `ad` | AE | FreeHeap% | % | |
| `ae` | AF | LargestBlock | KB | |

⚠️ **HVAC gebruikt key `ac` voor RSSI** — conform alle andere controllers (ECO gebruikt afwijkend `p`).

### 3.7 Openstaande punten HVAC

- **kWh-berekening**: echte `Q = m × Cp × ΔT / 3600` per pompbeurt implementeren
- **HTML compressie**: zelfde aanpak als ROOM — witte pagina op iPhone bij ventilatieslider wijst op heap-krapte bij page reload

---

## 4. ECO Boiler Controller — specifiek

### 4.1 Hardware

| Component | Detail |
|---|---|
| Board | ESP32-C6 32-pin clone (blote controller, MAC `58:8C:81:32:2B:D4`) |
| Voeding | Test: 5V USB-C / Productie: 5V via VIN (Zarlar shield, PTC 500 mA) |
| Static IP | 192.168.0.71 |
| Temperatuursensoren | 6× DS18B20 op OneWire (IO3) — 2 per boilerlaag: Top/Mid/Bot × H/L |
| Zonnecollector | PT1000 via MAX31865 SPI-module (CS=IO20, MOSI=IO21, MISO=IO22, SCK=IO23) |
| Pomprelais | IO1 — digitaal aan/uit |
| Circulatiepomp | PWM op IO5 (0–255), freq 1 kHz, 8-bit resolutie |

### 4.2 ECO /json output (keys a..s → naar Zarlar → Google Sheets)

| Key | Sheet | Label | Eenheid |
|-----|-------|-------|---------|
| `a` | B | uptime_sec | s |
| `b` | C | ETopH | °C |
| `c` | D | ETopL | °C |
| `d` | E | EMidH | °C |
| `e` | F | EMidL | °C |
| `f` | G | EBotH | °C |
| `g` | H | EBotL | °C |
| `h` | I | EAv | °C |
| `i` | J | EQtot | kWh |
| `j` | K | dEQ (delta kWh) | kWh |
| `k` | L | yield_today | kWh |
| `l` | M | Tsun | °C |
| `m` | N | dT (Tsun−Tboil) | °C |
| `n` | O | pwm_value | 0–255 |
| `o` | P | pump_relay | 0/1 |
| `p` | Q | **RSSI** | dBm |
| `q` | R | FreeHeap% | % |
| `r` | S | MaxAllocHeap | KB |
| `s` | T | MinFreeHeap | KB |

⚠️ **ECO gebruikt key `p` voor RSSI** — alle andere controllers gebruiken `ac`. Kritiek voor Dashboard RSSI-extractie.

### 4.3 Openstaande punten ECO

- **Heap-analyse**: baseline meten, ArduinoJson v7 check, `String(i)` NVS-keys → `snprintf`
- **kWh-berekening**: echte `Q = m × Cp × ΔT / 3600` per pompbeurt
- **Reactietijden**: IO-pinnen direct aansturen vanuit webUI-handlers
- **Versieheader**: `* /` met spatie in commentaar

---

## 5. ROOM Controller — specifiek

### 5.1 Hardware

| Component | Detail |
|---|---|
| Board | ESP32-C6 32-pin clone (MAC `58:8C:81:32:2F:48` productie / `58:8C:81:32:29:54` experimenteerbord) |
| Voeding | Test: 5V USB-C / Productie: 5V via VIN (Zarlar shield, PTC 500 mA) |
| Static IP | 192.168.0.80 |
| Sensoren | DHT22 (temp+vocht), DS18B20 (OneWire), MH-Z19 (CO2), Sharp GP2Y (dust), TSL2561 (lux), LDR |
| PIR sensoren | **AM312** (natively 3.3V, push-pull active HIGH — vervangen HC-SR501) |
| Actuatoren | NeoPixel strip (tot 30 pixels), laserbeam LDR |
| Verwarming | TSTAT output + setpoint |
| Shield | Nieuwe ESP32C6 shields aangekomen van JLCPCB (12 april 2026) — vervanging Photon shields |

#### PIR sensors — AM312 (v2.19+)

De HC-SR501 PIR sensors (gemodificeerd voor 3.3V) zijn onbruikbaar gebleken door BISS0001 gain-degradatie bij lagere voedingsspanning. Vervangen door **AM312 modules**:

| Eigenschap | HC-SR501 (oud) | AM312 (nieuw) |
|---|---|---|
| Voedingsspanning | 5V (gemodificeerd naar 3.3V → kapot) | **3.3V natively** |
| Output | Active LOW (INPUT_PULLUP vereist) | **Active HIGH, push-pull (geen pull-up nodig)** |
| Sketch logica | `== LOW` + `INPUT_PULLUP` | `== HIGH`, geen `INPUT_PULLUP` |

⚠️ **Sketch aanpassing bij overgang naar AM312:** `INPUT_PULLUP` → `INPUT`, `!= LOW` → `!= HIGH` (of omgekeerde logica). In v2.19 wordt de bestaande `!p1 && last1` (falling edge) aanpak gebruikt — dit vereist dat `INPUT_PULLUP` nog aanwezig is voor de AM312. Controleer bij ingebruikname of de logica overeenkomt.

#### Pinout ROOM (Photon → ESP32-C6 conversie)

| ESP32-C6 Pin | `#define` | Photon | Functie | Opmerking |
|---|---|---|---|---|
| IO1 | `LDR_ANALOG 1` | A3 | LDR1 analog | ⚠️ 10k pull-up IO1→3V3 op shield! |
| IO2 | `OPTION_LDR 2` | A7 | LDR2 analog (beam/MOV2) | 0–3.3V, geschaald 0–100 |
| IO3 | `ONE_WIRE_PIN 3` | D3 | DS18B20 OneWire | 3.3V pull-up |
| IO4 | `NEOPIXEL_PIN 4` | D4 | NeoPixels data | NEO_GRB + NEO_KHZ800 |
| IO5 | `PIR_MOV1 5` | D5 | MOV1 PIR (AM312) | INPUT_PULLUP — zie noot AM312 hierboven |
| IO6 | `DHT_PIN 6` | D6 | DHT22 data | 3.3V pull-up |
| IO7 | `SHARP_ANALOG 7` | A2 | Sharp dust analog (RX) | Voltage divider indien >3.3V |
| IO10 | `TSTAT_PIN 10` | A6 | TSTAT switch (GND = AAN) | INPUT_PULLUP |
| IO11 | — | D1 | I2C SCL → TSL2561 | `Wire.begin(13, 11)` — 4.7k pull-up naar 5V |
| IO12 | `SHARP_LED 12` | D7 | Sharp dust LED (TX) | OUTPUT, HIGH = uit |
| IO13 | — | D0 | I2C SDA → TSL2561 | 4.7k pull-up naar 5V |
| IO18 | `CO2_PWM 18` | A4 | CO2 PWM input (MH-Z19) | ⚠️ MH-Z19 heeft 5V voedingspin nodig! |
| IO19 | `PIR_MOV2 19` | A5 | MOV2 PIR (AM312) | INPUT_PULLUP — zie noot AM312 hierboven |

⚠️ **IO1 heeft een 10k pull-up naar 3V3** op de Roomsense-shield — beïnvloedt analoge meting van LDR1.

⚠️ **MH-Z19 CO2-sensor** heeft aparte 5V voedingspin — PWM-signaal zelf is 3.3V compatibel. Op de oude Photon-shield is de voedingsspanning slechts 4.3–4.4V → sensor leest niet. De `pulseIn()` call blokkeert dan toch nog tot 400ms per 60s cyclus (beide calls lopen volledig op timeout).

### 5.2 Libraries (ROOM)

| Library | Gebruik |
|---|---|
| `DHT` | DHT22 temperatuur + vochtigheid |
| `OneWireNg_CurrentPlatform` | DS18B20 — C6-compatibel |
| `Adafruit_TSL2561_U` | TSL2561 luxmeter op I2C |
| `Adafruit_NeoPixel` | NeoPixel strip (NEO_GRB, 800 kHz) |
| `AsyncTCP` + `ESPAsyncWebServer` | Webserver (chunked streaming) |
| `Preferences` | NVS opslag |

### 5.3 Sensordrempelwaarden (sketch-constanten)

| `#define` | Waarde | Betekenis |
|---|---|---|
| `SENSOR_TEMP_MIN` | 5.0 °C | Onder = sensor defect (rood in UI) |
| `SENSOR_TEMP_MAX` | 40.0 °C | Boven = sensor defect (rood in UI) |
| `SENSOR_HUMI_MIN` | 10 % | Onder = sensor defect |
| `SENSOR_HUMI_MAX` | 99 % | Boven = sensor defect |
| `SENSOR_RSSI_WARN` | −75 dBm | Zwak signaal (oranje) |
| `SENSOR_RSSI_CRIT` | −85 dBm | Kritiek signaal (rood) |
| `SENSOR_LUX_MAX` | 65000 lux | ≥ 65535 = I2C garbage (TSL2561) |

### 5.4 Heap-baseline (v2.10, ongewijzigd in v2.19)

```
Setup:   23% free  (62936 bytes)   Largest block: 45 KB
Runtime: 20% free  (~74 KB)        Largest block: 31 KB  ✅
Crashdrempel: 25 KB — marge: 6 KB  ✅
```

Matter kost ~214 KB heap — niet te vermijden. Alle andere optimalisaties zijn gedaan.

### 5.5 Heap-optimalisaties doorgevoerd (v2.10)

| Maatregel | Winst |
|---|---|
| `pixel_nicknames[30]` String[] → `char[30][32]` BSS | ~1.5 KB heap |
| `ds_nicknames[4]`, `room_id`, `wifi_*`, `static_ip_str` → `char[]` | Permanente heap weg |
| `getFormattedDateTime()` String return → `const char*` static buf | Heap-alloc per call weg |
| Captive portal handlers → AP-only registratie | ~600 bytes handler-heap |
| `MOV_BUF_SIZE` 50 → 20 | 240 bytes BSS |
| N pixel-handlers → 2 universele handlers (`?idx=`) | ~600 bytes handler-heap |
| `mdns_name` en `hsvToRgb()` verwijderd | Flash + BSS |

### 5.6 Matter endpoints (v2.19, werkend)

| # | Type | Variabele | Opmerking |
|---|---|---|---|
| EP1 | MatterThermostat | `room_temp` + `heating_setpoint` | Setpoint instelbaar vanuit Apple Home |
| EP2 | MatterHumiditySensor | `humi` | |
| EP3 | MatterOccupancySensor | `mov1_light` | MOV1 PIR |
| EP4 | MatterOccupancySensor | `mov2_light` | Alleen als `mov2_enabled` |
| EP5 | MatterColorLight | `neo_r/g/b` | `HsvColor_t` API; on/off altijd `true` |
| EP6 | MatterOnOffLight | `pixel_mode[0]` | SW1: pixel 0 MOV-override |
| EP7 | MatterOnOffLight | `pixel_on[1]` | SW2: pixel 1 |
| EP8 | MatterOnOffLight | `pixel_on[2..N]` | SW3: pixels 2..pixels_num-1 samen |

### 5.7 Matter — ROOM-specifieke valkuilen

**"Aparte tegels" in Apple Home — hypothese endpoint-volgorde:**
In de werkende v2.1 (3 maart 2026) stond de thermostat op positie 7 ná 6 sensor-endpoints → Apple Home bood splitsen aan. In v2.10+ staat de thermostat op EP1 → geen splitsen.

Te proberen volgende sessie:
```
EP1: MatterTemperatureSensor (los)
EP2: MatterHumiditySensor
EP3: MatterOccupancySensor MOV1
EP4: MatterOccupancySensor MOV2
EP5: MatterColorLight
EP6: MatterOnOffLight SW1
EP7: MatterOnOffLight SW2
EP8: MatterOnOffLight SW3
EP9: MatterThermostat  ← achteraan
```
Let op: Matter reset + herpairing vereist bij endpoint-volgorde wijziging.

**Andere ROOM Matter-valkuilen:**

| Valkuil | Oplossing |
|---|---|
| `MatterEnhancedColorLight` blokkeert aparte tegels | Gebruik `MatterColorLight` |
| `espHsvColor_t` vs `HsvColor_t` | `MatterColorLight` = `HsvColor_t` |
| Pixels buiten `pixels_num` bij boot | `updateLength(30)` + `clear()` + `show()` eerst, dan `updateLength(pixels_num)` |

### 5.8 RSSI key in /json

| Key | Label |
|---|---|
| `ac` | RSSI (dBm) |

### 5.9 ROOM /json output (keys a..ai → naar Zarlar → Google Sheets)

JSON ongewijzigd t.o.v. v2.10. Geen nieuwe keys toegevoegd in v2.12–v2.19.

| Key | Sheet | Label | Eenheid |
|-----|-------|-------|---------|
| `a` | B | uptime_sec | s |
| `b` | D | Heating_on | 0/1 |
| `c` | E | Heating_setpoint | °C |
| `d` | F | TSTATon | 0/1 |
| `e` | G | Temp1 (DHT22) | °C |
| `f` | H | Temp2 (DS18B20) | °C |
| `g` | I | Vent_percent | % |
| `h` | J | Humi (DHT22) | % |
| `i` | K | Dew (dauwpunt) | °C |
| `j` | L | DewAlert | 0/1 |
| `k` | M | CO2 | ppm |
| `l` | N | Dust | — |
| `m` | O | Light LDR | 0–100 |
| `n` | P | SUNLight lux | lux |
| `o` | Q | Night (licht>50) | 0/1 |
| `p` | R | Bed switch | 0/1 |
| `q`..`s` | S..U | Neopixel R/G/B | 0–255 |
| `t` | V | Pixel_on_str | tekst |
| `u` | W | Pixel_mode_str | tekst |
| `v` | X | Home switch | 0/1 |
| `w`..`x` | Y..Z | MOV1/MOV2 triggers/min | /min |
| `y`..`z` | AA..AB | MOV1/MOV2 lamp aan | 0/1 |
| `aa` | AC | BEAMvalue | 0–100 |
| `ab` | AD | BEAMalert | 0/1 |
| `ac` | AE | RSSI | dBm |
| `ad` | AF | FreeHeap% | % |
| `ae` | AG | LargestBlock | KB |
| `af` | AH | MinFreeHeap | KB |
| `ag` | AI | ds_count | — |
| `ah` | AJ | Tds2 | °C |
| `ai` | AK | Tds3 | °C |

**Dashboard matrix ROOM-kolom 6/7:** gebruikt `y`/`z` (lamp aan) voor MOV-pixels. Voor bewegingsdetectie ongeacht licht → gebruik `w`/`x` (triggers/min > 0). **Matrix-update nog te doen** (zie §5.11).

### 5.10 Versiehistorie ROOM (recente wijzigingen)

| Versie | Datum | Wijziging |
|---|---|---|
| v2.21 | 13 apr 2026 | KISS: `heating_mode` + `vent_mode` verwijderd. Matter `onChangeMode` → `home_mode`. CO2 dot bij ventilatieslider. Slider volgt werkelijke vent_percent. Vent default 25%. |
| v2.20 | 13 apr 2026 | Crash-stabiliteit: NVS crashlog feedback loop fix, Matter interval 5s→30s, CO2 pulseIn timeout 200ms→50ms |
| v2.19 | 12 apr 2026 | Dim snelheid + Licht tijd: JS handler leest slider DOM-waarde; format "N s" / "N min" |
| v2.18 | 12 apr 2026 | MOV triggers direct herberekend bij PIR-event via `countRecent()` na `pushEvent()` |
| v2.17 | 12 apr 2026 | JSON terug naar origineel (680 bytes, geen nieuwe keys) — v2.13/14/15/16 NG |
| v2.16 | 12 apr 2026 | Heropbouw: timer fix (DOMContentLoaded + updateClock), alle features v2.13–v2.14 correct |
| v2.13–v2.15 | 12 apr 2026 | NG — JS grote block vervanging brak submitAjax en setInterval |
| v2.12 | 12 apr 2026 | UI: binaire waarden → gekleurde .dot cirkels; MOV AUTO-kleur bug fix (hardcoded groen → neo_r/g/b) |
| v2.11 | 16 mrt 2026 | `/set_home` endpoint voor Dashboard HOME/UIT broadcast |
| v2.10 | 15 mrt 2026 | Matter fixes + heap-optimalisatie (String → char[]), stabiele baseline |

### 5.11 ROOM UI — features (v2.12–v2.21)

#### Dot-cirkels in statuspagina

Binaire waarden worden getoond als gekleurde cirkels (`.dot` CSS class, 14×14px, border-radius 50%):

| Label | Kleur true | JSON key | Opmerking |
|---|---|---|---|
| Dauwpunt alarm | 🔴 `#c00` | `j` | |
| Verwarming aan | 🟠 `#e05c00` | `b` | |
| Hardware thermostaat | 🟢 `#2a9d2a` | `d` | |
| Thuis | 🟢 `#2a9d2a` | `v` | Live update + Apple Home thermostat synchroon |
| MOV1 | 🔴 `#c00` | `w > 0` | Beweging gedetecteerd, ongeacht licht |
| MOV2 | 🔴 `#c00` | `x > 0` | Beweging gedetecteerd, ongeacht licht |
| Pixel 0..N | ⬤ neo-kleur | `y`/`z`/`t` | Werkelijke neopixel RGB als lamp aan |
| Bed modus | 🟣 `#7b2fbe` | `p` | |
| Beam alert | 🔴 `#c00` | `ab` | |
| CO2 dot (naast vent slider) | 🔵 `#3aafe0` / grijs | `k > 0` | Lichtblauw = CO2 stuurt ventilatie, grijs = slider bepaalt |

**Verwijderd in v2.21:** "Verwarming AUTO" en "Ventilatie AUTO" dots — zie §5.14.

#### Verwarmings- en ventilatielogica (v2.21)

**Verwarming — altijd automatisch:**
```
Thuis (home_mode=1) + TSTAT aanwezig → volg hardware thermostaat pin
Weg (home_mode=0) of geen TSTAT     → setpoint vs kamertemp + dauwpuntbeveiliging
```
Anti-condensbeveiliging is altijd actief: `effective_setpoint = max(heating_setpoint, dew + margin)`

**Ventilatie:**
```
co2_enabled && co2 > 0  → vent_percent = map(co2, 400–800 ppm, 0–100%)  [CO2 dot lichtblauw]
Anders                  → vent_percent = slider-waarde                   [CO2 dot grijs]
Slider updatet automatisch mee met werkelijke vent_percent via JS
Default bij boot: vent_request_default (NVS, standaard 25%)
```

**Apple Home thermostat koppeling:**
```
Apple Home thermostat → UIT   = home_mode = 0 (Weg) + NVS opslaan
Apple Home thermostat → HEAT  = home_mode = 1 (Thuis) + NVS opslaan
```
Apple Home thermostat en "Thuis" toggle in webUI zijn volledig synchroon en persistent.

#### Licht-aan tijd slider (`light_on_min`)

```cpp
// NVS key: "light_on_min"  (int, 0–30, default 0)
inline unsigned long lightOnDuration() {
  return (unsigned long)light_on_min * 60000UL + 5000UL;
}
// Slider = 0 → 5s, Slider = 5 → 5 min + 5s
```
- Endpoint: `/set_light_on_min?mins=N`
- Positie in UI: direct onder MOV2, vóór NeoPixel kleurkiezer

### 5.12 Crash-analyse ROOM (12 april 2026)

Analyse op basis van Google Sheets log (4–12 april 2026, 8331 rijen).

**Twee niet-manuele crashes:**

| Crash | Tijdstempel | Uptime | heap_block op crashmoment | heap_min bereikt in run |
|---|---|---|---|---|
| 1 | 7 apr 21:41 | 83,2h (299.640s) | 35 KB ✅ | **2 KB** ⚠️ |
| 2 | 12 apr 08:02 | 50,1h (180.840s) | 34 KB ✅ | 10 KB ⚠️ |

**Conclusie:** geen directe OOM-crashes. heap_block was normaal op het crashmoment → waarschijnlijk **WDT-crash** door een geblokte taak tijdens een gefragmenteerd heap-moment.

**Gevaarlijke heap-episode (4 april 18:01, uptime 27.360s):**
- heap_block viel plots van 34 → **13 KB** en bleef 2 uur onder 25 KB
- heap_min bereikte **2 KB** — vrijwel nul
- Tegelijk triggerde de crashlog NVS feedback loop ~120× (elke 60s) → zie §2.8

**Patroon ~5h na boot:**
Runs 2 en 3 toonden consistent een heap_min daling naar 12–13 KB rond uptime 18.000–20.000s (~5h). Waarschijnlijk een intern Matter/WiFi-stack event (re-commissioning check of attribute sync timer).

**Openstaande fixes:**

| Fix | Prioriteit | Status |
|---|---|---|
| NVS crashlog feedback loop (§2.8) | Hoog | ⏳ Niet geïmplementeerd |
| CO2 `pulseIn()` blokkeert 400ms per 60s als sensor niet leest | Middel | ⏳ Niet geïmplementeerd |
| Matter update-interval verlagen van 5s naar 30s | Laag | ⏳ Niet geïmplementeerd |

**CO2 opmerking:** op de oude Photon-shield geeft de 5V pin slechts 4.3–4.4V → sensor leest niet, maar `co2_enabled = true` → `pulseIn()` blokkeert toch 400ms per 60s. Samen met `delay(750)` DS18B20: main loop is elke minuut ~1.150ms aaneengesloten geblokkeerd.

### 5.14 KISS-simplificatie verwarmings- en ventilatielogica (v2.21)

**Wat verwijderd is en waarom:**

`heating_mode` en `vent_mode` zijn verwijderd. Ze waren:
- **Niet NVS-persistent** → na reboot altijd terug op AUTO, instelling verloren
- **Zonder meerwaarde** → de setpoint-slider doet hetzelfde veiliger voor verwarming; de ventilatiesilider doet hetzelfde voor ventilatie
- **`heating_mode = MANUEEL`** negeerde het setpoint, de TSTAT én de dauwpuntbeveiliging — gevaarlijk
- **`vent_mode = AUTO`** gaf altijd 0% omdat CO2 niet werkte — nutteloos
- **Dot-cirkels updatenden niet live** omdat er geen JSON-key was — verwarrend

**Wat er voor in de plaats komt:**

Niets — de bestaande sliders en `home_mode` dekken alle use cases. Eenvoudiger, veiliger, volledig persistent.

**Matter `onChangeMode` — nieuwe koppeling:**

Apple Home thermostat heeft een UIT-knop. Vroeger: deed niets nuttig (`heating_mode = MANUEEL`). Nu:
```
Thermostat UIT  → home_mode = 0 (Weg) + NVS persistent
Thermostat HEAT → home_mode = 1 (Thuis) + NVS persistent
```
Apple Home thermostat en de "Thuis" toggle in de webUI zijn nu volledig synchroon.

**CO2 dot naast ventilatieslider:**
- 🔵 lichtblauw: CO2 > 0 → CO2 stuurt de ventilatiesnelheid automatisch
- ⚪ grijs: CO2 = 0 of uitgeschakeld → slider bepaalt
- Slider volgt automatisch de werkelijke `vent_percent` waarde (ook als CO2 die instelt)
- `document.activeElement` check: slider wordt niet overgeschreven terwijl je hem versleept

### 5.15 Openstaande punten ROOM

**Prioriteit 1 — Crashstabiliteit (v2.20 geflasht, 24u evaluatie lopende):**
- NVS crashlog feedback loop: max 1x per episode ✅ gedaan
- Matter interval 5s → 30s ✅ gedaan
- CO2 pulseIn timeout 200ms → 50ms ✅ gedaan

**Prioriteit 2 — AM312 integratie:**
- Controleer sketch-logica bij overgang van HC-SR501 naar AM312 (active HIGH vs LOW)

**Prioriteit 3 — Aparte tegels Apple Home:**
Thermostat naar EP9, losse `MatterTemperatureSensor` als EP1. Matter reset vereist. Referentie: `Oude_MATTER_ROOM_3mar.ino`.

**Prioriteit 4 — Dashboard matrix MOV-kolommen:**
Kolommen 6 en 7 in de ROOM matrix-rij gebruiken `y`/`z` (lamp aan). Aanpassen naar `w > 0` / `x > 0` (beweging ongeacht licht).

**Prioriteit 5 — Gedeeld CSS endpoint:**
Geschatte winst: ~2–3 KB minder fragmentatie per request.

---

## 6. Zarlar Dashboard — specifiek

### 6.1 Hardware

| Component | Detail |
|---|---|
| Board | ESP32-C6 **30-pin** clone (blote controller, MAC `A8:42:E3:4B:FA:BC`) |
| Voeding | Test: 5V USB-C / Productie: 5V via VIN (Zarlar shield, PTC 500 mA) |
| Static IP | 192.168.0.60 |
| WebServer | `WebServer` (blocking) — bewust, niet `AsyncWebServer` |
| Sketch | `ESP32_C6_Zarlar_Dashboard_MATTER_v5_0.ino` v5.0 |
| Statusmatrix | 16×16 WS2812B op IO4 via Pixel-line connector (shield R3=33Ω) |

⚠️ **30-pin vs 32-pin:** het Dashboard gebruikt een 30-pin board. De pinvolgorde verschilt van het 32-pin board — gebruik de pinout van het 30-pin board bij hardware-aanpassingen. IO14 ontbreekt op beide formfactors.

⚠️ **Matrix voeding:** de 16×16 WS2812B matrix (256 pixels) wordt rechtstreeks op 5V gevoed — **niet** via de PTC-zekering van het shield (max 500 mA). Bij volle helderheid kan de matrix >3A trekken. Aparte 5V voeding verplicht.

### 6.2 Wat het Dashboard doet

- Pollt alle actieve ESP32 controllers elke N minuten via HTTP GET `/json`
- POST data naar Google Sheets via Google Apps Script
- Serveert web-UI op `http://192.168.0.60/`
- Stuurt HOME/UIT broadcast naar alle actieve ROOM controllers
- Toont live systeemstatus op een **16×16 WS2812B statusmatrix** (zie §6.8)
- WiFi Strength Tester op `/wifi`
- Matter HOME/UIT toggle (1 endpoint: `MatterOnOffPlugin`)

### 6.3 NVS namespace: `zarlar`

| Key | Type | Inhoud |
|---|---|---|
| `wifi_ssid` | String | WiFi netwerknaam |
| `wifi_pass` | String | WiFi wachtwoord |
| `poll_min` | Int | Poll interval in minuten |
| `room_script` | String | Google Apps Script URL voor ROOM controllers |
| `home_global` | Bool | HOME/UIT toestand (persistent) |
| `c0_act`..`c21_act` | Bool | Controller actief/inactief |
| `c0_url`..`c21_url` | String | Google Sheets URL per S-controller |

### 6.4 Controller tabel — RSSI keys

| Controller | Type | RSSI key in /json |
|---|---|---|
| S-HVAC | TYPE_SYSTEM | `ac` |
| S-ECO | TYPE_SYSTEM | **`p`** ← afwijkend! |
| R-* (alle rooms) | TYPE_ROOM | `ac` |
| P-* (Photon) | TYPE_PHOTON | — (geen polling) |

### 6.5 Captive portal — OS-specifieke handlers

Voor correcte werking op alle platformen zijn expliciete handlers vereist:

```cpp
auto cpRedirect = []() {
  if (ap_mode) {
    server.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/settings");
    server.send(302, "text/plain", "");
  } else {
    server.send(404, "text/plain", "404");
  }
};
server.on("/hotspot-detect.html",       HTTP_GET, cpRedirect); // macOS/iOS Safari
server.on("/library/test/success.html", HTTP_GET, cpRedirect); // macOS ouder
server.on("/generate_204",              HTTP_GET, cpRedirect); // Android Chrome
server.on("/connecttest.txt",           HTTP_GET, cpRedirect); // Windows
server.onNotFound(cpRedirect);
```

### 6.6 Matter endpoint Dashboard

| # | Type | Variabele | Opmerking |
|---|---|---|---|
| EP1 | MatterOnOffPlugin | `home_mode_global` | HOME=aan, WEG=uit |

**Synchronisatie-punten:**
- Boot: `matter_home.setOnOff(home_mode_global)` na `Matter.begin()`
- Apple Home → Dashboard: callback schrijft naar NVS + roept `setAllRoomsHomeMode()` aan
- Dashboard webUI → Matter: `matter_home.setOnOff()` in `/set_home_global` handler
- Loop sync elke 5s: `matter_home.setOnOff(home_mode_global)` als veiligheidsnet

### 6.7 WiFi Strength Tester (`/wifi`)

Endpoints:
- `/wifi` — HTML pagina
- `/wifi_json` — `{"r":-62,"q":76}` (~22 bytes, `char buf` op stack, geen String)
- `/wifi_snap?start=1` — start TCP-ping + async WiFi scan
- `/wifi_snap` (poll) — geeft scanresultaat wanneer klaar

Heap-impact: 3 primitieven (`bool` + 2× `int` = 6 bytes permanent).

Scan-resultaat: alle netwerken behalve eigen SSID, max 4, gesorteerd sterkste eerst. ESP32-C6 is 2.4GHz-only → geen channel-filter.

### 6.8 Statusmatrix 16×16 WS2812B

De Zarlar Matrix is een **16×16 WS2812B LED-paneel** (256 pixels, serpentine adressering) gemonteerd in een houten kader met glazen voorkant. Een geprinte transparant bovenop het glas labelleert elke pixel.

#### Fysieke opstelling

| Component | Detail |
|---|---|
| Panel | 16×16 WS2812B, 160×160mm, serpentine adressering, kabelingang onderaan |
| Kader | Bleekhouten lasercut omlijsting met glazen voorkant |
| Achtergrond | Vergrijsd hout — matrix verzonken achter het glas |
| Transparant | A4 kleurenlaserprint op transparantfolie — labels boven elke pixel |
| Data | IO4 via Pixel-line connector (shield R3=33Ω aanwezig) |
| Voeding | 5V rechtstreeks op matrix — **niet** via shield PTC |
| Helderheid | Instelbaar via `/settings` UI (NVS key `matrix_br`, default 60) |

#### Rij-indeling

| Rij | Controller | Type | Kleur groep |
|-----|-----------|------|------------|
| 0 | S-HVAC | Systeem | Rood |
| 1 | S-ECO | Systeem | Oranje |
| 2 | S-OUTSIDE | Gereserveerd | Donkergrijs (nog niet actief) |
| 3 | S-ACCESS | Gereserveerd | Donkergrijs (nog niet actief) |
| 4 | — | Separator | Lichtblauw — "ROOMS" |
| 5 | R-BandB | Room | Blauw |
| 6 | R-BADK | Room | Blauw |
| 7 | R-INKOM | Room | Blauw |
| 8 | R-KEUKEN | Room | Blauw |
| 9 | R-WASPL | Room | Blauw |
| 10 | R-EETPL | Room | Blauw |
| 11 | R-ZITPL | Room | Blauw |
| 12–15 | — | Leeg | Donkergrijs |

#### Kolom-indeling HVAC (rij 0)

| Col | JSON key | Label | Kleurlogica |
|-----|----------|-------|------------|
| 0 | — | Status | Groen=online, rood=offline |
| 1–7 | `p`–`v` | C1–C7 (BB/WP/BK/ZP/EP/KK/IK) | Groen=verwarming aan, dim=uit |
| 8 | `y` | Pomp SCH | Cyaan=actief |
| 9 | `aa` | Pomp WON | Cyaan=actief |
| 10 | `x` | Ventilatie % | Cyaan gradient 0–100% |
| 11 | `h` | KSAv boiler gem. | Boilertemp kleurschaal |
| 12–13 | — | Gereserveerd | Dim |
| 14 | `ae` | Heap KB | Groen>35 / geel>25 / rood |
| 15 | `ac` | RSSI | Groen≥-60 / oranje / rood |

#### Kolom-indeling ECO (rij 1)

| Col | JSON key | Label | Kleurlogica |
|-----|----------|-------|------------|
| 0 | — | Status | Groen=online, rood=offline |
| 1 | `l` | Tsun | Collector temp — blauw→groen→oranje→rood |
| 2 | `m` | dT | Rendement (Tsun−Tboiler) — groen=goed |
| 3 | `b` | ETopH | Boiler laag 1 — boilerTempPx |
| 4 | `c` | ETopL | Boiler laag 2 — boilerTempPx |
| 5 | `d` | EMidH | Boiler laag 3 — boilerTempPx |
| 6 | `e` | EMidL | Boiler laag 4 — boilerTempPx |
| 7 | `f` | EBotH | Boiler laag 5 — boilerTempPx |
| 8 | `g` | EBotL | Boiler laag 6 — boilerTempPx |
| 9 | `h` | EAv | Boiler gemiddeld — boilerTempPx |
| 10 | `n` | PWM pomp | Cyaan gradient 0–255 |
| 11 | `k` | Yield vandaag | kWh — groen gradient |
| 12 | `i` | EQtot | Energie-inhoud — amber gradient |
| 13 | `j` | dEQ | Delta kWh — groen=laden, zwart=stilstand |
| 14 | `q` | FreeHeap% | Groen>35% / geel>20% / rood |
| 15 | `p` | RSSI ⚠️ | Key `p` — afwijkend t.o.v. andere controllers! |

#### Kolom-indeling ROOM (rijen 5–11)

| Col | JSON key | Label | Kleurlogica |
|-----|----------|-------|------------|
| 0 | — | Status | Groen=online, rood=offline |
| 1 | `v` | HOME | Blauw=HOME, dim=WEG |
| 2 | `b` | Verwarming | Rood=aan, dim=uit |
| 3 | `e` | Temp DHT22 | Blauw<18° / groen / geel / rood>26° |
| 4 | `h` | Vochtigheid | Groen<50% / geel / oranje / rood>85% |
| 5 | `k` | CO2 | Groen<800 / geel / oranje / rood≥1500 ppm |
| 6 | `y` | MOV1 lamp | Warm wit=lamp aan ⚠️ **update naar `w>0` gewenst** |
| 7 | `z` | MOV2 lamp | Warm wit=lamp aan ⚠️ **update naar `x>0` gewenst** |
| 8 | `d` | TSTAT | Rood=warmtevraag, dim=uit |
| 9 | `j` | Dauw alert | Rood=alert, dim blauw=OK |
| 10 | `q`/`r`/`s` | Kamerkleur | Werkelijke RGB van NeoPixels (geschaald) |
| 11 | `o` | Dag/Nacht | Geel=dag, donker purper=nacht |
| 12 | `m` | Licht LDR1 | Geel omgekeerd: fel=donker in kamer |
| 13 | `t` | Pixels aan | Wit, evenredig met aantal `1`s in pixel_on_str |
| 14 | `ae` | Heap KB | Groen>35 / geel>25 / rood |
| 15 | `ac` | RSSI | Groen≥-60 / oranje / rood |

#### Kleurschalen (gedeeld)

| Schaal | Functie | Kleuren |
|--------|---------|---------|
| `boilerTempPx()` | ECO boilertemperatuur | Blauw<40° → cyaan/groen 40–60° → oranje 60–75° → rood>75° |
| `tempPx()` | Kamertemperatuur | Blauw<18° → groen 18–22° → geel 22–26° → rood>26° |
| `rssiPx()` | WiFi signaalsterkte | Groen≥-60 → geelgroen≥-70 → oranje≥-80 → rood<-80 dBm |
| `heapPx()` | Heap largest block | Groen>35KB → geel 25–35KB → rood<25KB |
| `cyanLevel()` | Ventilatie / PWM | Donker cyaan (0%) → helder cyaan (100%) |
| `statusPx()` | Controller status | Groen=online · rood=offline · geel=pending · grijs=inactief |

#### Boot-animatie (v4.2+)

Bij elke herstart doorloopt de matrix een korte animatie:
1. **Rode pixel** met lichtgevend staartje loopt door rij 0 (HVAC) en rij 1 (ECO)
2. **Blauwe pixel** met staartje loopt door rijen 5→11 (alle zeven rooms)
3. Korte witte flash op alle actieve rijen → matrix wordt donker → live data start

#### Web-endpoints matrix

| Endpoint | Functie |
|----------|---------|
| `/matrix_bright?v=N` | Helderheid instellen (0–255) + NVS opslaan |
| `/matrix_test` | Testpatroon activeren (regenboog per rij) |
| `/matrix_update` | Forceer onmiddellijke live data refresh |

Serial commando's: `matrix-test`, `matrix-update`

#### Kolom-indeling ROOM via Photon fallback (rijen 5–11, tijdelijk)

| Col | Photon key | Label | Zelfde logica als ROOM |
|-----|-----------|-------|----------------------|
| 0 | — | Status | groen=online, rood=offline |
| 1 | — | zwart | HOME niet beschikbaar op Photon |
| 2 | `l` | TSTATon | Rood=verwarming aan |
| 3 | `g` | Temp DHT22 | tempPx() |
| 4 | `d` | Humi % | vochtlogica |
| 5 | `a` | CO2 ppm | CO2 kleurschaal |
| 6 | `i` | MOV1 | warm wit=beweging |
| 7 | `j` | MOV2 | warm wit=beweging |
| 8 | — | zwart | niet beschikbaar |
| 9 | `k` | DewAlert | rood/dim blauw |
| 10 | `s`/`t`/`u` | RGB pixels | werkelijke kleur (geschaald) |
| 11 | `q` | Night | geel=dag, purper=nacht |
| 12 | `e` | LDR licht 0–100 | geel omgekeerd |
| 13 | — | zwart | pixel_on_str niet beschikbaar |
| 14 | `x` | FreeMem% | groen/geel/rood |
| 15 | — | zwart | RSSI niet in worker response |

#### Automatische ESP32/Photon fallback (v5.0+)

```
1. ESP32-controller actief + json aanwezig  → renderRoomRow()   (definitief)
2. ESP32 inactief of geen data              → renderPhotonRow() (tijdelijk)
3. Geen enkele controller beschikbaar       → zwart
```

Dit wordt bestuurd via de `MatrixRowDef` struct:
```cpp
struct MatrixRowDef {
  int esp_idx;     // ESP32 R-controller idx (-1 = geen)
  int photon_idx;  // Photon P-controller idx (-1 = geen)
  int sys_idx;     // Systeem-controller (-1 = geen, -2 = separator)
};
```

**Transitie-workflow:** zodra een nieuwe ESP32-controller klaar is, activeer hem in `/settings` → de matrix schakelt automatisch om van Photon naar ESP32. **Geen reflash nodig.**

#### Huidige MROW-mapping (v5.0)

| Matrix rij | SVG label | ESP32 idx | Photon idx | Actief als |
|-----------|----------|-----------|-----------|-----------|
| 0 | S-HVAC | — | — | sys 0 |
| 1 | S-ECO | — | — | sys 1 |
| 2–3 | S-OUTSIDE/ACCESS | — | — | gereserveerd |
| 4 | separator | — | — | — |
| 5 | R-BandB | 6 | 14 | Photon offline → zwart |
| 6 | R-BADK | 7 | 15 | Photon P-Badkamer |
| 7 | R-INKOM | 8 | 16 | Photon P-Inkom |
| 8 | R-KEUKEN | 9 | 17 | Photon P-Keuken |
| 9 | R-WASPL | 10 | 18 | Photon P-Waspl |
| 10 | R-EETPL | 11 | 19 | **ESP32 actief** |
| 11 | R-ZITPL | 12 | 20 | leeg (P-Zitpl inactief) |
| 12–15 | leeg | — | — | — |

### 6.9 Openstaande punten Dashboard

- **OTA testen** — nog niet gedaan op Dashboard
- **Matter pairing** uitvoeren en testen met Apple Home
- **Heap-baseline** meten na Matter-activatie + matrix
- **Matrix kolommen 6/7 ROOM:** aanpassen van `y`/`z` (lamp aan) naar `w>0`/`x>0` (beweging ongeacht licht) — conform ROOM v2.19 UI

---

## 7. Schimmelbescherming via vochtigheid

Elke ROOM controller meet lokaal de luchtvochtigheid. Bij overschrijding van een drempelwaarde beslist de room controller autonoom om ventilatie te vragen via JSON key `g` (Vent_percent). De HVAC neemt de hoogste ventilatievraag van alle zones en stuurt de centrale ventilator aan. De volledige schimmelbeschermingslogica zit in de room controllers — de HVAC is enkel uitvoerder.

---

## 8. Bestanden

| Bestand | Beschrijving |
|---|---|
| `ESP32_C6_Zarlar_Dashboard_MATTER_v5_0.ino` | Dashboard v5.0 — Matrix + Photon fallback + Matter HOME/UIT |
| `Zarlar_Matrix_Labels_v5.svg` | Transparant A4 voor matrix — kleurlaser, houtfoto als achtergrond in Inkscape |
| `worker-status.js` | Cloudflare Worker v2.0 — `/sensor` endpoint voor Photon data |
| `ESP32_C6_MATTER_HVAC_v1.19.ino` | HVAC productieversie — huidig |
| `HVAC_GoogleScript_v4.gs` | GAS HVAC — 31 kolommen A–AE |
| `ESP32_C6_MATTER_ECO_v1.23.ino` | ECO productieversie — huidig |
| `ECO_GoogleScript.gs` | GAS ECO — 20 kolommen A–T |
| `ESP32-C6_MATTER_ROOM_13apr_v221.ino` | ROOM v2.21 — huidig productie |
| `ROOM_GoogleScript_v1_4.gs` | GAS ROOM — 37 kolommen A–AK |
| `Oude_MATTER_ROOM_3mar.ino` | Referentie: werkende Matter endpoint-volgorde (aparte tegels) |
| `partitions_16mb.csv` | Custom partitietabel voor alle vier controllers |
| `Zarlar_Master_Overnamedocument.md` | Dit document |

---

## 9. Instructies voor nieuwe sessie

1. **Upload** de actuele sketch als bijlage + dit document
2. **Vraag Claude** het document te lezen en samen te vatten vóór hij iets aanpast
3. **Eerst een plan** — Claude mag pas beginnen coderen na expliciete goedkeuring
4. **Heap-baseline** laten meten als eerste stap bij elke nieuwe functie
5. **Versie per versie** werken met testmoment ertussen
6. **Herinner Claude** bij aanvang:
   - `* /` met spatie in commentaar (geen `*/` in tekst)
   - Versieheader aanpassen bij elke wijziging
   - Bij JSON-structuurwijziging: alle consumers nalopen (HVAC, Zarlar, Google Script)
   - IO-pins altijd onmiddellijk aansturen — nooit wachten op pollcyclus
   - `#define Serial Serial0` alleen aanwezig als Matter effectief geïntegreerd is
   - ECO gebruikt RSSI key `p`, alle andere controllers gebruiken `ac`
   - Dashboard gebruikt `WebServer` (blocking), niet `AsyncWebServer`
   - Pairing code altijd in webUI tonen, niet alleen in Serial
   - **Nooit de volledige JS-block in één str_replace vervangen** — altijd chirurgisch
   - **`DOMContentLoaded` gebruiken** in plaats van `window.addEventListener('load')` voor inline scripts
   - **Geen nieuwe JSON-keys toevoegen** tenzij expliciete toestemming — bestaande consumers (Sheets, Dashboard, Matrix) breken stil
   - **Geen state-variabelen toevoegen die niet NVS-persistent zijn** — na reboot verloren, verwarrend voor gebruiker
   - **KISS:** als een feature vervangen kan worden door een bestaande slider of toggle, doe dat. Geen AUTO/MANUEEL lagen boven sliders die al volledig functioneel zijn.
   - PIR triggers direct herberekenen na `pushEvent()` via `countRecent()` in loop

---

*Zarlar project — Filip Delannoy — bijgewerkt 13 april 2026*
