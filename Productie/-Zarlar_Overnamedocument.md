# Zarlar Thuisautomatisering — Master Overnamedocument
**ESP32-C6 · Arduino IDE · Matter · Google Sheets**
*Filip Delannoy — Zarlardinge (BE) — bijgewerkt 23 maart 2026*

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
| **HVAC** | ESP32_HVAC.local | 192.168.0.70 | 58:8C:81:32:2B:90 | 32-pin clone | v1.19 | ✅ Productie, stabiel |
| **ECO Boiler** | ESP32_ECO Boiler | 192.168.0.71 | 58:8C:81:32:2B:D4 | 32-pin clone | v1.22 | ✅ Productie, stabiel |
| **ROOM / Eetplaats** | ESP32_EETPLAATS | 192.168.0.80 | 58:8C:81:32:2F:48 | 32-pin clone | v2.10 | ✅ Matter + heap stabiel |
| **Testroom** | ESP32_EETPLAATS | 192.168.0.80 | 58:8C:81:32:29:54 | 32-pin clone (experimenteerbord) | v2.10 | 🔄 Zelfde IP als EETPL |
| **Zarlar Dashboard** | ESP32_ZARLAR.local | 192.168.0.60 | A8:42:E3:4B:FA:BC | 32-pin clone | v3.0 | ✅ Matter HOME/UIT, WiFi tester |

**MAC-adressen** zijn enkel nodig voor identificatie in de client-lijst van de router — niet functioneel voor de werking van het systeem. Alle controllers gebruiken vaste IP-adressen ingesteld via de `/settings` pagina.

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

### 1.3 Partitietabel (identiek voor ALLE vier controllers)

Bestand: `partitions_16mb.csv` — plaatsen naast het `.ino` bestand in de schetsmap.

| Naam | Type | Offset | Grootte |
|------|------|--------|---------|
| nvs | data/nvs | 0x9000 | 20 KB |
| otadata | data/ota | 0xe000 | 8 KB |
| app0 | app/ota_0 | 0x10000 | 6 MB |
| app1 | app/ota_1 | 0x610000 | 6 MB |
| spiffs | data/spiffs | 0xC10000 | ~4 MB |

⚠️ **Nooit `huge_app` gebruiken** — had maar één app-slot en brak OTA. `app0` + `app1` elk 6 MB → OTA volledig werkend.

⚠️ **Nooit een 4MB controller gebruiken** — de `partitions_16mb.csv` past niet op 4MB flash en veroorzaakt een bootloop (`partition size 0x600000 exceeds flash chip size 0x400000`). Alle Zarlar-controllers gebruiken uitsluitend **16MB** boards.

### 1.5 ESP32-C6 hardware — modules en boards

#### Controller-module

Alle Zarlar-controllers draaien op de **ESP32-C6-WROOM-1N16** module (Espressif), met:
- Ingebouwde PCB-antenne (standaard)
- 16 MB flash
- Wi-Fi 6 (2.4 GHz), BLE 5, Zigbee/Thread (IEEE 802.15.4)
- 3.3V IO (niet 5V-tolerant)

Voor locaties met zwakke WiFi-verbinding: **ESP32-C6-WROOM-1U** (identieke module, maar met U.FL/IPEX-connector voor externe antenne). Bereik interne antenne: ~20–50m indoors, ~80–200m buiten. Externe antenne: 2–3× beter indoors.

#### Dev boards — gestandaardiseerd op 32-pin

Alle Zarlar-controllers die op een shield worden gemonteerd gebruiken het **32-pin clone board** (AliExpress, €2.52/stuk). Het shield is ontworpen voor de 32-pin footprint.

| Type | Pins | Prijs | Gebruik |
|---|---|---|---|
| 32-pin clone | 32 | €2.52/stuk (AliExpress, 25 dec 2025, 10 stuks €65) | **Alle controllers op shield** |
| 30-pin clone | 30 | €9/stuk (3 stuks, €27) | Enkel voor projecten zonder shield (bijv. losse testopstellingen) |

⚠️ **Het Zarlar shield v2.0 is NIET compatibel met het 30-pin board** — andere pinvolgorde en footprint. Gebruik uitsluitend 32-pin boards op het shield.

⚠️ **Clone-boards:** de AliExpress-boards zijn clones — niet officieel Espressif. Ze werken identiek maar hebben soms afwijkende pinlabels t.o.v. de officiële Espressif DevKitC-1. Gebruik altijd de pinout uit §3.1 / §5.1 van dit document, niet de opdruk op het board.

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
| IO1 | LDR1 analog (⚠️ spanningsdeler 10k pull-up naar 3V3!) |
| IO2 | LDR2 analog (beam) (⚠️ zelfde spanningsdeler nodig als LDR1!) |
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

Momenteel geen IO-pinnen in gebruik buiten WiFi en USB. Het board wordt enkel gebruikt als netwerk-aggregator en Matter-endpoint.

**Toekomstig idee:** een NeoPixel-matrix toevoegen aan het Dashboard om de status van alle controllers visueel weer te geven (kleur per controller = online/offline/alarm, of een live heatmap van de verwarmingskringen). Dit past goed bij de centrale rol van het Dashboard en maakt de systeemstatus zichtbaar zonder een scherm of app. Te documenteren in §6 zodra dit uitgewerkt wordt.

### 1.7 Shield — connectoroverzicht

Overzicht van alle aansluitingen die het Zarlar-shield moet voorzien per controller. Aansluitingen gemarkeerd met ✅ zijn vereist, 〇 zijn optioneel, — niet van toepassing.

| Connector | Pins | Voeding | ROOM | HVAC | ECO | Dashboard | Opmerking |
|---|---|---|---|---|---|---|---|
| **Roomsense** (RJ45) | 8 | 3V3 + GND | ✅ | — | ✅ | — | ROOM: DHT22, MOV1, Dust, LDR1 / ECO: IO1=relay, IO5=PWM pomp |
| **OPTION** (RJ45) | 8 | 5V + 3V3 + GND | ✅ | — | — | — | MOV2 PIR, CO2 PWM, TSTAT, LDR2/beam |
| **T-BUS** (3-pin) | 3 | 3V3 + GND | ✅ | ✅ | ✅ | — | DS18B20 OneWire — één bus, meerdere sensoren |
| **Pixel-line** (3-pin) | 3 | 5V + GND | ✅ | — | — | 〇 | NeoPixel data + 5V voeding |
| **I2C** (5-pin Qwiic) | 5 | VCC_5V + 3V3 + GND | ✅ | ✅ | — | — | SDA + SCL, 4.7k pull-ups naar 3.3V |
| **SPI** (4-pin header) | 6 | 3V3 + GND | — | — | ✅ | — | IO20-23: CS, MOSI, MISO, SCK + 3V3 + GND |
| **UART** (4-pin) | 4 | 3V3 + GND | 〇 | — | — | — | TX, RX + 3V3 + GND |

#### Voltage-specificaties per connector

| Connector | Voedings-pin | Signaal-niveau | Opmerking |
|---|---|---|---|
| Roomsense | 3V3 | 3.3V | ROOM: PIR op 3.3V (beweging=LOW), DHT22 pull-up. ECO: zelfde connector — IO1=relay (actief-laag), IO5=PWM pomp. |
| OPTION | 5V (VIN) + 3V3 | 3.3V | MH-Z19 CO2 heeft **5V voedingspin** nodig; PWM-signaal is 3.3V. |
| T-BUS | 3V3 | 3.3V | DS18B20 werkt op 3.3V met 4.7k pull-up naar 3.3V. |
| Pixel-line | **5V (VIN)** | 3.3V | NeoPixels (WS2812) vereisen 5V voeding; 3.3V data-signaal werkt. |
| I2C | 3V3 | 3.3V | 4.7k pull-ups naar **3.3V**. TSL2561 (ROOM): 4-pin header zodat VCC_5V onbereikbaar is. |
| SPI | 3V3 + GND | 3.3V | MAX31865 heeft ingebouwde 3.3V LDO + level shifter — direct compatibel. |

#### Toekomstige uitbreiding Dashboard

Het Dashboard-shield heeft momenteel geen IO-aansluitingen. Een **NeoPixel-matrix** is voorzien als toekomstige toevoeging: visuele statusweergave van alle controllers (kleur per controller = online/offline/alarm, of heatmap verwarmingskringen). Hiervoor is een Pixel-line aansluiting (5V + GND + data IO4) te voorzien op het Dashboard-shield.



| Instelling | Waarde |
|---|---|
| Board | ESP32C6 Dev Module |
| Flash Size | **16 MB** |
| Partition Scheme | Custom → `partitions_16mb.csv` uit schetsmap |
| USB CDC On Boot | **Enabled** (verplicht voor Serial over USB-C) |
| Upload (eerste keer) | USB |
| Upload (daarna) | OTA via Arduino IDE → Sketch → Upload via OTA |

---

### 1.8 Shield v1.0 — ontwerp en review

**ESP32-C6 shield v1.0** — ontwerp iTroniX / FiDel, 30 dec 2025.

#### Connectors

| Connector | Type | Voeding | Signalen |
|---|---|---|---|
| **Roomsense** | RJ45 | 3V3 + GND | ROOM: DS18B20, DHT22, DUST-LED, DUST-ANA, LDR1, MOV1, T-BUS / ECO: IO1=relay, IO5=PWM |
| **Option** | RJ45 | 5V + 3V3 + GND | TSTAT, MOV2, CO2, LDR2, Pixels |
| **Pixel-line** | 3-pin header | 5V + GND | PIXELS (IO4) |
| **I2C-2** | 5-pin JST SH (Qwiic) | VCC_5V + 3V3 + GND | SCL (IO11), SDA (IO13) |
| **T-BUS** | 3-pin header | 3V3 + GND | DO (IO3) |
| **Serial** | 4-pin header | 3V3 + GND | TX, RX |
| **SPI ECO** | 4-pin header + 3V3 + GND | 3V3 + GND | IO20 CS, IO21 MOSI, IO22 MISO, IO23 SCK |
| **Power IN** | 2-pin | 6–23V input | Via regelaar naar 5V |

#### Passieve componenten

| Ref | Waarde | Functie |
|---|---|---|
| PTC | 500 mA | Resetbare zekering op VCC_5V |
| PTC | 500 mA | Resetbare zekering op 3V3 — beschermt ESP32-C6 bij kortsluiting op sensoren |

**PTC 500 mA — voldoende voor alle aangesloten lasten:** de PIXEL-LINE connector voert enkel het 3.3V datasignaal naar de **PowerPixels** (eigen ontwerp Filip Delannoy) — de zware verlichtingscircuits worden aangestuurd via optocouplers en MOSFETs op aparte voedingscircuits, volledig gescheiden van het shield. Het shield draagt dus geen verlichtingsstromen.
| U1 | 6–23V → 5V | Spanningsregelaar (knipbaar als externe 5V gebruikt wordt) |
| C2 | 10 µF 25V X5R | Bulk afvlakking VCC_5V |
| C4 | 0.1 µF 50V X7R | Ontkoppeling |
| C5 | 10 µF 25V X5R | Bulk afvlakking 3V3 |
| C1, C6, C7 | 0.1 µF 50V X7R | Ontkoppeling per rail |
| R1, R2 | 4k7 | I2C pull-ups → 3V3 |
| R3 | **33 Ω** | Serieregelaar PIXELS data-lijn (IO4) |
| R4 | 499 Ω | Serieregelaar UART TX-lijn |

#### Controller-footprint en voeding

Het shield gebruikt de **ESP32-C6-DEVKITC-1-N8** 32-pin footprint. De AliExpress clone-boards passen hierop. Het 30-pin Dashboard-board past **niet** op dit shield.

Voeding: 6–23V op Power IN → ingebouwde regelaar → 5V. De regelaar-jumper is knipbaar bij gebruik van externe 5V. PTC zekering 500 mA op VCC_5V.

#### I2C connector — TSL2561 aansluiting

De I2C connector heeft VCC_5V op pin 1. De TSL2561 lux-sensor (ROOM) heeft max 3.6V en zou bij 5V beschadigen. **Oplossing:** een 4-pin header op pins 2–5 maakt pin 1 (VCC_5V) fysiek onbereikbaar. Elegante hardwareoplossing zonder risico.

| Module | Max VCC | Aansluiting |
|---|---|---|
| TSL2561 (lux, ROOM) | 3.6V | 4-pin header pins 2–5 — VCC_5V onbereikbaar |
| MCP23017 (HVAC I/O expander) | 5.5V | Volledige 5-pin header mogelijk |

#### ECO Boiler aansluitingen

De ECO Boiler gebruikt twee bestaande connectors op het shield:

**SPI — 4-pin header (IO20–IO23) + 3V3 + GND:**

| Pin | Label | ECO-functie |
|---|---|---|
| IO20 | IO (CS) | SPI_CS → MAX31865 |
| IO21 | IO (MOSI) | SPI_MOSI → MAX31865 |
| IO22 | IO (MISO) | SPI_MISO → MAX31865 |
| IO23 | IO (SCK) | SPI_SCK → MAX31865 |

De 4-pin header heeft bijkomend 3V3 en GND voor de voeding van de MAX31865 module.

**Relay + PWM — via ROOMSENSE RJ45 (IO1 + IO5):**

Geen aparte connector nodig — dezelfde ROOMSENSE RJ45 wordt hergebruikt, maar IO1 en IO5 krijgen een andere functie bij de ECO-controller:

| Pin | ROOM-gebruik | ECO-gebruik |
|---|---|---|
| IO1 | LDR1 analog | Pomprelais (actief-laag: LOW = AAN) |
| IO5 | MOV1 PIR | PWM circulatiepomp (1 kHz, 8-bit) |

⚠️ Vrijloop-diode verplicht op de relay coil. IO5 PWM vereist externe driver voor de pomp.

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

Dit is **bewuste duplicatie** — niet refactoren naar een centrale `applyRelay()` functie tenzij er een vierde pad bijkomt. Elk pad is onafhankelijk leesbaar. In embedded code is dat een voordeel: bij een probleem weet je exact waar te kijken zonder andere functies op te zoeken.

⚠️ **Valkuil vóór v1.19:** de Matter `onChangeOnOff` callback zette enkel de override-vlag maar deed **geen** `mcp.digitalWrite()`. Gevolg: relais reageerden pas na de volgende `pollRooms()` cyclus (~10-60s vertraging) bij bediening vanuit Apple Home. Via de webUI werkte het al onmiddellijk. Fix: `circuits[i].heating_on = on_off` + `mcp.digitalWrite()` toegevoegd in de callback.

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
| IO20 | `VENT_FAN_PIN 20` | PWM ventilator | `ledcAttach(pin, 1000, 8)` — 3.3V PWM → externe OPAMP → 10V (vereist door Begetube ventilatie) |

#### MCP23017 poortindeling

| MCP pin | pinMode | Functie |
|---|---|---|
| 0–6 | OUTPUT | Relay circuits 1–7 (actief-laag: LOW = AAN) |
| 7 | INPUT_PULLUP | Pomp feedback — actief gebruikt: vergelijkt verwachte vs. werkelijke pompstatus, ALERT in Serial bij afwijking |
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

### 3.6 Openstaande punten HVAC

- **kWh-berekening**: echte `Q = m × Cp × ΔT / 3600` per pompbeurt implementeren
- **HTML compressie**: zelfde aanpak als ROOM v3.1 — witte pagina op iPhone bij ventilatieslider wijst op heap-krapte bij page reload

---

## 4. ECO Boiler Controller — specifiek

### 4.1 Hardware

| Component | Detail |
|---|---|
| Board | ESP32-C6 32-pin clone (MAC `58:8C:81:32:2B:D4`) |
| Voeding | Test: 5V USB-C / Productie: 5V via VIN (Zarlar shield, PTC 500 mA) |
| Static IP | 192.168.0.71 |
| Temperatuursensoren | 6× DS18B20 op OneWire (IO3) — 2 per boilerlaag: Top/Mid/Bot × H/L |
| PT-sensor (collector) | MAX31865 SPI-module (CS=IO20, MOSI=IO21, MISO=IO22, SCK=IO23) — **instelbaar PT100 of PT1000** |
| Pomprelais | IO1 — digitaal aan/uit (actief-laag) |
| Circulatiepomp | PWM op IO5 (0–255), freq 1 kHz, 8-bit resolutie |

#### PT-sensor — type en configuratie

De ECO sketch ondersteunt zowel PT100 als PT1000 via de `/settings` UI:

| Setting | RREF | RNOMINAL | Gebruik |
|---|---|---|---|
| **PT100** (default) | 430 Ω | 100 Ω | Testbord / proefopstelling |
| **PT1000** | 4000 Ω | 1000 Ω | Dakcollector productie |

- Omschakelen via `/settings` → "PT-sensor type" → herstart vereist
- Temperatuurberekening via `pt1000.temperature(RNOMINAL, RREF)` — Callendar-Van Dusen polynoom (nauwkeuriger dan lineaire formule)
- Altijd **2-wire** configuratie (`MAX31865_2WIRE`) — voldoende nauwkeurig voor pompsturing (±0.5–2°C fout aanvaardbaar bij ΔT-drempel van 3°C)

#### PT100 sensor — identificatie

⚠️ **Meet altijd de weerstand bij kamertemperatuur voor aansluiting:**
- ~108 Ω bij 20°C → **PT100** ✅
- ~1078 Ω bij 20°C → PT1000

#### MAX31865 module — soldeerbruggen (paarse Chinese clone)

| Brug | Status | Betekenis |
|---|---|---|
| "2/3 Wire" links | **Gesloten** | 3-wire modus (module-instelling) |
| "2 Wire" rechts | Open | 2-wire niet actief op module |
| "24/3" druppeltjes | **Onaangeroerd** | Rref selectie via weerstand |
| Rref weerstand | **427 Ω ≈ 430 Ω** | Correct voor PT100 ✅ |

⚠️ Ondanks "2/3 Wire" gesloten op de module gebruikt de sketch `MAX31865_2WIRE` — dit is correct en bewust. De module-jumper en de sketch-instelling zijn onafhankelijk.

#### Relaismodule — voeding (duaal relaisblok met optocoupler)

| Pin module | Aansluiting | Reden |
|---|---|---|
| JD-VCC | 5V | Relaisspoel (Tongling JQC-3FF is 5V DC coil) |
| VCC | 3V3 | Optocoupler-kant — zodat IO1 (3.3V) de ingang kan sturen |
| GND | GND | Gemeenschappelijk |
| IN1 | IO1 | Stuursignaal (actief-laag: LOW = relais AAN) |

⚠️ **JD-VCC jumper verwijderen** — scheidt relay-spoel (5V) van optocoupler-kant (3V3) galvanisch.

### 4.2 Testopstelling op shield v1.0

De ECO controller kan getest worden op het **Zarlar shield v1.0** (zonder SMD-componenten):

**Minimale SMD-bestukking voor testopstelling:**

| Component | Monteren | Reden |
|---|---|---|
| PTC×2 (500mA) | ✅ | Beveiliging voedingslijnen |
| C2, C5 (10µF) | ✅ | Bulk afvlakking |
| C4 (0.1µF) | ✅ | Ontkoppeling |
| Rest (R1–R5, C1/C6/C7, LED1) | ❌ | Niet nodig voor ECO |

**Aansluitingen testopstelling (manueel bedraden):**

| Connector | Pins | Functie |
|---|---|---|
| ROOMSENSE RJ45 | IO1 + GND + 5V | Relaismodule (JD-VCC=5V, VCC=3V3, GND, IN1=IO1) |
| ROOMSENSE RJ45 | IO5 + GND | PWM circulatiepomp |
| T-BUS 3-pin | IO3 + 3V3 + GND | DS18B20 sensoren |
| SPI 4-pin header | IO20–23 + 3V3 + GND | MAX31865 PT-sensor (6-aderig, 3V3-uitgang module niet verbinden!) |

### 4.3 ECO /json output (keys a..s → naar Zarlar → Google Sheets)

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

### 4.4 Versiehistorie ECO (recente wijzigingen)

| Versie | Wijziging |
|---|---|
| v1.23 | PT-sensor type instelbaar via UI: PT100 (testbord) / PT1000 (dakcollector). `readPT1000()` gebruikt Adafruit `pt1000.temperature()` — Callendar-Van Dusen. 2-wire gestandaardiseerd. |
| v1.22 | Matter geïntegreerd, heap-monitoring, crash-log NVS, chunked streaming UI |
| v1.21 | Google Sheets logging naar Dashboard gedelegeerd |

### 4.5 Openstaande punten ECO

- **PWM output LED**: visuele indicator voor pompsnelheid — eerst IO5 stroom meten voor aansluiting (pompdriver type onbekend, mogelijk externe driver nodig)
- **kWh-berekening**: echte `Q = m × Cp × ΔT / 3600` per pompbeurt
- **Heap-baseline v1.23**: meten na Matter-activatie op testopstelling

---


### 5.1 Hardware

| Component | Detail |
|---|---|
| Board | ESP32-C6 32-pin clone (MAC `58:8C:81:32:2F:48` productie / `58:8C:81:32:29:54` experimenteerbord) |
| Voeding | Test: 5V USB-C / Productie: 5V via VIN (Zarlar shield, PTC 500 mA) |
| Static IP | 192.168.0.80 |
| Sensoren | DHT22 (temp+vocht), DS18B20 (OneWire), MH-Z19 (CO2), Sharp GP2Y (dust), TSL2561 (lux), LDR, PT1000 |
| Actuatoren | NeoPixel strip (tot 30 pixels), PIR×2, laserbeam LDR |
| Verwarming | TSTAT output + setpoint |
| Shield | PhotoniX-compatible Roomsense connector (RJ45) |

#### Pinout ROOM (Photon → ESP32-C6 conversie)

| ESP32-C6 Pin | `#define` | Photon | Functie | Opmerking |
|---|---|---|---|---|
| IO1 | `LDR_ANALOG 1` | A3 | LDR1 analog | ⚠️ 10k spanningsdeler (pull-up IO1→3V3) op shield! |
| IO2 | `OPTION_LDR 2` | A7 | LDR2 analog (beam/MOV2) | ⚠️ Zelfde spanningsdeler nodig als LDR1! |
| IO3 | `ONE_WIRE_PIN 3` | D3 | DS18B20 OneWire | 3.3V pull-up |
| IO4 | `NEOPIXEL_PIN 4` | D4 | NeoPixels data | NEO_GRB + NEO_KHZ800 |
| IO5 | `PIR_MOV1 5` | D5 | MOV1 PIR | INPUT_PULLUP — beweging = LOW |
| IO6 | `DHT_PIN 6` | D6 | DHT22 data | 3.3V pull-up |
| IO7 | `SHARP_ANALOG 7` | A2 | Sharp dust analog (RX) | Voltage divider indien >3.3V |
| IO10 | `TSTAT_PIN 10` | A6 | TSTAT switch (GND = AAN) | INPUT_PULLUP |
| IO11 | — | D1 | I2C SCL → TSL2561 | `Wire.begin(13, 11)` — 4.7k pull-up naar 5V |
| IO12 | `SHARP_LED 12` | D7 | Sharp dust LED (TX) | OUTPUT, HIGH = uit |
| IO13 | — | D0 | I2C SDA → TSL2561 | 4.7k pull-up naar 5V |
| IO18 | `CO2_PWM 18` | A4 | CO2 PWM input (MH-Z19) | ⚠️ MH-Z19 heeft 5V voedingspin nodig! |
| IO19 | `PIR_MOV2 19` | A5 | MOV2 PIR | INPUT_PULLUP — beweging = LOW |

⚠️ **Beide LDRs (IO1 en IO2) hebben een spanningsdeler nodig** — een 10k pull-up naar 3V3 vormt een spanningsdeler met de LDR-weerstand zodat de analoge waarde correct geschaald wordt naar 0–3.3V. IO1 heeft deze al op de Roomsense-shield; IO2 (OPTION-connector) heeft dezelfde schakeling nodig.

⚠️ **MH-Z19 CO2-sensor** heeft aparte 5V voedingspin — PWM-signaal zelf is 3.3V compatibel.

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

### 5.4 Heap-baseline (v2.10)

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

### 5.6 Matter endpoints (v2.10, werkend)

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
In de werkende v2.1 (3 maart 2026) stond de thermostat op positie 7 ná 6 sensor-endpoints → Apple Home bood splitsen aan. In v2.10 staat de thermostat op EP1 → geen splitsen.

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
| `w`..`x` | Y..Z | MOV1/MOV2 /min | /min |
| `y`..`z` | AA..AB | MOV1/MOV2 light | 0/1 |
| `aa` | AC | BEAMvalue | 0–100 |
| `ab` | AD | BEAMalert | 0/1 |
| `ac` | AE | RSSI | dBm |
| `ad` | AF | FreeHeap% | % |
| `ae` | AG | LargestBlock | KB |
| `af` | AH | MinFreeHeap | KB |
| `ag` | AI | ds_count | — |
| `ah` | AJ | Tds2 | °C |
| `ai` | AK | Tds3 | °C |

### 5.10 Openstaande punten ROOM

**Prioriteit 1 — Aparte tegels:**
Thermostat naar EP9, losse `MatterTemperatureSensor` als EP1. Matter reset vereist. Referentie: `Oude_MATTER_ROOM_3mar.ino`.

**Prioriteit 2 — Gedeeld CSS endpoint:**
```cpp
server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
  AsyncWebServerResponse *r = request->beginResponse(200, "text/css", gedeelde_css);
  r->addHeader("Cache-Control", "public, max-age=86400");
  request->send(r);
});
```
Elke pagina vervangt `<style>...</style>` door `<link rel="stylesheet" href="/style.css">`. Geschatte winst: ~2–3 KB minder fragmentatie per request.

---

## 6. Zarlar Dashboard — specifiek

### 6.1 Hardware

| Component | Detail |
|---|---|
| Board | ESP32-C6 **32-pin** clone (MAC `A8:42:E3:4B:FA:BC`) |
| Voeding | Test: 5V USB-C / Productie: 5V via VIN (Zarlar shield, PTC 500 mA) |
| Static IP | 192.168.0.60 |
| WebServer | `WebServer` (blocking) — bewust, niet `AsyncWebServer` |
| Sketch | `ESP32_C6_Zarlar_Dashboard_17mar_wifi.ino` v3.0 |

### 6.2 Wat het Dashboard doet

- Pollt alle actieve ESP32 controllers elke N minuten via HTTP GET `/json`
- POST data naar Google Sheets via Google Apps Script
- Serveert web-UI op `http://192.168.0.60/`
- Stuurt HOME/UIT broadcast naar alle actieve ROOM controllers
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

### 6.8 Openstaande punten Dashboard

- **Heap-baseline meten** na Matter-activatie (eerste flash op 16MB controller)
- **OTA testen** — nog niet gedaan op Dashboard
- **Matter pairing** uitvoeren en testen met Apple Home

---

## 7. PowerPixels — ControlpiXel systeem

### 7.1 Concept

**PowerPixels** zijn een eigen ontwerp van iTroniX / Filip Delannoy. Ze combineren de eenvoudige NeoPixel-aansturing (WS2812B protocol, 3.3V datasignaal) met zware verlichtingscircuits via galvanische scheiding.

```
ESP32-C6 IO4 (3.3V)
        │
        ▼
  NeoPixel chip (WS2812B)
        │
   Optocouplers
        │
   MOSFET gates
        │
   3–24V verlichtingscircuits (aparte voeding, aparte GND)
```

De ESP32-C6 en het shield dragen **geen verlichtingsstromen** — enkel het 3.3V datasignaal gaat via de PIXEL-LINE connector. De zware stromen lopen volledig gescheiden op eigen voedingscircuits.

### 7.2 ControlpiXel TRIO v1.0

| Kenmerk | Detail |
|---|---|
| Ontwerp | iTroniX / FiDel — v1.0, 1 okt 2020 |
| Kanalen | 3 — R, G, B (afzonderlijk aanstuurbaar) |
| Protocol | NeoPixel / WS2812B (daisy-chain via DI → DO) |
| Voedingsspanning | 3–24V op aparte PGND |
| Scheiding | Optocouplers → MOSFET gates |
| Extra | PWM-OUT beschikbaar als bijkomende uitgang |

### 7.3 Koppeling met ROOM-controller sketch

De ROOM-sketch gebruikt de standaard `Adafruit_NeoPixel` library. De ControlpiXel TRIO ziet er vanuit de sketch identiek uit als een gewone NeoPixel-strip.

**Configuratie:**

| NVS-sleutel | Default | Beschrijving |
|---|---|---|
| `pixels_num` | 8 | Aantal PowerPixels (1–30, instelbaar via `/settings`) |
| `pixel_nick_N` | — | Nickname per pixel (bijv. "Bureaulamp", "Spots") |
| `pixel_on_N` | false | Aan/uit toestand per pixel (persistent in NVS) |
| `pixel_user_on_N` | false | Manuele intentie per pixel |
| `pixel_mode_0` | 0 | Pixel 0 modus: 0 = AUTO (PIR MOV1), 1 = MANUEEL |
| `pixel_mode_1` | 0 | Pixel 1 modus: 0 = AUTO (PIR MOV2), 1 = MANUEEL |

**Pixellogica:**

| Pixel | Type | Gedrag |
|---|---|---|
| 0 | MOV-pixel | AUTO: aan bij beweging MOV1 + donker. MANUEEL: altijd aan (vaste kleur) |
| 1 | MOV-pixel | AUTO: aan bij beweging MOV2 + donker (enkel als `mov2_enabled`). MANUEEL: altijd aan |
| 2 … pixels_num-1 | Normale pixels | Aan/uit via webUI, Matter of NVS |

**Boot-gedrag:** pixels buiten `pixels_num` worden bij boot expliciet gewist (`updateLength(30)` + `clear()` + `show()`), daarna teruggeschaald naar `pixels_num`. Voorkomt spookpixels op een langere fysieke strip.

### 7.4 Matter-endpoints voor pixels (ROOM v2.10)

| Endpoint | Type | Wat het aanstuurt |
|---|---|---|
| EP5 | `MatterColorLight` | Globale kleur (R/G/B) voor alle pixels — on/off altijd `true` |
| EP6 | `MatterOnOffLight` | SW1: pixel 0 MOV-override (MANUEEL/AUTO) |
| EP7 | `MatterOnOffLight` | SW2: pixel 1 aan/uit |
| EP8 | `MatterOnOffLight` | SW3: pixels 2…pixels_num-1 samen aan/uit |

⚠️ **`MatterColorLight` niet `MatterEnhancedColorLight`** — de Enhanced variant blokkeert de "aparte tegels" optie in Apple Home.

⚠️ **Color light on/off wordt genegeerd** — altijd op `true` gehouden. Schakelen gebeurt via de drie `MatterOnOffLight` endpoints.

### 7.5 Toekomstig: PowerPixels op Dashboard

Een NeoPixel-matrix (of ControlpiXel) is voorzien als toekomstige uitbreiding op het Zarlar Dashboard-shield. Doel: visuele statusweergave van alle controllers (kleur per controller = online/offline/alarm, of live heatmap van verwarmingskringen). Hiervoor is een PIXEL-LINE aansluiting (5V + GND + data IO4) te voorzien op het Dashboard-shield.

---

## 8. Schimmelbescherming via vochtigheid

Elke ROOM controller meet lokaal de luchtvochtigheid. Bij overschrijding van een drempelwaarde beslist de room controller autonoom om ventilatie te vragen via JSON key `g` (Vent_percent). De HVAC neemt de hoogste ventilatievraag van alle zones en stuurt de centrale ventilator aan. De volledige schimmelbeschermingslogica zit in de room controllers — de HVAC is enkel uitvoerder.

---

## 9. Shield fabricage — JLCPCB workflow

### 9.1 Software vereisten

| Tool | Versie | OS | Opmerking |
|---|---|---|---|
| **Eagle CAD** | **9.6.2** | macOS Catalina 10.15+ | ⚠️ Eagle 7.7.0 crasht bij CAM export op macOS 10.15+ — altijd 9.6.2 gebruiken |
| Eagle 9.6.2 download | — | — | https://eagle-updates.circuits.io/downloads/latest.html |

⚠️ **Eagle 7.7.0 is onbruikbaar voor Gerber-export op moderne macOS** — crasht met `EXC_BAD_ACCESS (SIGSEGV)` bij Process Job, zowel op Apple Silicon (Rosetta) als native Intel. Dit is een bug in Eagle 7.7.0 zelf, niet een OS-probleem. Eagle 9.6.2 werkt correct op Catalina ondanks de verouderde systeemvereistenlijst in de installer.

### 9.2 CAM files — juiste versie per Eagle versie

| Eagle versie | CAM file | Download |
|---|---|---|
| 9.6.2 | `jlcpcb_2_layer_v9.cam` | https://raw.githubusercontent.com/JLCPCBofficial/jlcpcb-eagle/master/cam/jlcpcb_2_layer_v9.cam |
| 7.2–8.5.2 | `jlcpcb_2_layer_v72.cam` | https://raw.githubusercontent.com/JLCPCBofficial/jlcpcb-eagle/master/cam/jlcpcb_2_layer_v72.cam |

Sla de CAM file op in `~/Documents/EAGLE/cam/` — dan verschijnt hij automatisch in het Control Panel onder CAM Jobs.

### 9.3 Stap 1 — Ground plane genereren (ratsnest)

De ground plane in Eagle is een "polygon pour" die **niet bewaard wordt** in het .brd bestand. Dit moet altijd opnieuw gedaan worden vlak vóór de CAM export.

1. Open het `.brd` bestand in Eagle — Board venster actief
2. Typ in de commandolijn onderaan: `ratsnest` → Enter
3. De ground plane verschijnt als gevulde zones
4. Controleer visueel: geen onverwachte isolaties of eilanden?
5. Resetten indien nodig: `ripup @;` → Enter → dan opnieuw `ratsnest`

⚠️ **Sla het .brd bestand NIET op na ratsnest** — anders bewaar je de gevulde polygons en kunnen toekomstige edits problemen geven.

### 9.4 Stap 2 — Gerber files genereren

1. Open de CAM Processor via het icoon bovenaan in het Board venster
2. Laad de JLCPCB job: in Eagle 9.x via de **Load** knop of **Open Job** bovenaan het CAM Processor venster → selecteer `jlcpcb_2_layer_v9.cam`
3. De secties verschijnen als tabs: Board Outline, Drill File, Top Copper Layer, etc.
4. Klik **Process Job** → alle bestanden worden gegenereerd in één ZIP

**Gegenereerde bestanden (11 stuks voor 2-laags board):**

| Extensie | Inhoud |
|---|---|
| `.GTL` | Top copper |
| `.GBL` | Bottom copper |
| `.GTS` | Top soldermask |
| `.GBS` | Bottom soldermask |
| `.GTO` | Top silkscreen |
| `.GBO` | Bottom silkscreen |
| `.GKO` | Board outline |
| `.XLN` | Drill file (Excellon) |
| + diverse | Paste, drill info, etc. |

⚠️ Negeer de `.gpi` en `.dri` bestanden — niet nodig voor JLCPCB.

### 9.5 Stap 3 — BOM en CPL genereren (voor SMD assemblage)

BOM en CPL worden gegenereerd via een **ULP-script**, niet via de CAM processor.

**Eenmalige installatie:**
1. Download `jlcpcb_smta_exporter.ulp` van https://github.com/oxullo/jlcpcb-eagle
2. Kopieer naar `~/Documents/EAGLE/ulps/`

**BOM + CPL exporteren:**
1. Board venster actief in Eagle 9.6.2
2. **File → Run ULP** → selecteer `jlcpcb_smta_exporter.ulp` → OK
3. Kies layer: **Top** (of Bottom)
4. Kies output map — maak een aparte map `smt-files` aan
5. Twee bestanden worden aangemaakt:
   - `boardname_top_bom.csv`
   - `boardname_top_cpl.csv`

**BOM formaat (kolommen):**

| Kolom | Inhoud |
|---|---|
| Comment | Waarde/type component |
| Designator | Referenties (bijv. C1 C4 C6) |
| Footprint | Package type |
| LCSC Part # | LCSC onderdeelnummer (zie §9.6) |
| Quantity | Aantal |

**Tip:** Voeg LCSC onderdeelnummers toe als attribuut `LCSC_PART` aan je componenten in het schema — de ULP exporteert deze automatisch naar de BOM en JLCPCB matcht de onderdelen automatisch bij bestelling.

### 9.6 Shield v2.0 — BOM componenten met LCSC nummers

| Ref | Waarde | Package | LCSC | Aantal |
|---|---|---|---|---|
| C1, C4, C6, C7 | 0.1µF 50V X7R | C0603 | C14663 | 4 |
| C2, C5 | 10µF 25V X5R | C0805 | C15850 | 2 |
| R1, R2, R3 | 4.7kΩ (I2C pull-ups) | 0805 | C17673 | 3 |
| R4 | 499Ω (UART TX) | R0402 | C4125 | 1 |
| R5 | 33Ω (Pixels datasignaal) | R0402 | C284519 | 1 |
| R9 | 3kΩ (Power LED serie) | 0805 | C2907263 | 1 |
| LED1 | Blauw (Power indicator) | CHIPLED_0603 | C19171394 | 1 |
| PTC, PTC1 | 500mA resetbare zekering | PTC-1206 | C52748003 | 2 |

**Niet door JLCPCB te plaatsen (THT/connectors — zelf solderen):**
- ESP32-C6 dev board (in pin headers)
- Alle connectors (RJ45, JST, pin headers)
- Spanningsregelaar U1 (knipbaar)

### 9.7 Stap 4 — Rotaties controleren in JLCPCB viewer

Na upload van BOM en CPL toont JLCPCB een interactieve 2D/3D preview.

**Polariteitscheck per componenttype:**

| Component | Gepolst? | Rotatie kritiek? | Actie |
|---|---|---|---|
| Keramische condensatoren (X5R, X7R) | ❌ | Nee | Geen controle nodig |
| Weerstanden | ❌ | Nee | Geen controle nodig |
| PTC zekeringen | ❌ | Nee | Geen controle nodig |
| **LED1 (CHIPLED_0603)** | ✅ | **Ja** | **Controleer kathode richting!** |

**LED1 polariteit (C19171394):**
- Circuit: `VCC_5V → LED1 anode → POWER-LED net → R9 (3kΩ) → GND`
- Pad **VCC_5V** = anode (+)
- Pad **POWER-LED** = kathode (−)

**Hoe de LED richting beoordelen in de JLCPCB 3D viewer:**
- JLCPCB toont een **roze/magenta stip** op pin 1 van elk gepolst component — dit is de **kathode (−)** van de LED
- De roze stip moet aan de **POWER-LED kant** staan (kant richting R9/GND), niet aan de VCC_5V kant
- In de shield v2.0 bestelling: LED stond initieel **dwars** (90° fout) — gecorrigeerd naar horizontaal met roze stip aan POWER-LED zijde ✅

**Rotatie corrigeren indien nodig:**
1. Open `boardname_top_cpl.csv` in Excel/Numbers
2. Zoek de designator (bijv. LED1)
3. Pas de Rotation waarde aan met stappen van 90° (bijv. 90 → 180, of 0 → 270)
4. Sla op en herlaad het bestand in de JLCPCB viewer
5. Herhaal tot de roze stip aan de juiste kant staat

**Na controle:** JLCPCB vraagt *"Can we proceed PCBA with corrected parts placement?"* → kies **"Yes, please proceed"** → Submit.

### 9.8 Stap 5 — Upload en bestellen bij JLCPCB

1. Ga naar jlcpcb.com → **Order Now**
2. Upload de Gerber ZIP
3. Stel PCB parameters in: 2 lagen, gewenste kleur soldermask, HASL of ENIG finish
4. Scroll naar beneden → activeer **PCB Assembly**
5. Kies **Top Side**, **Economic assembly**
6. Klik Next → upload `_bom.csv` en `_cpl.csv`
7. Controleer de component preview
8. Bevestig onderdelen en rotaties → bestelling plaatsen

### 9.9 Bekende valkuilen

| Probleem | Oorzaak | Oplossing |
|---|---|---|
| Eagle 7.7.0 crasht bij Process Job | Bug in Eagle 7.7.0 op macOS 10.15+ | Gebruik Eagle 9.6.2 |
| CAM file niet zichtbaar in Control Panel | Verkeerde map | Kopieer naar `~/Documents/EAGLE/cam/` |
| "No board loaded" bij CAM Processor | CAM Processor geopend vanuit Control Panel | Open CAM Processor via icoon in het Board venster |
| Ground plane verdwenen na opslaan | ratsnest-resultaat niet persistent | Altijd opnieuw `ratsnest` uitvoeren vóór CAM export |
| LED verkeerd om gesoleerd | Rotatie in CPL incorrect | Controleer roze stip in JLCPCB viewer — stip = kathode (−), moet aan POWER-LED/GND kant staan. CPL aanpassen in stappen van 90°. |
| LED staat dwars in viewer | CPL rotatie 90° fout | Pas Rotation aan met ±90° in CPL file, herlaad in viewer |

---

## 10. Bestanden

| Bestand | Beschrijving |
|---|---|
| `ESP32_C6_Zarlar_Dashboard_17mar_wifi.ino` | Dashboard v3.0 — Matter HOME/UIT, WiFi tester |
| `ESP32_C6_MATTER_HVAC_v1.19.ino` | HVAC productieversie — huidig |
| `HVAC_GoogleScript_v4.gs` | GAS HVAC — 31 kolommen A–AE |
| `ESP32_C6_MATTER_ECO_v1.22.ino` | ECO productieversie — huidig |
| `ECO_GoogleScript.gs` | GAS ECO — 20 kolommen A–T |
| `ESP32-C6_MATTER_ROOM_15mar_2200.ino` | ROOM v2.10 — Matter + heap stabiel |
| `ROOM_GoogleScript_v1_4.gs` | GAS ROOM — 37 kolommen A–AK |
| `Oude_MATTER_ROOM_3mar.ino` | Referentie: werkende Matter endpoint-volgorde (aparte tegels) |
| `partitions_16mb.csv` | Custom partitietabel voor alle vier controllers |
| `Zarlar_Master_Overnamedocument.md` | Dit document |

---

## 11. Instructies voor nieuwe sessie

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

---

*Zarlar project — Filip Delannoy — bijgewerkt 23 maart 2026*
