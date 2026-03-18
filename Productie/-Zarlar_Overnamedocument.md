# Zarlar Thuisautomatisering — Master Overnamedocument
**ESP32-C6 · Arduino IDE · Matter · Google Sheets**
*Filip Delannoy — Zarlardinge (BE) — bijgewerkt 18 maart 2026*

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

| Controller | IP | MAC | Versie | Status |
|---|---|---|---|---|
| **HVAC** | 192.168.0.70 | 58:8C:81:32:29:54 | v1.19 | ✅ Productie, stabiel |
| **ECO Boiler** | 192.168.0.71 | 58:8C:81:32:2B:D4 | v1.22 | ✅ Productie, stabiel |
| **ROOM** | 192.168.0.80 | 58:8C:81:5D:B0:88 | v2.10 | ✅ Matter + heap stabiel |
| **Zarlar Dashboard** | 192.168.0.60 | — | v3.0 | ✅ Matter HOME/UIT, WiFi tester |

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

⚠️ **Nooit een 4MB controller gebruiken voor het Dashboard** — de `partitions_16mb.csv` past niet op 4MB flash. Dit veroorzaakte een bootloop (`partition size 0x600000 exceeds flash chip size 0x400000`). Het Dashboard gebruikt nu een 32-pin 16MB controller, identiek aan de andere drie.

### 1.4 Arduino IDE instellingen (alle vier controllers)

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
| Static IP | 192.168.0.70 |
| I/O expander | MCP23017 I2C — pins 0..6 relay circuits, 7 pump feedback, 8..9 distributiepompen, 10..12 TSTAT inputs (INPUT_PULLUP) |
| Temperatuursensoren | 6× DS18B20 op OneWire pin 3 (SCH boiler) |
| Ventilator | PWM op GPIO 20 via `ledcWrite` (0–255 → 0–100%) |

### 3.2 Heap-baseline (v1.18)

```
Setup:   free=~180KB  largest=~55KB
Runtime: largest_block stabiel >35KB  ✅
```

### 3.3 Matter endpoints (v1.18)

| # | Type | Variabele | Opmerking |
|---|---|---|---|
| EP1 | MatterTemperatureSensor | `sch_temps[0]` | Boiler top |
| EP2–EP8 | MatterOnOffPlugin | `circuits[0..6]` | Kringen 1–7 |
| EP9 | MatterFan | `vent_percent` | Ventilatie % |

### 3.4 RSSI key in /json

| Key | Label |
|---|---|
| `ac` | RSSI (dBm) |

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
| Static IP | 192.168.0.71 |
| Temperatuursensoren | 6× DS18B20 (2 per laag: Top/Mid/Bot × H/L) |
| Zonnecollector | PT1000 temperatuursensor → `Tsun` |
| Circulatiepomp | PWM-aangestuurd op basis van `dT = Tsun − Tboil` |

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
| Static IP | 192.168.0.80 |
| Sensoren | DHT22 (temp+vocht), DS18B20, MH-Z19 (CO2), Sharp GP2Y (dust), TSL2561 (lux), LDR, PT1000 |
| Actuatoren | NeoPixel strip (tot 30 pixels), PIR×2, laserbeam LDR |
| Verwarming | TSTAT output + setpoint |

### 5.2 Heap-baseline (v2.10)

```
Setup:   23% free  (62936 bytes)   Largest block: 45 KB
Runtime: 20% free  (~74 KB)        Largest block: 31 KB  ✅
Crashdrempel: 25 KB — marge: 6 KB  ✅
```

Matter kost ~214 KB heap — niet te vermijden. Alle andere optimalisaties zijn gedaan.

### 5.3 Heap-optimalisaties doorgevoerd (v2.10)

| Maatregel | Winst |
|---|---|
| `pixel_nicknames[30]` String[] → `char[30][32]` BSS | ~1.5 KB heap |
| `ds_nicknames[4]`, `room_id`, `wifi_*`, `static_ip_str` → `char[]` | Permanente heap weg |
| `getFormattedDateTime()` String return → `const char*` static buf | Heap-alloc per call weg |
| Captive portal handlers → AP-only registratie | ~600 bytes handler-heap |
| `MOV_BUF_SIZE` 50 → 20 | 240 bytes BSS |
| N pixel-handlers → 2 universele handlers (`?idx=`) | ~600 bytes handler-heap |
| `mdns_name` en `hsvToRgb()` verwijderd | Flash + BSS |

### 5.4 Matter endpoints (v2.10, werkend)

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

### 5.5 Matter — ROOM-specifieke valkuilen

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

### 5.6 RSSI key in /json

| Key | Label |
|---|---|
| `ac` | RSSI (dBm) |

### 5.7 ROOM /json output (keys a..ai → naar Zarlar → Google Sheets)

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

### 5.8 Openstaande punten ROOM

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
| Static IP | 192.168.0.60 |
| Board | ESP32-C6 Dev Module, 32-pin, **16MB flash** |
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

## 7. Schimmelbescherming via vochtigheid

Elke ROOM controller meet lokaal de luchtvochtigheid. Bij overschrijding van een drempelwaarde beslist de room controller autonoom om ventilatie te vragen via JSON key `g` (Vent_percent). De HVAC neemt de hoogste ventilatievraag van alle zones en stuurt de centrale ventilator aan. De volledige schimmelbeschermingslogica zit in de room controllers — de HVAC is enkel uitvoerder.

---

## 8. Bestanden

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

---

*Zarlar project — Filip Delannoy — bijgewerkt 17 maart 2026*
