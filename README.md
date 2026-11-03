# ESP32-C6 HVAC-controller (Zarlardinge)

De centrale controller in de kelder die de **verwarmingskringen**, de **ventilatie** en de **energie-uitwisseling met de ECO-boiler** aanstuurt.

| | |
|---|---|
| **Actieve versie** | **v2.0** (1 november 2026) — `Productie/ESP32C6_HVAC.ino`, zonder Matter |
| **Adres** | `http://192.168.0.70` (vast IP, geen mDNS) |
| **Board** | ESP32-C6 32-pin clone, MAC `58:8C:81:32:2B:90` |
| **Status** | 🧪 **Testopstelling** (nog niet in productie) — kringen met testinstellingen, alle 7 op room80 |
| **Volgende stap** | v2.0 ✅ getest (48 u zonder herstart, heap 68 % / 247 KB — §10 stap 4). Nu: eigen Sheets-push weg → **v2.1** (§10 stap 5b) |
| **Systeemdocument** | [Zarlar Master Overnamedocument](https://github.com/FidelDworp/ZarlarApp/blob/main/ESP32_Zarlar/Zarlar_Master_Overnamedocument.md) — §4 gaat over deze controller |

> **Leeswijzer.** §1 en §2 zijn voor wie het systeem leert kennen. §3 t.e.m. §9 beschrijven wat er nu draait. §10 en §11 beschrijven wat er nog moet gebeuren. Bij twijfel is **de sketch de waarheid**, niet dit document.

---

## Inhoud

1. [Wat doet deze controller?](#1-wat-doet-deze-controller)
2. [Woordenlijst](#2-woordenlijst)
3. [Bestanden in deze repo](#3-bestanden-in-deze-repo)
4. [Hardware](#4-hardware)
5. [Hoe beslist de controller?](#5-hoe-beslist-de-controller)
6. [Ventilatie: Begetube 350 R + AnaBoX](#6-ventilatie-begetube-350-r--anabox)
7. [Webinterface, instellingen en serial commando's](#7-webinterface-instellingen-en-serial-commandos)
8. [Data: `/json` en Google Sheets](#8-data-json-en-google-sheets)
9. [Veiligheid en robuustheid](#9-veiligheid-en-robuustheid)
10. [Migratie zonder Matter — v2.0](#10-migratie-zonder-matter--v20)
11. [Openstaande punten](#11-openstaande-punten)
12. [Compileren en uploaden](#12-compileren-en-uploaden)
13. [Versiegeschiedenis](#13-versiegeschiedenis)

---

## 1. Wat doet deze controller?

In elke kamer hangt een **room controller** (een eigen ESP32). Die weet hoe warm het is, wat de gewenste temperatuur is, en of er iemand thuis is. De HVAC-controller in de kelder is de **uitvoerder**: hij vraagt elke kamer om de paar seconden wat ze nodig heeft en zet dan de juiste relais aan of uit.

```
 Kamers (room controllers)            ECO-boiler controller (.71)
   "ik wil warmte" / "ventilatie 40%"     "boiler 85 °C, 18 kWh"
              │                                  │
              └────────────┬─────────────────────┘
                           ▼  (HTTP /json, elke 10 s)
                ┌──────────────────────┐
                │  HVAC-controller .70 │──► 7 relais: verwarmingskringen
                │      (kelder)        │──► 2 relais: pompen SCH en WON
                └──────────────────────┘──► PWM → AnaBoX → ventilatie-unit
                           │
                           ▼
          webpagina · /json · Apple Home (via de RPi-bridge)
                           │
                           ▼  (Zarlar Dashboard .60 haalt /json op)
                     Google Sheets
```

Drie taken:

1. **Verwarming** — 7 kringen (vloerverwarming/radiatoren per zone). Per kring één relais. Een kring gaat aan als de kamer erom vraagt, of als een gewone muurthermostaat (TSTAT) erom vraagt.
2. **Ventilatie** — elke kamer vraagt een percentage; de hoogste vraag wint. Dat percentage gaat via een PWM-signaal en de AnaBoX als 0,2-10 V naar de ventilatie-unit.
3. **ECO-pompen** — als de ECO-boiler (zonne-/restwarmte) vol en heet genoeg is, pompt de controller die warmte beurtelings naar de **schuur (SCH)** en de **woning (WON)**.

De controller is bewust "dom": de slimheid (setpoints, aanwezigheid) zit in de room controllers. Valt een kamer weg, dan blijft de rest werken.

---

## 2. Woordenlijst

| Woord | Betekenis |
|---|---|
| **ESP32-C6** | Kleine computer-chip met WiFi. Hierop draait de sketch. |
| **Sketch** | Het programma (`.ino`-bestand) dat je met de Arduino IDE op de ESP32 zet. |
| **Relais** | Elektrische schakelaar die de ESP32 kan bedienen. Hier **actief-laag**: signaal LOW = relais AAN, HIGH = UIT. |
| **MCP23017** | Extra chip met 16 aansluitingen (in/uit), via de I2C-bus. De ESP32 heeft zelf te weinig pinnen. |
| **I2C** | Bus met 2 draden (SDA, SCL) om chips te laten praten. |
| **DS18B20** | Digitale temperatuursensor. Meerdere sensoren op één draad (OneWire), elk met een eigen adres. |
| **PWM** | Snel aan/uit-schakelen. Hoe langer "aan" per periode, hoe hoger de gemiddelde spanning (0-100%). |
| **0-10V** | Standaard stuursignaal in ventilatie: 0 V = min, 10 V = max. |
| **TSTAT** | Gewone muurthermostaat met een schakelcontact. |
| **Override** | Handmatig forceren via de webpagina (tijdelijk, vervalt vanzelf). |
| **JSON / `/json`** | Tekstformaat voor data. Elke controller toont zijn waarden op `http://<ip>/json`. |
| **NVS** | Klein geheugen in de ESP32 dat instellingen bewaart na een herstart of update. |
| **Heap** | Het werkgeheugen (RAM) tijdens het draaien. Raakt het op of versnipperd, dan crasht de ESP32. "Largest block" = grootste stuk vrij geheugen in één geheel. |
| **OTA** | "Over the air": nieuwe sketch uploaden via WiFi in plaats van via een kabel. |
| **Matter** | Standaard om toestellen in Apple Home te tonen. Draait sinds v2.0 enkel nog op de Raspberry Pi (bridge), niet meer op de ESP32 (§10). |
| **SCH / WON** | Schuur / Woning — de twee kanten die warmte uit de ECO-boiler krijgen. |

---

## 3. Bestanden in deze repo

| Bestand | Wat |
|---|---|
| `README.md` | Dit document (in de root, zodat GitHub het op de voorpagina van de repo toont) |
| `Productie/ESP32C6_HVAC.ino` | **Actieve sketch v2.0** (zonder Matter) |
| `Productie/partitions.csv` | Geheugenindeling van de flash (16 MB, 2 × 6 MB app voor OTA). Moet naast het `.ino`-bestand staan. |
| `Productie/HVAC_GoogleScript.gs` | Google Apps Script v5: ontvangt de HVAC-data (via het Dashboard) en schrijft ze in een Google Sheet |
| `Old/Old HVAC_Photon.cpp` | De Particle Photon-sketch. ⚠️ **Draait nog in productie** op de Photon S-HVAC, die vandaag de echte verwarming stuurt |
| `Old/ESP32C6_MATTER_HVAC.ino` | Vorige versie v1.19 (met Matter). **Alleen ter referentie** |
| `Old/Old ESP32_HVAC-NoMatter.ino` | Oude ESP32-versie zonder Matter (januari 2026, V53.x). **Alleen ter referentie** |
| `Old/Old HVAC_GoogleScript.gs` | Oud Apps Script (lange JSON-sleutels) — niet meer compatibel |
| `Old/CHECKLIST` | Oude werklijst |

---

## 4. Hardware

### 4.1 ESP32-C6-pinnen

| Pin | `#define` | Functie |
|---|---|---|
| IO3 | `ONE_WIRE_PIN` | DS18B20 OneWire — 6 sensoren in de SCH-boiler |
| IO13 | `I2C_SDA` | I2C data → MCP23017 |
| IO11 | `I2C_SCL` | I2C klok → MCP23017 |
| IO20 | `VENT_FAN_PIN` | PWM ventilatie (5 kHz, 8-bit) → AnaBoX → 0-10V |

Alle andere pinnen zijn vrij. **Nooit IO8 of IO9 als ingang gebruiken** (strapping pins: ze bepalen hoe de chip opstart).

### 4.2 MCP23017 (I2C-adres 0x20)

| MCP-pin | Richting | Functie |
|---|---|---|
| 0-6 | uitgang | Relais kring 1-7 (actief-laag) |
| 7 | ingang (pull-up) | Terugmelding circulatiepomp (LOW = pomp draait) |
| 8 | uitgang | Relais pomp SCH |
| 9 | uitgang | Relais pomp WON |
| 10-12 | ingang (pull-up) | TSTAT-ingangen (LOW = thermostaat vraagt warmte) |
| 13-15 | ingang (pull-up) | Vrij / reserve |

Bij het opstarten zet de sketch **alle relais eerst UIT** (HIGH), vóór WiFi start. Wordt de MCP23017 niet gevonden, dan blijft de controller draaien maar schakelt hij niets.

### 4.3 Temperatuursensoren

Zes DS18B20's met vaste adressen in de sketch (`sensor_addresses[6]`), van boven (sensor 0) naar onder (sensor 5) in de SCH-boiler. Ze worden **elke 60 s** gelezen, alle zes tegelijk (één wachttijd van 750 ms). Een sensor met CRC-fout krijgt de waarde −127 °C en telt niet mee in het gemiddelde.

---

## 5. Hoe beslist de controller?

### 5.1 Verwarming per kring

Elke **10 s** (instelbaar: `poll_interval`) vraagt de controller elke kamer haar `/json` op. Per kring geldt deze volgorde:

1. **Override actief?** → die waarde wint (vervalt na **10 min**).
2. **Kamer offline?** → enkel de TSTAT telt.
3. **Kamer online en iemand thuis** (`home_status` = 1) → AAN als TSTAT **of** kamer warmte vraagt.
4. **Kamer online en niemand thuis** → enkel de vraag van de kamer telt.

TSTAT-ingangen (MCP 10-12) worden daarnaast **elke 100 ms** gecontroleerd. Verandert een thermostaat, dan schakelt het relais meteen, zonder op de volgende poll te wachten.

Per kring houdt de controller bij hoe lang hij aan stond: een **duty-cyclus over 4 uur** (12 vakjes van 20 min, rollend). Die gaat naar `/json` en Google Sheets.

**Wat de controller van een kamer leest** (keys uit de room-sketch):

| Key | Betekenis |
|---|---|
| `b` | Warmtevraag (0/1) |
| `g` | Ventilatievraag (%) |
| `c` | Setpoint (°C) |
| `e` | Kamertemperatuur (°C) |
| `v` | Thuis (1) / weg (0) |

### 5.2 Ventilatie

```
effectief % = max( hoogste vraag van alle kamers , override via webslider )
spanning    = 0,2 V + effectief % × 9,8 V / 100      (minstens 0,3 V)
PWM         = ventPwm(effectief %)  → IO20            (correctie voor de AnaBoX, §6.4)
```

⚠️ **0 % is de laagste stand, niet "uit".** Onder 0,2 V draait de Begetube op volle kracht (§6.2). Echt uitschakelen kan enkel met klem 5-6 (§11, punt 6).

De override via de slider op de webpagina werkt als **minimum** en vervalt na **3 uur**. Slider op 0 = automatisch.

### 5.3 ECO-pompen SCH en WON

De controller leest de ECO-boiler (`.71`) uit: `b` = temperatuur boven, `g` = onder, `h` = gemiddelde, `i` = opgeslagen energie (kWh).

- **Start** als de boiler online is **en** (bovenste temp > `eco_max_temp` (90 °C) **of** energie > `eco_threshold` (15 kWh)).
- **Stop** als de boiler offline is **of** bovenste temp < `eco_min_temp` (80 °C) **of** energie < drempel − hysterese (15 − 5 = 10 kWh).

Daarna een vaste cyclus, beurtelings voor eerlijke verdeling:

```
IDLE → pomp SCH 1 min → wacht 1 min → pomp WON 1 min → wacht 2 min → opnieuw (zolang "start" geldt)
```

Er draait **nooit** meer dan één pomp tegelijk. Via de webpagina kan je een pomp handmatig aan of uit zetten; dat vervalt na **60 s**.

De **opgeslagen energie in de SCH-boiler** wordt berekend met 6 lagen van elk `boiler_volume` liter (default 50 l): per laag (gemiddelde laagtemperatuur − `boiler_ref_temp`) × volume × 1,16 Wh/(l·K).

> ⚠️ De **kWh per pompbeurt** is nog een vaste waarde (0,5 kWh), geen echte meting. Zie §11.

### 5.4 Terugmelding circulatiepomp

Als er minstens één kring aan staat, hoort de circulatiepomp te draaien (MCP 7 = LOW). Klopt dat niet, dan schrijft de controller een waarschuwing op de Serial Monitor. Er volgt geen andere actie.

---

## 6. Ventilatie: Begetube 350 R + AnaBoX

### 6.1 Waarom een omzetter?

De ESP32-C6 heeft **geen echte analoge uitgang** (geen DAC). IO20 geeft enkel een PWM-signaal van 3,3 V. De ventilatie-unit verwacht een gelijkspanning van 0-10 V. Daartussen zit dus een omzetter: de **AnaBoX**.

**Huidige toestand:** de unit wordt nog aangestuurd door een **Particle Photon met de oude AnaBoX** (met op-amp). Doel: overschakelen naar de HVAC-controller met de **AnaBoX v2.0**. De AnaBoX v2.0 is getest met de `ESP32_PWM_Test`-sketch op de Begetube in de kelder (1/11/2026): werkt.

### 6.2 Aansluitingen op de unit (groene 8-polige stekker)

Bron: gebruikershandleiding + mail van de technische dienst. Alle contacten zijn potentiaalvrij.

| Klemmen | Functie | Opmerking |
|---|---|---|
| 1-4 | Calamiteitscontact (brandsensor) | Standaard een brugje — **nooit verwijderen**, anders werkt de unit niet |
| 1-2 | Stand 2 (P–T1) | Driestandenschakelaar — niet combineren met de 0-10V-sturing |
| 1-3 | Stand 3 (P–T2) | idem |
| **5-6** | **PIR/timer-contact** | Gesloten = unit volgt de 0-10V; open = unit schakelt uit. Standaard een brugje |
| **7-8** | **0-10V-ingang** | Tekening: rood = 7 (+), blauw = 8 (−). **Eerst nameten** vóór je aansluit |

**Gemeten (1/11/2026):**
- **0,2-10 V = 0-100 %, lineair.** **Onder 0,2 V (ook 0 V of een losse draad) draait de unit op volle kracht.**
- Ingangsweerstand ±**200 kΩ** (0,05 mA bij 10 V). Geen interne pull-up (open: −0,12 V).

### 6.3 AnaBoX v2.0 (eigen ontwerp, iTroniX)

Twee kleine transistors (2N7002, Q1 en Q2) zetten het 3,3V-PWM-signaal om naar een PWM van 0 tot VCC. Een RC-filter (R4 = 10 kΩ, C1 = 1 µF) maakt daar een vloeiende gelijkspanning van.

- **Aansluiten:** PWM-IN ← IO20 + GND · ANA-OUT pin 1 → klem 7, GND → klem 8
- **Bij uitval** (geen PWM, controller uit, draad los): uitgang 0 V → ventilator op **volle kracht**. Voor ventilatie een veilige terugval
- **Aandachtspunten:**
  1. **VCC moet exact 10 V zijn** en stabiel: de uitgangsspanning schaalt mee met VCC.
     **Voeding AnaBoX:** 5 V van de controller → klein step-up-printje → **10,0 V** op Vin (geen aparte voeding nodig). Stel de 10 V in met de multimeter vóór je de AnaBoX aansluit, controleer na een uur draaien en zet het potje vast. De AnaBoX zelf trekt maar enkele mA. Op de HVAC-controller (Velleman PSSE510, 5 V / 1 A, 10 relais achter de 5V-PTC): de step-up aansluiten **vóór de 5V-PTC** (voedingskant), zodat schakelende relais geen dips op de 10 V geven.
  2. De AnaBoX v2.0 heeft **geen buffer** (de oude wel). Met de Begetube (200 kΩ) zakt de spanning maar ±6 %: **R3 is niet nodig**.
  3. Tijdens het opstarten van de ESP32 is de uitgang even 0 V (= volle kracht) tot `setup()` de laagste stand schrijft.
  4. C1 minstens 25 V (X7R). ESP32 en unit delen dezelfde GND.
  5. **PWM-frequentie:** de gelijkspanning is gelijk bij 1, 5 en 20 kHz. Rimpel: 0,18 / 0,03 / 0,01 V. Sinds v2.0 **5 kHz** (rimpel ≈ één PWM-stap).

### 6.4 Metingen AnaBoX v2.0 en correctie in de sketch

Vin = 10 V, zonder R3, multimeter (31/10 en 1/11/2026):

| PWM | Onbelast | Met Begetube |
|---|---|---|
| 0 | 0,00 V | 0,00 V (= volle kracht!) |
| 64 | 1,43 V | 1,36 V |
| 128 | 3,34 V | 3,14 V |
| 191 | 5,97 V | 5,53 V |
| 255 | 9,97 V | ≈ 9,3 V (berekend) |

De AnaBoX is **niet lineair**: `U = Vmax · D / (D + k·(1 − D))`, met D = duty (0-1), **k ≈ 2** en **Vmax ≈ 9,3 V** met de unit aangesloten. De sketch rekent omgekeerd (`ventPwm()`): `u = U / Vmax` → `D = k·u / (1 + (k − 1)·u)`. Dat voorspelt de metingen tot op 0,04 V.

| Ventilatie | Spanning | PWM |
|---|---|---|
| 0 % (laagste stand) | 0,30 V | 16 |
| 25 % | 2,65 V | 113 |
| 50 % | 5,12 V | 181 |
| 75 % | 7,58 V | 229 |
| 100 % | 9,3 V (max. haalbaar ≈ 93 %) | 255 |

Constanten bovenaan de sketch: `VENT_VMAX`, `VENT_K`, `VENT_U_MIN_VEILIG`, `VENT_PWM_FREQ`. Meet bij de overstap de spanning op klem 7-8 bij 100 % en pas `VENT_VMAX` aan als die afwijkt.

---

## 7. Webinterface, instellingen en serial commando's

### 7.1 Pagina's en endpoints

| Adres | Wat |
|---|---|
| `/` | Hoofdpagina: kringen, pompen, ventilatie, boiler, geheugenstatus |
| `/settings` | Instellingen (zie §7.2) |
| `/update` | Nieuwe sketch uploaden (OTA, `.bin`-bestand) |
| `/json` | Compacte data voor Sheets en Dashboard (§8) |
| `/json_ui` | Data voor de kringentabel op de hoofdpagina |
| `/json_settings` | Data voor de instellingenpagina |
| `/scan` | WiFi-netwerken in de buurt |
| `/reboot` | Herstarten |
| `/circuit_override_on`, `_off`, `_cancel` | Kring forceren aan/uit (10 min) of terug naar automatisch |
| `/pump_sch_on`, `_off`, `_cancel` · `/pump_won_…` | Pomp handmatig (60 s) |
| `/set_vent?vent=0..100` | Ventilatie-minimum (3 u); 0 = automatisch |
| `/clear_crash_log` | Crash-teller wissen |

### 7.2 Instellingen (`/settings`, bewaard in NVS namespace `hvac-config`)

| Groep | Velden |
|---|---|
| Algemeen | naam (`room_id`), WiFi SSID + wachtwoord, vast IP (leeg = DHCP), poll-interval (s), Google Script-URL (**leeg laten** — zie §8.2; verdwijnt in v2.1) |
| Kringen (1-7) | naam, IP of mDNS-naam van de kamer, vermogen (kW), TSTAT ja/nee + pin (10/11/12) |
| ECO-boiler | IP of mDNS-naam, energiedrempel + hysterese (kWh), start- en stoptemperatuur (°C) |
| SCH-boiler | referentietemperatuur (°C), volume per laag (l) |

Nu staan er **testinstellingen** ("Een"-"Zeven", 1-7 kW, alle kringen op room80). Bij ingebruikname in productie: namen BB, WP, BK, ZP, EP, KK, IK, het echte vermogen en per kring de juiste room controller.

Opslaan → de controller herstart. Instellingen blijven bewaard bij een update.

### 7.3 Serial commando's (Serial Monitor, 115200 baud)

| Commando | Wat |
|---|---|
| `R` binnen 5 s na het opstarten | Alle instellingen wissen (factory reset) |
| `status` | Overzicht van alle kringen en de ventilatie |
| `reset-all` | Volledige NVS wissen + herstart |

> Door `#define Serial Serial0` verschijnt de output op de USB-C-poort **"UART"** van het board, niet op de poort "USB".

### 7.4 Geen WiFi?

Lukt de WiFi-verbinding niet (5 pogingen van 20 s), dan start de controller een eigen netwerk **`HVAC-Setup`**. Verbind daarmee en surf naar `http://192.168.4.1` om de WiFi in te stellen.

---

## 8. Data: `/json` en Google Sheets

### 8.1 `/json` — compacte sleutels

Korte sleutels (`a`, `b`, …) houden het bericht klein. Dezelfde volgorde wordt in Google Sheets gebruikt (kolom A = tijdstempel).

| Key | Sheet | Betekenis | Eenheid |
|---|---|---|---|
| `a` | B | Uptime | s |
| `b`-`g` | C-H | Boilertemperaturen KST1-KST6 (boven → onder) | °C |
| `h` | I | Boilergemiddelde | °C |
| `i`-`o` | J-P | Duty-cyclus 4 u, kring 1-7 | % |
| `p`-`v` | Q-W | Kring 1-7 aan (1) / uit (0) | 0/1 |
| `w` | X | Totaal vermogen kringen die aan staan | kW |
| `x` | Y | Ventilatie effectief | % |
| `y` | Z | Pomp SCH aan | 0/1 |
| `z` | AA | kWh laatste SCH-beurt | kWh |
| `aa` | AB | Pomp WON aan | 0/1 |
| `ab` | AC | kWh laatste WON-beurt | kWh |
| `ac` | AD | WiFi-signaal (RSSI) | dBm |
| `ad` | AE | Vrije heap | **%** |
| `ae` | AF | Grootste vrije blok | KB |

> ⚠️ Een nieuwe of gewijzigde key raakt ook het Apps Script, het Zarlar Dashboard en de RPi-portal. **Nooit zonder afspraak wijzigen** (Master §3.10).

### 8.2 Google Sheets

**Afspraak (beslist 18 okt 2026): controllers posten nooit zelf naar Google.** Het **Zarlar Dashboard** (`192.168.0.60`) haalt van elke controller de `/json` op en stuurt die door naar Google Sheets. Zo doet geen enkele controller zelf zware HTTPS-verbindingen (die vroeger crashes door geheugenkrapte veroorzaakten), en is er één plek voor alle logging.

```
HVAC .70  /json  ──►  Dashboard .60 (pollt, om de 5 cycli een POST)  ──►  HVAC_GoogleScript.gs  ──►  Sheet
```

- De URL van `HVAC_GoogleScript.gs` staat in **`http://192.168.0.60/settings`**, bij de controller **S-HVAC**.
- Het Dashboard voegt een veld `"room"` toe; het HVAC-script negeert dat.
- Het script schrijft één rij per bericht en houdt maximaal 1000 rijen bij; oudere rijen worden gewist.

**Overgang:** v2.0 bevat nog de oude code om **zelf** te posten, maar het veld Google Script-URL in de HVAC-`/settings` is **leeg** (sinds 1 nov 2026), dus alleen het Dashboard post. Laat het leeg, anders komt elke meting dubbel in de Sheet. De code verdwijnt in **v2.1** (§10).

**Een scriptwijziging publiceren:** altijd *Implementaties beheren → potlood (bewerken) → nieuwe versie*, **nooit** "Nieuwe implementatie". Anders verandert de URL en moet die opnieuw in de Dashboard-`/settings`.

---

## 9. Veiligheid en robuustheid

| Wat | Hoe |
|---|---|
| Opstarten | Alle relais UIT vóór WiFi start; ventilatie op de laagste stand (0,3 V), niet 0 V |
| Overrides | Vervallen vanzelf: kring 10 min, pomp 60 s, ventilatie 3 u |
| ECO-pompen | Nooit twee tegelijk; stoppen als de ECO-boiler offline is |
| Kamer offline | Kring valt terug op de TSTAT |
| Sensorfout | CRC-controle; foutieve sensor telt niet mee |
| Geheugen | Elke 60 s gecontroleerd. Grootste blok < 25 KB → melding in de crash-log (zichtbaar in `/settings`). Kleurcode op de hoofdpagina: ≥ 35 KB OK, ≥ 25 KB laag, daaronder kritiek |
| Webpagina's | In kleine stukjes verstuurd (streaming), zodat er nooit veel geheugen tegelijk nodig is |
| Geen WiFi | Eigen netwerk `HVAC-Setup` (§7.4) |

---

## 10. Migratie zonder Matter — v2.0

*Plan van 18 okt 2026, naar het voorbeeld van ROOM v3.0 (Master §7.9 en §16, Portal Plan §16). Werkwijze: één stap tegelijk, elke stap met een meetpunt, code pas na GO.*

**Waarom:** Matter kostte veel werkgeheugen en maakte de sketch ingewikkeld. Apple Home praat nu alleen nog met de **Raspberry Pi** (`192.168.0.50`), die via een Matter-bridge de HVAC aanbiedt.

### Stappen 1-4 ✅ afgerond (19 okt – 3 nov 2026)

| Stap | Wat | Resultaat |
|---|---|---|
| 1 | Geheugen meten met Matter (v1.19) | Matter kostte bij het opstarten 61 KB; tijdens het draaien grootste blok ~30 KB (zone "LAAG"), vrij 17-18 % |
| 2 | De RPi neemt Apple Home over (`server.js` v2.10 + plugin `matterbridge-zarlar` v0.7, Portal Plan §16.12) | Toestel "HVAC" in de woning Zarlardinge: boiler, 7 kringen, ventilatie, ECO => SCHUUR / WONING. Alle tegels werken (3/11) |
| 3 | **v2.0**: Matter weg + ventilatie (§5.2, §6.4) + bestandsnaam `ESP32C6_HVAC.ino` | OTA geflasht 1/11; oude ESP32-tegels uit Apple Home ("Matter Home") verwijderd |
| 4 | Geheugen meten zonder Matter (48 u, Sheet 1/11 17:02 – 3/11 17:02, 576 rijen) | **0 herstarts** (v1.19: 4 in 11 dagen) · vrij **67-68 %** · grootste blok **239-251 KB** · Sheet volledig (1 rij per 5 min) |

> De Photon-knoppen van S-HVAC in Apple Home (*ECO => SCHUUR*, *ECO => WONING*, *HOME*) blijven: de **Photon S-HVAC stuurt nog de echte verwarming**.

### Stap 5 — Google Sheets enkel via het Dashboard (v2.1)

*Beslist 18 okt 2026: geen enkele controller post nog zelf naar Google (§8.2). Twee delen: eerst instellingen (geen code), daarna pas de code weghalen. Nooit samen met v2.0.*

**5a — Instellingen nakijken (geen codewijziging)** ✅ *1 nov 2026: HVAC-veld leeg, het Dashboard post (48 u-test: Sheet volledig)*

| # | Waar | Wat | Meetpunt |
|---|---|---|---|
| 1 | `http://192.168.0.60/settings` | Bij **S-HVAC** staat de URL van `HVAC_GoogleScript.gs` ingevuld | — |
| 2 | Google Sheet | Er komen nieuwe rijen binnen via het Dashboard | Om de ±5 min één nieuwe rij |
| 3 | `http://192.168.0.70/settings` | Veld **Google Script-URL leegmaken** en opslaan (controller herstart) | Na 30 min: geen dubbele rijen meer, wel nog rijen via het Dashboard |

**5b — v2.1: de eigen Sheets-push uit de sketch halen**

- **Weg:** `pushToGoogleSheets()`, de 5 min-timer in `loop()`, `gas_url` + `last_gas_push`, de NVS-key `gas_url`, het veld in `/settings` en `/save_settings`, `#include <WiFiClientSecure.h>`.
- **Blijft:** `buildLogJson()` — dat is de inhoud van `/json`, die het Dashboard ophaalt. **De JSON-keys veranderen niet.**
- **Meetpunt:** `/json` identiek aan v2.0 · Sheet blijft gevuld via het Dashboard · heap gelijk of beter.

**Ook voor de andere controllers** (ROOM, ECO, S-ENERGY): dezelfde regel, telkens in hun eigen repo en in een eigen versie, eerst 5a (instellingen), dan pas de code. Master §1.1 en §3 zijn de plaats om die regel systeembreed vast te leggen.

### Daarna — elk een eigen versie, na GO

| Versie | Wat | Meetpunt |
|---|---|---|
| v2.2 | `#define Serial Serial0` beslissen: poort "UART" of "USB" (Master §3.9) | Serial zichtbaar op de gekozen poort |
| v2.3 | mDNS terug (`hvac.local`), enkel als WiFi verbonden is met een geldig IP (Master §3.14) | `http://hvac.local` werkt; `.70` blijft werken |
| v2.4 | Geen polling/sensoren in AP-mode (Master §3.14) — eerst nakijken of dat al zo is | `/settings` blijft vlot in AP-mode |
| ~~v2.5~~ | Ventilatie — **zit al in v2.0** | — |
| v2.6 | **Firmwareversie zichtbaar** onderaan de hoofdpagina (en in `/settings`), enkel bij het laden van de pagina — **niet** in `/json` (geen nieuwe key). Eén `#define HVAC_VERSION` bovenaan, gebruikt in de webpagina én de opstarttekst, zodat ze nooit meer uit elkaar lopen (v1.19 toont bij het opstarten nog "v1.10") | Versie leesbaar op `http://192.168.0.70/` |

---

## 11. Openstaande punten

| # | Wat | Wanneer |
|---|---|---|
| 1b | **Bij ingebruikname in productie:** testinstellingen in `/settings` vervangen (namen BB, WP, BK, ZP, EP, KK, IK, echt vermogen, juiste room controller per kring). Geen codewijziging | bij overgang naar productie |
| 1c | **NTP-timeout** bij het opstarten: tijd synchroniseert niet. Mogelijke oorzaak: met vast IP wordt de router (192.168.0.1) als DNS gebruikt. Nakijken in de opstartlog van v2.0 | open |
| 1d | **WiFi-signaal** opnieuw meten op de definitieve plaats (testopstelling op de grond: −71 tot −79 dBm) | bij overgang naar productie |
| 2 | **Overstap van de Photon naar de AnaBoX v2.0** (metingen klaar, §6.4): spanning bij 100 % meten en `VENT_VMAX` nakijken | bij overgang naar productie |
| 3 | **kWh per pompbeurt** echt berekenen (Q = m × Cp × ΔT) i.p.v. vaste 0,5 kWh | later |
| 4 | **Witte pagina op de iPhone** bij de ventilatieslider (vermoedelijk geheugenkrapte bij herladen) | herbekijken — waarschijnlijk opgelost (grootste blok nu ~247 KB) |
| 5 | **Google Sheets enkel via het Dashboard** — beslist; instellingen nakijken (5a) en daarna de eigen push uit de sketch halen (v2.1). Zelfde regel voor ROOM, ECO en S-ENERGY | §10 stap 5 |
| 6 | *Optie:* klem 5-6 van de Begetube via een vrije MCP-pin (13-15) als harde aan/uit. Rusttoestand zo kiezen dat de unit **blijft ventileren** bij een storing. Nieuwe JSON-key enkel na afspraak | later |
| 7 | *Optie:* Sjalay-verbeteringen, één per versie (live-JSON vs. waarden enkel bij het laden van de pagina, statuscirkels, korte sectietitels) | later |

---

## 12. Compileren en uploaden

**Arduino IDE (Tools):**

| Instelling | Waarde |
|---|---|
| Board | ESP32C6 Dev Module |
| Flash Size | 16 MB |
| Partition Scheme | `partitions.csv` in de schetsmap (wordt automatisch gebruikt) |
| USB CDC On Boot | Enabled |
| Core Debug Level | None (spaart 20-30 KB heap) |

**Libraries:** `OneWireNg`, `Adafruit MCP23017 Arduino Library` (`Adafruit_MCP23X17`), `AsyncTCP` + `ESPAsyncWebServer`, `ArduinoJson` (v7), en uit de ESP32-core: `WiFi`, `HTTPClient`, `Preferences`, `Update`, `DNSServer`.

**Uploaden:**
- **Eerste keer:** via USB-kabel.
- **Daarna via OTA:** *Sketch → Export Compiled Binary*. In de map `build/esp32.esp32.esp32c6/` staan meerdere `.bin`-bestanden: kies **enkel `ESP32C6_HVAC.ino.bin`**, niet `.bootloader.bin`, `.partitions.bin` of `.merged.bin`. Upload dat op `http://192.168.0.70/update`; de controller herstart vanzelf.
- **Flash Size moet op 16 MB staan** (zichtbaar in de compile-uitvoer als `FlashSize=16M`). Staat ze op 4 MB, dan kan de OTA mislukken: eerst juist zetten en opnieuw exporteren. "Not connected" in de IDE bij het exporteren is normaal.
- **Na een update:** kijk op de hoofdpagina of alle kringen en pompen er normaal uitzien.

---

## 13. Versiegeschiedenis

| Versie | Datum | Wat |
|---|---|---|
| **v2.0** | 1 nov 2026 | Zonder Matter (bestand `ESP32C6_HVAC.ino`); ventilatie met correctie voor AnaBoX v2.0, laagste stand 0,3 V, 5 kHz |
| v1.19 | 18 mrt 2026 | Matter: relais schakelen meteen vanuit Apple Home |
| v1.18 | 17 mrt | ECO-keys aangepast aan de nieuwe ECO-sketch (`b`, `g`, `h`, `i`) |
| v1.17 | 17 mrt | Room-keys aangepast aan de nieuwe room-sketch (`b`, `g`, `c`, `e`, `v`) |
| v1.16 | 13 mrt | Duty-cyclus per kring over 4 u (rollend) → `/json` en Sheets |
| v1.15 | 13 mrt | TSTAT elke 100 ms: relais schakelt meteen bij een thermostaatwijziging |
| v1.14 | 13 mrt | Overrides schakelen het relais meteen; ventilatie = max(kamers, minimum) |
| v1.13 | 12 mrt | Heap-fixes ArduinoJson v7; Google Sheets-push vanuit de sketch |
| v1.12 | 12 mrt | `/json` compact (a/b/c); 2 Matter-endpoints weg (~7 KB) |
| v1.11 | 12 mrt | Heap: tekstvariabelen naar vaste buffers |
| v1.10 | 12 mrt | Betrouwbare Matter-reset; heap-meting bij het opstarten |
| v1.9 | 12 mrt | Uptime in `/json`; 7 i.p.v. 16 kringen in het geheugen |
| v1.8 | 12 mrt | Matter 14 → 11 endpoints (~10 KB); webpagina's in stukjes (streaming) |
| v1.7 | 11 mrt | `Serial0`-fix; sensoren elke 60 s i.p.v. 2 s (watchdog-crash opgelost); crash-log; mDNS weg (conflict met Matter) |
| v1.3-v1.6 | 1-3 mrt | Matter toegevoegd, `/matter`-pagina |
| v1.2 | 26 feb | Vast IP 192.168.0.70 |
| v1.1 | 10 jan | UI- en JSON-verbeteringen |
| V53.x | jan 2026 | Eerste ESP32-versie zonder Matter (nu `Old/Old ESP32_HVAC-NoMatter.ino`) |

Volledige details staan in de versieheader bovenaan de sketch.

---

*Zarlar thuisautomatisering — Filip Delannoy — bijgewerkt 3 november 2026*
