# Overdracht — HVAC-controller (ESP32C6_HVAC)

*Stand 7 november 2026 — Filip Delannoy (FiDel), Zarlardinge (BE)*

Dit document start een nieuw gesprek met Claude over de HVAC-controller in de kelder: verwarmingskringen, ventilatie (Begetube 350 R via AnaBoX) en de warmte-uitwisseling met de ECO-boiler. Alle details: [`README.md`](../README.md).

**Werkwijze en afspraken:** zie de [overkoepelende overdracht](https://github.com/FidelDworp/Zarlardinge-Home-Automation/blob/main/docs/ZARLAR_OVERDRACHT.md). Kort: Nederlands · nooit een sketch of script zonder GO · eerst een plan, één stap per versie (versiekop + changelog) en daarna een 48 u-test · alles via OTA · `/json`-keys en bedieningsadressen niet wijzigen zonder akkoord · README en deze overdracht mee bijwerken · na GO compileert Claude en zet de `.bin` in `bin/`.

---

## 1. Wat en waar

| | |
|---|---|
| Adres | `http://192.168.0.70` (vast IP, nog geen mDNS) |
| Sketch | `ESP32C6_HVAC/ESP32C6_HVAC.ino` **v2.0** (1/11/2026), zonder Matter |
| Klaar om te uploaden | `bin/ESP32C6_HVAC_v2.0.ino.bin` |
| Bibliotheken | OneWireNg, Adafruit MCP23X17 (+ BusIO), AsyncTCP + ESPAsyncWebServer, ArduinoJson v7 |
| Sheet | S-HVAC `1LL7swNRIVl2qfREHxAU7nCq9fH9zTUxa37s0IduHTXY` (logging hoort via het Dashboard te lopen) |
| Systeemdocument | Master-document §4 (`Zarlardinge-Home-Automation/docs/`) |

## 2. Stand (7/11)

- **Testopstelling**, nog niet in productie: de Photon S-HVAC stuurt nog de echte verwarming (`old/HVAC_Photon.cpp`).
- v2.0 getest: 48 u zonder herstart, heap 68 % / grootste blok 247 KB.
- Compileert in de sessie met core 3.3.12 (1,33 MB); heeft automatische prototypes nodig.

## 3. Open punten

1. **v2.1:** eigen Google Sheets-push uit de sketch (het Dashboard logt al). README §10 stap 5.
2. Bij ingebruikname: testinstellingen in `/settings` vervangen (kringnamen, vermogens, room per kring); WiFi-signaal meten op de definitieve plaats.
3. NTP-timeout bij het opstarten (vermoedelijk DNS via de router bij vast IP).
4. Overstap van de Photon naar de AnaBoX: spanning bij 100 % meten, `VENT_VMAX` nakijken.
5. Later: kWh per pompbeurt echt berekenen; mDNS (`hvac.local`); optie harde aan/uit van de Begetube via een vrije MCP-pin.

## 4. Praktisch

- Compileren: ESP32C6 Dev Module, Flash 16 MB, Partition Custom, USB CDC On Boot Enabled, Core Debug Level None.
- OTA: enkel `ESP32C6_HVAC.ino.bin` via `/update`. Na een update: hoofdpagina nakijken (kringen en pompen).
