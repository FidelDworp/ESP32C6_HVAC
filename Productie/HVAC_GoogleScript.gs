// ============================================================
// HVAC CONTROLLER DATA LOGGER - Google Apps Script v4
// Ontvangt JSON-push van de HVAC ESP32 en logt naar Google Sheet
//
// v4 — aangepast voor compacte a/b/c JSON-keys (ESP32 v1.12)
//   Kolommen: Tijdstempel + 32 datavelden (A t/m AF)
//   ECO-velden verwijderd (komen uit ECO controller)
//   Nieuw: SCH/WON pomp, kWh, RSSI, Free heap, Heap block
//
// INSTALLATIE:
//   1. Voer setupHeaders() eenmalig uit
//   2. Implementeren → Web-app → Toegang: Iedereen → URL opslaan
// ============================================================

function doPost(e) {
  try {
    const data = JSON.parse(e.postData.contents);
    const sheet = SpreadsheetApp.getActiveSpreadsheet().getActiveSheet();

    const timestamp = Utilities.formatDate(
      new Date(),
      "Europe/Brussels",
      "yyyy-MM-dd HH:mm:ss"
    );

    const row = [
      timestamp,            // A:  Tijdstempel

      data.a  || 0,         // B:  Uptime (s)

      // C–I: SCH Boiler temperaturen
      data.b  || 0,         // C:  TopH (°C)
      data.c  || 0,         // D:  TopL (°C)
      data.d  || 0,         // E:  MidH (°C)
      data.e  || 0,         // F:  MidL (°C)
      data.f  || 0,         // G:  BotH (°C)
      data.g  || 0,         // H:  BotL (°C)
      data.h  || 0,         // I:  Gem (°C)

      // J–P: Duty cycles circuits 1–7 (%)
      data.i  || 0,         // J:  BB duty %
      data.j  || 0,         // K:  WP duty %
      data.k  || 0,         // L:  BK duty %
      data.l  || 0,         // M:  ZP duty %
      data.m  || 0,         // N:  EP duty %
      data.n  || 0,         // O:  KK duty %
      data.o  || 0,         // P:  IK duty %

      // Q–W: Relay-staten circuits 1–7 (0/1)
      data.p  || 0,         // Q:  R1
      data.q  || 0,         // R:  R2
      data.r  || 0,         // S:  R3
      data.s  || 0,         // T:  R4
      data.t  || 0,         // U:  R5
      data.u  || 0,         // V:  R6
      data.v  || 0,         // W:  R7

      // X–Y: Vermogen & ventilatie
      data.w  || 0,         // X:  HeatDem (kW)
      data.x  || 0,         // Y:  Ventilatie (%)

      // Z–AC: ECO distributiepompen
      data.y  || 0,         // Z:  SCH Pomp (1/0)
      data.z  || 0,         // AA: SCH Overgepompt (kWh)
      data.aa || 0,         // AB: WON Pomp (1/0)
      data.ab || 0,         // AC: WON Overgepompt (kWh)

      // AD–AF: Systeemstatus
      data.ac || 0,         // AD: WiFi RSSI (dBm)
      data.ad || 0,         // AE: Free heap (%)
      data.ae || 0,         // AF: Heap largest block (KB)
    ];

    sheet.appendRow(row);

    return ContentService
      .createTextOutput(JSON.stringify({
        status:    "success",
        message:   "Data gelogd",
        timestamp: timestamp,
        uptime:    data.a
      }))
      .setMimeType(ContentService.MimeType.JSON);

  } catch (error) {
    Logger.log("doPost fout: " + error.toString());
    return ContentService
      .createTextOutput(JSON.stringify({
        status:  "error",
        message: error.toString()
      }))
      .setMimeType(ContentService.MimeType.JSON);
  }
}


// ============================================================
// SETUP — voer eenmalig uit om kolomhoofden aan te maken
// ============================================================
function setupHeaders() {
  const sheet = SpreadsheetApp.getActiveSpreadsheet().getActiveSheet();

  if (sheet.getLastRow() > 0) {
    const firstCell = sheet.getRange(1, 1).getValue();
    if (firstCell === "Tijdstempel") {
      sheet.deleteRow(1);
      Logger.log("Bestaande koprij verwijderd.");
    }
  }

  const headers = [
    "Tijdstempel",          // A  (1)
    "Uptime (s)",           // B  (2)
    "TopH (°C)",            // C  (3)
    "TopL (°C)",            // D  (4)
    "MidH (°C)",            // E  (5)
    "MidL (°C)",            // F  (6)
    "BotH (°C)",            // G  (7)
    "BotL (°C)",            // H  (8)
    "Gem (°C)",             // I  (9)
    "BB Duty (%)",          // J  (10)
    "WP Duty (%)",          // K  (11)
    "BK Duty (%)",          // L  (12)
    "ZP Duty (%)",          // M  (13)
    "EP Duty (%)",          // N  (14)
    "KK Duty (%)",          // O  (15)
    "IK Duty (%)",          // P  (16)
    "R1",                   // Q  (17)
    "R2",                   // R  (18)
    "R3",                   // S  (19)
    "R4",                   // T  (20)
    "R5",                   // U  (21)
    "R6",                   // V  (22)
    "R7",                   // W  (23)
    "HeatDem (kW)",         // X  (24)
    "Ventilatie (%)",       // Y  (25)
    "SCH Pomp",             // Z  (26)
    "SCH kWh",              // AA (27)
    "WON Pomp",             // AB (28)
    "WON kWh",              // AC (29)
    "RSSI (dBm)",           // AD (30)
    "Free heap (%)",        // AE (31)
    "Heap block (KB)",      // AF (32)
  ];

  sheet.insertRowBefore(1);
  sheet.getRange(1, 1, 1, headers.length).setValues([headers]);

  const headerRange = sheet.getRange(1, 1, 1, headers.length);
  headerRange.setFontWeight("bold");
  headerRange.setBackground("#FFCC00");
  headerRange.setHorizontalAlignment("center");

  sheet.setColumnWidth(1, 160);  // Tijdstempel
  for (let i = 2; i <= headers.length; i++) sheet.setColumnWidth(i, 100);
  sheet.setFrozenRows(1);

  Logger.log("Headers aangemaakt! (32 kolommen, A t/m AF)");
}


// ============================================================
// TEST — simuleer een POST zoals de HVAC sketch die stuurt
// ============================================================
function test() {
  const testData = {
    postData: {
      contents: JSON.stringify({
        "a": 3721,
        "b": 52.3, "c": 49.1, "d": 44.5, "e": 41.2,
        "f": 35.8, "g": 32.0, "h": 42.5,
        "i": 65, "j": 40, "k": 55, "l": 30, "m": 45, "n": 20, "o": 10,
        "p": 1, "q": 0, "r": 1, "s": 0, "t": 1, "u": 0, "v": 0,
        "w": 4.35, "x": 45,
        "y": 0, "z": 0.5,
        "aa": 1, "ab": 0.5,
        "ac": -60, "ad": 18, "ae": 28
      })
    }
  };

  const result = doPost(testData);
  Logger.log(result.getContent());
}


// ============================================================
// DAGELIJKSE SAMENVATTING — optionele e-mail trigger
// Trigger: Tools → Triggers → sendDailySummary → 23u–0u
// Kolomnummers aangepast voor v4 layout
// ============================================================
function sendDailySummary() {
  const sheet = SpreadsheetApp.getActiveSpreadsheet().getActiveSheet();
  const lastRow = sheet.getLastRow();
  if (lastRow < 2) { Logger.log("Nog geen data"); return; }

  const today = new Date();
  let count = 0, maxHeat = 0, sumHeat = 0, sumVent = 0;
  let schPompCount = 0, wonPompCount = 0;
  const sumDuty = [0, 0, 0, 0, 0, 0, 0];

  // Kolom-indices (1-based):
  // X=24 HeatDem, Y=25 Vent, Z=26 SCH Pomp, AB=28 WON Pomp
  // J–P = 10–16 duty cycles, Q–W = 17–23 relays

  for (let i = lastRow; i > 1; i--) {
    const ts = new Date(sheet.getRange(i, 1).getValue());
    if (ts.toDateString() !== today.toDateString()) break;
    count++;

    const heat    = sheet.getRange(i, 24).getValue() || 0;  // X: HeatDem
    const vent    = sheet.getRange(i, 25).getValue() || 0;  // Y: Vent
    const schPomp = sheet.getRange(i, 26).getValue() || 0;  // Z: SCH Pomp
    const wonPomp = sheet.getRange(i, 28).getValue() || 0;  // AB: WON Pomp

    if (heat > maxHeat) maxHeat = heat;
    sumHeat += heat;
    sumVent += vent;
    if (schPomp > 0) schPompCount++;
    if (wonPomp > 0) wonPompCount++;

    // Duty cycles J–P = kolommen 10–16
    for (let c = 0; c < 7; c++) {
      sumDuty[c] += sheet.getRange(i, 10 + c).getValue() || 0;
    }
  }

  if (count === 0) return;

  const expected = Math.round(24 * 60 / 5);
  const pct = (count / expected * 100).toFixed(1);
  const names = ["BB", "WP", "BK", "ZP", "EP", "KK", "IK"];

  let circuitLines = "";
  names.forEach((n, i) => {
    circuitLines += "  " + n + ": " + (sumDuty[i] / count).toFixed(0) + "%\n";
  });

  MailApp.sendEmail({
    to: "filip.delannoy@gmail.com",
    subject: "🏠 HVAC Dagelijkse Samenvatting - " +
      Utilities.formatDate(today, "Europe/Brussels", "dd/MM/yyyy"),
    body:
      "=== HVAC DAGELIJKSE SAMENVATTING ===\n\n" +
      "📊 Data logging:\n" +
      "  Entries: " + count + "/" + expected + " (" + pct + "%)\n" +
      "  Status: " + (pct > 95 ? "✓ Uitstekend" : pct > 80 ? "⚠ Matig" : "❌ Slecht") + "\n\n" +
      "🔥 Verwarming:\n" +
      "  Gem. vermogen: " + (sumHeat / count).toFixed(2) + " kW\n" +
      "  Piek vermogen: " + maxHeat.toFixed(2) + " kW\n\n" +
      "💧 Pompen:\n" +
      "  SCH actief: " + schPompCount + "x\n" +
      "  WON actief: " + wonPompCount + "x\n\n" +
      "💨 Ventilatie gem.: " + (sumVent / count).toFixed(0) + "%\n\n" +
      "🔌 Circuit duty cycles:\n" + circuitLines +
      "\nBekijk alle data: " + SpreadsheetApp.getActiveSpreadsheet().getUrl() + "\n"
  });

  Logger.log("Dagelijkse samenvatting verstuurd.");
}
