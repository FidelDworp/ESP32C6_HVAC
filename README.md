# ESP32-HVAC = Centrale HVAC Controller (Vervanging van Particle sketch: HVAC_Photon.cpp)

**FiDel, 2 januari 2026**

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

3. **Duty-cycle**:
   - Per circuit Ton/Toff bijgehouden → % berekend en getoond.

4. **Boiler monitoring**:
   - 12 hardcoded DS18B20 addresses.
   - Qtot berekening voor SCH en ECO (placeholder-formule, later kalibreren).

5. **ECO-overpompen**:
   - Automatisch als ECO Qtot > threshold (default 12 kWh) en er is verwarmingsvraag.
   - Hysteresis en max 5 min pomptijd.
   - Prioriteit SCH.

6. **Ventilatie**:
   - Max vent_percent van alle kamers → PWM op GPIO20.

7. **Webinterface**:
   - Hoofdpagina: status boilers, ventilatie, circuits (state + duty-cycle)
   - /settings: alle configuratie (namen, IP's rooms, thresholds, etc.) → persistent in NVS
   - /logdata: JSON voor Google Sheets pull (elke 10 min)

8. **Toekomst**:
   - Matter integratie (switches voor kleppen/pompen, fan voor ventilatie)

### Hardware

- ESP32-C6 devboard
- MCP23017 (adres 0x20) voor relays
- OneWire bus op GPIO3 (12 DS18B20)
- PWM ventilatie op GPIO20 → externe 0-10V converter
- I2C op GPIO13 (SDA) / GPIO11 (SCL)

### Installatie

1. Flash de sketch.
2. Eerste boot → AP "HVAC-Setup" → configureer WiFi.
3. Ga naar http://hvac.local/settings → vul room IP's/mDNS, circuitnamen, etc.
4. Save → reboot → klaar!
 
FiDel
Zarlardinge, België
