/* -S-HVAC-Schuur.ino = For Photon "HVAC_schuur".

Version:
- 5dec25: Removed more unnecessary particle.publish commands, Reduce frequency of HeatSetInterval = 60s ipv 5s.
- 21oct23: Removed unnecessary particle.publish commands
- 04mei23: Added auto transfer of excess solar heat from ECO to SCH boiler: 1) Catch ECO energy (kWh) then call ECOtransfer() function.
- 04apr21: Added R5-WASPL controller heating demand
- 10aug20: Error: HVAC kept restarting! => Used new controller names to be monitored: From line 1328: R1-BandB, R2-BADK, R3-INKOM.
- 29jan20: Corrected JSON string: Replaced variables for heatincircuit status (BBon etc...)
- 28jan20: Changed Duty-cycle program: Set initial Duty-cycle for all ON floors to 100% after 10 min ON time and OFF time still = 100 as initialized. Set OFF time to 200 to avoid repeating.
          + Modified commands for ventilation: FULL = 0
- 23dec19: Include relay status (LastState_R1 => R7) and HeatDemand in JSON string.
- 16dec19: Duty-cycle improvements: After 6h ON and OFF time = 0 change to DC=100, After 6h OFF and ON time = 0, change to DC=0, Add 7 DC values to JSON string
- 14dec19: Duty-cycle improvements: Update DC also when OFF, 15dec19: Include DC in JSON string.
- 8dec19: Modified all variables with millis() to type "uint32_t". Calculates & publishes Duty-cycle for all floor heating circuits.
- 7nov19: JSON string published for the status panel.
- 28oct19: New Sensor to replace defective TOP-H: 0x28,0xDB,0xB5,0x03,0x00,0x00,0x80,0xBB
- 14oct19: Created JSON publish string with all temperatures. Included also Av and Qtot.
- 13oct19: Activated the 12 DS18B20 temp sensors (6 for SCHuur, 6 for WONing).
  Attention: De-activated the 12 variables: Cause erratic restarts... => TO DO: Replace by a JSON string variable.
- 15apr19: Added VENTILATION control: EETPL, BandB, SLAK send their CO2 levels.
  We use pin A4 as PWM output. Function VENTilation() sets the analog output value (0-5V)
  For the electromagnetic valve opening duct b & c, we use "Relay 11" operated through the I2C connector: Pin 13 of the MCP23017 16port Expander chip.
  TEMPORARILY: Made 2 remote functions: venton/ventoff => Without CO2 sensors (fixed @1.2v max) Disabled function VENTilate() !!!
- 6feb19: Added automatic 5v power reset function (+ Counter variable for report) in case the remote thermostat rooms (BADK, INKOM, BandB) take too long to respond. (WASPL not yet!)
- 4feb19: Added 5V line reset (With SSR)
- 26jan19: Added BADK + INKOM thermostats and a concatenated report string for floor heating.

Includes following functions:
* Installation help:
- 1-wire address scanner: Lists all DS18B20 sensors on the T-Bus (D3)
* Floor heating control: 12 circuit valves + 1 pump; 3 modes: 1) HOME = Thermostat & Condens, 2) OUT = Condens, 3) MANUAL (for testing)
* Reporting:
- The floor heating circuits working a this moment are reported in one string.
- Transfer pumping between ECO boiler and 2 kelder boilers (SCH & WON)
- Energy reporting of both KELDER boilers: KS + KW.
  For 12 "DS18B20" sensors in both boilers. LABELS on sensors:
  1) KEL-WON: #1=KWTopH, #2=KWTopL, #3=KWMidH, #4=KWMidL, #5=KWBotH, #6=KWBotL
  2) KEL-SCH: #7=KSTopH, #8=KSTopL, #9=KSMidH, #10=KSMidL, #11=KSBotH, #12=KSBotL
- Calculating hourly heat energy added or consumed + accumulating total energy added or consumed for a full season.
- Variable(s) for this monitoring are retained in ROM memory so that the accumulated energy demand can be kept for a full year...
  => Make sure Vbat remains powered by a 3V Lithium button cell!

---------------------------------------
PhotoniX shield v.1.0	I/O connections: (* = used)

*D0 - I2C-SDA           => MCP23017 16 port expander for HVAC control (Currently one, later two?)
*D1 - I2C-SCL           => idem
 D2 - TOUCH-COM
*D3 - T-BUS             => 12+ temp sensors: Both KEL-SCH & KEL-WON Heating boilers (KS & KW) + later other "hotspots"...
 D4 - PIXELS            => Later: Cellar LIGHTS = 1 PowerPiXel (3 channels)
 D5 - RoomSense PIR
 D6 - RoomSense TEMP/HUM
 D7 - RoomSense GAS-DIG
 A0 - TOUCH-1
 A1 - TOUCH-2
 A2 - RoomSense GAS-ANA
 A3 - RoomSense LIGHT
 A4 - RF out
 A5 - OP3
 A6 - OP1
 A7 - OP2
 TX/RX - Serial comms
---------------------------------------

IO-eXboX connections
Note: Below list must be maintained also on the schematic drawing of the HVAC control system
---------------------------------------
IO-eXboX-1: MCP23017 (16 port expander) #1: SCHuur HVAC control
Code setting on PCB: A0, A1, A2 to GND => Address = 0X20

  0 - Heating solenoid Relay 1 "BB" (R wire)
  1 - Heating solenoid Relay 2 "WP" (O wire)
  2 - Heating solenoid Relay 3 "IK" (Y wire)
  3 - Heating solenoid Relay 4 "KK" (G wire)
  4 - Heating solenoid Relay 5 "EP" (Bu wire)
  5 - Heating solenoid Relay 6 "ZP" (Vi wire)
  6 - Heating solenoid Relay 7 "BK" (Gr wire)
  7 - Heating Pump feedback signal (W wire) => LOW = Relay 8 ON!
  8 - ECO-Pump Relay 9 "SCH" (Gr wire)
  9 - ECO-Pump Relay 10 "WON" (W wire)
 10 - Thermostat "ZP" (R wire) => LOW = Heating ON!
 11 - Thermostat "EP" (O wire) => LOW = Heating ON!
 12 - Thermostat "KK" (Y wire) => LOW = Heating ON!
 13 - VentValve1 Relay 11 (G wire)
 14 - VentValve2 Relay 12 (Bu wire) => Later expansion? (Not yet in use!)
 15 - SSR module for 5V power supply (Vi wire) => 5V reset: HIGH = Power ON, LOW = Power OFF
---------------------------------------
----------TEMPORARY!-------------------
IO-eXboX-2: MCP23017 (16 port expander) #2: WONing HVAC control (Currently not in use)
Code setting on PCB: A0 to Vcc, A1, A2 to GND => Address = 0X21
=> Currently hardwired... (Possibly later copy SCH system)
---------------------------------------


Hardware: One Photon on a PhotoniX shield. I2C connection with 2 IO-EXBOX modules. Both control the relays and receive the thermostat signals.

This sketch controls the HVAC in the barn (our new home) @ Zarlardinge.
- Solenoids and pump of the floor heating
- Pump between the solar boiler and floor heating boiler: When there is heating demand and excess heat (> 2 kW) in the ECO-boiler
    Practically: If heating demand of SCHUur is >0 (Pump = ON) and Qtot >2, pump 1 kWh to one heating boiler. Stop the pump when Qtot < 1 kW.

Inputs:
- 3 Hardwired thermostats in the "living space": ZP, EP, KK.
- 4 "Communicating" thermostats: BK, IK, BB, WP (WP controller not yet in use)
- 12 One-Wire Temp sensors: 6 in SCH boiler, 6 in WON boiler

Results:
- The system with the 8 SCHuur relays worked well for 6 months, except for the inductive pulses upsetting the sensitive I2C line...

TO DO:

* Lines temporarily commented out:
- Line 991: Temporarily disconnected until KW sensors are connected. //message = "Bad reading on Sensor: ";
- Line 995: Temporarily disconnected until KW sensors are connected. //Particle.publish("Alert", message + String(i), 60, PRIVATE);

1) "Manual" mode: Add a timer which switches back to HOME mode after an hour.
2) Operate the WONing ECO boiler pump: (= same like for SCHuur)
3) Control fan of the ventilation units:
- Monitor CO2 % in both houses
- Start fan of the unit where CO2% > 800 (Use the ANA-DIGI PCB with OpAmp)
4) Control the heat-pump energy management:
- Switch between FULL power and normal setting: FULL if there is sunshine (solar energy) and heat demand)

*/

STARTUP(WiFi.selectAntenna(ANT_AUTO)); // Select an antenna for the Photon: "AUTO" continually switches at high speed between antennas
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

// *COMMON

 // Strings for publishing
 char str[255]; // Temporary string for all messages
 char JSON_hvac[400]; // Publish all temperature variables in one string

 char myText [40]; // String to publish status of all ON heating circuits
 double wifiRSSI;// WiFi reception
 double wifiPERCENT;

//Initialize libraries:
// *D0 + D1 = I2C-SDA + SCL
 #include <Adafruit_MCP23017.h>
 Adafruit_MCP23017 mcp1; // First instance MCP chip (For SCHuur)
 //Adafruit_MCP23017 mcp2; // Second instance MCP chip => Later for WONing...


// *D3 - T-BUS (12 temp sensors)
 #include <OneWire.h>
 const int oneWirePin = D3; // Compatible to PhotoniX shield...
 OneWire ds = OneWire(oneWirePin);


//Initialize global variables:
// *D0 + D1 = I2C-SDA + SCL
 // Heating
 int HeatMode = 1;  // 1 = "HOME" using the Thermostats, 2 = "OUT" with Room temperature monitoring, 3 = "MANUAL": Manual control only (For testing)
 double kilowatts = 0;  // To calculate max heating demand
 double heatdemand; // Momentary heat energy demand: Total (max) power dissipation of circuits circulating hot water
 uint32_t HeatSetInterval = 60 * 1000; // Set heating & publish every 5 s
 uint32_t HeatSetLastTime = millis() - HeatSetInterval;  // Make function run as soon as loop() starts
 uint32_t HeatTransferInterval = 1 * 60 * 1000; // Check transfer ECO boiler energy to heating boiler(s) every 1 min
 uint32_t HeatTransferLastTime = millis() - HeatTransferInterval;  // Make function run as soon as loop() starts
 int BusErrorCount = 0; // In case of I2C bus error: Count till 5 errors before rebooting controller

 // Monitoring "remote thermostats": BB, IK, BK (WP not yet!)
 uint32_t TstatChkInterval = 10 * 60 * 1000; // If remote Tstat reporting takes longer than this, reset 5v power line @ end of loop().
 uint32_t BandBChkLastTime = millis(); // BandB remote monitoring last time.
 uint32_t INKOMChkLastTime = millis(); // INKOM remote monitoring last time.
 uint32_t BADKChkLastTime = millis(); // BADK remote monitoring last time.
 uint32_t WASPLChkLastTime = millis(); // WASPL remote monitoring last time.
 int BandBResetCount = 0;  // To record Nr of 5v power resets due to BandB controller
 int INKOMResetCount = 0;  // To record Nr of 5v power resets due to INKOM controller
 int BADKResetCount = 0;  // To record Nr of 5v power resets due to BADK controller
 int WASPLResetCount = 0;  // To record Nr of 5v power resets due to WASPL controller

 // ECO boiler pumping
 float ECOQtot = 0; // Create a global variable for the ECO boiler energy: The eventDecoder() creates it also but it's a local variable...
 float EQtotStart = 0; // ECO boiler energy before pumping
 float EQtotStop = 0; // ECO boiler energy after pumping
 float Qpumped = 0; // Pumped ECO boiler energy after pumping
 retained int QECOSCH; // TOTAL received ECO energy in SCH boiler (Retained in memory!)
 retained int QECOWON; // TOTAL received ECO energy in WON boiler (Retained in memory!)
 uint32_t MaxECOpumpTime =  5 * 60 * 1000; // Maximum ECO boiler pumping time = 5 min (Stop if the bottom limit is not reached)
 uint32_t ECOpumpSCHLastTime = millis();
 uint32_t ECOpumpWONLastTime = millis();

 // Heating demand reporting
 uint32_t getDemandInterval = 60 * 1000; // Sample energy Demand: Every 1 minute! (Important: Keep this 1 minute, as energy calculation is based on this sampling)
 uint32_t getDemandLastTime = millis();
 retained double total_Demand = 0; // Accumulated energy Demand (TOTAL) every minute (Retained in memory!)
 retained double hour_Demand = 0; // Accumulated energy Demand during last hour (Retained in memory!)
 retained double prev_Demand = 0; // Accumulated energy Demand (TOTAL) exactly on the previous hour (Retained in memory!)

  // Floorheating circuit BB Duty-Cycle:
  double BB_Ton = 1; // To make initial D/C small
  double BB_Toff = 100; // To avoid dangerous division by 0!
  uint32_t BB_Laston = 0;
  uint32_t BB_Lastoff = 0;
  double BB_DC = 0;
  // Floorheating circuit WP Duty-Cycle:
  double WP_Ton = 1; // To make initial D/C small
  double WP_Toff = 100; // To avoid dangerous division by 0!
  uint32_t WP_Laston = 0;
  uint32_t WP_Lastoff = 0;
  double WP_DC = 0;
  // Floorheating circuit BK Duty-Cycle:
  double BK_Ton = 1; // To make initial D/C small
  double BK_Toff = 100; // To avoid dangerous division by 0!
  uint32_t BK_Laston = 0;
  uint32_t BK_Lastoff = 0;
  double BK_DC = 0;
  // Floorheating circuit ZP Duty-Cycle:
  double ZP_Ton = 1; // To make initial D/C small
  double ZP_Toff = 100; // To avoid dangerous division by 0!
  uint32_t ZP_Laston = 0;
  uint32_t ZP_Lastoff = 0;
  double ZP_DC = 0;
  // Floorheating circuit EP Duty-Cycle:
  double EP_Ton = 1; // To make initial D/C small
  double EP_Toff = 100; // To avoid dangerous division by 0!
  uint32_t EP_Laston = 0;
  uint32_t EP_Lastoff = 0;
  double EP_DC = 0;
  // Floorheating circuit KK Duty-Cycle:
  double KK_Ton = 1; // To make initial D/C small
  double KK_Toff = 100; // To avoid dangerous division by 0!
  uint32_t KK_Laston = 0;
  uint32_t KK_Lastoff = 0;
  double KK_DC = 0;
  // Floorheating circuit IK Duty-Cycle:
  double IK_Ton = 1; // To make initial D/C small
  double IK_Toff = 100; // To avoid dangerous division by 0!
  uint32_t IK_Laston = 0;
  uint32_t IK_Lastoff = 0;
  double IK_DC = 0;

 // Initialize "remote thermostats": As long as no message is received we assume they're OFF
 bool ThermostatBB = 0; // BandB Photon publishes this
 bool ThermostatWP = 0; // WASPL Photon publishes this
 bool ThermostatIK = 0; // INKOM Photon publishes this
 bool ThermostatBK = 0; // BADK Photon publishes this

 // Initialize "remote condensation protection": As long as no message is received we assume they're OFF
 bool CondensProtBB = 0; // BandB room
 bool CondensProtWP = 0; // WASPL room
 bool CondensProtBK = 0; // BADK room
 bool CondensProtZP = 0; // ZITPL room
 bool CondensProtEP = 0; // EETPL room
 bool CondensProtKK = 0; // KEUK room
 bool CondensProtIK = 0; // INKOM room

 // Turn room heating ON (1) or OFF (0)
 bool BBon = 0; // BandB room
 bool WPon = 0; // WASPL room
 bool BKon = 0; // BADK room
 bool ZPon = 0; // ZITPL room
 bool EPon = 0; // EETPL room
 bool KKon = 0; // KEUK room
 bool IKon = 0; // INKOM room
 bool SCHon = 0; // ECO => SCH Pump
 bool WONon = 0; // ECO => SCH Pump

 // "state" Variables of the 7 heating solenoid control Relays: Put them all in OFF state so that the relay state is published when it first turns ON
 bool LastState_R1 = 0;
 bool LastState_R2 = 0;
 bool LastState_R3 = 0;
 bool LastState_R4 = 0;
 bool LastState_R5 = 0;
 bool LastState_R6 = 0;
 bool LastState_R7 = 0;
 bool LastState_R9 = 0;// ECO => SCH Pump
 bool LastState_R10 = 0;// ECO => WON Pump


// *D3 - T-BUS (12 temp sensors)

 // Group sensor addresses of different types in their bus (on one I/O pin) ; [12][8] means: 12 sensor addresses with 8 bytes. (Attention: Inactive lines are "//commented out"!)
 // 1) Store addresses of DS18B20 sensors (Starting with 0x28,) here and activate "getTemperatures(0);" in loop() function:
 byte addrs0[12][8] =
 {{0x28,0xDB,0xB5,0x03,0x00,0x00,0x80,0xBB},
  {0x28,0x7C,0xF0,0x03,0x00,0x00,0x80,0x59},
  {0x28,0x72,0xDB,0x03,0x00,0x00,0x80,0xC2},
  {0x28,0xAA,0xFB,0x03,0x00,0x00,0x80,0x5F},
  {0x28,0x49,0xDD,0x03,0x00,0x00,0x80,0x4B},
  {0x28,0xC3,0xD6,0x03,0x00,0x00,0x80,0x1E},
  {0x28,0x3A,0xBC,0x07,0x00,0x00,0x80,0x58},
  {0x28,0x72,0x03,0x04,0x00,0x00,0x80,0x24},
  {0x28,0xD4,0xE7,0x03,0x00,0x00,0x80,0x89},
  {0x28,0x78,0xF9,0x03,0x00,0x00,0x80,0x76},
  {0x28,0x70,0xAD,0x07,0x00,0x00,0x80,0x53},
  {0x28,0x40,0xE1,0x03,0x00,0x00,0x80,0x78}}; // = For 12 "DS18B20" sensors in both "KEL-WON" & "KEL-SCH" boilers.

 // 2) Store addresses of DS18S20 sensors (Starting with 0x10,) here and activate "getTemperatures(1);" in loop() function:
 byte addrs1[3][8] = {{},{},{}}; // = For a number of TO92 "DS18S20" sensors (Currently none used)

 // Initialize the variable names of the sensors as "double" variables: (=> can be published as "Particle.variables")
 double KSTopH, KSTopL, KSMidH, KSMidL, KSBotH, KSBotL, KWTopH, KWTopL, KWMidH, KWMidL, KWBotH, KWBotL; // Names of the 12 sensors, stored in the above array(s). 12 sensors (0,1,2,3,4,5,6,7,8,9,10,11) in the KEL-SCH + KEL-WON boilers
 double* temps[] = {&KSTopH, &KSTopL, &KSMidH, &KSMidL, &KSBotH, &KSBotL, &KWTopH, &KWTopL, &KWMidH, &KWMidL, &KWBotH, &KWBotL}; // Group1: 12 waterproof sensors

 // Initialize the globals for time stamp (Faulty sensor reporting via CRC checking):
 char crcErrorJSON[128];
 int crcErrorCount[sizeof(temps)/sizeof(temps[0])];
 uint32_t tmStamp[sizeof(temps)/sizeof(temps[0])];

 // Initialize the global variables for temperature calculations:
 uint32_t getTemperaturesInterval = 10 * 1000; // Sample rate for temperatures = 10s
 uint32_t getTemperaturesLastTime = millis() - getTemperaturesInterval; // Reset interval to sample immediately at start-up!
 double celsius;
 double KSTmin = 25; // Minimum KS boiler temperature to calculate "spare" energy (Securing Floor Heating)
 double KWTmin = 25; // Minimum KW boiler temperature to calculate "spare" energy (Securing Floor Heating)
 double KSAv1, KSAv2, KSAv3, KSAv4, KSAv5, KSAv, KWAv1, KWAv2, KWAv3, KWAv4, KWAv5, KWAv; // To store average temperatures
 double KSQ1, KSQ2, KSQ3, KSQ4, KSQ5, KSQtot, KWQ1, KWQ2, KWQ3, KWQ4, KWQ5, KWQtot; // To store "spare" energy

 // For energy reporting of KEL-SCH boiler:
 double prev_KSQtot = 0;
 uint32_t getKSplusInterval = 60 * 1000; // Sample rate for energy Demand: Every 1 minute
 uint32_t getKSplusLastTime = millis();
 retained double total_KSplus = 0; // Accumulated energy Demand (TOTAL) every minute
 retained double hour_KSplus = 0; // Accumulated energy Demand during last hour
 retained double prev_H_KSplus = 0; // Accumulated energy Demand (TOTAL) exactly on the previous hour

 // For energy reporting of KEL-WON boiler:
 double prev_KWQtot = 0;
 uint32_t getKWplusInterval = 60 * 1000; // Sample rate for energy Demand: Every 1 minute
 uint32_t getKWplusLastTime = millis() + 1000; // 1s delay not to start together with KEL-SCH...
 retained double total_KWplus = 0; // Accumulated energy Demand (TOTAL) every minute
 retained double hour_KWplus = 0; // Accumulated energy Demand during last hour
 retained double prev_H_KWplus = 0; // Accumulated energy Demand (TOTAL) exactly on the previous hour

 // For VENTILATION control:
 int fancontrolPin = A4; // We use pin A4 as PWM output (Available: A4, A5, A7, D0, D1, D2, D3, RX, TX)
 int CO2a = 0; // CO2 excess level (received from EETPL minus 400)
 int CO2b = 0; // CO2 excess level (received from BandB minus 400)
 int CO2c = 0; // CO2 excess level (received from SLAK minus 400)
 int Fanspeed; // This value is the sum of all CO2 excess levels. It determines the ventilation fan speed.




void setup()
{
// *GENERAL Settings:
  Serial.begin(9600);
  Time.zone(+1); // Set clock to Belgium time

// *D0 + D1 = I2C-SDA + SCL
  mcp1.begin(1); // First instance MCP chip (0 = 0X21) => For schuur
  //mcp2.begin(0); // Second instance MCP chip (0 = 0X20) => Later for WONing...

  // Initialize the MCP23017 I/O pins as INPUT (turn on 100k pullup) or OUTPUT (set to HIGH):
  // IC Side 1: Pins 0 to 7
  mcp1.pinMode(0, OUTPUT); mcp1.digitalWrite(0, HIGH);// Relay 1 = Kring 1 & 2 (BB)
  mcp1.pinMode(1, OUTPUT); mcp1.digitalWrite(1, HIGH);// Relay 2 = Kring 3 (WP)
  mcp1.pinMode(2, OUTPUT); mcp1.digitalWrite(2, HIGH);// Relay 3 = Kring 4&5 (IK)
  mcp1.pinMode(3, OUTPUT); mcp1.digitalWrite(3, HIGH);// Relay 4 = Kring 6&7 (KK)
  mcp1.pinMode(4, OUTPUT); mcp1.digitalWrite(4, HIGH);// Relay 5 = Kring 8 & 9 (EP)
  mcp1.pinMode(5, OUTPUT); mcp1.digitalWrite(5, HIGH);// Relay 6 = Kring 10 (ZP)
  mcp1.pinMode(6, OUTPUT); mcp1.digitalWrite(6, HIGH);// Relay 7 = Kring 11 & 12 (BK)
  mcp1.pinMode(7, INPUT); mcp1.pullUp(7, HIGH); // Relay 8 = PUMP => Hardwired HIGH when any relay output is HIGH (using 7 diodes) => Gives pump relay state Feedback: 0 = "HEATING DEMAND"
  // IC Side 2: Pins 8 to 15
  mcp1.pinMode(8, OUTPUT); mcp1.digitalWrite(8, HIGH); // Relay 9 = ECO PUMP relay SCH
  mcp1.pinMode(9, OUTPUT); mcp1.digitalWrite(9, HIGH); // Relay 10 = ECO PUMP relay WON
  mcp1.pinMode(10, INPUT); mcp1.pullUp(10, HIGH);// Thermostat 6 (ZP)
  mcp1.pinMode(11, INPUT); mcp1.pullUp(11, HIGH);// Thermostat 5 (EP)
  mcp1.pinMode(12, INPUT); mcp1.pullUp(12, HIGH);// Thermostat 4 (KK) => Temporarily operates also IK
  mcp1.pinMode(13, INPUT); mcp1.pullUp(13, HIGH);// N/C (Use for lighting)
  mcp1.pinMode(14, INPUT); mcp1.pullUp(14, HIGH);// N/C (Use for lighting)
  mcp1.pinMode(15, OUTPUT); mcp1.digitalWrite(15, HIGH);// NEW: For 5V reset. (Initial setting HIGH => Power = ON)


  // Initialize Particle variables:
  Particle.variable("JSON_hvac", JSON_hvac, STRING); // Publishes all system temperatures

  Particle.variable("Heatdemand", heatdemand);
  Particle.variable("Hourlydemand", hour_Demand);
  //Particle.variable("ECO-energy SCH", QECOSCH);// For SCHuur => ERROR! I tried also with "double", still not!
  //Particle.variable("ECO-energy WON", QECOWON);// For WONing

  //As Particle.variable does not work: Alternative for publishing retained variables:
  sprintf(str, "Pumped to SCH, WON = %2.2f, %2.2f", QECOSCH, QECOWON);
  //Particle.publish("Status-HEAT:HVAC", str,60,PRIVATE);

  // Initialize Particle functions:
  Particle.function("Manual", manual);

  // Initialize Particle subscribe function:
  Particle.subscribe("Status-HEAT", eventDecoder, MY_DEVICES); // Subscribe will listen for the event "Status-HEAT" and, when it receives it, it will run the function eventDecoder()


// *D3 - T-BUS (12 temp sensors)
 // @BulldogLowell: Initialize the timestamp array: prevent wrong messages if you get a bad CRC error on the first reading after startup...
 for (int i = 0; i < sizeof(temps)/sizeof(temps[0]); i++)
 {
   tmStamp[i] = Time.now();
 }

 // Report the CRC errors with sensor ID:
  //Particle.variable("CRC_Errors", crcErrorJSON, STRING); // For debugging; Creates array of errorcounts of all active sensors. Example: {"errorCount":[17,4,4,14,8,3]} => 17 = sensor 0, 4 = sensor 1, etc...

 // Ventilation control:
  pinMode(fancontrolPin, OUTPUT);

 // Turn ventilation ON initially:
  analogWrite(fancontrolPin, 120); // Fanspeed: 255 - 0 => 0 - 80% fan speed)

}  // END setup





void loop()
{
// *D0 + D1 = I2C-SDA + SCL
 // Set the heating in all rooms
 if ((millis()-HeatSetLastTime)>HeatSetInterval)
 {
   SetHeating();//Check all thermostats and switch related solenoids ON/OFF accordingly

   // ATTENTION: Reset Photon if problem with I2C bus persists (After 4 errors)
    if (BusErrorCount > 4)
    {
      System.reset();
    }

   HeatSetLastTime = millis(); // Reset the timer
 }
 delay(50);

 // Collect & report heat demand data
 if ((millis()-getDemandLastTime)>getDemandInterval)
 {
   getDemand();
   getDemandLastTime = millis();
  }
 delay(50);

// *D3 - T-BUS (Get all system temperatures: 2 Boilers + 2 Heat Pumps, 2 ECO Pumps, 2 Floor heating pumps (= 2x 12 sensors)
 if ((millis()-getTemperaturesLastTime)>getTemperaturesInterval) // Set the sampling rate in "getTemperaturesInterval"
 {
  // Read sensor values in array 0 and 1. (The argument selects the array of addresses (addrs0, addrs1 ...) in the "getTemperatures" function
  // TEMPORARY: Switch off next line. Activate again when boiler sensors are installed.
  getTemperatures(0); // Update all sensor variables of array 0 (DS18B20 type)
  //getTemperatures(1); // Update all sensor variables of array 1 (DS18S20 type) => Currently not used...

  // 294 liter KS Boiler:
  // Calculate the average KS tank temperature in each of the 5 zones:
  KSAv1 = (KSTopH + KSTopL)/2;
  KSAv2 = (KSTopL + KSMidH)/2;
  KSAv3 = (KSMidH + KSMidL)/2;
  KSAv4 = (KSMidL + KSBotH)/2;
  KSAv5 = (KSBotH + KSBotL)/2;
  KSAv = (KSAv1+KSAv2+KSAv3+KSAv4+KSAv5)/5;

  // Calculate the "spare" 294 liter KS tank energy in each of the 5 zones:
  KSQ1 = (KSAv1-KSTmin)*66*1.163/1000; // Spare energy in 66 liter top zone (kWh)
  KSQ2 = (KSAv2-KSTmin)*54*1.163/1000; // Spare energy in 54 liter top/mid zone (kWh)
  KSQ3 = (KSAv3-KSTmin)*54*1.163/1000; // Spare energy in 54 liter middle zone (kWh)
  KSQ4 = (KSAv4-KSTmin)*54*1.163/1000; // Spare energy in 54 liter mid/bottom zone (kWh)
  KSQ5 = (KSAv5-KSTmin)*66*1.163/1000; // Spare energy in 66 liter bottom zone (kWh)
  KSQtot = KSQ1+KSQ2+KSQ3+KSQ4+KSQ5; // Total spare energy in tank (kWh)

  // 294 liter KW Boiler:
  // Calculate the average KW tank temperature in each of the 5 zones:
  KWAv1 = (KWTopH + KWTopL)/2;
  KWAv2 = (KWTopL + KWMidH)/2;
  KWAv3 = (KWMidH + KWMidL)/2;
  KWAv4 = (KWMidL + KWBotH)/2;
  KWAv5 = (KWBotH + KWBotL)/2;
  KWAv = (KWAv1+KWAv2+KWAv3+KWAv4+KWAv5)/5;

  // Calculate the "spare" 294 liter KW tank energy in each of the 5 zones:
  KWQ1 = (KWAv1-KWTmin)*66*1.163/1000; // Spare energy in 66 liter top zone (kWh)
  KWQ2 = (KWAv2-KWTmin)*54*1.163/1000; // Spare energy in 54 liter top/mid zone (kWh)
  KWQ3 = (KWAv3-KWTmin)*54*1.163/1000; // Spare energy in 54 liter middle zone (kWh)
  KWQ4 = (KWAv4-KWTmin)*54*1.163/1000; // Spare energy in 54 liter mid/bottom zone (kWh)
  KWQ5 = (KWAv5-KWTmin)*66*1.163/1000; // Spare energy in 66 liter bottom zone (kWh)
  KWQtot = KWQ1+KWQ2+KWQ3+KWQ4+KWQ5; // Total spare energy in tank (kWh)

  // Update the Temperatures JSON string:
  snprintf(JSON_hvac,400,"{\"KSTopH\":%.1f,\"KSTopL\":%.1f,\"KSMidH\":%.1f,\"KSMidL\":%.1f,\"KSBotH\":%.1f,\"KSBotL\":%.1f,\"KSAv\":%.1f,\"KSQtot\":%.3f,\"KWTopH\":%.1f,\"KWTopL\":%.1f,\"KWMidH\":%.1f,\"KWMidL\":%.1f,\"KWBotH\":%.1f,\"KWBotL\":%.1f,\"KWAv\":%.1f,\"KWQtot\":%.3f,\"BB\":%.0f,\"WP\":%.0f,\"BK\":%.0f,\"ZP\":%.0f,\"EP\":%.0f,\"KK\":%.0f,\"IK\":%.0f,\"R1\":%d,\"R2\":%d,\"R3\":%d,\"R4\":%d,\"R5\":%d,\"R6\":%d,\"R7\":%d,\"HeatDem\":%.1f}",KSTopH,KSTopL,KSMidH,KSMidL,KSBotH,KSBotL,KSAv,KSQtot,KWTopH,KWTopL,KWMidH,KWMidL,KWBotH,KWBotL,KWAv,KWQtot,BB_DC,WP_DC,BK_DC,ZP_DC,EP_DC,KK_DC,IK_DC,BBon,WPon,BKon,ZPon,EPon,KKon,IKon,heatdemand);
  //Particle.publish("Status-HEAT:HVAC", JSON_hvac,60,PRIVATE);
  delay(500);

  // Publish energy levels in 5 layers of the boilers in 1 string:
  sprintf(str, "*KS: %2.1f,%2.1f,%2.1f,%2.1f,%2.1f=%2.2f(%2.0f)",KSQ1,KSQ2,KSQ3,KSQ4,KSQ5,KSQtot,KSAv);
  //Particle.publish("Status-HEAT:HVAC", str,60,PRIVATE);
  delay(500);
  sprintf(str, "*KW: %2.1f,%2.1f,%2.1f,%2.1f,%2.1f=%2.2f(%2.0f)",KWQ1,KWQ2,KWQ3,KWQ4,KWQ5,KWQtot,KWAv);
  //Particle.publish("Status-HEAT:HVAC", str,60,PRIVATE);

  // Reset the timer
  getTemperaturesLastTime = millis();
 }

 // For energy reporting of KEL-SCH boiler:
  if ((millis()-getKSplusLastTime)>getKSplusInterval)
  {
    getKSplus();
    getKSplusLastTime = millis();
  }

 // For energy reporting of KEL-WON boiler:
  if ((millis()-getKWplusLastTime)>getKWplusInterval)
  {
    getKWplus();
    getKWplusLastTime = millis();
  }

  // If one of the remote tstats did not react recently, restart the 5v power line!
  if ((millis()-BandBChkLastTime)>TstatChkInterval)
  {
    manual("reset5v");
    BandBResetCount = BandBResetCount + 1;
    Particle.publish("Status-ALERT:HVAC", "ALERT: reset5v for BandB!",60,PRIVATE);
    BandBChkLastTime = millis();
  }

  if ((millis()-INKOMChkLastTime)>TstatChkInterval)
  {
    manual("reset5v");
    INKOMResetCount = INKOMResetCount + 1;
    Particle.publish("Status-ALERT:HVAC", "ALERT: reset5v for INKOM!",60,PRIVATE);
    INKOMChkLastTime = millis();
  }

  if ((millis()-BADKChkLastTime)>TstatChkInterval)
  {
    manual("reset5v");
    BADKResetCount = BADKResetCount + 1;
    Particle.publish("Status-ALERT:HVAC", "ALERT: reset5v for BADK!",60,PRIVATE);
    BADKChkLastTime = millis();
  }

  if ((millis()-WASPLChkLastTime)>TstatChkInterval) // Activate only when WASPL is in use!
  {
    manual("reset5v");
    WASPLResetCount = WASPLResetCount + 1;
    Particle.publish("Status-ALERT:HVAC", "ALERT: reset5v for WASPL!",60,PRIVATE);
    WASPLChkLastTime = millis();
  }

} // END loop



// FUNCTIONS:


// *D0 + D1 = I2C-SDA + SCL
void SetHeating() // Checks thermostats + manual input and switches Relays to a different state. Publishes only what is ON.
{ // Attention: Create minimal I2C communication and publishing to avoid I2C bus and publishing overflow! Check state of relays and only send command to change state...

/* For HeatMode-1 "home": Thermostats & condensation rule! (Hard-wired or remote) Mode = Activated if both our iPhones are within 100 km radius from home. ("Life360" app + IFTTT)
   Heating is ON if thermostat is ON OR if condensprotection is ON. This is set for every room in the eventDecoder() function. */
  if (HeatMode == 1)
  {
      Particle.publish("Status-HEAT:HVAC","Heatingmode HOME",60,PRIVATE);
      delay(1000);

      // Thermostat 1 (REMOTE): Relay 1 = Kring 1 & 2 (BandB) = 1254 W (BB Photon publishes events!)
      if (ThermostatBB == 1 || CondensProtBB == 1) // We heat also if we are close to the condensation limit!
      {
        BBon = 1;
      }
      else
      {
        BBon = 0;
      }

      // Thermostat 2 (REMOTE): Relay 2 = Kring 3 (WASPL) = 1096 W (WASPL Photon publishes events!)
      if (ThermostatWP == 1 || CondensProtWP == 1) // We heat also if we are close to the condensation limit!
      {
        WPon = 1;
      }
      else
      {
        WPon = 0;
      }

      // Thermostat 3 (REMOTE): Relay 3 = Kring 4&5 (INKOM) = 1025 W (INKOM Photon publishes events!)
      if (ThermostatIK == 1 || CondensProtIK == 1) // We heat also if we are close to the condensation limit!
      {
        IKon = 1;
      }
      else
      {
        IKon = 0;
      }

      // Thermostat 4 (Wired to HVAC controller): Relay 4 = Kring 6&7 (KEUK) = 1018 W
      if (mcp1.digitalRead(12) == LOW || CondensProtKK == 1) // We heat also if we are close to the condensation limit!)
      {
        KKon = 1;
      }
      else
      {
        KKon = 0;
      }

      // Thermostat 5 (Wired to HVAC controller): Relay 5 = Kring 8 & 9 (EETPL) = 916 W
      if (mcp1.digitalRead(11) == LOW || CondensProtEP == 1) // We heat also if we are close to the condensation limit!)
      {
        EPon = 1;
      }
      else
      {
        EPon = 0;
      }

      // Thermostat 6 (Wired to HVAC controller): Relay 6 = Kring 10 (ZITPL) = 460 W
      if (mcp1.digitalRead(10) == LOW || CondensProtZP == 1) // We heat also if we are close to the condensation limit!
      {
        ZPon = 1;
      }
      else
      {
        ZPon = 0;
      }

      // Thermostat 7 (REMOTE): Relay 7 = Kring 11 & 12 (BADK) = 832 W (BADK Photon publishes events!)
      if (ThermostatBK == 1 || CondensProtBK == 1) // We heat also if we are close to the condensation limit!
      {
        BKon = 1;
      }
      else
      {
        BKon = 0;
      }
  }


  /* In HeatMode-2 => "Out" = Economy mode: Check all room temperatures and compare with set points!
  Check actual temperature and heat to a safe level, a few °C above condensation limit.
  Active if we are further out of above range (circle of 100 km). */

  if (HeatMode == 2)
  {
    // Currently same like "Manual" mode. TO DO: Roomtemperatures can be regulated to a safe margin above the dew point level
    Particle.publish("Status-HEAT:HVAC","Heatingmode OUT (= still Manual)",60,PRIVATE);
    delay(1000);

      // Condensation protection: Relay 1 = Kring 1 & 2 (BandB) = 1254 W (BandB Photon publishes events!)
      if (CondensProtBB == 1) // We heat also if we are close to the condensation limit!
      {
        BBon = 1;
      }
      else
      {
        BBon = 0;
      }

      // Condensation protection: Relay 2 = Kring 3 (WASPL) = 1096 W (WASPL Photon publishes events!)
      if (CondensProtWP == 1) // We heat also if we are close to the condensation limit!
      {
        WPon = 1;
      }
      else
      {
        WPon = 0;
      }

      // Condensation protection: Relay 3 = Kring 4&5 (INKOM) = 1025 W (INKOM Photon publishes events!)
      if (CondensProtIK == 1) // We heat also if we are close to the condensation limit!
      {
        IKon = 1;
      }
      else
      {
        IKon = 0;
      }

      // Condensation protection: Relay 4 = Kring 6&7 (KEUK) = 1018 W (KEUK Photon publishes events!)
      if (CondensProtKK == 1) // We heat also if we are close to the condensation limit!)
      {
        KKon = 1;
      }
      else
      {
        KKon = 0;
      }

      // Condensation protection: Relay 5 = Kring 8 & 9 (EETPL) = 916 W (EETPL Photon publishes events!)
      if (CondensProtEP == 1) // We heat also if we are close to the condensation limit!)
      {
        EPon = 1;
      }
      else
      {
        EPon = 0;
      }

      // Condensation protection: Relay 6 = Kring 10 (ZITPL) = 460 W (ZITPL Photon publishes events!)
      if (CondensProtZP == 1) // We heat also if we are close to the condensation limit!
      {
        ZPon = 1;
      }
      else
      {
        ZPon = 0;
      }

      // Condensation protection: Relay 7 = Kring 11 & 12 (BADK) = 832 W (BADK Photon publishes events!)
      if (CondensProtBK == 1) // We heat also if we are close to the condensation limit!
      {
        BKon = 1;
      }
      else
      {
        BKon = 0;
      }
  }

  // In HeatMode-3 => "Manual": Switch heating on in each room with a command and switch off after certain time!
  if (HeatMode == 3)
  {
    // Do nothing: The relays can be switched remotely.
    Particle.publish("Status-HEAT:HVAC","Heatingmode MANUAL",60,PRIVATE);
    delay(1000);
    // TO DO: Add a timer which switches OFF after a few hours...
  }

  // ACTIVATE FLOORHEATING RELAYS if they have been "named" ON... (Common for ALL HeatModes)
  strncpy(myText, "*VV: ", sizeof(myText)); // Create the string to collect floorheating ("VV") ON circuits
  kilowatts = 0; // Reset before collecting heat demand...


  // Relay 1 = Kring 1 & 2 (BandB) = 1254 W
  if (BBon == 1)
  {
    if (!LastState_R1)
    {
      mcp1.digitalWrite(0, LOW);
      LastState_R1 = 1;
      // Reset ON time counter and record new OFF time
      BB_Laston = millis();
      BB_Toff = millis() - BB_Lastoff; // OFF time (ms)
      // Calculate new circuit Duty-Cycle:
      BB_DC = (BB_Ton * 100)/(BB_Ton + BB_Toff);
    }
    strncat(myText, " BB", sizeof(myText)); // Concatenate current string with name of this circuit
    kilowatts = kilowatts + 1.254;
    delay(1000);
  }
  else
  {
    if (LastState_R1)
    {
      mcp1.digitalWrite(0, HIGH);
      LastState_R1 = 0;
      // Reset OFF time counter and record new ON time:
      BB_Lastoff = millis();
      BB_Ton = millis() - BB_Laston; // ON time (ms)
      // Calculate new circuit Duty-Cycle:
      BB_DC = (BB_Ton * 100)/(BB_Ton + BB_Toff);
    }
  }

  // Relay 2 = Kring 3 (WASPL) = 1096 W
  if (WPon == 1)
  {
    if (!LastState_R2)
    {
      mcp1.digitalWrite(1, LOW);
      LastState_R2 = 1;
      // Reset ON time counter and record new OFF time
      WP_Laston = millis();
      WP_Toff = millis() - WP_Lastoff; // OFF time (ms)
      // Calculate new circuit Duty-Cycle:
      WP_DC = (WP_Ton * 100)/(WP_Ton + WP_Toff);
    }
    strncat(myText, " WP", sizeof(myText)); // Concatenate current string with name of this circuit
    kilowatts = kilowatts + 1.096;
    delay(1000);
  }
  else
  {
    if (LastState_R2)
    {
      mcp1.digitalWrite(1, HIGH);
      LastState_R2 = 0;
      // Reset OFF time counter and record new ON time:
      WP_Lastoff = millis();
      WP_Ton = millis() - WP_Laston; // ON time (ms)
      // Calculate new circuit Duty-Cycle:
      WP_DC = (WP_Ton * 100)/(WP_Ton + WP_Toff);
    }
  }

  // Relay 3 = Kring 4&5 (INKOM) = 1025 W
  if (IKon == 1)
  {
    if (!LastState_R3)
    {
      mcp1.digitalWrite(2, LOW);
      LastState_R3 = 1;
      // Reset ON time counter and record new OFF time
      IK_Laston = millis();
      IK_Toff = millis() - IK_Lastoff; // OFF time (ms)
      // Calculate new circuit Duty-Cycle:
      IK_DC = (IK_Ton * 100)/(IK_Ton + IK_Toff);
    }
    strncat(myText, " IK", sizeof(myText)); // Concatenate current string with name of this circuit
    kilowatts = kilowatts + 1.025;
    delay(1000);
  }
  else
  {
    if (LastState_R3)
    {
      mcp1.digitalWrite(2, HIGH);
      LastState_R3 = 0;
      // Reset OFF time counter and record new ON time:
      IK_Lastoff = millis();
      IK_Ton = millis() - IK_Laston; // ON time (ms)
      // Calculate new circuit Duty-Cycle:
      IK_DC = (IK_Ton * 100)/(IK_Ton + IK_Toff);
    }
  }

  // Relay 4 = Kring 6&7 (KEUK) = 1018 W (Thermostat 4)
  if (KKon == 1)
  {
    if (!LastState_R4)
    {
      mcp1.digitalWrite(3, LOW);
      LastState_R4 = 1;
      // Reset ON time counter and record new OFF time
      KK_Laston = millis();
      KK_Toff = millis() - KK_Lastoff; // OFF time (ms)
      // Calculate new circuit Duty-Cycle:
      KK_DC = (KK_Ton * 100)/(KK_Ton + KK_Toff);
    }
    strncat(myText, " KK", sizeof(myText)); // Concatenate current string with name of this circuit
    kilowatts = kilowatts + 1.018;
    delay(1000);
  }
  else
  {
    if (LastState_R4)
    {
      mcp1.digitalWrite(3, HIGH);
      LastState_R4 = 0;
      // Reset OFF time counter and record new ON time:
      KK_Lastoff = millis();
      KK_Ton = millis() - KK_Laston; // ON time (ms)
      // Calculate new circuit Duty-Cycle:
      KK_DC = (KK_Ton * 100)/(KK_Ton + KK_Toff);
    }
  }

  // Relay 5 = Kring 8 & 9 (EETPL) = 916 W (Thermostat 5)
  if (EPon == 1)
  {
    if (!LastState_R5)
    {
      mcp1.digitalWrite(4, LOW);
      LastState_R5 = 1;
      // Reset ON time counter and record new OFF time
      EP_Laston = millis();
      EP_Toff = millis() - EP_Lastoff; // OFF time (ms)
      // Calculate new circuit Duty-Cycle:
      EP_DC = (EP_Ton * 100)/(EP_Ton + EP_Toff);
    }
    strncat(myText, " EP", sizeof(myText)); // Concatenate current string with name of this circuit
    kilowatts = kilowatts + 0.916;
    delay(1000);
  }
  else
  {
    if (LastState_R5)
    {
      mcp1.digitalWrite(4, HIGH);
      LastState_R5 = 0;
      // Reset OFF time counter and record new ON time:
      EP_Lastoff = millis();
      EP_Ton = millis() - EP_Laston; // ON time (ms)
      // Calculate new circuit Duty-Cycle:
      EP_DC = (EP_Ton * 100)/(EP_Ton + EP_Toff);
    }
  }

  // Relay 6 = Kring 10 (ZITPL) = 460 W (Thermostat 6)
  if (ZPon == 1)
  {
    if (!LastState_R6)
    {
      mcp1.digitalWrite(5, LOW);
      LastState_R6 = 1;
      // Reset ON time counter and record new OFF time
      ZP_Laston = millis();
      ZP_Toff = millis() - ZP_Lastoff; // OFF time (ms)
      // Calculate new circuit Duty-Cycle:
      ZP_DC = (ZP_Ton * 100)/(ZP_Ton + ZP_Toff);
    }
    strncat(myText, " ZP", sizeof(myText)); // Concatenate current string with name of this circuit
    kilowatts = kilowatts + 0.460;
    delay(1000);
  }
  else
  {
    if (LastState_R6)
    {
      mcp1.digitalWrite(5, HIGH);
      LastState_R6 = 0;
      // Reset OFF time counter and record new ON time:
      ZP_Lastoff = millis();
      ZP_Ton = millis() - ZP_Laston; // ON time (ms)
      // Calculate new circuit Duty-Cycle:
      ZP_DC = (ZP_Ton * 100)/(ZP_Ton + ZP_Toff);
    }
  }

  // Relay 7 = Kring 11 & 12 (BADK) = 832 W
  if (BKon == 1)
  {
    if (!LastState_R7)
    {
      mcp1.digitalWrite(6, LOW);
      LastState_R7 = 1;
      // Reset ON time counter and record new OFF time
      BK_Laston = millis();
      BK_Toff = millis() - BK_Lastoff; // OFF time (ms)
      // Calculate new circuit Duty-Cycle:
      BK_DC = (BK_Ton * 100)/(BK_Ton + BK_Toff);
    }
    strncat(myText, " BK", sizeof(myText)); // Concatenate current string with name of this circuit
    kilowatts = kilowatts + 0.832;
    delay(1000);
  }
  else
  {
    if (LastState_R7)
    {
      mcp1.digitalWrite(6, HIGH);
      LastState_R7 = 0;
      // Reset OFF time counter and record new ON time:
      BK_Lastoff = millis();
      BK_Ton = millis() - BK_Laston; // ON time (ms)
      // Calculate new circuit Duty-Cycle:
      BK_DC = (BK_Ton * 100)/(BK_Ton + BK_Toff);
    }
  }

  // Calculate actual heatdemand (kW)
  heatdemand = kilowatts;  // Updates this Particle.variable, published to the Particle cloud

  // Report heat demand & Duty-Cycle:
  sprintf(str, "%s = %2.2f kW, BB:%2.0f,WP:%2.0f,IK:%2.0f,KK:%2.0f,EP:%2.0f,ZP:%2.0f,BK:%2.0f,",myText,heatdemand,BB_DC,WP_DC,IK_DC,KK_DC,EP_DC,ZP_DC,BK_DC); // For: string %s, floating %2.2f, integer = "%d"
  //Particle.publish("Status-HEAT:HVAC",str,60,PRIVATE); // Publish the full string of currently working heating circuits

  // ANOMALY CHECKS: Read pin 7 on the MCP23017 IC: Reflects Pump status (This allows us to check if the I2C bus is still working!)
  if (mcp1.digitalRead(7) == LOW) // Pin 7 = LOW => The Pump relay is ON!
  {
    if (heatdemand > 0) // The pump should be ON and it is: Normal!
    {
      //Particle.publish("Status-HEAT:HVAC","All OK, Pump = ON",60,PRIVATE);
    }
    else // The pump should be OFF and it's not: ALERT!
    {
      Particle.publish("Status-HEAT:HVAC","Pump = ON, no heat demand!!!",60,PRIVATE);
      delay(1000);
      Particle.publish("Alerts","Heating problem!",60,PRIVATE); // Catch this event with IFTTT and let it send a notification to your smartphone...
      delay(1000);
      BusErrorCount = BusErrorCount + 1; // Monitor if the error is persistent (after a few times, reset controller in loop function!)
    }
  }

  else // Pin 7 = HIGH => The Pump relay is OFF!
  {
    if (heatdemand > 0) // The pump should be ON and it's not: ALERT!
    {
      Particle.publish("Status-HEAT:HVAC","Pump = OFF with heat demand!!!",60,PRIVATE);
      delay(1000);
      Particle.publish("Alerts","Heating problem!",60,PRIVATE); // Catch this event with IFTTT and let it send a notification to your smartphone...
      delay(1000);
      BusErrorCount = BusErrorCount + 1; // Monitor if the error is persistent (after a few times, reset controller in loop function!)
    }
    else // The pump should be OFF and it is: Normal!
    {
      //Particle.publish("Status-HEAT:HVAC","No heat demand, all OK.",60,PRIVATE);
    }
  }

} // End SetHeating()



// *D0 + D1 = I2C-SDA + SCL
void ECOtransfer() // Transfer ECO-boiler SOLAR energy to the heating boilers. Function = called from the eventDecoder(), each time ECO boiler variables are received!
{
  // Check ECO boiler energy and pump excess (15=>12 kWh) equally to both HEATING boiler(s)
  if ((millis() - HeatTransferLastTime) > HeatTransferInterval)
  {
    if (ECOQtot > 15) // 2nd condition removed: " && hour_Demand > 0" => If there's enough energy in ECO boiler AND heat demand (in the previous hour) from each heating boiler.
    {
      manual("SCHon"); // Currently we use only the SCH pump...
    }
    HeatTransferLastTime = millis(); // Reset this timer
  }

  if (ECOQtot <= 12) // Avoid a cold shower! Stop immediately if below this value...
  {
    manual("SCHoff"); // Currently we use only the SCH pump...
  }

    /* TO DO:
    - Include "kWh pumped" in the message...
    - Before pumping, check energy space in WON & SCH boilers (with both KQtot)
    - Distribute energy evenly between WON & SCH boilers: Alternate between both, pump to the smalles total pumped energy... (Use permanent memory variables!)
    - Stop pumping after a maximum time (if something goes wrong)
    */


  // Operate Relay 9 = Pump from "ECO" boiler to "SCH" heating boiler
  if (SCHon == 1)
  {
    if (!LastState_R9)
    {
      mcp1.digitalWrite(8, LOW);
      LastState_R9 = 1;
      ECOpumpSCHLastTime = millis(); // Start the timer
      Particle.publish("Status-HEAT:HVAC","START pumping ECO => SCH boiler",60,PRIVATE);
      delay(1000);
      // Record the ECO energy before pumping
      EQtotStart = ECOQtot;
    }

    // Check timer for max pump time:
                    // ATTENTION: The pump stops, but restarts again if the conditions are met... (ECOQtot > 4 && heatdemand > 0)
    if ((millis() - ECOpumpSCHLastTime) > MaxECOpumpTime)
    {
      manual("SCHoff"); // Switch pump OFF
      Particle.publish("Status-HEAT:HVAC","SCH ECO pump time-out!",60,PRIVATE);
      delay(1000);
    }
  }
  else
  {
    if (LastState_R9)
    {
      mcp1.digitalWrite(8, HIGH);
      LastState_R9 = 0;
      Particle.publish("Status-HEAT:HVAC","STOP pumping ECO => SCH boiler",60,PRIVATE);
      delay(1000);
      // Update boiler received energy:
      EQtotStop = ECOQtot;// Record the ECO energy after pumping
      Qpumped = EQtotStart - EQtotStop;
      QECOSCH = Qpumped + QECOSCH; // Permanent memory!
      // Publish the pumped energy:
      sprintf(str, "Pumped to SCH = %2.2f", QECOSCH);
      Particle.publish("Status-HEAT:HVAC", str,60,PRIVATE);
      delay(500);
    }
  }


  // Operate Relay 10 = Pump from "ECO" boiler to "WON" heating boiler
  if (WONon == 1)
  {
    if (!LastState_R10)
    {
      mcp1.digitalWrite(9, LOW);
      LastState_R10 = 1;
      ECOpumpWONLastTime = millis(); // Start the timer
      Particle.publish("Status-HEAT:HVAC","START pumping ECO => WON boiler",60,PRIVATE);
      delay(1000);
      // Record the ECO energy before pumping
      EQtotStart = ECOQtot;
    }
    // Check timer for max pump time:
    if ((millis() - ECOpumpWONLastTime) > MaxECOpumpTime)
    {
      // Switch pump OFF
      manual("WONoff");
      Particle.publish("Status-HEAT:HVAC","WON ECO pump time-out!",60,PRIVATE);
      delay(1000);
    }

  }

  else
  {
    if (LastState_R10)
    {
      mcp1.digitalWrite(9, HIGH);
      LastState_R10 = 0;
      Particle.publish("Status-HEAT:HVAC","STOP pumping ECO => WON boiler",60,PRIVATE);
      delay(1000);
      // Update boiler received energy:
      EQtotStop = ECOQtot;// Record the ECO energy after pumping
      Qpumped = EQtotStart - EQtotStop;
      QECOWON = Qpumped + QECOWON; // Permanent memory!

    }
  }
}




// *D0 + D1 = I2C-SDA + SCL
void getDemand() // Collect & report heat demand data
{
  double minute_Demand = heatdemand / 60;
  total_Demand = total_Demand + minute_Demand;

  if (Time.minute() == 0) // Exactly on every hour!
  {
    hour_Demand = total_Demand - prev_Demand;
    prev_Demand = total_Demand;
  }

  // Publish current, hourly & accumulated energy Demand: ATTENTION: Do not change as status panel catches this format...
  sprintf(str, "Heat Demand: %2.2f, Hourly= %2.2f kWh, Total= %2.2f kWh",heatdemand,hour_Demand,total_Demand); // For integer use "%d"
  //Particle.publish("Status-HEAT:HVAC", str,60,PRIVATE);
  delay(500);
  // Serial output
  Serial.println(str);
}





// *D3 - T-BUS (12 temp sensors)
// @Ric's function modified by @BulldogLowell to include CRC checking + Faulty sensor reporting (many errors in given time: TIMEOUT alert!):
void getTemperatures(int select)
{
    ds.reset();
    ds.skip();
    ds.write(0x44, 0);
    delay(1000);
    ds.reset();

    for (int i=0; i< sizeof(temps)/sizeof(temps[0]); i++)
    {
        switch (select)
        {
            case 0:
                ds.select(addrs0[i]);
                break;
            case 1:
                ds.select(addrs1[i]);
                break;
        }

        ds.write(0xBE,0);

        byte scratchpadData[9];
        for (int i = 0; i < 9; i++) // we only need 9 bytes
        {
          scratchpadData[i] = ds.read();
        }

        byte currentCRC = (OneWire::crc8(scratchpadData, 8));

        ds.reset();

        if (currentCRC != scratchpadData[8])
        {
            String message;
            if (Time.now() - tmStamp[i] > 3600UL)  // one hour in this example
            {
                message = "Sensor Timeout on sensor: ";
            }
            else
            {
               //message = "Bad reading on Sensor: "; Temporarily disconnected until KW sensors are connected.
            }
            //Particle.publish("Alert", message + String(i), 60, PRIVATE);
            crcErrorCount[i]++;
            delay(1000);
            continue;
        }

        tmStamp[i] = Time.now();

        if (select == 0)
        {
            //int16_t raw = (data1 << 8) | data0;
            int16_t raw = (scratchpadData[1] << 8) | scratchpadData[0];
            celsius = (double)raw * 0.0625;
        }
        else if (select == 1)
        {
            int16_t raw = scratchpadData[0];
            celsius = (double)raw * 0.5;
        }
        // construct the CRC error array
        strcpy(crcErrorJSON, "{\"errorCount\":[");
        for (int i = 0; i < sizeof(temps)/sizeof(temps[0]); i++)
        {
            char buffer[10];
            itoa(crcErrorCount[i], buffer, 10);
            strcat(crcErrorJSON, buffer);
            if(i < sizeof(temps)/sizeof(temps[0]) - 1)
              strcat(crcErrorJSON, ",");
        }
        strcat(crcErrorJSON, "]}");
        *temps[i] = celsius;
    }
}





void discoverOneWireDevices(void) // List the addresses of all 1-wire devices on the T-BUS (D3) => discoverOneWireDevices();
{
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[12];
  int sensorCount = 0;

  Particle.publish("OneWire", "Looking for 1-wire addresses:", 60, PRIVATE); delay(1000);

  while(ds.search(addr) and sensorCount < 13)
  {
    sensorCount++;
    char newAddress[96] = ""; // Make space for the (12 x 8 =) 96 characters of our 12 sensor addresses.
    snprintf(newAddress, sizeof(newAddress), "0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7], addr[8], addr[9], addr[10], addr[11]);
    Particle.publish("OneWire", newAddress, 60, PRIVATE); delay(1000);

    if ( OneWire::crc8( addr, 7) != addr[7])
    {
      Particle.publish("OneWire", "CRC is not valid!", 60, PRIVATE); delay(1000);
      return;
    }
  }

  ds.reset_search();
  Particle.publish("OneWire", "No more addresses!", 60, PRIVATE); delay(1000);
  return;
}





// For energy reporting of KEL-SCH boiler:
void getKSplus()
{
  double minute_KSplus = KSQtot - prev_KSQtot;

  if (minute_KSplus > 0)
  {
    total_KSplus = total_KSplus + minute_KSplus;
    //Particle.publish("Status-HEAT:HVAC", "KSQtot UP!",60,PRIVATE);
  }
  else
  {
    //Particle.publish("Status-HEAT:HVAC", "KSQtot DOWN...",60,PRIVATE);
  }

  prev_KSQtot = KSQtot;

  if (Time.minute() == 0) // Exactly on every hour!
  {
    hour_KSplus = total_KSplus - prev_H_KSplus;
    prev_H_KSplus = total_KSplus;
  }
  sprintf(str, "KS energy,kWh: %2.2f-Diff: %2.2f",KSQtot,minute_KSplus);
  //Particle.publish("Status-HEAT:HVAC", str,60,PRIVATE);
  delay(500);
  sprintf(str, "KS-plus,kWh: Hourly: %2.2f-Total: %2.2f",hour_KSplus,total_KSplus);
  //Particle.publish("Status-HEAT:HVAC", str,60,PRIVATE);
  delay(500);
}




// For energy reporting of KEL-WON boiler:
void getKWplus()
{
  double minute_KWplus = KWQtot - prev_KWQtot;

  if (minute_KWplus > 0)
  {
    total_KWplus = total_KWplus + minute_KWplus;
    //Particle.publish("Status-HEAT:HVAC", "KWQtot UP!",60,PRIVATE);
  }
  else
  {
    //Particle.publish("Status-HEAT:HVAC", "KWQtot DOWN...",60,PRIVATE);
  }

  prev_KWQtot = KWQtot;

  if (Time.minute() == 0) // Exactly on every hour!
  {
    hour_KWplus = total_KWplus - prev_H_KWplus;
    prev_H_KWplus = total_KWplus;
  }
  sprintf(str, "KW energy,kWh: %2.2f-Diff: %2.2f",KWQtot,minute_KWplus);
  //Particle.publish("Status-HEAT:HVAC", str,60,PRIVATE);
  delay(500);
  sprintf(str, "KW-plus,kWh: Hourly: %2.2f-Total: %2.2f",hour_KWplus,total_KWplus);
  //Particle.publish("Status-HEAT:HVAC", str,60,PRIVATE);
  delay(500);
}






// Check if ventilation speed must be adjusted...
void VENTilation()
{
  Fanspeed = CO2a + CO2b + CO2c; // The sum of all CO2 excess levels is a measure for fan speed...
  Fanspeed = constrain(Fanspeed, 0, 255); // Constrain value from 0 to 255.
  //analogWrite(fancontrolPin, Fanspeed); // Fanspeed: 255 - 0 => 0 - 80% fan speed) => UNUSED AS LONG AS NO CO2 sensors are integrated...
  // For the electromagnetic valves opening both PULSE ducts, we use Relays 11 & 12 operated through the I2C connector: Pin 13 & 14 of the MCP23017 16port Expander chip.
  // Create commands: IF CO2b or CO2c > 0, then switch relay 11 ON, else switch it OFF.
}






// *COMMON functions:

// Catch "Status-HEAT" messages from our controllers:
void eventDecoder(const char *event, const char *data) // This function is called when the event "Status-HEAT" is published
{
  char* Subject = strtok(strdup(data), ":"); // Take first string until delimiter = ":" (If there is no : then the full string is copied!)

  // Catch events from the room controllers and take action. 3 rooms in use: BB, IK, BK. (WP not yet!)

  // * "BandB" controller:
  if (String(event) == "Status-HEAT:R1-BandB")// Room controllers automatically include their name in events
  {
    if (String(data) == "No Tstat heat demand") // Heating - Home mode
    {
      ThermostatBB = 0;
      Particle.publish("Status-HEAT:HVAC","BandB heating OFF (Tstat)",60,PRIVATE);
      BandBChkLastTime = millis(); // Reset time since when not heard news...
    }
    if (String(data) == "Tstat heat demand") //
    {
      ThermostatBB = 1;
      Particle.publish("Status-HEAT:HVAC","BandB heating ON (Tstat)",60,PRIVATE);
      BandBChkLastTime = millis(); // Reset time since when not heard news...
    }

    if (String(data) == "No Tout heat demand (Humidity)") // Heating - Out mode
    {
      CondensProtBB = 0; // Condensation protection OFF
      manual("Bboff");
      Particle.publish("Status-HEAT:HVAC","BandB heating OFF (No condens risk)",60,PRIVATE);
      BandBChkLastTime = millis(); // Reset time since when not heard news...
    }
    if (String(data) == "Tout heat demand (Humidity)") //
    {
      CondensProtBB = 1; // Condensation protection ON: Heat, even if thermostat is OFF in Home mode
      manual("Bbon");
      Particle.publish("Status-HEAT:HVAC","BADK heating ON (Condens risk!)",60,PRIVATE);
      BandBChkLastTime = millis(); // Reset time since when not heard news...
    }
  } // * End room "BandB"

  // * "INKOM" controller:
  if (String(event) == "Status-HEAT:R3-INKOM")// Room controllers automatically include their name in events
  {
    if (String(data) == "No Tstat heat demand") // Heating - Home mode
    {
      ThermostatIK = 0;
      Particle.publish("Status-HEAT:HVAC","INKOM heating OFF (Tstat)",60,PRIVATE);
      INKOMChkLastTime = millis(); // Reset time since when not heard news...
    }
    if (String(data) == "Tstat heat demand") //
    {
      ThermostatIK = 1;
      Particle.publish("Status-HEAT:HVAC","INKOM heating ON (Tstat)",60,PRIVATE);
      INKOMChkLastTime = millis(); // Reset time since when not heard news...
    }

    if (String(data) == "No Tout heat demand (Humidity)") // Heating - Out mode
    {
      CondensProtIK = 0; // Condensation protection OFF
      manual("Ikoff");
      Particle.publish("Status-HEAT:HVAC","INKOM heating OFF (No condens risk)",60,PRIVATE);
      INKOMChkLastTime = millis(); // Reset time since when not heard news...
    }
    if (String(data) == "Tout heat demand (Humidity)") //
    {
      CondensProtIK = 1; // Condensation protection ON: Heat, even if thermostat is OFF in Home mode
      manual("Ikon");
      Particle.publish("Status-HEAT:HVAC","INKOM heating ON (Condens risk!)",60,PRIVATE);
      INKOMChkLastTime = millis(); // Reset time since when not heard news...
    }
  } // * End room "INKOM"

  // * "BADK" controller:
  if (String(event) == "Status-HEAT:R2-BADK")// Room controllers automatically include their name in events
  {
    if (String(data) == "No Tstat heat demand") // Heating - Home mode
    {
      ThermostatBK = 0;
      Particle.publish("Status-HEAT:HVAC","BADK heating OFF (Tstat)",60,PRIVATE);
      BADKChkLastTime = millis(); // Reset time since when not heard news...
    }
    if (String(data) == "Tstat heat demand") //Tstat Heat demand
    {
      ThermostatBK = 1;
      Particle.publish("Status-HEAT:HVAC","BADK heating ON (Tstat)",60,PRIVATE);
      BADKChkLastTime = millis(); // Reset time since when not heard news...
    }

    if (String(data) == "No Tout heat demand (Humidity)") // Heating - Out mode
    {
      CondensProtBK = 0; // Condensation protection OFF
      manual("Bkoff");
      Particle.publish("Status-HEAT:HVAC","BADK heating OFF (No condens risk)",60,PRIVATE);
      BADKChkLastTime = millis(); // Reset time since when not heard news...
    }
    if (String(data) == "Tout heat demand (Humidity)") //
    {
      CondensProtBK = 1; // Condensation protection ON: Heat, even if thermostat is OFF in Home mode
      manual("Bkon");
      Particle.publish("Status-HEAT:HVAC","BADK heating ON (Condens risk!)",60,PRIVATE);
      BADKChkLastTime = millis(); // Reset time since when not heard news...
    }
  } // * End room "BADK"

  // * "WASPL" controller:
  if (String(event) == "Status-HEAT:R5-WASPL")// Room controllers automatically include their name in events
  {
    if (String(data) == "No Tstat heat demand") // Heating - Home mode
    {
      ThermostatWP = 0;
      Particle.publish("Status-HEAT:HVAC","WASPL heating OFF (Tstat)",60,PRIVATE);
      WASPLChkLastTime = millis(); // Reset time since when not heard news...
    }
    if (String(data) == "Tstat heat demand") //Tstat Heat demand
    {
      ThermostatWP = 1;
      Particle.publish("Status-HEAT:HVAC","WASPL heating ON (Tstat)",60,PRIVATE);
      WASPLChkLastTime = millis(); // Reset time since when not heard news...
    }

    if (String(data) == "No Tout heat demand (Humidity)") // Heating - Out mode
    {
      CondensProtWP = 0; // Condensation protection OFF
      manual("Bkoff");
      Particle.publish("Status-HEAT:HVAC","WASPL heating OFF (No condens risk)",60,PRIVATE);
      WASPLChkLastTime = millis(); // Reset time since when not heard news...
    }
    if (String(data) == "Tout heat demand (Humidity)") //
    {
      CondensProtWP = 1; // Condensation protection ON: Heat, even if thermostat is OFF in Home mode
      manual("Bkon");
      Particle.publish("Status-HEAT:HVAC","WASPL heating ON (Condens risk!)",60,PRIVATE);
      WASPLChkLastTime = millis(); // Reset time since when not heard news...
    }
  } // * End room "WASPL"


  // Catch heating-related variables: If "Subject" starts with a 3-character string followed by ":" => Create variable names, depending on the "Subject" name... (ex: "ECO: 10.13 kWh")
  // 1) ECO-boiler energy: Check if energy can be transferred to the heating boilers.
  if (strncmp(Subject, "ECO", strlen("ECO")) == 0) // String = ECO
  {
    double ECOQtot = atof(strtok(NULL, ",")); // Convert next string until delimiter = "," to double variable
    // Report the ECO boiler energy level received:
    sprintf(str, "SOLAR data received: %2.2f kWh",ECOQtot);
    Particle.publish("ECHO!", str,60,PRIVATE); delay(500);

    ECOtransfer(); // Check if "ECOQtot" heat must be transferred from ECO boiler to both heating boilers...
  }

  // 2) Important roomdata: temperatures, humidity, CO2 ppm...
  //    (Currently only for ventilation control: CO2 ppm of BandB, EETPL, SLAK)

  // * "EETPL" controller:
  if (strncmp(Subject, "EETPL", strlen("EETPL")) == 0) // String = EETPL
  {
    int CO2EETPL = atof(strtok(NULL, ",")); // Convert next string until delimiter = "," to integer variable
    CO2a = CO2EETPL - 400; // Subtract 400 from CO2 value: This is the "excess CO2" used to set fan speed.
    VENTilation(); // Check if ventilation speed must be adjusted...
  }

  // * "BandB" controller:
  if (strncmp(Subject, "BandB", strlen("BandB")) == 0) // String = BandB
  {
    int CO2BandB = atof(strtok(NULL, ",")); // Convert next string until delimiter = "," to integer variable
    CO2b = CO2BandB - 400; // Subtract 400 from CO2 value: This is the "excess CO2" used to set fan speed.
    VENTilation(); // Check if ventilation speed must be adjusted...
  }

  // * "SLAK" controller:
  if (strncmp(Subject, "SLAK", strlen("SLAK")) == 0) // String = SLAK
  {
    int CO2SLAK = atof(strtok(NULL, ",")); // Convert next string until delimiter = "," to integer variable
    CO2c = CO2SLAK - 400; // Subtract 400 from CO2 value: This is the "excess CO2" used to set fan speed.
    VENTilation(); // Check if ventilation speed must be adjusted...
  }


  free(Subject); // To avoid memory leak, this memory must be freed every time...
}





// *COMMON function: Receive "commands" from outside.
int manual(String command) // = Particle.function receiving commands (From this sketch or from Particle cloud: iPhone App etc...)
{
// 3 heat control modes:
  if((command == "home") || (command == "Home=1}")  || (command == "Manual=0}")) // Mode = Activated if both our iPhones are <100 km radius from home: Command = manual, Parameter = Home ("Life360" app + IFTTT)
  {
    HeatMode = 1;  // Thermostats rule!
    return 1;
  }

  if((command == "out") || (command == "Home=0}")) // Mode = Activated if both our iPhones are >100 km radius from home: Command = manual, Parameter = Out ("Life360" app + IFTTT)
  {
    HeatMode = 2;  // Automatic lower "safe" temperature to a few °C above dewpoint
    return 2;
  }

  if((command == "manual") || (command == "Manual=1}")) // Mode = Activated from iPhone App: Command = manual, Parameter = Manual
  {
    HeatMode = 3;  // Pure manual operation: Turn ON & OFF per room from iPhones App: Command = manual, Parameter = Bbon ...
    return 3;
  }

// 8 heating RELAYS:
  if((command == "bbon") || (command == "BB=1}"))
  {
    BBon = 1;
    return 1254;
  }

  if((command == "bboff") || (command == "BB=0}"))
  {
    BBon = 0;
    return 0;
  }

  if((command == "wpon") || (command == "WP=1}"))
  {
    WPon = 1;
    return 1096;
  }

  if((command == "wpoff") || (command == "WP=0}"))
  {
    WPon = 0;
    return 0;
  }

  if((command == "bkon") || (command == "BK=1}"))
  {
    BKon = 1;
    return 832;
  }

  if((command == "bkoff") || (command == "BK=0}"))
  {
    BKon = 0;
    return 0;
  }

  if((command == "zpon") || (command == "ZP=1}"))
  {
    ZPon = 1;
    return 460;
  }

  if((command == "zpoff") || (command == "ZP=0}"))
  {
    ZPon = 0;
    return 0;
  }

  if((command == "epon") || (command == "EP=1}"))
  {
    EPon = 1;
    return 916;
  }

  if((command == "epoff") || (command == "EP=0}"))
  {
    EPon = 0;
    return 0;
  }

  if((command == "kkon") || (command == "KK=1}"))
  {
    KKon = 1;
    return 1018;
  }

  if((command == "kkoff") || (command == "KK=0}"))
  {
    KKon = 0;
    return 0;
  }

  if((command == "ikon") || (command == "IK=1}"))
  {
    IKon = 1;
    return 1025;
  }

  if((command == "ikoff") || (command == "IK=0}"))
  {
    IKon = 0;
    return 0;
  }

  if((command == "allon") || (command == "ALL=1}"))
  {
    BBon = 1;
    WPon = 1;
    BKon = 1;
    ZPon = 1;
    EPon = 1;
    KKon = 1;
    IKon = 1;

    return 6601;
  }

  if((command == "alloff") || (command == "ALL=0}"))
  {
    BBon = 0;
    WPon = 0;
    BKon = 0;
    ZPon = 0;
    EPon = 0;
    KKon = 0;
    IKon = 0;
    return 0;
  }

// 2 ECO pump RELAYS:
  if((command == "schon") || (command == "SCH=1}")) // Pump from ECO => SCH boiler
  {
    SCHon = 1;
    return 9;
  }

  if((command == "schoff") || (command == "SCH=0}"))
  {
    SCHon = 0;
    return 0;
  }

  if((command == "wonon") || (command == "WON=1}")) // Pump from ECO => SCH boiler
  {
    WONon = 1;
    return 10;
  }

  if((command == "wonoff") || (command == "WON=0}"))
  {
    WONon = 0;
    return 0;
  }

// RESET Photon in case of problems:
  if(command == "reset") // You can remotely RESET the photon with this command... (It won't reset the IO-eXbox!)
  {
    System.reset();
    return -10000;
  }

  // RESET 5V power in case of controller connection problems:
  if(command == "reset5v") // You can remotely Power cycle the 5V power supply with this command...
  {
    mcp1.digitalWrite(15, LOW);// Turn 5V Power OFF
    delay(5000);
    mcp1.digitalWrite(15, HIGH);// Turn 5V Power ON again
    return 5;
  }

  // Turn 5V power OFF:
  if(command == "5voff")
  {
    mcp1.digitalWrite(15, LOW);
    return 5;
  }

  // Turn 5V power ON:
  if(command == "5von")
  {
    mcp1.digitalWrite(15, HIGH);
    return 5;
  }

  // Turn ventilation fans ON: (Pin A4)
  if(command == "venton")
  {
    analogWrite(fancontrolPin, 120); // Fanspeed: 255 - 0 => 0 - 80% fan speed)
    return 120;
  }

  // Turn ventilation fans ON: (Pin A4)
  if(command == "ventfull")
  {
    analogWrite(fancontrolPin, 0); // Fanspeed: 255 - 0 => 0 - 80% fan speed)
    return 0;
  }

  // Turn ventilation fans OFF: (Pin A4)
  if(command == "ventoff")
  {
    analogWrite(fancontrolPin, 255); // Fanspeed: 255 - 0 => 0 - 80% fan speed)
    return 255;
  }

// REPORT all important data:
  if(command == "reportstatus")
  {
    // Time stamp
    Particle.publish("Status-HEAT:HVAC", Time.timeStr(), 60,PRIVATE);// Wed May 21 01:08:47 2014
    delay(500);

    // WiFi reception
    wifiRSSI = WiFi.RSSI();
    wifiPERCENT = wifiRSSI+120;
    sprintf(str, "RSSI:%2.0f, PCT:%2.0f",wifiRSSI,wifiPERCENT);
    Particle.publish("Status-ROOM", str,60,PRIVATE);
    delay(500);

    // Nr of resets for non-responsive controllers
    sprintf(str, "BandB:%2.0f, INKOM:%2.0f, BADK:%2.0f, WASPL:%2.0f",BandBResetCount,INKOMResetCount,BADKResetCount,WASPLResetCount);
    Particle.publish("Status-ROOM", str,60,PRIVATE);
    delay(500);

    // Heating circuits ON?
    Particle.publish("Status-HEAT:HVAC",myText,60,PRIVATE);

    // Pumped energy:
    sprintf(str, "Pumped to SCH:%2.2f, WON:%2.2f", QECOSCH, QECOWON);
    Particle.publish("Status-HEAT:HVAC", str,60,PRIVATE);
    delay(500);

    return 1001;
  }

  if(command == "reportsensors")
  {
    // 1-wire addresses (DS18B20 sensors)
    discoverOneWireDevices();

    return 1002;
  }

  Particle.publish("Status-HEAT:HVAC", command,60,PRIVATE);// If it does not match one of the above, publish the received string to see what it's "payload" was...
  return -1;// If none above

}
