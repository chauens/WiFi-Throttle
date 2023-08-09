/* "WiCAB" WiFi Single/Multi Throttle for JMRI. Version V3.8.6.
   Christoph Hauenstein. Nov/2021, additions during 2022.

   The pin assignments are for EzSBC or Adafruit ESP32-Feather.
   Prerequisites:
   Install libraries: in Arduino IDE - select File, Preferences >
    after “Additional Boards Manager URLs:”  paste:
    https://dl.espressif.com/dl/package_esp32_index.json
    In the Board Manager select "ESP Dev Module".
   Install Additional lib via zip file: https://github.com/geekfactory/TimeLib
   For more details please refer to the document "WiFiThrottle Documentation".
   Turn on JMRI debug: add below line to file JMRI/default.icf.
   log4j.category.jmri.jmrit.withrottle=DEBUG
*************************************************************************************************
*** JMRI: if using Heart Beat set it to min 10 sec. Withrottle > Preferences > Use eStop.     ***
*** When using Serial Monitor: set its speed to 115200 baud & No Line Ending.                 ***
*************************************************************************************************
*** Before loading this program for the first time the keypad has to be configured first:     ***                                                              ***
*** upload sketch_Keypad_Config-V1 and run it with Serial Monitor.                            ***
*** Follow the messeages in SM. More details in Document "WiCab_Documention.docx".            ***                                          ***
*************************************************************************************************
*/
#include <TimeLib.h>
#include <EEPROM.h>
#include <WiFi.h>
#include "SH1106Wire.h"
//#inlcude "SSD1306Wire.h"
// Initialize the OLED display:
SH1106Wire display(0x3c, 23, 22);  // I2C addr, SDA, SCK
//SSD1306Wire  display(0x3c, 23, 22);  // I2C addr, SDA, SCK
String vrs = "Version 3.8.9.0";  // y.m.d.t#
WiFiClient client;
/////////////////////////////////////////////////////////////////////////////////
// compile options:
//#define LOG
//#define EUROSOUND  // european sound files use F1 to turn on/off sound. This moves "Fade" to F1.
#define BATLEVEL   // Battery level sensor
#define CLOCK      // Display JMRI clock
//////////////////////////////////////////////////////////////////////////////////
// WiFi Server Definitions
const char* ssid = "RPi-JMRI";     // local wireless network id
const char* pass = "rpI-jmri";     // key
const char* host = "192.168.6.1";  // WiThrottle server IP Adr. (Device running JMRI-WiThrottle-server)
const int port = 12090;            // WiThrottle server Port
///////////////////////////////////////////////////////////////////////////////////////////
// JMRI-Roster mapping for dedicated function switches >>
char* fctname[] = { "Whistle-short", "Whistle-long", "Bell", "s1map", "Coupler", "Coupler-front", "Shunting", "Drive-Hold", "Fade" };
#define numswfcs 9  // nbr of switched functions = nbr of array elements
//---------------------------------------------------
// fctname-array elements dedicated switch assignments:
// element:  0   1   2   3   4   5   6   7    8
// switch:  SWs SWl SB  S1  S2f S2r B12 Hold Fade
// s=short,l=long,f=forward,r=reverse.
// Hold=ESU Full Throttle Drive Hold Function, mapped
// to center position of direction switch.
//-----------------------------------------------------------------------------------------
//*******************************************************************************
byte numthrottles = 1;  // number of throttles. 4 or less. 1 defines single throttle.
char* cfgthrottle[] = { "1 throttle", "2 throttles", "3 throttles", "4 throttles" };
//*******************************************************************************
byte fctswtmap[numswfcs];  // mapping dedicated switches to functions
byte fctswtmapT1[numswfcs];
byte fctswtmapT2[numswfcs];
byte fctswtmapT3[numswfcs];
byte fctswtmapT4[numswfcs];
byte fctswtmapdft[]{ 2, 3, 5, 99, 4, 99, 6, 99, 99 };  // default mapping switches to functions
byte MThrottle = 1;      // multi throttle number
bool MT1active = false;  // active throttle number
bool MT2active = false;
bool MT3active = false;
bool MT4active = false;
byte sdhswtmap[numswfcs];  // mapping switches to double header functions
byte sdhswtmapT1[numswfcs];
byte sdhswtmapT2[numswfcs];
byte sdhswtmapT3[numswfcs];
byte sdhswtmapT4[numswfcs];
#define numfcts 29  // number of functions (0-28)
byte fctarray[numfcts];
byte fctarrayT1[numfcts];
byte fctarrayT2[numfcts];
byte fctarrayT3[numfcts];
byte fctarrayT4[numfcts];
byte t_fctarray[numfcts];
byte t_fctarrayT1[numfcts];
byte t_fctarrayT2[numfcts];
byte t_fctarrayT3[numfcts];
byte t_fctarrayT4[numfcts];
byte dblhdrfct;         // for a function that is also sent to the consist (double header) address
byte blkdfks[numfcts];  // blocked function keys while using dedicated function switches.
byte blkdfksT1[numfcts];
byte blkdfksT2[numfcts];
byte blkdfksT3[numfcts];
byte blkdfksT4[numfcts];
byte fctnos[numfcts];  // function name start offsets for get fct by name
byte fctnosT1[numfcts];
byte fctnosT2[numfcts];
byte fctnosT3[numfcts];
byte fctnosT4[numfcts];
byte fctnoe[numfcts];  // function name end offsets for get fct by name
byte fctnoeT1[numfcts];
byte fctnoeT2[numfcts];
byte fctnoeT3[numfcts];
byte fctnoeT4[numfcts];
byte fctnox[numfcts];  // function numbers
byte fctnoxT1[numfcts];
byte fctnoxT2[numfcts];
byte fctnoxT3[numfcts];
byte fctnoxT4[numfcts];
byte numfcnt = 0;      // number of fcts counted
byte numfcntT1 = 0;
byte numfcntT2 = 0;
byte numfcntT3 = 0;
byte numfcntT4 = 0;
String dspfctname;     // display function name
bool setfctbn = false;
#define maxnbrrts 64    // max number of routes (defined in JMRI)
#define maxnbrtrns 128  // max number of turnouts (defined in JMRI)
// routes >>>
byte rsosa[maxnbrrts];  // route sysname offsets array
byte ruosa[maxnbrrts];  // route usrname offsets array
byte numrtos = 0;       // route offset in array
byte numrts = 0;        // number of routes found
bool setroute = false;  // setting route or accessory decoder address
bool sltroute = false;
String rsysname;         // route system name
String rusrname;         // route user name
bool rts_Avail = false;  // routes found in JMRI
// turnouts >>>
byte tsosa[maxnbrtrns];   // turnout sysname offsets array
byte tuosa[maxnbrtrns];   // turnout usrname offsets array
byte tstosa[maxnbrtrns];  // turnout state offsets array
byte numtos = 0;          // turnout offset in array
byte numts = 0;           // number of turnouts found
String tsysname;          // turnout system name
String tusrname;          // turnout user name
byte tostate;             // turnout state
String dsptostate;
char* tostarray[] = { "*", "Unknown", "Closed", "*", "Thrown", "*", "*", "*", "Inconsistent" };
//char *tostarray[] = {"*", "?", "C", "*", "T", "*", "*", "*", "!"};
bool trn_Avail = false;  // turnouts available in JMRI
//////////////////////////////////////////////////////////////////////////////////////
// pin definitions:
#define switch1pin 27  // default-if not mapped: Release loco (button)
#define switch2pin 21  // Coupler (button) Mapped to 2 functions - forward/reverse
#define switch3pin 16  // Brake
#define switchBpin 13  // Bell Switch (SPST) Mapped to 1 function
#define switchWpin 12  // whistle/Horn (button) Mapped to 2 functions - short/long
#define switchFpin 18  // Forward  Switch (SPDT pin1)
#define switchRpin 19  // Reverse  Switch (SPDT pin3)
#define encoderA 32    // Encoder A-DT (ADC#1)
#define encoderB 33    // Encoder B-CLK (ADC#1)
#define switchEpin 17  // Emergency stop
#define keypadin 34    // (A2) analog pin connecting keypad
int keypadarray[32];   // holding keypad button values
/////////////////////////////////////////////////////////////////////////////////////////
// Definitions of variables:
String tname = "WiCab";
String dsptname;         // display throttle name
String ctname;           // current throttle name
int cd = 60;             // timeout for the screen saver (in seconds)
int cb = 180;            // time in seconds between battery monitoring
#define numrose 100      // max number of JMRI roster entries processed (defines array size, increase if needed).
byte numspst = 28;       // current max speed steps (from DCC)
byte spdstpm = 2;        // speed step mode
byte chgspstm = 0;       // change speed step mode
bool pushshort = false;  // switch E pushed short = brake (300ms)
bool pushlong = false;   // switch E pushed long = change speed step mode
byte dbcdly = 100;       // debounce delay
#define fdelay 10
#define ldelay 350
#define sdelay 500  // waiting for jmri replies to arrive in throttle.
#define mdelay 200  // momentary functions - delay off after on.
/////////////////////////////////////////////////////////////////////////////////////////////
bool noop = false;
bool thrconfig = false;  // config mode by pushing STP button while power on
bool thrchg;
bool throff = false;
#define voltagepin 35  // voltage sensor (internal #35 = A13)
int batin;             // calc battery voltage
float battmp, batvolt;
/////////////////////////////////////////////////////////////////////////////////////////////
String MacString;       // MAC Address
String unixtime;        //hold unix time stamp from jmri
String unixsub;         //holding time substring
String ratiosub;        //holding fast clock ratio substring
int cntc = 0;           // counter for connecting to WiFi
int cntt = 0;           // connection timeout counter
int cnth;               // counter for the heart beat timer
byte ch;                // for the heartbeat timer (received from jmri)
int hbitv;              // heart beat interval from JMRI
String hbres;           // receive heart beat interval from server
bool hbuse = false;     // heart beat off (on)
float cntf;             // calc throttle heart beat interval
int cntd;               // counter for the screen saver
int cntb;               // counter for the battery monitor timer
bool dsplon = true;     // screen saver tracking
bool DEBUG = false;     // turn on/off serial monitor debug info on the fly
bool DEBUGENC = false;  // turn on/off encoder debug info
/////////////////////////////////////////////////////////////////////
int btnmin = 0;      // for finding min value
int btnmax = 0;      // for finding max value
int btnprvl = 5000;  // previous low
int btnprvh = 0;     // previous high
int btnnum = 0;      // button number
/////////////////////////////////////////////////////////////////
IPAddress ip;  // the IP address of this device
byte mac[6];   // MAC Address of the throttle sent to server for identification
//
unsigned long prvsMs = 0;
unsigned long crntMs;
unsigned long startTime;
bool drivehold = false;
bool driveholdON = false;
//
int btnread = 0;  // holding keypad button values
byte btnnbr;
int btncnt = 0;  // button count for entering a dcc address (up to 4 digits)
int dccadr = 0;  // DCC Address
int dccadrT1 = 0;
int dccadrT2 = 0;
int dccadrT3 = 0;
int dccadrT4 = 0;
int sdhadr = 0;  // simple double header address
int sdhadrT1 = 0;
int sdhadrT2 = 0;
int sdhadrT3 = 0;
int sdhadrT4 = 0;
int dccadrrlsd = 0;  // DCC address that was released
bool sdhdct = true;  // double header direction - true is same as lead
bool sdhdctT1 = true;
bool sdhdctT2 = true;
bool sdhdctT3 = true;
bool sdhdctT4 = true;

int dccadrtmp;
int lastdccadr = 0;
int lastdccadrT1 = 0;
int lastdccadrT2 = 0;
int lastdccadrT3 = 0;
int lastdccadrT4 = 0;
int dccstck[6];  // stack array;
byte stkx = 0;   // stack index
bool updstck;    // save or not to save to stack
int btnval = 0;  // the number of the keypad button
char dspadr[6];  // for displaying DCCadr with leading zeros
char dspadrT1[6];
char dspadrT2[6];
char dspadrT3[6];
char dspadrT4[6];
int dspmin;   // for displaying fast clock minute with leading zero
int dsphour;  // for displaying fast clock hour with leading zero
String dsptime;

bool adrbtnstate = false;
bool setdccadr = false;
bool setsdhadr = false;
bool setsdhadrT1 = false;
bool setsdhadrT2 = false;
bool setsdhadrT3 = false;
bool setsdhadrT4 = false;
bool kpbtnstate = false;
bool estop = false;
bool pshadr = false;
int rosbtncnt = 0;
//
byte accbtnstate = 0;
bool setaccadr = false;
int accadr = 0;
char dspaccadr[6];
String setacc;
//
#define EEPROM_SIZE 96  // 0+1=dccadr, 2-13=stack, 19=numthrottles, 20-84=keypad, 95=debug flag.

bool shortwhstl = false;
bool longwhstl = false;
bool swtWon = false;
bool swt1on = false;
bool swt2on = false;
bool swtBon = false;
bool swtCon = false;
bool swt3on = false;
bool swtEon = false;

int crntAState;
int prvsAState;

// functions:
bool fctbtnstate = false;  // function group button
byte fcgstate = 0;         // function group state (0,1,2)
byte fcgstateT1 = 0;
byte fcgstateT2 = 0;
byte fcgstateT3 = 0;
byte fcgstateT4 = 0;
bool dctstate = true;  // direction. true is forward
bool dctstateT1 = true;
bool dctstateT2 = true;
bool dctstateT3 = true;
bool dctstateT4 = true;
bool fc1btnstate = false;
bool fc2btnstate = false;
bool fc3btnstate = false;
bool fc4btnstate = false;
bool fc5btnstate = false;
bool fc6btnstate = false;
bool fc7btnstate = false;
bool fc8btnstate = false;
bool fc9btnstate = false;
bool fc0btnstate = false;
byte fnum;          // function number
byte sfnum;         // fct nbr for sound fade
byte t_sfnum;       // trailer fct nbr for sound fade
bool fading;        // fading active/inactive in loco
bool t_fading;      // fading for trailer
bool fade = false;  // use fade
bool t_fade = false;
bool faded = false;  // fade on/off
bool t_faded = false;
byte fdfctst = 0;  //fade function state
byte t_fdfctst = 0;
byte fnumts;                  // function number for toggle switch which could be on while other functions are handled
byte fn;                      // fct nbr array pointer
byte rosnamoffsets[numrose];  // array for loco names from roster
byte rosadroffsets[numrose];  // array for loco addresses from roster

// function buttons 0-28:
byte dspfos[] = { 30, 40, 50, 60, 70, 80, 90, 100, 110, 120 };  // display functions offsets
String dspfnum[] = { "0", "1", "2", "3", "4", "5", "6", "7", "8", "9" };
bool fctsavail = false;
byte fctbtnst;  // function state (1=on, 0=off)
byte fctstate;  // function state feedback from jmri
float vtemp = 0;
byte vspeed = 0;
byte vspeedT1 = 0;
byte vspeedT2 = 0;
byte vspeedT3 = 0;
byte vspeedT4 = 0;
byte dspeed = 0;
byte vcntT1 = 0;
byte vcntT2 = 0;
byte vcntT3 = 0;
byte vcntT4 = 0;
byte prvspeed = 0;
byte prvspeedT1 = 0;
byte prvspeedT2 = 0;
byte prvspeedT3 = 0;
byte prvspeedT4 = 0;
byte curspeed = 0;
bool encoder_moved = false;  //encoder moved flag
volatile byte ecnt = 0;      // encoder counter
volatile byte pecnt = 0;     // previous ecnt
volatile byte vcnt = 0;      // encoder step counter for speed
volatile byte rcnt = 0;      // encoder step counter for roster
volatile byte rtcnt = 0;     // encoder step counter for routes
volatile byte fcnt = 0;      // encoder step counter for functions
volatile byte tcnt = 0;      // encoder step counter for turnouts
volatile byte thrcnt = 0;    // encoder step counter for nbr throttles
byte savrcnt = 0;
byte savrtcnt = 0;
byte hcnt = 0;  // drive hold counter for locked speed
byte lastvcnt = 0;
int rxn, rxnn, numre, numos;
int fxi, fxn, fxm, fxs, fxe;
String xmits;             //send buffer
String jmri_res;          //buffer to hold incoming response
String jmri_sub;          //buffer to hold substring of incoming response
String functions;         //buffer to hold function list, JMRI reply
String functionsT1;
String functionsT2;
String functionsT3;
String functionsT4;
String roster;            //buffer to hold roster list
String routes;            //buffer to hold route list
String turnouts;          //buffer to hold turnout list
String rosne;             //to hold nbr of roster entries
bool ros_eAvail = false;  // roster entry available
String loconame;          // loco name from server. should not exceed about 10 char.
String loconameT1;
String loconameT2;
String loconameT3;
String loconameT4;
String locoadr;   // display dcc adr
String locoacts;  // to hold an action string
String locoactsT1;
String locoactsT2;
String locoactsT3;
String locoactsT4;
String sdhacts;  // hold actions for the consist address
String sdhactsT1;
String sdhactsT2;
String sdhactsT3;
String sdhactsT4;
String lkf;  // to hold search argument for functions
String lkfT1;
String lkfT2;
String lkfT3;
String lkfT4;
String lkr;  // to hold search argument for direction
String lkrT1;
String lkrT2;
String lkrT3;
String lkrT4;
String lks;  // hold search argument for speed step mode
String lksT1;
String lksT2;
String lksT3;
String lksT4;
bool svr = true;
bool recon = false;
int swtBstate = 0;
int lastswtBstate = 0;
int swt3state = 0;
int lastswt3state = 0;
int swtWstate = 0;
int lastswtWstate = 0;
int swt1state = 0;
int lastswt1state = 0;
int swt2state = 0;
int lastswt2state = 0;
int swtFstate = 0;
int lastswtFstate = 0;
int swtRstate = 0;
int lastswtRstate = 0;
int swtEstate = 0;
int lastswtEstate = 0;
int swtPstate = 0;
int lastswtPstate = 0;
// encoder interrupt subroutine:----------------------------
void IRAM_ATTR encisr() {
  crntAState = digitalRead(encoderA);
  if (crntAState != prvsAState) {
    if (digitalRead(encoderB) != crntAState) {  //rotating cw
      ecnt++;
      if (vcnt < numspst) vcnt++;      // count up speed steps
      rcnt++;                          // count up roster entries
      if (rcnt > numos) rcnt = 0;      // end of roster, back to beginning
      rtcnt++;                         // count up route entries
      if (rtcnt > numrtos) rtcnt = 0;  // end of route entries
      fcnt++;                          // count up functions
      if (fcnt > numfcnt) fcnt = 0;    // max number of functions
      tcnt++;                          // count up turnout entries
      if (tcnt > numtos) tcnt = 0;     // end of turnout entries
      thrcnt++;                        // count up throttle entries
      if (thrcnt > 3) thrcnt = 0;      // end of throttle entries
    } else {                           // rotating ccw
      ecnt--;
      if (vcnt > 0) vcnt--;                                   // count down speed steps
      rcnt--;                                                 // count down roster entries
      if ((rcnt < 0) || (rcnt > numos)) rcnt = numos;         // begin of roster, return to end
      rtcnt--;                                                // count down route entries
      if ((rtcnt < 0) || (rtcnt > numrtos)) rtcnt = numrtos;  // begin of route entries
      fcnt--;
      if ((fcnt < 0) || (fcnt > numfcnt)) fcnt = numfcnt;
      tcnt--;                                            // count down turnout entries
      if ((tcnt < 0) || (tcnt > numtos)) tcnt = numtos;  // begin of turnout entries
      thrcnt--;                                          // count down throttle entries
      if ((thrcnt < 0) || (thrcnt > 3)) thrcnt = 3;      // begin of throttle entries
    }
  }
  prvsAState = crntAState;
  encoder_moved = true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  display.init();
  //display.flipScreenVertically();
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setColor(WHITE);
  display.setFont(ArialMT_Plain_10);  // available sizes: 10,16,24
  display.drawString(0, 0, vrs);
  display.display();
  delay(2000);  // start serial monitor to catch startup.
  // start setup----
  pinMode(switchWpin, INPUT_PULLUP);
  pinMode(switch1pin, INPUT_PULLUP);
  pinMode(switch2pin, INPUT_PULLUP);
  pinMode(switchFpin, INPUT_PULLUP);
  pinMode(switchRpin, INPUT_PULLUP);
  pinMode(switchBpin, INPUT_PULLUP);
  pinMode(switch3pin, INPUT_PULLUP);
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  pinMode(switchEpin, INPUT_PULLUP);
#ifdef BATLEVEL
  pinMode(2, OUTPUT);
  pinMode(voltagepin, INPUT);
  digitalWrite(2, LOW);  // inactive
#endif
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  for (int i = 0; i < numfcts; i++) {
    fctarray[i] = 0;
    fctarrayT1[i] = 0;
    fctarrayT2[i] = 0;
    fctarrayT3[i] = 0;
    fctarrayT4[i] = 0;
  }
  for (int i = 0; i < numfcts; i++) {
    t_fctarray[i] = 0;
    t_fctarrayT1[i] = 0;
    t_fctarrayT2[i] = 0;
    t_fctarrayT3[i] = 0;
    t_fctarrayT4[i] = 0;
  }
  for (int i = 0; i < numfcts; i++) {
    blkdfks[i] = 0;
    blkdfksT1[i] = 0;
    blkdfksT2[i] = 0;
    blkdfksT3[i] = 0;
    blkdfksT4[i] = 0;
  }
  // ------------------------------------
  EEPROM.begin(EEPROM_SIZE);
  dccadr = EEPROM.read(0);
  dccadr = (dccadr << 8) + EEPROM.read(1);
  if ((dccadr <= 0) || (dccadr > 9999)) {  // as supported by Lenz LZV100 V3.6 and later.
    dccadr = 3;
    EEPROM.write(0, 0);
    EEPROM.write(1, 3);
    EEPROM.commit();
  }
  numthrottles = EEPROM.read(19);
  if ((numthrottles < 1) || (numthrottles > 4)) {
    numthrottles = 1;
    EEPROM.write(19, 1);
    EEPROM.commit();
  }
  sprintf(dspadr, "%04d", dccadr);  // print with leading zeros
  #ifdef LOG
  DEBUG = EEPROM.read(95);
  if (DEBUG == true) {
    Serial.println("Setup:-----------------------------");
    Serial.print(vrs);
    Serial.println(" Running in DEBUG mode.");
  }
  #endif
  // retrieve stack:
  retrievestack();
  // -keypad config:-----------------------------------
  #ifdef LOG
  if (DEBUG == true) {
    Serial.println("Setup: show keypad values in EEPROM:");
    byte k = 20;
    for (byte i = 0; i < 32; i++) {
      int v = EEPROM.read(k);
      k++;
      v = (v << 8) + EEPROM.read(k);
      k++;
      Serial.print(v);
      Serial.print(",");
    }
    Serial.println();
  }
  #endif
  //Get keypad values from EEPROM:
  byte j = 20;
  for (byte i = 0; i < 32; i++) {
    keypadarray[i] = EEPROM.read(j);
    j++;
    keypadarray[i] = (keypadarray[i] << 8) + EEPROM.read(j);
    j++;
  }
  #ifdef LOG
  if (DEBUG == true) {
    Serial.println("Setup: Keypad values in array: ");
    for (byte i = 0; i < 32; i++) {
      int val = keypadarray[i];
      Serial.print(val);
      Serial.print(",");
    }
    Serial.println();
  }
  #endif
  //................................................................................
  prvsAState = digitalRead(encoderA);  // Read the "initial" state of encoderA
  attachInterrupt(encoderA, encisr, CHANGE);
  //................................................................................
  ctname = tname + String(MThrottle);  // start with default throttle name (1)
  #ifdef LOG
  if (DEBUG == true) dsptname = ctname + "d";
  else dsptname = ctname;
  #endif
  #ifndef LOG
  dsptname = ctname;
  #endif
  display.setFont(ArialMT_Plain_10);
  display.setColor(BLACK);
  display.fillRect(0, 0, 132, 15);
  display.setColor(WHITE);
  display.drawString(0, 0, dsptname);
  if (numthrottles == 1) display.drawString(55, 0, "Single Throttle");
  else if (numthrottles > 1) display.drawString(55, 0, String(numthrottles) + " Throttles");
  // enter config mode:
  display.drawString(0, 10, "Press Stop for Setup");
  display.display();
  unsigned long pushtime = millis();
  while (millis() < pushtime + 5000) {  //wait up to 5 sec
    swt3state = digitalRead(switch3pin);
    if (swt3state == LOW) {
      thrconfig = true;
      break;
    }
  }
  if (thrconfig == true) {
    thrchg = false;
    thrcnt = numthrottles - 1;  //set counter to current setting (array starts at 0)
    display.setColor(BLACK);
    display.fillRect(0, 10, 132, 15);
    display.fillRect(45, 0, 87, 15);
    display.setColor(WHITE);
    display.drawString(45, 0, "Set nbr throttles");
    display.drawString(0, 10, "Current setting:");
    display.setFont(ArialMT_Plain_16);
    display.drawString(10, 24, String(cfgthrottle[thrcnt]));
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 43, "Turn knob to select");
    display.drawString(0, 53, "Push knob to accept");
    display.display();
  }
  while (thrconfig == true) {
    if (encoder_moved == true) {
      display.setColor(BLACK);
      display.fillRect(0, 10, 132, 18);
      display.setFont(ArialMT_Plain_16);
      display.fillRect(0, 24, 132, 18);
      display.setColor(WHITE);
      display.drawString(10, 24, String(cfgthrottle[thrcnt]));
      display.display();
      encoder_moved = false;
      thrchg = true;
    }
    swtEstate = digitalRead(switchEpin);
    if (swtEstate == LOW) {
      delay(dbcdly);
      if (thrchg == true) {
        numthrottles = thrcnt + 1;  // (array starts at 0, numthrottles starts at 1)
        #ifdef LOG
        if (DEBUG == true) Serial.println("New number of throttles: " + String(numthrottles));
        #endif
        EEPROM.write(19, numthrottles);
        EEPROM.commit();
      }
      thrchg = false;
      thrconfig = false;
    }
  }
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, dsptname);
  /* connecting to a WiFi network **************************************************/
  display.drawString(45, 0, "connecting WiFi");
  display.display();
  #ifdef LOG
  if (DEBUG == true) Serial.print("Setup: Starting WiFi");
  #endif
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    #ifdef LOG
    if (DEBUG == true) Serial.print('.');
    #endif
    delay(1000);
    cntt++;
    swt3state = digitalRead(switch3pin);
    if ((cntt > 60) || (swt3state == LOW)) {
      #ifdef LOG
      if (DEBUG == true) {
        Serial.println();
        Serial.println("Setup: WiFi connection failed");
      }
      #endif
      svr = false;
      WiFi.disconnect();
      display.setColor(BLACK);
      display.fillRect(45, 0, 92, 15);
      display.setColor(WHITE);
      display.drawString(45, 0, "WiFi offline");
      display.display();
      #ifdef LOG
      if (DEBUG == true) Serial.println("Setup: WiFi disconnected");
      #endif
      goto bailout;
    }
  }
  display.setColor(BLACK);
  display.fillRect(45, 0, 132, 15);
  display.setColor(WHITE);
  display.drawString(45, 0, "WiFi connected");
  display.display();
  #ifdef LOG
  if (DEBUG == true) {
    Serial.println("Setup: WiFi connected ");
    ip = WiFi.localIP();
    Serial.print("Client IP address: ");
    Serial.println(ip);
  }
  #endif
  MacString = WiFi.macAddress();
  delay(2000);
  display.clear();
  display.drawString(0, 0, dsptname + "  Connecting svr");
  display.display();
  #ifdef LOG
  if (DEBUG == true) {
    Serial.println("Setup: starting connection to WiThrottle server");
    Serial.print("Server IP: ");
    Serial.print(host);
    Serial.println(":" + String(port));
    Serial.print("connecting");
  }
  #endif
  cntc = 6;
  cntt = 0;
  client.connect(host, port);
  while (!client.connected()) {
    #ifdef LOG
    if (DEBUG == true) {
      Serial.print(cntc);
      Serial.print('.');
    }
    #endif
    delay(1000);
    cntc--;
    cntt++;
    if (cntc <= 0) {
      cntc = 6;
      #ifdef LOG
      if (DEBUG == true) Serial.println("retrying");
      #endif
      client.connect(host, port);
    }
    if (cntt > 60) {
      #ifdef LOG
      if (DEBUG == true) {
        Serial.println();
        Serial.println("Setup: server not responding, closing.");
      }
      #endif
      client.stop();
      display.setColor(BLACK);
      display.fillRect(43, 0, 95, 12);
      display.setColor(WHITE);
      display.drawString(45, 0, "svr offline");
      display.display();
      WiFi.disconnect();
      #ifdef LOG
      if (DEBUG == true) {
        Serial.println("Setup: WiFi disconnected");
      }
      #endif
      svr = false;
      //return;
      goto bailout;  // continue without WiFi for testing
    }
  }
  //connected >>>
  display.setFont(ArialMT_Plain_10);
  display.setColor(BLACK);
  display.fillRect(43, 0, 95, 12);
  display.setColor(WHITE);
  display.drawString(45, 0, "online");
  display.display();
  #ifdef LOG
  if (DEBUG == true) {
    Serial.println();
    Serial.println("Setup: Sending throttle name: N" + tname);
  }
  #endif
  client.print("N" + tname + "\r\n");  // set Name of Throttle
  client.flush();
  delay(sdelay);
  while (client.available()) {
    jmri_res = client.readStringUntil('\n');
    #ifdef LOG
    if (DEBUG == true) {
      Serial.print("Setup: Info received from WiThrottle server: ");
      Serial.println(jmri_res);
    }
    #endif
    if (jmri_res.startsWith("RL")) {
      roster = jmri_res;  // save roster
    }
    if (jmri_res.startsWith("PRL")) {
      routes = jmri_res;  // save route list
      rts_Avail = true;
    }
    if (jmri_res.startsWith("PTL")) {
      turnouts = jmri_res;  // save turnout list
      trn_Avail = true;
    }
    if (jmri_res.startsWith("*")) {
      #ifdef LOG
      if (DEBUG == true) {
        Serial.print("heart beat from server: ");
        Serial.println(jmri_res);
      }
      #endif
      hbres = jmri_res;                    // save heartbeat value
      String hbs = hbres.substring(1, 3);  // get heart beat interval
      hbitv = hbs.toInt();
      #ifdef LOG
      if (DEBUG == true) {
        Serial.print("Setup: heart beat itv response: ");
        Serial.println(hbitv);
      }
      #endif
      if (hbitv > 5) {
        cntf = hbitv / 1.5;
        ch = (int)cntf;
        hbuse = true;
      }
    }
  }
  if (hbuse == false) ch = 12;  //value needed for checking connection status.
  cnth = ch;
  #ifdef LOG
  if (DEBUG == true) {
    Serial.println("Setup: heart beat interval = " + String(ch));
    Serial.println("Sending MAC: HU" + MacString);
  }
  #endif
  client.print("HU" + MacString + "\r\n");  // set identifier
  client.flush();
  delay(fdelay);
  /**end of WiFi connection startup ***********************************************/
  #ifdef LOG
  if (DEBUG == true) {
    Serial.println("roster list: " + roster);
    Serial.println("route list: " + routes);
    Serial.println("turnout list: " + turnouts);
  }
  #endif
  //----build index of roster entries----:
  rxn = roster.indexOf('L');  // find first roster entry for nbr of entries
  rxn = rxn + 1;
  rxnn = roster.indexOf(']');
  rosne = roster.substring(rxn, rxnn);  // get nbr of roster entries
  numre = rosne.toInt();                // convert to integer
  if (numre > 0) {                      // making sure we got a working value
    numos = numre - 1;                  // offsets start with 0
    for (int i = 0; i <= numos; i++) {
      rxn = roster.indexOf('[', rxnn);
      rxn++;
      rosnamoffsets[i] = rxn;
      rxnn = rxn + 1;
      int rxa = roster.indexOf('{', rxnn);
      rxa++;
      rosadroffsets[i] = rxa;
      rxnn = rxa + 1;
    }
    #ifdef LOG
    if (DEBUG == true) {
      Serial.print("nbr of roster entries: ");
      Serial.println(numre);
      Serial.print("roster name offsets: ");
      for (int i = 0; i <= numos; i++) {
        Serial.print(rosnamoffsets[i]);
        Serial.print(",");
      }
      Serial.println();
      Serial.print("roster address offsets: ");
      for (int i = 0; i <= numos; i++) {
        Serial.print(rosadroffsets[i]);
        Serial.print(",");
      }
      Serial.println();
    }
    #endif
  }
  //----build index of route entries---->>>:
  if (rts_Avail == true) {
    rxnn = 0;
    rxn = 0;
    // PRL]\[IO...}|{usrname}|{
    for (byte i = 0; i < maxnbrrts; i++) {
      rxn = routes.indexOf("IO", rxnn);  // count sysname entries
      if (rxn == -1) break;
      numrts++;
      rxn = rxn + 2;
      rxnn = rxn + 1;
    }
    #ifdef LOG
    if (DEBUG == true) {
      Serial.print("Setup: Routes counted: ");
      Serial.println(numrts);
    }
    #endif
    if (numrts > 0) {
      rxnn = 0;
      numrtos = numrts - 1;
      for (byte i = 0; i < numrts; i++) {
        rxn = routes.indexOf("IO", rxnn);
        rsosa[i] = rxn;  // store index of sysname
        rxnn = rxn + 1;
        int rxa = routes.indexOf('{', rxnn);
        rxa++;
        ruosa[i] = rxa;  // store index of usrname
        rxnn = rxa + 1;
      }
      #ifdef LOG
      if (DEBUG == true) {
        Serial.println("route sysname offsets");
        for (int i = 0; i < numrts; i++) {
          Serial.print(rsosa[i]);
          Serial.print(",");
        }
        Serial.println();
        Serial.println("route usrname offsets");
        for (int i = 0; i < numrts; i++) {
          Serial.print(ruosa[i]);
          Serial.print(",");
        }
        Serial.println();
      }
      #endif
    } else rts_Avail = false;
  }
  //----build index of turnout entries----->>>:
  if (trn_Avail == true) {
    rxnn = 0;
    rxn = 0;
    // PTL]\[XT301}|{usrname}|{1]  (X=Xpressnet, 201=accadr, 1=state)
    for (byte i = 0; i < maxnbrtrns; i++) {
      //rxn = turnouts.indexOf("T", rxnn);  // count sysname entries
      rxn = turnouts.indexOf("T", rxnn);  // count sysname entries
      if (rxn == -1) break;
      numts++;
      rxn++;
      rxnn = rxn + 1;
    }
    #ifdef LOG
    if (DEBUG == true) {
      Serial.print("Setup: Turnouts counted: ");
      Serial.println(numts);
    }
    #endif
    if (numts > 0) {
      rxnn = 0;
      numtos = numts - 1;
      for (byte i = 0; i < numts; i++) {
        rxn = turnouts.indexOf('[', rxnn);
        rxnn = rxn + 3;   // point to address
        tsosa[i] = rxnn;  // store index of accessory address
        int rxa = turnouts.indexOf('{', rxnn);
        rxa++;
        tuosa[i] = rxa;  // store index of usrname
        rxnn = rxa + 1;
        int rxb = turnouts.indexOf('{', rxnn);
        rxb++;
        tstosa[i] = rxb;  // store index of state
        rxnn = rxb + 1;
      }
      #ifdef LOG
      if (DEBUG == true) {
        Serial.println("turnout sysname (address) offsets");
        for (int i = 0; i < numts; i++) {
          Serial.print(tsosa[i]);
          Serial.print(",");
        }
        Serial.println();
        Serial.println("turnout usrname offsets");
        for (int i = 0; i < numts; i++) {
          Serial.print(tuosa[i]);
          Serial.print(",");
        }
        Serial.println();
        Serial.println("turnout state offsets");
        for (int i = 0; i < numts; i++) {
          Serial.print(tstosa[i]);
          Serial.print(",");
        }
        Serial.println();
      }
      #endif
    } else trn_Avail = false;
  }
  /*--------------------------------------------------------*/
  rtvrosnam();
  if (ros_eAvail == true) getrosnamen();
bailout:  // continue without server connection for testing
  //display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setColor(WHITE);
  display.drawString(80, 13, String(dspadr));  // dccadr with leading zero's
  display.drawString(120, 13, "?");
  display.display();
  #ifdef LOG
  if (DEBUG == true) Serial.println("Setup: last dcc addr: " + String(dspadr));
  #endif
  dsprosnam();
  dsprosadr();
  if (dctstate == true) {
    display.drawString(0, 13, "F");
  } else {
    display.drawString(0, 13, "R");
  }
  display.drawString(15, 13, String(vspeed));
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 50, "Fg");
  display.drawString(15, 50, String(fcgstate));
  display.display();
  delay(1000);
  // simulate first address button push, so next push takes address from stack
  adrbtnstate = true;
  setdccadr = true;
  pshadr = true;
  if (hbuse == true) {
    #ifdef LOG
    if (DEBUG == true) Serial.println("Setup: Sending *+ start heart beat");
    #endif
    client.print("*+\r\n");  // starting heartbeat
    client.flush();
  }
  cntd = cd;
  cntb = cb;
#ifdef BATLEVEL
  getbatlvl();
#endif
#ifdef LOG
  if (DEBUG == true) {
    if (numthrottles > 1) {
      Serial.println("This device is set as Multi Throttle.");
      Serial.println("Active Throttle: " + String(MThrottle));
    }
    if (numthrottles == 1) {
      Serial.println("This device is set as Single Throttle.");
    }
    Serial.println("Setup complete.--------------------------");
  }
  #endif
}
/*************************************************************************************/
void loop() {
  if (Serial.available() > 0) {
    int indata = Serial.read();
    if ((indata == 'V') || (indata == 'v')) {
      Serial.println(vrs);
    }
    if ((indata == 'C') || (indata == 'c')) {
      Serial.println(vrs);
      Serial.println("4x4 keypad config.");
      Serial.println("Please put your serial monitor in 'No Line Ending' mode.");
      Serial.println("To cancel or to end enter 99 or press the Stop-Button.");
      keypad_check();
      keypad_config();
    }
    if ((indata == 'T') || (indata == 't')) {
      Serial.println(vrs);
      Serial.println("Single/Multi-Throttle config.");
      Serial.println("Please put your serial monitor in 'No Line Ending' mode.");
      Serial.println("Current number of throttles: " + String(numthrottles));
      Serial.println("Enter the nbr of Throttles, up to 4. For Single Throttle enter 1.");
      Serial.println("To cancel or to end enter 9 or press the Stop-Button.");
      cfgthr();
    }
    #ifdef LOG
    if ((indata == 'D') || (indata == 'd')) {
      DEBUG = !DEBUG;
      if (DEBUG == true) {
        EEPROM.write(95, DEBUG);  // keep beyond pwr off
        EEPROM.commit();
        Serial.println(vrs);
        Serial.println("Running in DEBUG mode.");
        ctname = tname + String(MThrottle);  // start with default throttle name
        dsptname = ctname + "d";
        display.setFont(ArialMT_Plain_10);
        display.setColor(BLACK);
        display.fillRect(0, 0, 45, 15);
        display.setColor(WHITE);
        display.drawString(0, 0, dsptname);
        display.display();
      }
      if (DEBUG == false) {
        EEPROM.write(95, DEBUG);  // keep at pwr off
        EEPROM.commit();
        Serial.println("DEBUG mode ended.");
        ctname = tname + String(MThrottle);
        dsptname = ctname;
        display.setFont(ArialMT_Plain_10);
        display.setColor(BLACK);
        display.fillRect(0, 0, 45, 15);
        display.setColor(WHITE);
        display.drawString(0, 0, dsptname);
        display.display();
      }
    }
    #endif
    if ((indata == 'E') || (indata == 'e')) {
      DEBUGENC = !DEBUGENC;
      if (DEBUGENC == true) {
        Serial.println(vrs);
        Serial.println("Running Encoder in DEBUG mode.");
      }
      if (DEBUGENC == false) Serial.println("Encoder DEBUG mode ended.");
    }
  }
#ifdef CLOCK
  // look for fast clock time stamp from jmri
  if (svr == true) {
    if (client.available()) {
      jmri_res = client.readStringUntil('\n');
      if (jmri_res.startsWith("PFT")) {
        getjmritime();
      }
    }
  }
#endif
  // heartbeat:
  crntMs = millis();
  if (crntMs - prvsMs >= 1000) {  // 1 second interval timer
    prvsMs = crntMs;
    cnth--;
    cntd--;
    cntb--;  // battery monitor
  }
  if (cnth <= 0) {  // heartbeat time
    // still connected?:
    if ((!client.connected()) && (svr == true)) {
      #ifdef LOG
      if (DEBUG == true) {
        Serial.println();
        Serial.println("Main: Throttle offline.");
      }
      #endif
      if (dsplon == false) dspreset();
      display.setFont(ArialMT_Plain_10);
      display.setColor(BLACK);
      display.fillRect(43, 0, 90, 12);
      display.setColor(WHITE);
      display.drawString(45, 0, "Svr offline");
      display.display();
      svr = false;
      WiFi.disconnect();
      delay(100);
      if (recon == false) {
        #ifdef LOG
        if (DEBUG == true) Serial.print("Main: Attempting to reconnect WiFi.");
        #endif
        cntt = 0;
        recon = true;
        WiFi.begin(ssid, pass);
        while (WiFi.status() != WL_CONNECTED) {
          #ifdef LOG
          if (DEBUG == true) Serial.print('.');
          #endif
          delay(1000);
          cntt++;
          if (cntt > 60) {
            #ifdef LOG
            if (DEBUG == true) {
              Serial.println();
              Serial.println("Main: WiFi connection failed");
            }
            #endif
            return;
          }
        }
        #ifdef LOG
        if (DEBUG == true) {
          Serial.println();
          Serial.println("Main: Reconnecting server");
        }
        #endif
        display.setColor(BLACK);
        display.fillRect(43, 0, 95, 12);
        display.setColor(WHITE);
        display.drawString(45, 0, "reconnecting");
        display.display();
        cntc = 6;
        cntt = 0;
        client.connect(host, port);
        while (!client.connected()) {
          if ((analogRead(keypadin) >= keypadarray[30]) && (btnread <= keypadarray[31])) cntt = 90;  //stop retry
          #ifdef LOG          
          if (DEBUG == true) {
            Serial.print(cntc);
            Serial.print('.');
          }
          #endif
          delay(1000);
          cntc--;
          cntt++;
          if (cntc <= 0) {
            cntc = 6;
            #ifdef LOG
            if (DEBUG == true) Serial.println("retrying the reconnect");
            #endif
            client.connect(host, port);
          }
          if (cntt > 90) {
            #ifdef LOG
            if (DEBUG == true) {
              Serial.println();
              Serial.println("Main: server reconnect failed");
            }
            #endif
            client.stop();
            display.setColor(BLACK);
            display.fillRect(43, 0, 95, 12);
            display.setColor(WHITE);
            display.drawString(45, 0, "reconnect failed");
            display.display();
            WiFi.disconnect();
            #ifdef LOG
            if (DEBUG == true) {
              Serial.println("Throttle offline");
              Serial.println("WiFi disconnected");
            }
            #endif
            svr = false;
            return;
          }
        }
        svr = true;
        display.setColor(BLACK);
        display.fillRect(43, 0, 90, 12);
        display.setColor(WHITE);
        display.drawString(45, 0, "svr online");
        display.display();
        #ifdef LOG
        if (DEBUG == true) {
          Serial.println();
          Serial.println("Main: Sending MAC HU" + MacString);
        }
        #endif
        client.print("N" + tname + "\r\n");  // set Name of Throttle
        delay(fdelay);
        client.print("HU" + MacString + "\r\n");  // set identifier
        delay(ldelay);
        recon = false;
      }
      if (hbuse == true) {
        client.print("*+\r\n");  // starting heartbeat
        delay(ldelay);
        client.flush();
      }
      svr = true;
      releaseadr(dccadr);
      adrbtnstate = true;
      setdccadr = true;
      pshadr = true;
      display.setFont(ArialMT_Plain_16);
      display.setColor(WHITE);
      display.drawString(120, 13, "?");
      display.display();
    }
    if ((svr == true) && (hbuse == true)) {
      client.print("*\r\n");  // send heart beat
    }
    cnth = ch;
  }
  if ((cntd <= 0) && (dsplon == true)) {  // time for turning display dark
    display.displayOff();
    dsplon = false;
  }
#ifdef BATLEVEL
  if (cntb <= 0) {
    getbatlvl();
    cntb = cb;
  }
#endif
  //-------Encoder---------------------------------------------------
  if (encoder_moved == true) {  // encoder has moved
    if (dsplon == false) {      // turn Display back on
      dspreset();
      if (ecnt > pecnt) vcnt--;
      pecnt = ecnt;
      encoder_moved = false;
      return;
    } else {
      cntd = cd;
      pecnt = ecnt;
    }
    // reset estop in case it was -----------------------
    if (estop == true) {
      if (dctstate == true) {
        set_forward();
      } else {
        set_reverse();
      }
      estop = false;
      display.setFont(ArialMT_Plain_16);
      display.setColor(BLACK);
      display.fillRect(80, 30, 70, 18);  // remove STOP from display
      display.display();
    }
    if (DEBUGENC == true) {
      Serial.print("Encoder cnt: ");
      Serial.println(ecnt);
      Serial.print("speed vcnt: ");
      Serial.println(vcnt);
      Serial.print("roster rcnt: ");
      Serial.println(rcnt);
      Serial.print("route rtcnt: ");
      Serial.println(rtcnt);
      Serial.print("function fcnt: ");
      Serial.println(fcnt);
      Serial.print("turnout tcnt: ");
      Serial.println(tcnt);
      Serial.print("throttle thrcnt: ");
      Serial.println(thrcnt);
    }
    //speed:----------------------------
    if ((setdccadr == false) && (setaccadr == false) && (setfctbn == false) && (sltroute == false) && (encoder_moved == true)) {
      //if ((setdccadr == false) && (setaccadr == false) && (setfctbn == false) && (encoder_moved == true)) {
      set_speed();
    }
    // ----roster--------------------------------------------
    if ((setdccadr == true) && (setaccadr == false)) {
      getrosnam(rcnt);
      dsprosnam();
      getrosadr(rcnt);
      dsprosadr();
      dspcnt(rcnt);
      #ifdef LOG
      if (DEBUG == true) {
        Serial.println("Encoder moved:");
        Serial.print("roster offset: ");
        Serial.println(rcnt);
        Serial.print("name from roster: ");
        Serial.println(loconame);
        Serial.print("address from roster: ");
        Serial.println(locoadr);
      }
      #endif
    }
    //-----------routes---------------------
    if ((setdccadr == false) && (sltroute == true) && (rts_Avail == true)) {
      setroute = true;
      clrrosnam();
      getroute(rtcnt);
      dspcnt(rtcnt);
      display.setFont(ArialMT_Plain_16);
      display.setColor(BLACK);
      display.fillRect(0, 30, 132, 18);
      display.setColor(WHITE);
      display.drawString(0, 30, rusrname);
      display.display();
    }
    //
    //---turnout names----------------------------
    if ((setdccadr == false) && (setaccadr == true) && (trn_Avail == true)) {
      clrrosnam();
      gettoname(tcnt);
      dspcnt(tcnt);
      display.setFont(ArialMT_Plain_16);
      display.setColor(BLACK);
      display.fillRect(0, 30, 132, 18);
      display.setColor(WHITE);
      display.drawString(0, 30, tusrname);
      display.display();
    }
    //-----------function names---------------------
    if ((setdccadr == false) && (setfctbn == true) && (fctsavail == true)) {
      clrrosnam();
      dspcnt(fcnt);
      getfctbn(fcnt);
      display.setFont(ArialMT_Plain_16);
      display.setColor(BLACK);
      display.fillRect(0, 30, 132, 18);
      display.setColor(WHITE);
      display.drawString(0, 30, dspfctname);
      display.display();
    }
    encoder_moved = false;
  }
  // dedicated switches: ////////////////////////////
  swtWstate = digitalRead(switchWpin);  // whistle
  if (swtWstate != lastswtWstate) {
    if (dsplon == false) {
      dspreset();
      delay(500);
      return;
    } else {
      cntd = cd;
    }
    lastswtWstate = swtWstate;
  }
  if ((swtWstate == LOW) && (longwhstl == false)) {
    if (swtWon == false) startTime = millis();
    swtWon = true;
    shortwhstl = true;
    if (((millis() - startTime) >= 500) && (longwhstl == false)) {
      longwhstl = true;
      shortwhstl = false;
      fnum = fctswtmap[1];                    // swtmap element 1 points to function "long whistle"
      if (fnum < numfcts) blkdfks[fnum] = 1;  // disable Function key
      fctbtnst = 1;
      sendfct(fnum);
      #ifdef LOG
      if (DEBUG == true) Serial.println("Switch W: long whistle on");
      #endif
    }
  }
  if ((swtWstate == HIGH) && (swtWon == true)) {
    swtWon = false;  // turn short whistle on & off, or long whistle off.
    if (shortwhstl == true) {
      fnum = fctswtmap[0];                    // swtmap element 0 points to function "short whistle"
      if (fnum < numfcts) blkdfks[fnum] = 1;  // disable Function key
      fctbtnst = 1;
      shortwhstl = false;
      #ifdef LOG
      if (DEBUG == true) Serial.println("Switch W: short whistle on");
      #endif
      sendfct(fnum);
      delay(mdelay);
      fctbtnst = 0;
      sendfct(fnum);
      if (fnum < numfcts) blkdfks[fnum] = 0;
      #ifdef LOG
      if (DEBUG == true) Serial.println("Switch W: short whistle off");
      #endif
    }
    if (longwhstl == true) {
      fnum = fctswtmap[1];                    // swtmap element 1 points to function "long whistle"
      if (fnum < numfcts) blkdfks[fnum] = 1;  // disable Function key
      fctbtnst = 0;
      sendfct(fnum);
      if (fnum < numfcts) blkdfks[fnum] = 0;
      longwhstl = false;
      #ifdef LOG
      if (DEBUG == true) Serial.println("Switch W: long whistle off");
      #endif
    }
  }
  //-------------------------------------------------------
  swtBstate = digitalRead(switchBpin);
  if (swtBstate != lastswtBstate) {
    if (dsplon == false) {
      lastswtBstate = swtBstate;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    if ((swtBstate == LOW) && (swtBon == false)) {
      swtBon = true;
      #ifdef LOG
      if (DEBUG == true) Serial.println("Switch B on");
      #endif
      fnumts = fctswtmap[2];  // swtmap element 2 points to function "Bell"
      if (fctarray[fnumts] == 1) {
        #ifdef LOG
        if (DEBUG == true) Serial.println("Switch blocked, function is on");
        #endif
        return;
      }
      if (fnum < numfcts) blkdfks[fnum] = 1;  // disable Function key
      fctbtnst = 1;
      sendfct(fnumts);
      delay(mdelay);
      fctbtnst = 0;  // simulate button
      sendfct(fnumts);
      #ifdef LOG
      if (DEBUG == true) Serial.println("Switch B fct on");
      #endif
    }
    if ((swtBstate == HIGH) && (swtBon == true)) {
      swtBon = false;
      #ifdef LOG
      if (DEBUG == true) Serial.println("Switch B off");
      #endif
      fctbtnst = 1;
      sendfct(fnumts);
      delay(mdelay);
      fctbtnst = 0;  // simulate momentary
      sendfct(fnumts);
      if (fnumts < numfcts) blkdfks[fnumts] = 0;
      #ifdef LOG
      if (DEBUG == true) Serial.println("Switch B fct off");
      #endif
    }
    lastswtBstate = swtBstate;
  }
  //----------------------------------------------------------------------
  swt2state = digitalRead(switch2pin);
  if (swt2state != lastswt2state) {
    if (dsplon == false) {
      lastswt2state = swt2state;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    if ((swt2state == LOW) && (swt2on == false)) {
      swt2on = true;
      delay(dbcdly);
      if ((dctstate == false) && (fctswtmap[5] != 99)) fnum = fctswtmap[5];  // map 5 = front coupler
      if ((dctstate == false) && (fctswtmap[5] == 99)) fnum = fctswtmap[4];  // map 4 = rear coupler
      if (dctstate == true) fnum = fctswtmap[4];                             // swtmap element 4 points to function "Coupler" (rear)
      if (fnum < numfcts) blkdfks[fnum] = 1;                                 // disable Function key
      fctbtnst = 1;
      #ifdef LOG
      if (DEBUG == true) Serial.println("Switch 2 - coupler fct on");
      #endif
      sendfct(fnum);
      if (setsdhadr == true) {
        if (sdhdct == true) dblhdrfct = sdhswtmap[4];
        if ((sdhdct == false) && (sdhswtmap[5] == 99)) dblhdrfct = sdhswtmap[4];
        if ((sdhdct == false) && (sdhswtmap[5] != 99)) dblhdrfct = sdhswtmap[5];
        sendsdhfct(dblhdrfct);
      }
    }
    if ((swt2state == HIGH) && (swt2on == true)) {
      swt2on = false;
      delay(dbcdly);
      fctbtnst = 0;
      #ifdef LOG
      if (DEBUG == true) Serial.println("Switch 2 - coupler fct off");
      #endif
      sendfct(fnum);
      if (setsdhadr == true) sendsdhfct(dblhdrfct);
      if (fnum < numfcts) blkdfks[fnum] = 0;
    }
    lastswt2state = swt2state;
  }
  //----Switch 1 defaults to Release loco ---->>----------------------------------------------------------
  swt1state = digitalRead(switch1pin);
  if (swt1state != lastswt1state) {
    if (dsplon == false) {
      lastswt1state = swt1state;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    //--------------------------------------
    if ((swt1state == LOW) && (swt1on == false)) {
      swt1on = true;
      delay(dbcdly);
      if (numthrottles == 1) {  // (defined as single throttle)
        // mapped function:----------
        if (fctswtmap[3] != 99) {
          fnum = fctswtmap[3];
          fctbtnst = 1;
          sendfct(fnum);
        }
        // switch not mapped to a function: ------
        if (fctswtmap[3] == 99) {
          if (setdccadr == false) {
            releaseadr(dccadr);
            adrbtnstate = true;
            setdccadr = true;
            clrchgadr();
            btncnt = 0;
            pshadr = false;
            vspeed = 0;
            vtemp = 0;
            vcnt = 0;
            lastvcnt = 0;
            kpbtnstate = true;
            display.setFont(ArialMT_Plain_16);
            display.setColor(BLACK);
            display.fillRect(14, 13, 30, 15);
            display.display();
            dspchgadr();
            dsp_speed();
          }
        }
      }
      if (numthrottles > 1) {                                              // (defined as multi throttle)
        MThrottle++;                                                       // current throttle number
        if ((MThrottle > numthrottles) || (MThrottle > 4)) MThrottle = 1;  // limit 4 throttles (1-4).
        ctname = tname + String(MThrottle);                                // start with default throttle name
        #ifdef LOG
        if (DEBUG == true) dsptname = ctname + "d";
        else dsptname = ctname;
        #endif
        #ifndef LOG
        dsptname = ctname;
        #endif
        display.setFont(ArialMT_Plain_10);
        display.setColor(BLACK);
        display.fillRect(0, 0, 45, 15);
        display.setColor(WHITE);
        display.drawString(0, 0, dsptname);
        display.display();
        //get dccadr etc. >>
        if (MThrottle == 1) {
          if ((numthrottles == 4) && (MT4active == true)) savethr4();
          if ((numthrottles == 3) && (MT3active == true)) savethr3();
          if ((numthrottles == 2) && (MT2active == true)) savethr2();
          if (MT1active == true) {
            dccadr = dccadrT1;
            lastdccadr = lastdccadrT1;
            loconame = loconameT1;
            vspeed = vspeedT1;
            vcnt = vcntT1;
            dspeed = vcnt;
            prvspeed = prvspeedT1;
            dctstate = dctstateT1;
            sdhadr = sdhadrT1;
            sdhdct = sdhdctT1;
            sdhacts = sdhactsT1;
            setsdhadr = setsdhadrT1;
            fcgstate = fcgstateT1;
            functions = functionsT1;
            numfcnt = numfcntT1;
            for (byte i = 0; i < 6; i++) {
              dspadr[i] = dspadrT1[i];
            }
            for (int i = 0; i < numfcts; i++) {
              fctarray[i] = fctarrayT1[i];
              t_fctarray[i] = t_fctarrayT1[i];
              blkdfks[i] = blkdfksT1[i];
              fctnos[i] = fctnosT1[i];
              fctnoe[i] = fctnoeT1[i];
              fctnox[i] = fctnoxT1[i];
            }
            for (int i = 0; i < numswfcs; i++) {
              fctswtmap[i] = fctswtmapT1[i];
              sdhswtmap[i] = sdhswtmapT1[i];
            }
            dspdccadr();
            clrrosnam();
            dsprosnam();
            dsp_speed();
            dsp_dctst();
            dspfcts();
          } else {
            setsdhadr = false;
            simlokbtn();
          }
        }
        if (MThrottle == 2) {
          if (MT1active == true) savethr1();
          if (MT2active == true) {
            dccadr = dccadrT2;
            lastdccadr = lastdccadrT2;
            loconame = loconameT2;
            vspeed = vspeedT2;
            vcnt = vcntT2;
            dspeed = vcnt;
            prvspeed = prvspeedT2;
            dctstate = dctstateT2;
            sdhadr = sdhadrT2;
            sdhdct = sdhdctT2;
            sdhacts = sdhactsT2;
            setsdhadr = setsdhadrT2;
            fcgstate = fcgstateT2;
            functions = functionsT2;
            numfcnt = numfcntT2;
            for (byte i = 0; i < 6; i++) {
              dspadr[i] = dspadrT2[i];
            }
            for (int i = 0; i < numfcts; i++) {
              fctarray[i] = fctarrayT2[i];
              t_fctarray[i] = t_fctarrayT2[i];
              blkdfks[i] = blkdfksT2[i];
              fctnos[i] = fctnosT2[i];
              fctnoe[i] = fctnoeT2[i];
              fctnox[i] = fctnoxT2[i];
            }
            for (int i = 0; i < numswfcs; i++) {
              fctswtmap[i] = fctswtmapT2[i];
              sdhswtmap[i] = sdhswtmapT2[i];
            }
            dspdccadr();
            clrrosnam();
            dsprosnam();
            dsp_speed();
            dsp_dctst();
            dspfcts();
            setdccadr = false;
          } else {
            setsdhadr = false;
            simlokbtn();
          }
        }
        if (MThrottle == 3) {
          if (MT2active == true) savethr2();
          if (MT3active == true) {
            dccadr = dccadrT3;
            lastdccadr = lastdccadrT3;
            loconame = loconameT3;
            vspeed = vspeedT3;
            vcnt = vcntT3;
            dspeed = vcnt;
            prvspeed = prvspeedT3;
            dctstate = dctstateT3;
            sdhadr = sdhadrT3;
            sdhdct = sdhdctT3;
            sdhacts = sdhactsT3;
            setsdhadr = setsdhadrT3;
            fcgstate = fcgstateT3;
            functions = functionsT3;
            numfcnt = numfcntT3;
            for (byte i = 0; i < 6; i++) {
              dspadr[i] = dspadrT3[i];
            }
            for (int i = 0; i < numfcts; i++) {
              fctarray[i] = fctarrayT3[i];
              t_fctarray[i] = t_fctarrayT3[i];
              blkdfks[i] = blkdfksT3[i];
              fctnos[i] = fctnosT3[i];
              fctnoe[i] = fctnoeT3[i];
              fctnox[i] = fctnoxT3[i];
            }
            for (int i = 0; i < numswfcs; i++) {
              fctswtmap[i] = fctswtmapT3[i];
              sdhswtmap[i] = sdhswtmapT3[i];
            }
            dspdccadr();
            clrrosnam();
            dsprosnam();
            dsp_speed();
            dsp_dctst();
            dspfcts();
            setdccadr = false;
          } else {
            setsdhadr = false;
            simlokbtn();
          }
        }
        if (MThrottle == 4) {
          if (MT3active == true) savethr3();
          if (MT4active == true) {
            dccadr = dccadrT4;
            lastdccadr = lastdccadrT4;
            loconame = loconameT4;
            vspeed = vspeedT4;
            vcnt = vcntT4;
            dspeed = vcnt;
            prvspeed = prvspeedT4;
            dctstate = dctstateT4;
            sdhadr = sdhadrT4;
            sdhdct = sdhdctT4;
            sdhacts = sdhactsT4;
            setsdhadr = setsdhadrT4;
            fcgstate = fcgstateT4;
            functions = functionsT4;
            numfcnt = numfcntT4;
            for (byte i = 0; i < 6; i++) {
              dspadr[i] = dspadrT4[i];
            }
            for (int i = 0; i < numfcts; i++) {
              fctarray[i] = fctarrayT4[i];
              t_fctarray[i] = t_fctarrayT4[i];
              blkdfks[i] = blkdfksT4[i];
              fctnos[i] = fctnosT4[i];
              fctnoe[i] = fctnoeT4[i];
              fctnox[i] = fctnoxT4[i];
            }
            for (int i = 0; i < numswfcs; i++) {
              fctswtmap[i] = fctswtmapT4[i];
              sdhswtmap[i] = sdhswtmapT4[i];
            }
            dspdccadr();
            clrrosnam();
            dsprosnam();
            dsp_speed();
            dsp_dctst();
            dspfcts();
            setdccadr = false;
          } else {
            setsdhadr = false;
            simlokbtn();
          }
        }
        #ifdef LOG
        if (DEBUG == true) {
          Serial.println("Throttle changed to: " + String(MThrottle));
          Serial.println("functions array copied: ");
          Serial.println(functions);
          Serial.println("number of functions: " + String(numfcnt));
          Serial.print("fctnos copied: ");
          for (byte i = 0; i < numfcts; i++) {
            Serial.print(fctnos[i]);
            Serial.print(",");
          }
          Serial.println();
          Serial.print("fctnoe copied: ");
          for (byte i = 0; i < numfcts; i++) {
            Serial.print(fctnoe[i]);
            Serial.print(",");
          }
          Serial.println();
          Serial.print("fctnox copied: ");
          for (byte i = 0; i < numfcts; i++) {
            Serial.print(fctnox[i]);
            Serial.print(",");
          }
          Serial.println();
        }
        #endif
      } 
    }
    if ((swt1state == HIGH) && (swt1on == true)) {
      swt1on = false;
      delay(dbcdly);
      if (numthrottles == 1) {
        if (fctswtmap[3] != 99) {
          fnum = fctswtmap[3];
          fctbtnst = 0;
          sendfct(fnum);
        }
      }
    }
    lastswt1state = swt1state;
  }
  //---Switch 3: Brake/Stop (not Estop) --------------------------------------------
  swt3state = digitalRead(switch3pin);
  if (swt3state != lastswt3state) {
    if (dsplon == false) {
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    lastswt3state = swt3state;
  }
  if ((swt3state == LOW) && (pushlong == false)) {
    if (swt3on == false) startTime = millis();
    swt3on = true;
    delay(dbcdly);
    pushshort = true;
    if (((millis() - startTime) >= 1000) && (pushlong == false)) {
      pushlong = true;
      pushshort = false;
      #ifdef LOG
      if (DEBUG == true) {
        if (vspeed != 0) Serial.println("Switch 3: long push ignored, speed is not 0)");
      }
      #endif
      if ((svr == true) && (vspeed == 0)) {
        #ifdef LOG
        if (DEBUG == true) Serial.println("Switch 3: long push - change speed step mode");
        #endif
        set_spstm();
      }
    }
  }
  if ((swt3state == HIGH) && (swt3on == true)) {
    swt3on = false;
    delay(dbcdly);
    pushlong = false;
    if (pushshort == true) {
      vspeed = 0;
      vtemp = 0;
      vcnt = 0;
      dspeed = vcnt;
      lastvcnt = 0;
      set_speed();
      pushshort = false;
      #ifdef LOG
      if (DEBUG == true) Serial.println("Switch 3: short push - brake");
      #endif
    }
  }
  //------------------------------------------------------------
  swtFstate = digitalRead(switchFpin);
  swtRstate = digitalRead(switchRpin);
  if (swtFstate != lastswtFstate) {
    if (dsplon == false) {
      lastswtFstate = swtFstate;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    if ((swtFstate == LOW) && (swtRstate == HIGH) && (setdccadr == false)) {
      if (dctstate == false) {
        vcnt = 0;
        set_speed();
      }
      if (dctstate == false) set_forward();
      if (driveholdON == true) {
        fnum = fctswtmap[7];
        fctbtnst = 0;
        sendfct(fnum);
        if (fnum < numfcts) blkdfks[fnum] = 0;
        driveholdON = false;
        #ifdef LOG
        if (DEBUG == true) Serial.println("Drive hold off");
        #endif
        display.setFont(ArialMT_Plain_10);
        display.setColor(BLACK);
        display.fillRect(50, 18, 17, 10);
        display.display();
      }
      swtCon = false;
    }
    lastswtFstate = swtFstate;
  }
  if ((swtRstate != lastswtRstate)) {
    if (dsplon == false) {
      lastswtRstate = swtRstate;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    if ((swtRstate == LOW) && (swtFstate == HIGH) && (setdccadr == false)) {
      if (dctstate == true) {
        vcnt = 0;
        set_speed();
      }
      if (dctstate == true) set_reverse();
      if (driveholdON == true) {
        fnum = fctswtmap[7];
        fctbtnst = 0;
        sendfct(fnum);
        if (fnum < numfcts) blkdfks[fnum] = 0;
        driveholdON = false;
        #ifdef LOG
        if (DEBUG == true) Serial.println("Drive hold off");
        #endif
        display.setFont(ArialMT_Plain_10);
        display.setColor(BLACK);
        display.fillRect(50, 18, 17, 10);
        display.display();
      }
      swtCon = false;
    }
    lastswtRstate = swtRstate;
  }
  // Function for Drive Hold >
  if (drivehold == true) {
    if ((swtFstate == HIGH) && (swtRstate == HIGH) && (setdccadr == false) && (driveholdON == false)) {
      if (swtCon == false) {
        startTime = millis();
        swtCon = true;
        #ifdef LOG
        if (DEBUG == true) Serial.println("Switch C on");
        #endif
      }
      if ((millis() - startTime) > 1000) {
        driveholdON = true;
        hcnt = vcnt;
        fnum = fctswtmap[7];
        if (fnum < numfcts) blkdfks[fnum] = 1;  // disable Function key
        fctbtnst = 1;
        sendfct(fnum);
        #ifdef LOG
        if (DEBUG == true) Serial.println("Drive hold on");
        #endif
        display.setFont(ArialMT_Plain_10);
        display.setColor(BLACK);
        display.fillRect(50, 18, 17, 10);
        display.setColor(WHITE);
        sprintf(dspadr, "%3d", hcnt);
        display.drawString(50, 18, String(dspadr));
        display.display();
        #ifdef LOG
        if (DEBUG == true) Serial.println("Drive hold ON: " + String(dspadr));
        #endif
      }
    }
  }
  //----Switch E: E-Stop----------------------------------------------
  swtEstate = digitalRead(switchEpin);
  if ((swtEstate != lastswtEstate)) {
    if (dsplon == false) {  //turn Display back on if off
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    lastswtEstate = swtEstate;
  }
  if ((swtEstate == LOW) && (pushlong == false) && (setdccadr == false) && (setfctbn == false)) {
    if (swtEon == false) startTime = millis();
    swtEon = true;
    pushshort = false;
    vspeed = 0;
    vcnt = 0;
    lastvcnt = 0;
    dspeed = vcnt;
    lastvcnt = 0;
    set_speed();
    if (svr == true) {
      client.print(locoacts + "X\r\n");
      client.flush();
      if (setsdhadr == true) {
        client.print(sdhacts + "X\r\n");
        client.flush();
      }
    }
    display.setFont(ArialMT_Plain_16);
    display.setColor(BLACK);
    display.fillRect(80, 30, 70, 18);
    display.fillRect(14, 13, 30, 15);
    display.setColor(WHITE);
    display.drawString(15, 13, String(dspeed));
    display.drawString(80, 30, "STOP");
    display.display();
    estop = true;
    if (((millis() - startTime) >= 1000) && (pushlong == false)) {
      pushlong = true;
      pushshort = false;
      #ifdef LOG
      if (DEBUG == true) {
        if (vspeed != 0) Serial.println("long push ignored, speed is not 0)");
      }
      #endif
      if ((svr == true) && (vspeed == 0)) {
        set_spstm();
        #ifdef LOG
        if (DEBUG == true) Serial.println("Switch E: long push - change speed step mode");
        #endif
        display.setFont(ArialMT_Plain_16);
        display.setColor(BLACK);
        display.fillRect(80, 30, 70, 18);  // remove STOP from display
        display.display();
      }
    }
  }
  if ((swtEstate == LOW) && (setdccadr == true) && (swtEon == false)) {
    swtEon = true;
    actdccadr();
    adrbtnstate = !adrbtnstate;
  }
  // set function:
  if ((swtEstate == LOW) && (setfctbn == true) && (swtEon == false)) {
    swtEon = true;
    fctbtnst = 1;
    sendfct(fnum);
    dspsfnum(fnum);
  }
  if ((swtEstate == HIGH) && (swtEon == true)) {
    swtEon = false;
    pushlong = false;
    pushshort = false;
    if (setfctbn == true) {
      fctbtnst = 0;
      sendfct(fnum);
      dspsfnum(fnum);
    }
  }
  //---------------------------------------------------------------------
  // read keypad:
  btnread = analogRead(keypadin);
  if (btnread > 10) {
    delay(dbcdly);
    kparrRead();  //button pressed - read it
  }
  /////////////////////////////////////////////////////////////
  // All keypad buttons released - none pushed:
  else if ((btnread == 0) && (kpbtnstate == true)) {
    kpbtnstate = false;
    noop = false;
    //  keypad buttons off:
    kpRelease();
  } else if ((btnread == 0) && (noop == true)) {
    kpbtnstate = false;
    noop = false;
    kpRelease();
  }
}
//-------------------------------------------------------
void kparrRead() {
  btnread = analogRead(keypadin);
  // Button 1: ***********************************
  if ((btnread >= keypadarray[0]) && (btnread <= keypadarray[1])) {
    if (dsplon == false) {
      kpbtnstate = true;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    btnnbr = 1;
    if (kpbtnstate == false) dspbtnnum();
    // set loco or assessory decoder address:
    if ((setdccadr == true) || (accbtnstate == 1)) {
      if ((btncnt < 5) && (kpbtnstate == false)) {
        btncnt++;
        btnval = 1;
        if (setaccadr == false) putdgt();
        if (setaccadr == true) putaccdgt();
        kpbtnstate = true;
      }
    }
// function button 1
#ifndef EUROSOUND
    if ((setdccadr == false) && (kpbtnstate == false)) {
      if ((blkdfks[1] == 1) || (blkdfks[11] == 1) || (blkdfks[21] == 1)) return;
      if (fc1btnstate == false) {
        fc1btnstate = true;
        fctbtnst = 1;
        if (fcgstate == 0) {
          fnum = 1;
          sendfct(fnum);
        }
        if (fcgstate == 1) {
          fnum = 11;
          sendfct(fnum);
        }
        if (fcgstate == 2) {
          fnum = 21;
          sendfct(fnum);
        }
      }
      kpbtnstate = true;
    }  // end of function button 1
#endif
#ifdef EUROSOUND
    if ((setdccadr == false) && (kpbtnstate == false)) {
      if (fc1btnstate == false) {
        fc1btnstate = true;
        fctbtnst = 1;
        if ((fading == true) && (vspeed > 0)) {
          fade = true;                // button used for fading
          sendfct(sfnum);             //send 'fade btn on' (F1xx)
          fdfctst = fctarray[sfnum];  // get fade function state
          if (fdfctst == 1) {
            faded = true;  // sound is faded
            display.setFont(ArialMT_Plain_16);
            display.setColor(BLACK);
            display.fillRect(70, 13, 10, 15);  //clear +
            display.setFont(ArialMT_Plain_10);
            display.setColor(WHITE);
            display.drawString(70, 18, "F");
            display.display();
            #ifdef LOG
            if (DEBUG == true) Serial.println("Sound is faded");
            #endif
          } else {
            faded = false;
            display.setFont(ArialMT_Plain_16);
            display.setColor(BLACK);
            display.fillRect(70, 13, 10, 15);
            if (setsdhadr == true) {
              display.setColor(WHITE);
              display.drawString(70, 13, "+");
            }
            display.display();
            #ifdef LOG
            if (DEBUG == true) Serial.println("Sound is not faded");
            #endif
          }
        }
        if ((faded == true) && (vspeed == 0)) {
          sendfct(sfnum);             //send 'fade btn on' (F1xx)
          fdfctst = fctarray[sfnum];  // get fade function state
          if (fdfctst == 1) {
            faded = true;
            #ifdef LOG
            if (DEBUG == true) Serial.println("Sound faded turned on");
            #endif
          } else {
            faded = false;
            display.setFont(ArialMT_Plain_16);
            display.setColor(BLACK);
            display.fillRect(70, 13, 10, 15);
            if (setsdhadr == true) {
              display.setColor(WHITE);
              display.drawString(70, 13, "+");
            }
            display.display();
            #ifdef LOG
            if (DEBUG == true) Serial.println("Sound faded turned off");
            #endif
          }
        }
        if ((fdfctst == 0) && (fcgstate == 0)) {
          if ((fade == false) && (vspeed == 0)) {
            fnum = 1;
            sendfct(fnum);  //send F11
          }
        }
        if (fcgstate == 1) {
          fnum = 11;
          sendfct(fnum);
        }
        if (fcgstate == 2) {
          fnum = 21;
          sendfct(fnum);
        }
        //-----------------------------------------------------
        if ((t_fading == true) && (vspeed > 0)) {
          t_fade = true;        // button used for fading
          sendsdhfct(t_sfnum);  //send 'fade btn on' (F1xx)
          if (setsdhadr == true) sendsdhfct(t_sfnum);
          t_fdfctst = t_fctarray[t_sfnum];  // get fade function state
          if (t_fdfctst == 1) {
            t_faded = true;
          } else {
            t_faded = false;
          }
        }
        if ((t_faded == true) && (vspeed == 0)) {
          sendsdhfct(t_sfnum);  //send 'trailer fade btn on' (F1xx)
          if (setsdhadr == true) sendsdhfct(t_sfnum);
          t_fdfctst = t_fctarray[t_sfnum];  // get fade function state
          if (t_fdfctst == 1) {
            t_faded = true;
          } else {
            t_faded = false;
          }
        }
        if (t_fdfctst == 0) {
          if (t_fade == false) {
            fnum = 1;
            if (setsdhadr == true) sendsdhfct(t_sfnum);
          }
        }
      }
      kpbtnstate = true;
    }  //end of function button 1
#endif
  }
  // Button 2: ***************************************
  else if ((btnread >= keypadarray[2]) && (btnread <= keypadarray[3])) {
    if (dsplon == false) {
      kpbtnstate = true;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    btnnbr = 2;
    if (kpbtnstate == false) dspbtnnum();
    // set loco or assessory decoder address:
    if ((setdccadr == true) || (accbtnstate == 1)) {
      if ((btncnt < 5) && (kpbtnstate == false)) {
        btncnt++;
        btnval = 2;
        if (setaccadr == false) putdgt();
        if (setaccadr == true) putaccdgt();
        kpbtnstate = true;
      }
    }
    // function button 2
    if ((setdccadr == false) && (kpbtnstate == false)) {
      if ((blkdfks[2] == 1) || (blkdfks[12] == 1) || (blkdfks[22] == 1)) return;
      if (fc2btnstate == false) {
        fc2btnstate = true;
        fctbtnst = 1;
        if (fcgstate == 0) {
          fnum = 2;
          sendfct(fnum);
        }
        if (fcgstate == 1) {
          fnum = 12;
          sendfct(fnum);
        }
        if (fcgstate == 2) {
          fnum = 22;
          sendfct(fnum);
        }
      }
      kpbtnstate = true;
    }  // end of function button 2
  }

  // Button 3: ***************************************
  else if ((btnread >= keypadarray[4]) && (btnread <= keypadarray[5])) {
    if (dsplon == false) {
      kpbtnstate = true;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    btnnbr = 3;
    if (kpbtnstate == false) dspbtnnum();
    // set loco or assessory decoder address:
    if ((setdccadr == true) || (accbtnstate == 1)) {
      if ((btncnt < 5) && (kpbtnstate == false)) {
        btncnt++;
        btnval = 3;
        if (setaccadr == false) putdgt();
        if (setaccadr == true) putaccdgt();
        kpbtnstate = true;
      }
    }
    // function button 3
    if ((setdccadr == false) && (kpbtnstate == false)) {
      if ((blkdfks[3] == 1) || (blkdfks[13] == 1) || (blkdfks[23] == 1)) return;
      if (fc3btnstate == false) {
        fc3btnstate = true;
        fctbtnst = 1;
        if (fcgstate == 0) {
          fnum = 3;
          sendfct(fnum);
        }
        if (fcgstate == 1) {
          fnum = 13;
          sendfct(fnum);
        }
        if (fcgstate == 2) {
          fnum = 23;
          sendfct(fnum);
        }
      }
      kpbtnstate = true;
    }  // end of function button 3
  }
  // Button 4: ***************************************
  else if ((btnread >= keypadarray[6]) && (btnread <= keypadarray[7])) {
    if (dsplon == false) {
      kpbtnstate = true;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    btnnbr = 4;
    if (kpbtnstate == false) dspbtnnum();
    if ((setdccadr == true) || (accbtnstate == 1)) {
      if ((btncnt < 5) && (kpbtnstate == false)) {
        btncnt++;
        btnval = 4;
        if (setaccadr == false) putdgt();
        if (setaccadr == true) putaccdgt();
        kpbtnstate = true;
      }
    }
    // function button 4
    if ((setdccadr == false) && (kpbtnstate == false)) {
      if ((blkdfks[4] == 1) || (blkdfks[14] == 1) || (blkdfks[24] == 1)) return;
      if (fc4btnstate == false) {
        fc4btnstate = true;
        fctbtnst = 1;
        if (fcgstate == 0) {
          fnum = 4;
          sendfct(fnum);
        }
        if (fcgstate == 1) {
          fnum = 14;
          sendfct(fnum);
        }
        if (fcgstate == 2) {
          fnum = 24;
          sendfct(fnum);
        }
      }
      kpbtnstate = true;
    }  // end of function button 4
  }
  // Button 5: ***************************************
  else if ((btnread >= keypadarray[8]) && (btnread <= keypadarray[9])) {
    if (dsplon == false) {
      kpbtnstate = true;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    btnnbr = 5;
    if (kpbtnstate == false) dspbtnnum();
    if ((setdccadr == true) || (accbtnstate == 1)) {
      if ((btncnt < 5) && (kpbtnstate == false)) {
        btncnt++;
        btnval = 5;
        if (setaccadr == false) putdgt();
        if (setaccadr == true) putaccdgt();
        kpbtnstate = true;
      }
    }
    // function button 5
    if ((setdccadr == false) && (kpbtnstate == false)) {
      if ((blkdfks[5] == 1) || (blkdfks[15] == 1) || (blkdfks[25] == 1)) return;
      if (fc5btnstate == false) {
        fc5btnstate = true;
        fctbtnst = 1;
        if (fcgstate == 0) {
          fnum = 5;
          sendfct(fnum);
        }
        if (fcgstate == 1) {
          fnum = 15;
          sendfct(fnum);
        }
        if (fcgstate == 2) {
          fnum = 25;
          sendfct(fnum);
        }
      }
      kpbtnstate = true;
    }  // end of function button 5
  }
  // Button 6: ***************************************
  else if ((btnread >= keypadarray[10]) && (btnread <= keypadarray[11])) {
    if (dsplon == false) {
      kpbtnstate = true;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    btnnbr = 6;
    if (kpbtnstate == false) dspbtnnum();
    if ((setdccadr == true) || (accbtnstate == 1)) {
      if ((btncnt < 5) && (kpbtnstate == false)) {
        btncnt++;
        btnval = 6;
        if (setaccadr == false) putdgt();
        if (setaccadr == true) putaccdgt();
        kpbtnstate = true;
      }
    }
    // function button 6
    if ((setdccadr == false) && (kpbtnstate == false)) {
      if ((blkdfks[6] == 1) || (blkdfks[16] == 1) || (blkdfks[26] == 1)) return;
      if (fc6btnstate == false) {
        fc6btnstate = true;
        fctbtnst = 1;
        if (fcgstate == 0) {
          fnum = 6;
          sendfct(fnum);
        }
        if (fcgstate == 1) {
          fnum = 16;
          sendfct(fnum);
        }
        if (fcgstate == 2) {
          fnum = 26;
          sendfct(fnum);
        }
      }
      kpbtnstate = true;
    }  // end of function button 6
  }
  // Button 7: ***************************************
  else if ((btnread >= keypadarray[12]) && (btnread <= keypadarray[13])) {
    if (dsplon == false) {
      kpbtnstate = true;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    btnnbr = 7;
    if (kpbtnstate == false) dspbtnnum();
    if ((setdccadr == true) || (accbtnstate == 1)) {
      if ((btncnt < 5) && (kpbtnstate == false)) {
        btncnt++;
        btnval = 7;
        if (setaccadr == false) putdgt();
        if (setaccadr == true) putaccdgt();
        kpbtnstate = true;
      }
    }
    // function button 7
    if ((setdccadr == false) && (kpbtnstate == false)) {
      if ((blkdfks[7] == 1) || (blkdfks[17] == 1) || (blkdfks[27] == 1)) return;
      if (fc7btnstate == false) {
        fc7btnstate = true;
        fctbtnst = 1;
        if (fcgstate == 0) {
          fnum = 7;
          sendfct(fnum);
        }
        if (fcgstate == 1) {
          fnum = 17;
          sendfct(fnum);
        }
        if (fcgstate == 2) {
          fnum = 27;
          sendfct(fnum);
        }
      }
      kpbtnstate = true;
    }  // end of function button 7
  }
  // Button 8: ***************************************
  else if ((btnread >= keypadarray[14]) && (btnread <= keypadarray[15])) {
    if (dsplon == false) {
      kpbtnstate = true;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    btnnbr = 8;
    if (kpbtnstate == false) dspbtnnum();
    if ((setdccadr == true) || (accbtnstate == 1)) {
      if ((btncnt < 5) && (kpbtnstate == false)) {
        btncnt++;
        btnval = 8;
        if (setaccadr == false) putdgt();
        if (setaccadr == true) putaccdgt();
        kpbtnstate = true;
      }
    }
    // function button 8
#ifdef EUROSOUND
    if ((setdccadr == false) && (kpbtnstate == false)) {
      if ((blkdfks[8] == 1) || (blkdfks[18] == 1) || (blkdfks[28] == 1)) return;
      if (fc8btnstate == false) {
        fc8btnstate = true;
        fctbtnst = 1;
        if (fcgstate == 0) {
          fnum = 8;
          sendfct(fnum);
        }
        if (fcgstate == 1) {
          fnum = 18;
          sendfct(fnum);
        }
        if (fcgstate == 2) {
          fnum = 28;
          sendfct(fnum);
        }
      }
      kpbtnstate = true;
    }  // end of function button 8
#endif
#ifndef EUROSOUND
    if ((setdccadr == false) && (kpbtnstate == false)) {
      if (fc8btnstate == false) {
        fc8btnstate = true;
        fctbtnst = 1;
        if ((fading == true) && (vspeed > 0)) {
          fade = true;                // button used for fading
          sendfct(sfnum);             //send 'fade btn on' (F8xx)
          fdfctst = fctarray[sfnum];  // get fade function state
          if (fdfctst == 1) {
            faded = true;  // sound is faded
            display.setFont(ArialMT_Plain_16);
            display.setColor(BLACK);
            display.fillRect(70, 13, 10, 15);  //clear +
            display.setFont(ArialMT_Plain_10);
            display.setColor(WHITE);
            display.drawString(70, 18, "F");
            display.display();
            #ifdef LOG
            if (DEBUG == true) Serial.println("Sound is faded");
            #endif
          } else {
            faded = false;
            display.setFont(ArialMT_Plain_16);
            display.setColor(BLACK);
            display.fillRect(70, 13, 10, 15);
            if (setsdhadr == true) {
              display.setColor(WHITE);
              display.drawString(70, 13, "+");
            }
            display.display();
            #ifdef LOG
            if (DEBUG == true) Serial.println("Sound is not faded");
            #endif
          }
        }
        if ((faded == true) && (vspeed == 0)) {
          sendfct(sfnum);             //send 'fade btn on' (F8xx)
          fdfctst = fctarray[sfnum];  // get fade function state
          if (fdfctst == 1) {
            faded = true;
            #ifdef LOG
            if (DEBUG == true) Serial.println("Sound faded turned on");
            #endif
          } else {
            faded = false;
            display.setFont(ArialMT_Plain_16);
            display.setColor(BLACK);
            display.fillRect(70, 13, 10, 15);
            if (setsdhadr == true) {
              display.setColor(WHITE);
              display.drawString(70, 13, "+");
            }
            display.display();
            #ifdef LOG
            if (DEBUG == true) Serial.println("Sound faded turned off");
            #endif
          }
        }
        if ((fdfctst == 0) && (fcgstate == 0)) {
          if ((fade == false) && (vspeed == 0)) {
            fnum = 8;
            sendfct(fnum);  //send F18
          }
        }
        if (fcgstate == 1) {
          fnum = 18;
          sendfct(fnum);
        }
        if (fcgstate == 2) {
          fnum = 28;
          sendfct(fnum);
        }
        //-----------------------------------------------------
        if ((t_fading == true) && (vspeed > 0)) {
          t_fade = true;        // button used for fading
          sendsdhfct(t_sfnum);  //send 'fade btn on' (F8xx)
          if (setsdhadr == true) sendsdhfct(t_sfnum);
          t_fdfctst = t_fctarray[t_sfnum];  // get fade function state
          if (t_fdfctst == 1) {
            t_faded = true;
          } else {
            t_faded = false;
          }
        }
        if ((t_faded == true) && (vspeed == 0)) {
          sendsdhfct(t_sfnum);  //send 'trailer fade btn on' (F8xx)
          if (setsdhadr == true) sendsdhfct(t_sfnum);
          t_fdfctst = t_fctarray[t_sfnum];  // get fade function state
          if (t_fdfctst == 1) {
            t_faded = true;
          } else {
            t_faded = false;
          }
        }
        if (t_fdfctst == 0) {
          if (t_fade == false) {
            fnum = 1;
            if (setsdhadr == true) sendsdhfct(t_sfnum);
          }
        }
      }
      kpbtnstate = true;
    }  //end of eurosound
#endif
    if ((setdccadr == false) && (kpbtnstate == false)) {
      if ((blkdfks[8] == 1) || (blkdfks[18] == 1) || (blkdfks[28] == 1)) return;
      if (fc8btnstate == false) {
        fc8btnstate = true;
        fctbtnst = 1;
        if (fcgstate == 0) {
          fnum = 8;
          sendfct(fnum);
        }
        if (fcgstate == 1) {
          fnum = 18;
          sendfct(fnum);
        }
        if (fcgstate == 2) {
          fnum = 28;
          sendfct(fnum);
        }
      }
      kpbtnstate = true;
    }  // end of function button 8
  }
  // Button 9: ***************************************
  else if ((btnread >= keypadarray[16]) && (btnread <= keypadarray[17])) {
    if (dsplon == false) {
      kpbtnstate = true;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    btnnbr = 9;
    if (kpbtnstate == false) dspbtnnum();
    if ((setdccadr == true) || (accbtnstate == 1)) {
      if ((btncnt < 5) && (kpbtnstate == false)) {
        btncnt++;
        btnval = 9;
        if (setaccadr == false) putdgt();
        if (setaccadr == true) putaccdgt();
        kpbtnstate = true;
      }
    }
    // function button 9
    if ((setdccadr == false) && (kpbtnstate == false)) {
      if ((blkdfks[9] == 1) || (blkdfks[19] == 1)) return;
      if (fc9btnstate == false) {
        fc9btnstate = true;
        fctbtnst = 1;
        if (fcgstate == 0) {
          fnum = 9;
          sendfct(fnum);
        }
        if (fcgstate == 1) {
          fnum = 19;
          sendfct(fnum);
        }
      }
      kpbtnstate = true;
    }  // end of function button 9
  }
  // Button 10 (function group/Acc dec close): *******
  else if ((btnread >= keypadarray[18]) && (btnread <= keypadarray[19])) {
    if (dsplon == false) {
      kpbtnstate = true;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    btnnbr = 10;
    if (kpbtnstate == false) dspbtnnum();
    // send "close" to accessory decoder 12/13>------------------------
    if ((kpbtnstate == false) && (accbtnstate == 2) && (setaccadr == true)) {
      kpbtnstate = true;
      setacc = "C";
      display.setFont(ArialMT_Plain_16);
      display.setColor(BLACK);
      display.fillRect(0, 30, 132, 18);
      display.setColor(WHITE);
      display.drawString(0, 30, tusrname + String("-/cls"));
      display.display();
      sendaccadr();
    }
    // toggle function group: -------------------------------------------
    if ((kpbtnstate == false) && (fctbtnstate == false) && (setaccadr == false)) {
      fctbtnstate = true;
      kpbtnstate = true;
    }
  }
  // Button 11 (zero/acc dec toggle): ****************
  else if ((btnread >= keypadarray[20]) && (btnread <= keypadarray[21])) {
    if (dsplon == false) {
      kpbtnstate = true;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    btnnbr = 11;
    if (kpbtnstate == false) dspbtnnum();
    // send "toggle" to accessory decoder >------------------------
    if ((kpbtnstate == false) && (accbtnstate == 2) && (setaccadr == true)) {
      kpbtnstate = true;
      setacc = "2";
      display.setFont(ArialMT_Plain_16);
      display.setColor(BLACK);
      display.fillRect(0, 30, 132, 18);
      display.setColor(WHITE);
      display.drawString(0, 30, tusrname + String("tgl"));
      display.display();
      sendaccadr();
    }
    // enter digit for dccadr: ----------------------------------
    if ((setdccadr == true) || (accbtnstate == 1)) {
      if ((btncnt < 5) && (kpbtnstate == false)) {
        btncnt++;
        btnval = 0;
        if (setaccadr == false) putdgt();
        if (setaccadr == true) putaccdgt();
        kpbtnstate = true;
      }
    }
    // function 0 button ------------------------------------------
    if ((setdccadr == false) && (setaccadr == false) && (kpbtnstate == false)) {
      if ((blkdfks[0] == 1) || (blkdfks[10] == 1) || (blkdfks[20] == 1)) return;
      if (fc0btnstate == false) {
        fc0btnstate = true;
        fctbtnst = 1;
        if (fcgstate == 0) {
          fnum = 0;
          sendfct(fnum);
        }
        if (fcgstate == 1) {
          fnum = 10;
          sendfct(fnum);
        }
        if (fcgstate == 2) {
          fnum = 20;
          sendfct(fnum);
        }
      }
      kpbtnstate = true;
    }  // end of function 0 button
  }
  // Button 12 (shunting/switch direction of consist/acc dec throw): *****
  else if ((btnread >= keypadarray[22]) && (btnread < keypadarray[23])) {
    if (dsplon == false) {
      kpbtnstate = true;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    btnnbr = 12;
    if (kpbtnstate == false) dspbtnnum();
    // send "thrown" to accessory decoder >------------------------
    if ((kpbtnstate == false) && (accbtnstate == 2) && (setaccadr == true)) {
      kpbtnstate = true;
      setacc = "T";
      display.setFont(ArialMT_Plain_16);
      display.setColor(BLACK);
      display.fillRect(0, 30, 132, 18);
      display.setColor(WHITE);
      display.drawString(0, 30, tusrname + String("+/thw"));
      display.display();
      sendaccadr();
    }
    // mapped function:
    if ((kpbtnstate == false) && (setsdhadr == false) && (setaccadr == false)) {
      kpbtnstate = true;
      fnum = fctswtmap[6];  // swtmap element 6 points to function "Shunting"
      fctbtnst = 1;
      #ifdef LOG
      if (DEBUG == true) Serial.println("Button 12 pushed for mapped function.");
      #endif
      sendfct(fnum);
      delay(mdelay);
      fctbtnst = 0;
      #ifdef LOG
      if (DEBUG == true) Serial.println("Button 12 released for mapped function.");
      #endif
      sendfct(fnum);
    }
    // change direction of consist:
    if ((kpbtnstate == false) && (setsdhadr == true) && (setaccadr == false)) {
      kpbtnstate = true;
      sdhdct = !sdhdct;
      #ifdef LOG
      if (DEBUG == true) {
        if (sdhdct == true) Serial.println("sdhdct = true");
        if (sdhdct == false) Serial.println("sdhdct = false");
      }
      #endif
      chgsdhdct();
    }
  }
  // Button 13 (enter address): **********************
  else if ((btnread >= keypadarray[24]) && (btnread <= keypadarray[25])) {
    if (dsplon == false) {
      kpbtnstate = true;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    btnnbr = 13;
    if (kpbtnstate == false) {
      dspbtnnum();
      //cancel route selection>>
      if ((sltroute == true) || (setaccadr == true) || (setfctbn == true)) {
        setfctbn = false;
        sltroute = false;
        setroute = false;
        setaccadr = false;
        accbtnstate = 0;
        clrrosnam();
        dsprosnam();
        dsprosadr();
        display.setFont(ArialMT_Plain_10);
        display.setColor(BLACK);
        display.fillRect(50, 18, 17, 10);  //remove display of cntr.
        display.display();
      }
      adrbtnstate = !adrbtnstate;
      if ((adrbtnstate == true) && (vspeed != 0)) adrbtnstate = !adrbtnstate;
      if ((adrbtnstate == true) && (vspeed == 0)) {
        stkx = 0; //always start stack at 0
        clrchgadr();
        btncnt = 0;
        setdccadr = true;
        #ifdef LOG
        if (DEBUG == true) {
          Serial.print("lastdccadr ");
          Serial.println(lastdccadr);
          Serial.print("dccadr ");
          Serial.println(dccadr);
        }
        #endif
        if (setsdhadr == false) releaseadr(dccadr);
        lastdccadr = dccadr;
        display.setColor(WHITE);
        display.drawString(80, 13, String(dspadr));
        display.drawString(120, 13, "?");
        display.display();
        #ifdef LOG
        if (DEBUG == true) Serial.println("Button 13: " + String(dspadr));
        #endif
      } else if (adrbtnstate == false) {
        actdccadr();
      }
      kpbtnstate = true;
    }
  }
  // Button 14 (accessory address): ******************
  else if ((btnread >= keypadarray[26]) && (btnread <= keypadarray[27])) {
    if (dsplon == false) {
      kpbtnstate = true;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    btnnbr = 14;
    // accessory decoder
    if (kpbtnstate == false) {
      dspbtnnum();
      kpbtnstate = true;
      //if ((sltroute == false) && (setdccadr == false)) {
      if (setdccadr == false) {
        accbtnstate++;  // 1 = address mode, 2 = action, 0 = off
        if (accbtnstate == 4) accbtnstate = 1;
        #ifdef LOG
        if (DEBUG == true) Serial.print("accbtnstate: " + String(accbtnstate));
        #endif
        if (accbtnstate == 1) {  // set accessory address mode
          btncnt = 0;
          accadr = 0;
          lastvcnt = vcnt;
          setaccadr = true;
          display.setFont(ArialMT_Plain_16);
          display.setColor(BLACK);
          display.fillRect(0, 30, 132, 18);  // remove locoadr and loconame
          display.setColor(WHITE);
          display.drawString(0, 30, "T");
          sprintf(dspaccadr, "%04d", accadr);
          display.drawString(85, 30, String(dspaccadr));
          display.drawString(120, 30, "?");
          display.display();
          #ifdef LOG
          if (DEBUG == true) {
            Serial.println("button state " + String(accbtnstate) + ": Set accessory address via keypad");
            Serial.println("or turn knob to select turnout name.");
            Serial.println("then press turnout button.");
          }
          #endif
        }
        if (accbtnstate == 2) {
          if (accadr != 0) {  // ready to enter "C","2","T" via kp-buttons 10,11,12.
            display.setFont(ArialMT_Plain_16);
            display.setColor(BLACK);
            display.fillRect(85, 30, 50, 15);
            display.setColor(WHITE);
            sprintf(dspaccadr, "%04d", accadr);
            display.drawString(85, 30, String(dspaccadr));
            display.display();
            #ifdef LOG
            if (DEBUG == true) {
              Serial.println("button state " + String(accbtnstate));
              Serial.print("accadr: ");
              Serial.println(accadr);
              Serial.print("dspaccadr: ");
              Serial.println(dspaccadr);
              Serial.println("Use buttons 10,11,12 for: close,toggle,throw.");
            }
            #endif
          }
          if (accadr == 0) {  // button pressed but nothing entered
            //dsprosnam();
            //dsprosadr();
            sltroute = true;
            setaccadr = false;
            display.setFont(ArialMT_Plain_16);
            display.setColor(BLACK);
            display.fillRect(50, 18, 17, 10);  //remove btn cnt
            display.fillRect(0, 30, 132, 18);
            display.setColor(WHITE);
            display.drawString(0, 30, "Select route?");
            display.display();
            #ifdef LOG
            if (DEBUG == true) Serial.println("Nothing entered. Select route?");
            #endif
          }
        }
        if (accbtnstate == 3) {
          clrrosnam();
          dsprosnam();
          dsprosadr();
          setaccadr = false;
          sltroute = false;
          vcnt = lastvcnt;
          display.setFont(ArialMT_Plain_10);
          display.setColor(BLACK);
          display.fillRect(50, 18, 17, 10);  //remove display of tcnt.
          display.setFont(ArialMT_Plain_16);
          display.fillRect(0, 50, 132, 15);
          display.display();
          dspfcts();
          #ifdef LOG
          if (DEBUG == true) Serial.println("button state 3: Set accessory address mode OFF");
          #endif
        }
      }
      if ((setroute == true) && (setaccadr == false)) {  // address was not selected with knob {  //------setroute------
        accbtnstate = 0;
        #ifdef LOG
        if (DEBUG == true) {
          Serial.print("Setting route: ");
          Serial.println(rsysname);
        }
        #endif
        sendroute();
        clrrosnam();
        dsprosnam();
        dsprosadr();
        setroute = false;
        setaccadr = false;
        vcnt = lastvcnt;
        display.setFont(ArialMT_Plain_10);
        display.setColor(BLACK);
        display.fillRect(50, 18, 17, 10);  //remove display of rtcnt.
        display.display();
      }
    }
  }
  // Button 15 (set/release consist adr/retrieve stack): ****************
  else if ((btnread >= keypadarray[28]) && (btnread <= keypadarray[29])) {
    if (dsplon == false) {
      kpbtnstate = true;
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    btnnbr = 15;
    if (kpbtnstate == false) dspbtnnum();
    // rls consist address: ----------------------------------------------
    if ((setsdhadr == true) && (setdccadr == false) && (kpbtnstate == false)) {
      kpbtnstate = true;
      if ((numthrottles == 1) && (sdhadr != 0)) {
        releaseadr(sdhadr);
        sdhadr = 0;
        display.setFont(ArialMT_Plain_16);
        display.setColor(BLACK);
        display.fillRect(70, 13, 10, 15);
        display.display();
        setsdhadr = false;
        t_fading = false;  //2.7.19
      }
      if (numthrottles > 1) {
        if (MThrottle == 1) {
          releaseadr(sdhadrT1);
          sdhadrT1 = 0;
        }
        if (MThrottle == 2) {
          releaseadr(sdhadrT2);
          sdhadrT2 = 0;
        }
        if (MThrottle == 3) {
          releaseadr(sdhadrT3);
          sdhadrT3 = 0;
        }
        if (MThrottle == 4) {
          releaseadr(sdhadrT4);
          sdhadrT4 = 0;
        }
      }
      display.setFont(ArialMT_Plain_16);
      display.setColor(BLACK);
      display.fillRect(70, 13, 10, 15);
      display.display();
      setsdhadr = false;
      t_fading = false;
    }
    // set consist address: -----------------------------------------------
    if ((setsdhadr == false) && (setdccadr == false) && (kpbtnstate == false)) {
      kpbtnstate = true;
      if (dccadr != 0) {
        setsdhadr = true;
        sdhadr = dccadr;
        // copy fctswtmap   to sdhswtmap  :
        for (int i = 0; i < numswfcs; i++) {
          sdhswtmap[i] = fctswtmap[i];
        }
        // 2.7.19 copy fctarray   to t_fctarray  :
        for (int i = 0; i < numfcts; i++) {
          t_fctarray[i] = fctarray[i];
        }
        t_sfnum = sfnum;
        #ifdef LOG
        if (DEBUG == true) {
          Serial.print("loconame " + loconame);
          Serial.print(" address " + String(sdhadr));
          Serial.println(" set as trailer.");
        }
        #endif
        compsdhadr();
        if (fading == true) t_fading = true;
        display.setFont(ArialMT_Plain_16);
        display.setColor(WHITE);
        display.drawString(70, 13, "+");
        display.display();
      }
    }
    // retrieve dcc address from stack array-------------------------
    if ((setdccadr == true) && (kpbtnstate == false)) {
      kpbtnstate = true;
      if (stkx > 5) stkx = 0;
      dspcnt(stkx);  // display counter
      dccadr = dccstck[stkx];
      stkx++;
      if ((stkx > 5) || (dccadr == 0)) stkx = 0;
      #ifdef LOG
      if (DEBUG == true) {
        Serial.print("stack array@");
        Serial.print(stkx);
        Serial.print(" reads: ");
        Serial.println(dccadr);
      }
      #endif
      rtvrosnam();
      if (ros_eAvail == true) {  //display with name
        clrrosnam();
        dsprosnam();
        dsprosadr();
      } else {  // display w/o name
        clrchgadr();
        dspchgadr();
      }
      if (dccadr > 0) {
        pshadr = true;
      }
    }
  }
  // Button 16 (throttle-off/restart): ****************
  else if ((btnread >= keypadarray[30]) && (btnread <= keypadarray[31])) {
    if (dsplon == false) {
      dspreset();
      noop = true;
      return;
    } else {
      cntd = cd;
    }
    btnnbr = 16;
    if (kpbtnstate == false) dspbtnnum();
    if ((kpbtnstate == false) && (throff == false) && (noop == false)) {
      kpbtnstate = true;
      dccadr = 0;
      sdhadr = 0;
      setsdhadr = false;
      clrrosnam();
      dspdccadr();
      // Start the 'button press timer'
      startTime = millis();
      if (numthrottles == 1) {
        client.print("M1-*<;>r");
        delay(fdelay);
        client.flush();
      }
      if (numthrottles > 1) {
        if (MT1active == true) {
          client.print("M1-*<;>r");
          delay(fdelay);
          client.flush();
          MT1active = false;
          dccadrT1 = 0;
          if (setsdhadrT1 == true) {
            setsdhadrT1 = false;
            sdhadrT1 = 0;
          }
        }
        if (MT2active == true) {
          client.print("M2-*<;>r");
          delay(fdelay);
          client.flush();
          MT2active = false;
          dccadrT2 = 0;
          if (setsdhadrT2 == true) {
            setsdhadrT2 = false;
            sdhadrT2 = 0;
          }
        }
        if (MT3active == true) {
          client.print("M3-*<;>r");
          delay(fdelay);
          client.flush();
          MT3active = false;
          dccadrT3 = 0;
          if (setsdhadrT3 == true) {
            setsdhadrT3 = false;
            sdhadrT3 = 0;
          }
        }
        if (MT4active == true) {
          client.print("M4-*<;>r");
          delay(fdelay);
          client.flush();
          MT4active = false;
          dccadrT4 = 0;
          if (setsdhadrT4 == true) {
            setsdhadrT4 = false;
            sdhadrT4 = 0;
          }
        }
        client.print("Q\r\n");
        delay(fdelay);
        client.flush();
      }
      display.setFont(ArialMT_Plain_16);
      display.setColor(BLACK);
      display.fillRect(90, 30, 132, 18);
      display.setColor(WHITE);
      display.drawString(95, 30, "OFF");
      display.drawString(120, 13, "?");
      display.display();
      svr = false;
      throff = true;
      #ifdef LOG
      if (DEBUG == true) Serial.println("Throttle disconnected");
      #endif
    }
    if ((kpbtnstate == false) && (throff == true) && (noop == false)) {
      kpbtnstate = true;
      throff = false;
      startTime = millis();  //restart button-press timer
      display.setColor(BLACK);
      display.fillRect(80, 30, 70, 18);
      display.display();
      reconnect();
    }
    // restart when button was pressed for 5 seconds
    if (kpbtnstate == true) {  // button pushed for 5 sec
      if ((millis() - startTime) >= 5000) {
        client.stop();
        delay(100);
        WiFi.disconnect();
        delay(100);
        #ifdef LOG
        if (DEBUG == true) Serial.println("restarting");
        #endif
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.setColor(WHITE);
        display.drawString(40, 0, "Restarting");
        display.display();
        ESP.restart();
      }
    }
  }
}  //end of kparrRead ('button pressed')
//*****************************************************************
void kpRelease() {
  // All buttons released - no button pushed:
  if (fc0btnstate == true) {
    fc0btnstate = false;
    fctbtnst = 0;
    if (fcgstate == 0) {
      fnum = 0;
    }
    if (fcgstate == 1) {
      fnum = 10;
    }
    if (fcgstate == 2) {
      fnum = 20;
    }
    sendfct(fnum);
  }
  if (fc1btnstate == true) {
    fc1btnstate = false;
    fctbtnst = 0;
    if (fcgstate == 0) {
      fnum = 1;
      if (fade == true) {
        sendfct(sfnum);  // send 'fade btn off'
      }
      if (t_fade == true) {
        sendfct(t_sfnum);  // send 'fade btn off' for trailer
      }
    }
    if (fcgstate == 1) {
      fnum = 11;
    }
    if (fcgstate == 2) {
      fnum = 21;
    }
    if ((fade == false) && (vspeed == 0)) sendfct(fnum);
    fdfctst = fctarray[sfnum];
    if (fdfctst == 0) {
      fade = false;
    }
    t_fdfctst = t_fctarray[t_sfnum];
    if (t_fdfctst == 1) {
      t_fade = false;
    }
  }
  if (fc2btnstate == true) {
    fc2btnstate = false;
    fctbtnst = 0;
    if (fcgstate == 0) {
      fnum = 2;
    }
    if (fcgstate == 1) {
      fnum = 12;
    }
    if (fcgstate == 2) {
      fnum = 22;
    }
    sendfct(fnum);
  }
  if (fc3btnstate == true) {
    fc3btnstate = false;
    fctbtnst = 0;
    if (fcgstate == 0) {
      fnum = 3;
    }
    if (fcgstate == 1) {
      fnum = 13;
    }
    if (fcgstate == 2) {
      fnum = 23;
    }
    sendfct(fnum);
  }
  if (fc4btnstate == true) {
    fc4btnstate = false;
    fctbtnst = 0;
    if (fcgstate == 0) {
      fnum = 4;
    }
    if (fcgstate == 1) {
      fnum = 14;
    }
    if (fcgstate == 2) {
      fnum = 24;
    }
    sendfct(fnum);
  }
  if (fc5btnstate == true) {
    fc5btnstate = false;
    fctbtnst = 0;
    if (fcgstate == 0) {
      fnum = 5;
    }
    if (fcgstate == 1) {
      fnum = 15;
    }
    if (fcgstate == 2) {
      fnum = 25;
    }
    sendfct(fnum);
  }
  if (fc6btnstate == true) {
    fc6btnstate = false;
    fctbtnst = 0;
    if (fcgstate == 0) {
      fnum = 6;
    }
    if (fcgstate == 1) {
      fnum = 16;
    }
    if (fcgstate == 2) {
      fnum = 26;
    }
    sendfct(fnum);
  }
  if (fc7btnstate == true) {
    fc7btnstate = false;
    fctbtnst = 0;
    if (fcgstate == 0) {
      fnum = 7;
    }
    if (fcgstate == 1) {
      fnum = 17;
    }
    if (fcgstate == 2) {
      fnum = 27;
    }
    sendfct(fnum);
  }
  if (fc8btnstate == true) {
    fc8btnstate = false;
    fctbtnst = 0;
    if (fcgstate == 0) {
      fnum = 8;
    }
    if (fcgstate == 1) {
      fnum = 18;
    }
    if (fcgstate == 2) {
      fnum = 28;
    }
    sendfct(fnum);
  }
  if (fc9btnstate == true) {
    fc9btnstate = false;
    fctbtnst = 0;
    if (fcgstate == 0) {
      fnum = 9;
    }
    if (fcgstate == 1) {
      fnum = 19;
    }
    sendfct(fnum);
  }
  if (fctbtnstate == true) {
    fcgstate++;
    if (fcgstate > 3) fcgstate = 0;
    if (fcgstate == 0) {
      if (setfctbn == true) {
        setfctbn = false;
        dsprosadr();
        dsprosnam();
        vcnt = lastvcnt;
        rcnt = savrcnt;
        rtcnt = savrtcnt;
        dspfcts();
      }
    }
    if ((fcgstate == 1) || (fcgstate == 2)) {
      dspfcts();
    }
    // select function:
    if (fcgstate == 3) {
      fcnt = 0;
      dspcnt(fcnt);
      display.setFont(ArialMT_Plain_10);
      display.setColor(BLACK);
      display.fillRect(0, 30, 132, 18);
      display.fillRect(0, 50, 132, 15);
      display.setColor(WHITE);
      display.drawString(0, 50, "F select");
      getfctbn(fcnt);
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 30, dspfctname);
      display.display();
      vcnt = lastvcnt;
      savrcnt = rcnt;
      savrtcnt = rtcnt;
      setfctbn = true;
    }
    fctbtnstate = false;
  }
  //--------------------------------------------------------
}  // end of kpRelease
//***********************************************************

//------------------------------------------
void dspcnt(byte x) {
  display.setFont(ArialMT_Plain_10);
  display.setColor(BLACK);
  display.fillRect(50, 18, 17, 10);
  display.setColor(WHITE);
  display.drawString(50, 18, String(x));
  display.display();
}
//----------------------------------------------------
void putdgt() {
  pshadr = true;
  clrrosnam();
  ros_eAvail = false;
  if (btncnt == 1) {
    clrchgadr();
    dccadr = btnval;
    dspchgadr();
  }
  if (btncnt == 2) {
    clrchgadr();
    dccadrtmp = btnval;
    dccadr = dccadr * 10 + dccadrtmp;
    dspchgadr();
  }
  if (btncnt == 3) {
    clrchgadr();
    dccadrtmp = btnval;
    dccadr = dccadr * 10 + dccadrtmp;
    dspchgadr();
  }
  if (btncnt == 4) {
    clrchgadr();
    dccadrtmp = btnval;
    dccadr = dccadr * 10 + dccadrtmp;
    dspchgadr();
  }
  if (btncnt == 5) {
    btncnt = 1;
    dccadr = btnval;
    clrchgadr();
    dspchgadr();
  }
}
//-----------------------------------------------------------
void putaccdgt() {
  //pshaccadr = true;
  if (btncnt == 1) {
    accadr = btnval;
    dspadcadr();
  }
  if (btncnt == 2) {
    dccadrtmp = btnval;
    accadr = accadr * 10 + dccadrtmp;
    dspadcadr();
  }
  if (btncnt == 3) {
    dccadrtmp = btnval;
    accadr = accadr * 10 + dccadrtmp;
    dspadcadr();
  }
  if (btncnt == 4) {
    dccadrtmp = btnval;
    accadr = accadr * 10 + dccadrtmp;
    dspadcadr();
  }
  if (btncnt == 5) {
    btncnt = 1;
    accadr = btnval;
    dspadcadr();
  }
}
//------------------------------------------------------
void dspreset() {
  cntd = cd;
  display.displayOn();
  dsplon = true;
}
//------------------------------------------------------
void dspbtnnum() {
  char dspbtnnbr[4];
  sprintf(dspbtnnbr, "%02d", btnnbr);
  display.setFont(ArialMT_Plain_10);
  display.setColor(BLACK);
  display.fillRect(100, 0, 28, 10);
  display.setColor(WHITE);
  display.drawString(110, 0, String(dspbtnnbr));
  display.display();
  #ifdef LOG
  if (DEBUG == true) Serial.println("Button " + String(dspbtnnbr) + " pressed");
  #endif
  cntb = 3;  // reset and display battery level after 3 sec.
}
//------------------------------------------------------
void setfcts(byte fnum) {
  //#ifdef LOG
  //if (DEBUG == true) {
  //  Serial.println("setfcts: num " + String(fnum));
  //  Serial.println("setfcts: state " + String(fctstate));
  //}
  //#endif
  fctarray[fnum] = fctstate;
}
//------------------------------------------------------
void dspfcts() {
  if (fcgstate != 3) {
    display.setFont(ArialMT_Plain_10);
    display.setColor(BLACK);
    display.fillRect(0, 50, 132, 15);
  }
  if (fcgstate == 0) {
    display.setColor(WHITE);
    display.drawString(0, 50, "Fg");
    display.drawString(15, 50, "0");
    display.display();
    for (int i = 0; i <= 9; i++) {  // 0-9 in fct grp 0
      if (fctarray[i] == 1) {
        display.drawString(dspfos[i], 50, dspfnum[i]);
        display.display();
      }
    }
  }
  if (fcgstate == 1) {
    display.setColor(WHITE);
    display.drawString(0, 50, "Fg");
    display.drawString(15, 50, "1");
    display.display();
    fn = -1;
    for (int i = 10; i <= 19; i++) {  // 0-9 in fct grp 1
      fn++;
      if (fctarray[i] == 1) {
        display.drawString(dspfos[fn], 50, dspfnum[fn]);
        display.display();
      }
    }
  }
  if (fcgstate == 2) {
    display.setColor(WHITE);
    display.drawString(0, 50, "Fg");
    display.drawString(15, 50, "2");
    display.display();
    fn = -1;
    for (int i = 20; i <= 28; i++) {  // 0-8 in fct grp 2
      fn++;
      if (fctarray[i] == 1) {
        display.drawString(dspfos[fn], 50, dspfnum[fn]);
        display.display();
      }
    }
  }
}
//------------------------------------------------------
void dspadcadr() {
  display.setFont(ArialMT_Plain_16);
  display.setColor(BLACK);
  display.fillRect(80, 30, 40, 15);
  display.setColor(WHITE);
  sprintf(dspaccadr, "%04d", accadr);
  display.drawString(85, 30, String(dspaccadr));
  display.drawString(120, 30, "?");
  display.display();
}
//------------------------------------------------------
void clrchgadr() {
  sprintf(dspadr, "%04d", 0);
  display.setFont(ArialMT_Plain_16);
  display.setColor(BLACK);
  display.fillRect(80, 13, 40, 15);
  display.display();
  #ifdef LOG
  if (DEBUG == true) Serial.println("clrchgadr: " + String(dspadr));
  #endif
}
//------------------------------------------------------
void clrrosnam() {
  display.setFont(ArialMT_Plain_16);
  display.setColor(BLACK);
  display.fillRect(0, 30, 132, 18);
  display.display();
}
//------------------------------------------------------
void dspchgadr() {
  display.setFont(ArialMT_Plain_16);
  display.setColor(WHITE);
  sprintf(dspadr, "%04d", dccadr);
  display.drawString(80, 13, String(dspadr));
  display.drawString(120, 13, "?");
  display.display();
  #ifdef LOG
  if (DEBUG == true) Serial.println("dspchgadr: " + String(dspadr));
  #endif
}
//------------------------------------------------------
void dsprosnam() {
  display.setFont(ArialMT_Plain_16);
  display.setColor(BLACK);
  display.fillRect(0, 30, 132, 18);
  display.setColor(WHITE);
  display.drawString(0, 30, loconame);
  display.display();
}
//------------------------------------------------------
void dsprosadr() {
  display.setFont(ArialMT_Plain_16);
  display.setColor(BLACK);
  //display.fillRect(92, 30, 50, 18);
  display.fillRect(80, 30, 50, 18);
  display.setColor(WHITE);
  display.drawString(92, 30, locoadr);
  display.display();
}
//------------------------------------------------------
void getrosnam(int ptr) {
  #ifdef LOG
  if (DEBUG == true) Serial.println("Get loco name from array element " + String(ptr));
  #endif
  int rs = rosnamoffsets[ptr];       // begin of loco name
  int re = roster.indexOf('}', rs);  // end of loco name
  loconame = roster.substring(rs, re);
  #ifdef LOG
  if (DEBUG == true) Serial.println(loconame);
  #endif
  ros_eAvail = true;
}
//--------------------------------------------------------------------------
void getrosadr(int ptr) {
  #ifdef LOG
  if (DEBUG == true) Serial.println("Get loco address from array element " + String(ptr));
  #endif
  int rs = rosadroffsets[ptr];       // begin of loco address
  int re = roster.indexOf('}', rs);  // end of loco address
  locoadr = roster.substring(rs, re);
  #ifdef LOG
  if (DEBUG == true) Serial.println(locoadr);
  #endif
}
//--------------------------------------------------------------------------
void getrosnamen() {
  for (int i = 0; i <= numos; i++) {
    rxn = roster.indexOf('[', rxn);  // first/next start of name
    rxn++;
    rxnn = rxn;
    rxnn = roster.indexOf('}', rxnn);  // first/next end of name
    if (roster.substring(rxn, rxnn) == loconame) {
      #ifdef LOG
      if (DEBUG == true) Serial.println("getrosnamen name offset: " + String(i));
      #endif
      break;
    }
  }
}
//-------------------------------------------------------------------------
void rtvrosnam() {
  ros_eAvail = false;
  #ifdef LOG
  if (DEBUG == true) Serial.println("retr. loco name for " + String(dccadr));
  #endif
  if ((dccadr > 0) && (dccadr <= 9999)) {
    for (byte i = 0; i < numre; i++) {
      getrosadr(i);
      int dccadrf = locoadr.toInt();
      if (dccadrf == dccadr) {
        getrosnam(i);
        break;
      } else {
        clrrosnam();
      }
    }
  }
}
//------------------------------------------------------
void getroute(int ptr) {
  #ifdef LOG
  if (DEBUG == true) Serial.println("Get route sysname from array element: " + String(ptr));
  #endif
  int rss = rsosa[ptr];
  int rse = routes.indexOf('}', rss);
  rsysname = routes.substring(rss, rse);
  #ifdef LOG
  if (DEBUG == true) {
    Serial.println(rsysname);
    Serial.println("Get route usrname from array element " + String(ptr));
  }
  #endif
  int rus = ruosa[ptr];
  int rue = routes.indexOf('}', rus);
  rusrname = routes.substring(rus, rue);
  #ifdef LOG
  if (DEBUG == true) Serial.println(rusrname);
  #endif
}
//-----turnouts---------------------------------------
void gettoname(int ptr) {
  #ifdef LOG
  if (DEBUG == true) {
    Serial.println("gettoname:");
    Serial.print("Get turnout sysname using array element: " + String(ptr) + ": ");
  }
  #endif
  int rss = tsosa[ptr];
  int rse = turnouts.indexOf('}', rss);
  tsysname = turnouts.substring(rss, rse);
  #ifdef LOG
  if (DEBUG == true) {
    Serial.println(tsysname);
    Serial.print("Get turnout usrname using array element " + String(ptr) + ": ");
  }
  #endif
  accadr = tsysname.toInt();
  int rus = tuosa[ptr];
  int rue = turnouts.indexOf('}', rus);
  tusrname = turnouts.substring(rus, rue);
  #ifdef LOG
  if (DEBUG == true) {
    Serial.println(tusrname);
    Serial.print("Get turnout state using array element " + String(ptr) + ": ");
  }
  #endif
}
//------------------------------------------------------
void getfctbn(byte ptr) {  // get function by name, ptr is fcnt
  //find function name in array "functions" and display
  #ifdef LOG
  if (DEBUG == true) Serial.println("Get function name using array element: " + String(ptr));
  #endif
  byte fs = fctnos[ptr];
  byte fe = fctnoe[ptr];
  fnum = fctnox[ptr];
  dspfctname = functions.substring(fs, fe);
  dspsfnum(fnum);
  #ifdef LOG
  if (DEBUG == true) {
    Serial.print(dspfctname);
    Serial.print(" fct nbr: ");
    Serial.println(fnum);
  }
  #endif
}
//--------------------------------------------------------
void dspsfnum(byte x) {
  display.setFont(ArialMT_Plain_10);
  display.setColor(BLACK);
  display.fillRect(60, 50, 72, 15);
  display.setColor(WHITE);
  display.drawString(60, 50, String(x));
  if (fctarray[x] == 1) display.drawString(90, 50, "ON");
  if (fctarray[x] == 0) display.drawString(90, 50, "OFF");
  display.display();
}
//--------------------------------------------------------
void set_speed() {
  lastvcnt = vcnt;
  dspeed = vcnt;
  if ((numspst == 28) && (numspst != 14)) {
    vtemp = vcnt * 4.5;
  } else {
    vtemp = vcnt;
  }
  if (prvspeed != vtemp) {
    prvspeed = vtemp;
    vspeed = vtemp;
    if (MThrottle == 1) xmits = locoactsT1 + "V" + String(vspeed);
    if (MThrottle == 2) xmits = locoactsT2 + "V" + String(vspeed);
    if (MThrottle == 3) xmits = locoactsT3 + "V" + String(vspeed);
    if (MThrottle == 4) xmits = locoactsT4 + "V" + String(vspeed);
    if (svr == true) {
      client.print(xmits + "\r\n");
      client.flush();
    }
    #ifdef LOG
    if (DEBUG == true) {
      if (setsdhadr == true) Serial.print("set_speed leader: ");
      else Serial.print("set_speed: ");
      Serial.println(xmits);
    }
    #endif
    if ((setsdhadr == true) && (dccadr != sdhadr)) {
      if (MThrottle == 1) xmits = sdhactsT1 + "V" + String(vspeed);
      if (MThrottle == 2) xmits = sdhactsT2 + "V" + String(vspeed);
      if (MThrottle == 3) xmits = sdhactsT3 + "V" + String(vspeed);
      if (MThrottle == 4) xmits = sdhactsT4 + "V" + String(vspeed);
      xmits = sdhacts + "V" + String(vspeed);
      if (svr == true) {
        client.print(xmits + "\r\n");
        client.flush();
      }
      #ifdef LOG
      if (DEBUG == true) {
        Serial.print("set_speed trailer: ");
        Serial.println(xmits);
      }
      #endif
    }
  }  //2.7.
  dsp_speed();
}
//-------------------------------------------------------
void dsp_speed() {
  display.setFont(ArialMT_Plain_16);
  display.setColor(BLACK);
  display.fillRect(15, 13, 27, 15);
  display.setColor(WHITE);
  display.drawString(15, 13, String(dspeed));
  display.display();
}
//------------------------------------------------------
void dsp_dctst() {
  display.setFont(ArialMT_Plain_16);
  display.setColor(BLACK);
  display.fillRect(0, 13, 10, 20);
  display.setColor(WHITE);
  if (dctstate == true) display.drawString(0, 13, "F");
  if (dctstate == false) display.drawString(0, 13, "R");
  display.display();
}
//---------------------------------------------------------
void dspdccadr() {
  sprintf(dspadr, "%04d", dccadr);
  display.setFont(ArialMT_Plain_16);
  display.setColor(BLACK);
  display.fillRect(70, 13, 62, 15);  //"+dccadr?"
  display.setColor(WHITE);
  display.drawString(80, 13, String(dspadr));
  if (setsdhadr == true) {
    display.setColor(WHITE);
    display.drawString(70, 13, "+");
  }
  display.display();
}
//------------------------------------------------------
void savethr1() {
  dccadrT1 = dccadr;
  lastdccadrT1 = lastdccadr;
  loconameT1 = loconame;
  vspeedT1 = vspeed;
  vcntT1 = vcnt;
  prvspeedT1 = prvspeed;
  dctstateT1 = dctstate;
  sdhadrT1 = sdhadr;
  sdhdctT1 = sdhdct;
  sdhactsT1 = sdhacts;
  setsdhadrT1 = setsdhadr;
  fcgstateT1 = fcgstate;
  functionsT1 = functions;
  numfcntT1 = numfcnt;
  for (byte i = 0; i < 6; i++) {
    dspadrT1[i] = dspadr[i];
  }
  for (int i = 0; i < numfcts; i++) {
    fctarrayT1[i] = fctarray[i];
    t_fctarrayT1[i] = t_fctarray[i];
    blkdfksT1[i] = blkdfks[i];
    fctnosT1[i] = fctnos[i];
    fctnoeT1[i] = fctnoe[i];
    fctnoxT1[i] = fctnox[i];
  }
  for (int i = 0; i < numswfcs; i++) {
    fctswtmapT1[i] = fctswtmap[i];
    sdhswtmapT1[i] = sdhswtmap[i];
  }
}
void savethr2() {
  dccadrT2 = dccadr;
  lastdccadrT2 = lastdccadr;
  loconameT2 = loconame;
  vspeedT2 = vspeed;
  vcntT2 = vcnt;
  prvspeedT2 = prvspeed;
  dctstateT2 = dctstate;
  sdhadrT2 = sdhadr;
  sdhdctT2 = sdhdct;
  sdhactsT2 = sdhacts;
  setsdhadrT2 = setsdhadr;
  fcgstateT2 = fcgstate;
  functionsT2 = functions;
  numfcntT2 = numfcnt;
  for (byte i = 0; i < 6; i++) {
    dspadrT2[i] = dspadr[i];
  }
  for (int i = 0; i < numfcts; i++) {
    fctarrayT2[i] = fctarray[i];
    t_fctarrayT2[i] = t_fctarray[i];
    blkdfksT2[i] = blkdfks[i];
    fctnosT2[i] = fctnos[i];
    fctnoeT2[i] = fctnoe[i];
    fctnoxT2[i] = fctnox[i];
  }
  for (int i = 0; i < numswfcs; i++) {
    fctswtmapT2[i] = fctswtmap[i];
    sdhswtmapT2[i] = sdhswtmap[i];
  }
}
void savethr3() {
  dccadrT3 = dccadr;
  lastdccadrT3 = lastdccadr;
  loconameT3 = loconame;
  vspeedT3 = vspeed;
  vcntT3 = vcnt;
  prvspeedT3 = prvspeed;
  dctstateT3 = dctstate;
  sdhadrT3 = sdhadr;
  sdhdctT3 = sdhdct;
  sdhactsT3 = sdhacts;
  setsdhadrT3 = setsdhadr;
  fcgstateT3 = fcgstate;
  functionsT3 = functions;
  numfcntT3 = numfcnt;
  for (byte i = 0; i < 6; i++) {
    dspadrT3[i] = dspadr[i];
  }
  for (int i = 0; i < numfcts; i++) {
    fctarrayT3[i] = fctarray[i];
    t_fctarrayT3[i] = t_fctarray[i];
    blkdfksT3[i] = blkdfks[i];
    fctnosT3[i] = fctnos[i];
    fctnoeT3[i] = fctnoe[i];
    fctnoxT3[i] = fctnox[i];
  }
  for (int i = 0; i < numswfcs; i++) {
    fctswtmapT3[i] = fctswtmap[i];
    sdhswtmapT3[i] = sdhswtmap[i];
  }
}
void savethr4() {
  dccadrT4 = dccadr;
  lastdccadrT4 = lastdccadr;
  loconameT4 = loconame;
  vspeedT4 = vspeed;
  vcntT4 = vcnt;
  prvspeedT4 = prvspeed;
  dctstateT4 = dctstate;
  sdhadrT4 = sdhadr;
  sdhdctT4 = sdhdct;
  sdhactsT4 = sdhacts;
  setsdhadrT4 = setsdhadr;
  fcgstateT4 = fcgstate;
  functionsT4 = functions;
  numfcntT4 = numfcnt;
  for (byte i = 0; i < 6; i++) {
    dspadrT4[i] = dspadr[i];
  }
  for (int i = 0; i < numfcts; i++) {
    fctarrayT4[i] = fctarray[i];
    t_fctarrayT4[i] = t_fctarray[i];
    blkdfksT4[i] = blkdfks[i];
    fctnosT4[i] = fctnos[i];
    fctnoeT4[i] = fctnoe[i];
    fctnoxT4[i] = fctnox[i];
  }
  for (int i = 0; i < numswfcs; i++) {
    fctswtmapT4[i] = fctswtmap[i];
    sdhswtmapT4[i] = sdhswtmap[i];
  }
}
//------------------------------------------------------
void sendsdhfct(byte fnum) {
  if (fnum != 99) {
    if (fctbtnst == 1) {
      xmits = sdhacts + "F1" + String(fnum);  // function button on
    }
    if (fctbtnst == 0) {
      xmits = sdhacts + "F0" + String(fnum);  // function button off
    }
    #ifdef LOG
    if (DEBUG == true) {
      Serial.print("sendsdhfct: ");
      Serial.println(xmits);
    }
    #endif
    if (svr == true) {
      client.print(xmits + "\r\n");
      client.flush();
    }
  }
}
//------------------------------------------------------
void sendfct(byte fnum) {
  if (fnum != 99) {
    if (fctbtnst == 1) {
      if (MThrottle == 1) {
        xmits = locoactsT1 + "F1" + String(fnum);
        lkf = lkfT1;
      }
      if (MThrottle == 2) {
        xmits = locoactsT2 + "F1" + String(fnum);
        lkf = lkfT2;
      }
      if (MThrottle == 3) {
        xmits = locoactsT3 + "F1" + String(fnum);
        lkf = lkfT3;
      }
      if (MThrottle == 4) {
        xmits = locoactsT4 + "F1" + String(fnum);
        lkf = lkfT4;
      }
    }
    if (fctbtnst == 0) {
      if (MThrottle == 1) {
        xmits = locoactsT1 + "F0" + String(fnum);
        lkf = lkfT1;
      }
      if (MThrottle == 2) {
        xmits = locoactsT2 + "F0" + String(fnum);
        lkf = lkfT2;
      }
      if (MThrottle == 3) {
        xmits = locoactsT3 + "F0" + String(fnum);
        lkf = lkfT3;
      }
      if (MThrottle == 4) {
        xmits = locoactsT4 + "F0" + String(fnum);
        lkf = lkfT4;
      }
    }
    #ifdef LOG
    if (DEBUG == true) {
      Serial.print("sendfct: ");
      Serial.println(xmits);
    }
    #endif
    if (svr == true) {
      client.print(xmits + "\r\n");
      client.flush();
      delay(sdelay);
      while (client.available()) {
        jmri_res = client.readStringUntil('\n');
        if (jmri_res.startsWith(lkf)) {
          #ifdef LOG
          if (DEBUG == true) {
            Serial.print("JMRI reply to sendfct: ");
            Serial.println(jmri_res);
          }
          #endif
          int ix = jmri_res.indexOf('F');
          ix++;
          jmri_sub = jmri_res.substring(ix, ix + 3);
          String fstate = jmri_sub.substring(0, 1);
          fctstate = fstate.toInt();
          String fctnum = jmri_sub.substring(1, 3);
          fnum = fctnum.toInt();
          if (fnum <= numfcts) {
            setfcts(fnum);
            dspfcts();
          }
        }
      }
    }
  }
}
//----------------------------------------------------------
void set_forward() {
  if (estop == true) {
    estop = false;
    display.setFont(ArialMT_Plain_16);
    display.setColor(BLACK);
    display.fillRect(80, 30, 70, 18);
    display.display();
  }
  if (MThrottle == 1) {
    xmits = locoactsT1 + "R1";
    lkr = lkrT1;
  }
  if (MThrottle == 2) {
    xmits = locoactsT2 + "R1";
    lkr = lkrT2;
  }
  if (MThrottle == 3) {
    xmits = locoactsT3 + "R1";
    lkr = lkrT3;
  }
  if (MThrottle == 4) {
    xmits = locoactsT4 + "R1";
    lkr = lkrT4;
  }
  #ifdef LOG
  if (DEBUG == true) {
    if (setsdhadr == true) Serial.print("set_forward: leader ");
    else Serial.print("set_forward: ");
    Serial.println(xmits);
  }
  #endif
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
    delay(sdelay);
    while (client.available()) {
      jmri_res = client.readStringUntil('\n');
      #ifdef LOG
      if (DEBUG == true) Serial.println("JMRI set_forward resp " + jmri_res);
      #endif
      if (jmri_res.startsWith(lkr)) {  // get direction
        int ix = jmri_res.indexOf('R');
        ix++;
        jmri_sub = jmri_res.substring(ix, ix + 1);
        int dct = jmri_sub.toInt();
        if (dct == 1) {
          dctstate = true;
          #ifdef LOG
          if (DEBUG == true) {
            Serial.print("set_forward(): dctstate = " + String(dctstate));
            Serial.println(" is forward");
          }
          #endif
          display.setFont(ArialMT_Plain_16);
          display.setColor(BLACK);
          display.fillRect(0, 13, 10, 20);
          display.setColor(WHITE);
          display.drawString(0, 13, "F");
          display.display();
        }
        if (dct == 0) {
          dctstate = false;
          #ifdef LOG
          if (DEBUG == true) {
            Serial.print("set_forward(): dctstate = " + String(dctstate));
            Serial.println("is reverse");
          }
          #endif
          display.setFont(ArialMT_Plain_16);
          display.setColor(BLACK);
          display.fillRect(0, 13, 10, 20);
          display.setColor(WHITE);
          display.drawString(0, 13, "R");
          display.display();
        }
      }
    }
  }
  xmits = locoacts + "V0";
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
  }
  #ifdef LOG
  if (DEBUG == true) Serial.println(xmits);
  #endif
  display.setFont(ArialMT_Plain_16);
  display.setColor(BLACK);
  display.fillRect(0, 13, 10, 20);
  display.setColor(WHITE);
  display.drawString(0, 13, "F");
  display.display();
  //-----------------------
  if ((setsdhadr == true) && (dccadr != sdhadr)) {
    if (sdhdct == true) {
      if (MThrottle == 1) xmits = sdhactsT1 + "R1";
      if (MThrottle == 2) xmits = sdhactsT2 + "R1";
      if (MThrottle == 3) xmits = sdhactsT3 + "R1";
      if (MThrottle == 4) xmits = sdhactsT4 + "R1";
    }
    if (sdhdct == false) {
      if (MThrottle == 1) xmits = sdhactsT1 + "R0";
      if (MThrottle == 2) xmits = sdhactsT2 + "R0";
      if (MThrottle == 3) xmits = sdhactsT3 + "R0";
      if (MThrottle == 4) xmits = sdhactsT4 + "R0";
    }
    if (svr == true) {
      client.print(xmits + "\r\n");
      client.flush();
      delay(fdelay);
    }
    #ifdef LOG
    if (DEBUG == true) {
      if (setsdhadr == true) Serial.print("set_forward: trailer ");
      else Serial.print("set_forward: ");
      Serial.println(xmits);
    }
    #endif
    xmits = locoacts + "V0";
    if (svr == true) {
      client.print(xmits + "\r\n");
      client.flush();
    }
    #ifdef LOG
    if (DEBUG == true) Serial.println(xmits);
    #endif
  }
}
//----------------------------------------------------------------
void set_reverse() {
  if (estop == true) {
    estop = false;
    display.setFont(ArialMT_Plain_16);
    display.setColor(BLACK);
    display.fillRect(80, 30, 70, 18);
    display.display();
  }
  if (MThrottle == 1) {
    xmits = locoactsT1 + "R0";
    lkr = lkrT1;
  }
  if (MThrottle == 2) {
    xmits = locoactsT2 + "R0";
    lkr = lkrT2;
  }
  if (MThrottle == 3) {
    xmits = locoactsT3 + "R0";
    lkr = lkrT3;
  }
  if (MThrottle == 4) {
    xmits = locoactsT4 + "R0";
    lkr = lkrT4;
  }
  #ifdef LOG
  if (DEBUG == true) {
    if (setsdhadr == true) Serial.print("set_reverse: leader ");
    else Serial.print("set_reverse: ");
    Serial.println(xmits);
  }
  #endif
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
    delay(sdelay);
    while (client.available()) {
      jmri_res = client.readStringUntil('\n');
      #ifdef LOG
      if (DEBUG == true) Serial.println("JMRI set_reverse resp " + jmri_res);
      #endif
      if (jmri_res.startsWith(lkr)) {  // get direction
        int ix = jmri_res.indexOf('R');
        ix++;
        jmri_sub = jmri_res.substring(ix, ix + 1);
        int dct = jmri_sub.toInt();
        if (dct == 1) {
          dctstate = true;
          #ifdef LOG
          if (DEBUG == true) {
            Serial.print("set_reverse(): dctstate = " + String(dctstate));
            Serial.println(" is forward");
          }
          #endif
          display.setFont(ArialMT_Plain_16);
          display.setColor(BLACK);
          display.fillRect(0, 13, 10, 20);
          display.setColor(WHITE);
          display.drawString(0, 13, "F");
          display.display();
        }
        if (dct == 0) {
          dctstate = false;
          #ifdef LOG
          if (DEBUG == true) {
            Serial.print("set_reverse(): dctstate = " + String(dctstate));
            Serial.println(" is reverse");
          }
          #endif
          display.setFont(ArialMT_Plain_16);
          display.setColor(BLACK);
          display.fillRect(0, 13, 10, 20);
          display.setColor(WHITE);
          display.drawString(0, 13, "R");
          display.display();
        }
      }
    }
  }
  xmits = locoacts + "V0";
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
  }
  #ifdef LOG
  if (DEBUG == true) Serial.println(xmits);
  #endif
  display.setFont(ArialMT_Plain_16);
  display.setColor(BLACK);
  display.fillRect(0, 13, 10, 20);
  display.setColor(WHITE);
  display.drawString(0, 13, "R");
  display.display();
  //-----------------------
  if ((setsdhadr == true) && (dccadr != sdhadr)) {
    if (sdhdct == true) {
      if (MThrottle == 1) xmits = sdhactsT1 + "R0";
      if (MThrottle == 2) xmits = sdhactsT2 + "R0";
      if (MThrottle == 3) xmits = sdhactsT3 + "R0";
      if (MThrottle == 4) xmits = sdhactsT4 + "R0";
    }
    if (sdhdct == false) {
      if (MThrottle == 1) xmits = sdhactsT1 + "R1";
      if (MThrottle == 2) xmits = sdhactsT2 + "R1";
      if (MThrottle == 3) xmits = sdhactsT3 + "R1";
      if (MThrottle == 4) xmits = sdhactsT4 + "R1";
    }
    if (svr == true) {
      client.print(xmits + "\r\n");
      client.flush();
    }
    #ifdef LOG
    if (DEBUG == true) {
      Serial.print("set_reverse: trailer ");
      Serial.println(xmits);
    }
    #endif
    delay(fdelay);
    xmits = locoacts + "V0";
    if (svr == true) {
      client.print(xmits + "\r\n");
      client.flush();
    }
    #ifdef LOG
    if (DEBUG == true) Serial.println(xmits);
    #endif
  }
}
//-------------------------------------------------------------
void chgsdhdct() {
  if (sdhdct == true) {
    if (MThrottle == 1) xmits = sdhactsT1 + "R1";
    if (MThrottle == 2) xmits = sdhactsT2 + "R1";
    if (MThrottle == 3) xmits = sdhactsT3 + "R1";
    if (MThrottle == 4) xmits = sdhactsT4 + "R1";
  }
  if (sdhdct == false) {
    if (MThrottle == 1) xmits = sdhactsT1 + "R0";
    if (MThrottle == 2) xmits = sdhactsT2 + "R0";
    if (MThrottle == 3) xmits = sdhactsT3 + "R0";
    if (MThrottle == 4) xmits = sdhactsT4 + "R0";
  }
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
    delay(fdelay);
  }
  #ifdef LOG
  if (DEBUG == true) {
    Serial.print("chgsdhdct: ");
    Serial.println(xmits);
  }
  #endif
  if (MThrottle == 1) xmits = locoactsT1 + "V0";
  if (MThrottle == 2) xmits = locoactsT2 + "V0";
  if (MThrottle == 3) xmits = locoactsT3 + "V0";
  if (MThrottle == 4) xmits = locoactsT4 + "V0";
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
  }
  #ifdef LOG
  if (DEBUG == true) Serial.println(xmits);
  #endif
}
/////////////////////////////////////////////////////////////////
void releaseadr(int rlsadr) {
  String MTprf;
  MTprf = "M" + String(MThrottle);
  String xmitsi;
  if (rlsadr < 128) {
    xmits = MTprf + "-S" + String(rlsadr) + "<;>r"; 
    xmitsi = MTprf + "AS" + String(rlsadr) + "<;>I"; 
  } else {
    if (rlsadr > 127) {
      xmits = MTprf + "-L" + String(rlsadr) + "<;>r";
      xmitsi = MTprf + "AL" + String(rlsadr) + "<;>I";
    }
  }
  #ifdef LOG
  if (DEBUG == true) Serial.println(xmitsi);
  #endif
  if (svr == true) {
    client.print(xmitsi + "\r\n");  //send "idle"
    client.flush();
  }
  #ifdef LOG
  if (DEBUG == true) {
    Serial.print("Releasing address ");
    Serial.println(xmits);
  }
  #endif
  if (svr == true) {
    client.print(xmits + "\r\n");  //send "release"
    client.flush();
    #ifdef LOG
    if (DEBUG == true) {
      delay(ldelay);
      Serial.println("JMRI-response: ");
      while (client.available()) {
        char c = client.read();
        Serial.write(c);
      }
      Serial.println();
    }
    #endif
    dccadrrlsd = rlsadr;
  }
}
//-------------------------------------------------------------
void compsdhadr() {
  String MTprf;
  MTprf = "M" + String(MThrottle);
  if (sdhadr < 128) {
    xmits = MTprf + "+S" + String(sdhadr) + "<;>S" + String(sdhadr);
    sdhacts = MTprf + "AS" + String(sdhadr) + "<;>";                  // set variable for actions
  }
  if (sdhadr > 127) {
    xmits = MTprf + "+L" + String(sdhadr) + "<;>L" + String(sdhadr);
    sdhacts = MTprf + "AL" + String(sdhadr) + "<;>";
  }
  if (MThrottle == 1) {
    sdhactsT1 = sdhacts;
    sdhadrT1 = sdhadr;
  }
  if (MThrottle == 2) {
    sdhactsT2 = sdhacts;
    sdhadrT2 = sdhadr;
  }
  if (MThrottle == 3) {
    sdhactsT3 = sdhacts;
    sdhadrT3 = sdhadr;
  }
  if (MThrottle == 4) {
    sdhactsT4 = sdhacts;
    sdhadrT4 = sdhadr;
  }
  #ifdef LOG
  if (DEBUG == true) Serial.println("compsdhadr: " + xmits);
  #endif
}
//---------------------------------------------------------------------
void sendadr() {
  // add a locomotive
  String MTprf;
  MTprf = "M" + String(MThrottle);
  if (dccadr < 128) {
    if (ros_eAvail == false) {
      xmits = MTprf + "+S" + String(dccadr) + "<;>S" + String(dccadr);
    } else {
      xmits = MTprf + "+S" + String(dccadr) + "<;>E" + loconame;
    }
    locoacts = MTprf + "AS" + String(dccadr) + "<;>";  // set variable for actions
  }
  if (dccadr > 127) {
    if (ros_eAvail == false) {
      xmits = MTprf + "+L" + String(dccadr) + "<;>L" + String(dccadr);
    } else {
      //xmits = "M0+L" + String(dccadr) + "<;>E" + loconame;
      xmits = MTprf + "+L" + String(dccadr) + "<;>E" + loconame;
    }
    //locoacts = "M0AL" + String(dccadr) + "<;>";
    locoacts = MTprf + "AL" + String(dccadr) + "<;>";
  }
  lkf = locoacts + "F";  // search argument to look for function states
  lkr = locoacts + "R";  // search argument to look for direction
  lks = locoacts + "s";  // search argument to look for speed step
  if (MThrottle == 1) {  // for use by other subroutines
    dccadrT1 = dccadr;
    locoactsT1 = locoacts;
    lkfT1 = lkf;
    lkrT1 = lkr;
    lksT1 = lks;
  }
  if (MThrottle == 2) {
    dccadrT2 = dccadr;
    locoactsT2 = locoacts;
    lkfT2 = lkf;
    lkrT2 = lkr;
    lksT2 = lks;
  }
  if (MThrottle == 3) {
    dccadrT3 = dccadr;
    locoactsT3 = locoacts;
    lkfT3 = lkf;
    lkrT3 = lkr;
    lksT3 = lks;
  }
  if (MThrottle == 4) {
    dccadrT4 = dccadr;
    locoactsT4 = locoacts;
    lkfT4 = lkf;
    lkrT4 = lkr;
    lksT4 = lks;
  }
  ros_eAvail = false;
  #ifdef LOG
  if (DEBUG == true) Serial.println("sendadr: " + xmits);
  #endif
  fctsavail = false;
  fcgstate = 0;
  display.setFont(ArialMT_Plain_10);
  display.setColor(BLACK);
  display.fillRect(15, 50, 100, 15);
  display.setColor(WHITE);
  display.drawString(30, 50, "mapping functions");
  display.display();
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
    delay(sdelay);
    while (client.available()) {
      jmri_res = client.readStringUntil('\n');
      String lkfcta = MTprf + "L";
      if (jmri_res.startsWith(lkfcta)) {  // get functions array
        functions = jmri_res;
        fctsavail = true;
        #ifdef LOG
        if (DEBUG == true) {
          Serial.println("JMRI list of functions: ");
          Serial.println(functions);
        }
        #endif
      }
      if (jmri_res.startsWith(lkf)) {  //get function states
        int ix = jmri_res.indexOf('F');
        ix++;
        jmri_sub = jmri_res.substring(ix, ix + 1);
        fctstate = jmri_sub.toInt();
        ix++;
        jmri_sub = jmri_res.substring(ix, ix + 2);
        fnum = jmri_sub.toInt();
        if (fnum < 10) {
          jmri_sub = jmri_res.substring(ix, ix + 1);
          fnum = jmri_sub.toInt();
        }
        if (fnum <= numfcts) {  // limit to configured nbr of functions
          setfcts(fnum);
        }
      }
      if (jmri_res.startsWith(lkr)) {  // get direction
        int ix = jmri_res.indexOf('R');
        ix++;
        jmri_sub = jmri_res.substring(ix, ix + 1);
        int dct = jmri_sub.toInt();
        if (dct == 1) {
          dctstate = true;
          #ifdef LOG
          if (DEBUG == true) {
            Serial.print("sendadr(): ");
            Serial.print("dctstate = " + String(dctstate));
            Serial.println(" is forward");
          }
          #endif
          display.setFont(ArialMT_Plain_16);
          display.setColor(BLACK);
          display.fillRect(0, 13, 10, 20);
          display.setColor(WHITE);
          display.drawString(0, 13, "F");
          display.display();
        }
        if (dct == 0) {
          dctstate = false;
          #ifdef LOG
          if (DEBUG == true) {
            Serial.print("sendadr(): ");
            Serial.print("dctstate = " + String(dctstate));
            Serial.println(" is reverse");
          }
          #endif
          display.setFont(ArialMT_Plain_16);
          display.setColor(BLACK);
          display.fillRect(0, 13, 10, 20);
          display.setColor(WHITE);
          display.drawString(0, 13, "R");
          display.display();
        }
      }
      if (jmri_res.startsWith(lks)) {  // look for speed step setting in jmri
        int ix = jmri_res.indexOf('s');
        ix++;
        jmri_sub = jmri_res.substring(ix, ix + 1);
        spdstpm = jmri_sub.toInt();
        if (spdstpm == 8) numspst = 14;
        if (spdstpm == 1) numspst = 126;  // speed step mode 128 (0=stop,1to126+Estop=128)
        if (spdstpm == 2) numspst = 28;   // speed step mode 28
        #ifdef LOG
        if (DEBUG == true) {
          Serial.print("current speed step mode: ");
          Serial.println(numspst);
        }
        #endif
        display.setFont(ArialMT_Plain_10);
        display.setColor(BLACK);
        display.fillRect(50, 18, 20, 10);
        display.setColor(WHITE);
        display.drawString(50, 18, String(numspst));
        display.display();
      }
    }
    //match direction setting of loco with switch setting:
    if (numthrottles == 1) {  // for single throttle only:
      swtFstate = digitalRead(switchFpin);
      swtRstate = digitalRead(switchRpin);
      if ((swtFstate == LOW) && (swtRstate == HIGH) && (dctstate == false)) set_forward();
      if ((swtFstate == HIGH) && (swtRstate == LOW) && (dctstate == true)) set_reverse();
    }
  }
  // build dedicated function switch mapping array based on JMRI Roster:
  if (fctsavail == true) {
    #ifdef LOG
    if (DEBUG == true) Serial.println("mapping switches");
    #endif
    for (int i = 0; i < numswfcs; i++) {  //reset all array elements to default
      fctswtmap[i] = 99;
    }
    for (int i = 0; i < numfcts; i++) {  //reset all array elements to default
      fctnos[i] = 99;
      fctnoe[i] = 99;
      fctnox[i] = 99;
    }
    fxi = 0;
    fxm = 0;
    for (byte j = 0; j < numswfcs; j++) {
      fxn = 0;
      fxs = 0;
      fxe = 0;
      while (fxn < numfcts) {  // look up all elements of functions for each switched function
        fxs = functions.indexOf("[", fxs);
        fxe = functions.indexOf("]", fxs);
        String lkn = fctname[fxi];                      // copy fct name to search for into search arg
        String lk = functions.substring(fxs + 1, fxe);  // copy JMRI entry to compare
        fxs++;
        if (lk == lkn) {  // if it is a match
          #ifdef LOG
          if (DEBUG == true) {
            Serial.print("Function: ");
            Serial.println(lk);
            Serial.print("Match @ (fxn):");
            Serial.println(fxn);
          }
          #endif
          fctswtmap[fxm] = fxn;
          break;
        }
        fxn++;
      }
      fxi++;
      fxm++;
    }
    #ifdef LOG
    if (DEBUG == true) {
      Serial.print("fctswtmap  : ");
      for (byte x = 0; x < numswfcs; x++) {
        Serial.print(fctswtmap[x]);
        Serial.print(",");
      }
      Serial.println();
    }
    #endif
    dspfcts();
  }
  if (fctsavail == false) {
    for (int i = 0; i < numswfcs; i++) {
      fctswtmap[i] = fctswtmapdft[i];
    }
    dspfcts();
    #ifdef LOG
    if (DEBUG == true) Serial.println("Using default Function Switch map");
    #endif
  }
  if (fctswtmap[7] == 99) {
    drivehold = false;
    #ifdef LOG
    if (DEBUG == true) Serial.println("drivehold not available for this loco");
    #endif
  } else {
    drivehold = true;
    #ifdef LOG
    if (DEBUG == true) Serial.println("drivehold available for this loco");
    #endif
  }
#ifdef EUROSOUND
  sfnum = fctswtmap[8];
  if (sfnum != 99) {
    fading = true;
    if (fctarray[sfnum] == 1) {
      faded = true;
      display.setColor(WHITE);
      display.drawString(70, 18, "F");
      display.display();
      #ifdef LOG
      if (DEBUG == true) Serial.println("Sound of this loco is faded");
      #endif
    }
    #ifdef LOG
    if (DEBUG == true) Serial.println("F1 + Speed > 0 = Sound Fading for this loco");
    #endif
  } else {
    fading = false;
    #ifdef LOG
    if (DEBUG == true) Serial.println("Normal F1 for this loco");
    #endif
  }
#endif
#ifndef EUROSOUND
  sfnum = fctswtmap[8];
  if (sfnum != 99) {
    fading = true;
    if (fctarray[sfnum] == 8) {
      faded = true;
      display.setColor(WHITE);
      display.drawString(70, 18, "F");
      display.display();
      #ifdef LOG
      if (DEBUG == true) Serial.println("Sound of this loco is faded");
      #endif
    }
    #ifdef LOG
    if (DEBUG == true) Serial.println("F8 + Speed > 0 = Sound Fading for this loco");
    #endif
  } else {
    fading = false;
    #ifdef LOG
    if (DEBUG == true) Serial.println("Normal F8 for this loco");
    #endif
  }
#endif //EUROSOUND
  bldfctidx();
  #ifdef LOG
  if (DEBUG == true) {
    Serial.println("--------------------------------------------");
    Serial.println("Loco address " + String(dccadr));
    if (setsdhadr == true) Serial.println("Trailer address: " + String(sdhadr));
    Serial.println("--------------------------------------------");
  }
  #endif
}
//--build index of function entries----:
void bldfctidx() {
  fxs = 0;
  fxe = 0;
  byte j = 0;
  numfcnt = 0;
  int fxl = functions.length();
  for (byte i = 0; i < numfcts; i++) {
    fxs = functions.indexOf("[", fxs);
    fxe = functions.indexOf("]", fxs);
    fxs++;
    String lk = functions.substring(fxs, fxe);  // copy JMRI entry
    if (fxe > fxl) fxe = fxl;
    if (fxe - fxs > 0) {  // not nothing between []
      numfcnt++;
      fctnos[j] = fxs;  //start of name
      fctnoe[j] = fxe;  //end of name
      fctnox[j] = i;    //fct number
      j++;
      #ifdef LOG
      if (DEBUG == true) {
        //Serial.println("Finding function name indices");
        //Serial.println("Start of name: " + String(fxs));
        //Serial.println("End of name: " + String(fxe));
        Serial.print("Function found: F" + String(i));
        Serial.println(" = " + String(lk));
      }
      #endif
    }
  }
  numfcnt--;
  #ifdef LOG
  if (DEBUG == true) {
    Serial.println("Building function name arrays:");
    Serial.println("Functions count: " + String(numfcnt));
    Serial.print("function name start offsets: ");
    for (byte i = 0; i < numfcts; i++) {
      Serial.print(fctnos[i]);
      Serial.print(",");
    }
    Serial.println();
    Serial.print("function name end offsets: ");
    for (byte i = 0; i < numfcts; i++) {
      Serial.print(fctnoe[i]);
      Serial.print(",");
    }
    Serial.println();
    Serial.print("function numbers: ");
    for (byte i = 0; i < numfcts; i++) {
      Serial.print(fctnox[i]);
      Serial.print(",");
    }
    Serial.println();
  }
  #endif
}
//---------------------------------------------------------------
void sendaccadr() {
  xmits = "PTA" + setacc + String(accadr);
  #ifdef LOG
  if (DEBUG == true) Serial.println("sendaccadr sends " + xmits);
  #endif
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
    delay(ldelay);
    while (client.available()) {
      jmri_res = client.readStringUntil('\n');
      if (jmri_res.startsWith("PTA")) {
        #ifdef LOG
        if (DEBUG == true) {
          Serial.print("JMRI reply to sendaccadr: ");
          Serial.println(jmri_res);
        }
        #endif
        int ix = jmri_res.indexOf('A');
        ix++;
        jmri_sub = jmri_res.substring(ix);
        String stostate = jmri_sub.substring(0, 1);
        tostate = stostate.toInt();
        if ((tostate >= 0) && (tostate < 9)) dsptostate = tostarray[tostate];
        #ifdef LOG
        if (DEBUG == true) {
          Serial.println("JMRI turnout state: '2'=Closed, '4'=Thrown, '1'=Unknown or '8'=Inconsistent.");
          Serial.print("tostate: ");
          Serial.println(tostate);
          Serial.print("dsptostate: ");
          Serial.println(dsptostate);
        }
        #endif
      }
    }
    dspaccstate();
  }
}
//-----------------------------------------------------------------
void dspaccstate() {
  display.setFont(ArialMT_Plain_10);
  display.setColor(BLACK);
  display.fillRect(0, 50, 132, 15);
  display.setColor(WHITE);
  display.drawString(0, 50, "T-state: " + String(dsptostate));
  display.display();
}
//-----------------------------------------------------------------
void sendroute() {
  xmits = "PRA2" + String(rsysname);
  #ifdef LOG
  if (DEBUG == true) Serial.println("sendroute: " + xmits);
  #endif
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
    delay(ldelay);
    #ifdef LOG
    if (DEBUG == true) {
      Serial.println("server response: ");
      while (client.available()) {
        char c = client.read();
        Serial.write(c);
      }
    }
    #endif
  }
}
/////////////////////////////////////////////////////////////////////////
#ifdef CLOCK
void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600;
  pctime = unixsub.toInt();
  if (pctime >= DEFAULT_TIME) {  // check the integer is a valid time (greater than Jan 1 2013)
    setTime(pctime);             // Sync Arduino clock to the time received on the serial port
  }
}
void getjmritime() {
  unixtime = jmri_res;  // save fastclock time
  int j = unixtime.indexOf("<;>");
  if (j > 0) {                           // some times a "<;>" isn't sent from jmri, ignore timestamp if it isn't.
    unixsub = unixtime.substring(3, j);  // time stamp only
    j = j + 3;
    ratiosub = unixtime.substring(j, j + 3);
    // process unix time stamp
    processSyncMessage();
    //display fastclock:
    dsphour = hour();
    dspmin = minute();
    display.setFont(ArialMT_Plain_10);
    display.setColor(BLACK);
    //display.fillRect(39, 0, 50, 10);
    display.fillRect(43, 0, 50, 10);
    display.setColor(WHITE);
    if (dspmin < 10) {
      display.drawString(45, 0, String(dsphour) + ":0" + String(dspmin) + "  r:" + ratiosub + "\r\n");
    } else {
      display.drawString(45, 0, String(dsphour) + ":" + String(dspmin) + "  r:" + ratiosub + "\r\n");
    }
    display.display();
    //getbatlvl();
  }
}
#endif
//--------------------------------------
void retrievestack() {
  byte j = 2;  // stack array saved to eeprom starting at element 2
  for (byte i = 0; i <= 5; i++) {
    dccadrtmp = EEPROM.read(j);
    j++;
    dccadrtmp = (dccadrtmp << 8) + EEPROM.read(j);
    j++;
    if ((dccadrtmp >= 0) && (dccadrtmp < 9999)) dccstck[i] = dccadrtmp;
  }
  #ifdef LOG
  if (DEBUG == true) {
    Serial.print("stack retrieved: ");
    for (byte i = 0; i <= 5; i++) {
      Serial.print(dccstck[i]);
      Serial.print(",");
    }
    Serial.println();
  }
  #endif
}
//--------------------------------------
void writestack() {
  for (byte i = 4; i >= 0; i--) {  // all entries shift right
    dccadrtmp = dccstck[i];
    byte j = i + 1;
    dccstck[j] = dccadrtmp;
    if (i == 0) {
      dccstck[0] = dccadr;  // add new entry as first
      break;
    }
  }
  #ifdef LOG
  if (DEBUG == true) {
    Serial.print("stack array new: ");
    for (byte i = 0; i <= 5; i++) {
      Serial.print(dccstck[i]);
      Serial.print(",");
    }
    Serial.println();
  }
  #endif
}
//---------------------------------------------------------
void savestack() {
  #ifdef LOG
  if (DEBUG == true) {
    Serial.print("saving stack: ");
    for (byte i = 0; i <= 5; i++) {
      Serial.print(dccstck[i]);
      Serial.print(",");
    }
    Serial.println();
  }
  #endif
  byte j = 2;  // stack array saved to eeprom starting at element 2
  for (byte i = 0; i <= 5; i++) {
    dccadrtmp = dccstck[i];
    if ((dccadr > 0) && (dccadr <= 9999)) {
      EEPROM.write(j, (dccadrtmp >> 8) & 0xff);
      j++;
      EEPROM.write(j, (dccadrtmp & 0xff));
      j++;
    }
  }
  EEPROM.commit();
}
//--------------------------------------
#ifdef BATLEVEL
void getbatlvl() {
  // check battery level:
  digitalWrite(2, HIGH);
  delay(50);
  batin = analogRead(voltagepin);
  digitalWrite(2, LOW);
  battmp = batin / 2047.5;
  //batvolt = battmp * 3.63;    // 3.3 refV * 1.1 ADC refV = 3.63
  batvolt = battmp * 3.53;  // this gets us closer to a meter reading
  char dspbat[4];
  dtostrf(batvolt, 4, 1, dspbat);
  display.setFont(ArialMT_Plain_10);
  display.setColor(BLACK);
  display.fillRect(100, 0, 28, 10);
  display.setColor(WHITE);
  display.drawString(100, 0, dspbat);
  display.drawString(120, 0, "V");
  display.display();
}
#endif
//--------------------------------------
void set_spstm() {
  #ifdef LOG
  if (DEBUG == true) {
    Serial.print("current speed step mode: ");
    Serial.println(spdstpm);
  }
  #endif
  if (spdstpm == 1) chgspstm = 2;
  if (spdstpm == 2) chgspstm = 1;
  if (MThrottle == 1) {
    xmits = locoactsT1 + "s" + String(chgspstm);
    lks = lksT1;
  }
  if (MThrottle == 2) {
    xmits = locoactsT2 + "s" + String(chgspstm);
    lks = lksT2;
  }
  if (MThrottle == 3) {
    xmits = locoactsT3 + "s" + String(chgspstm);
    lks = lksT3;
  }
  if (MThrottle == 4) {
    xmits = locoactsT4 + "s" + String(chgspstm);
    lks = lksT4;
  }
  #ifdef LOG
  if (DEBUG == true) {
    Serial.print("set_spstm: ");
    Serial.println(xmits);
  }
  #endif
  client.print(xmits + "\r\n");
  client.flush();
  delay(ldelay);
  while (client.available()) {
    jmri_res = client.readStringUntil('\n');
    if (jmri_res.startsWith(lks)) {  // look for speed step setting in jmri
      int ix = jmri_res.indexOf('s');
      ix++;
      jmri_sub = jmri_res.substring(ix, ix + 1);
      spdstpm = jmri_sub.toInt();
      //if (spdstpm   == 8) numspst   = 14;
      if (spdstpm == 1) numspst = 126;  // speed step mode 128 (0=stop,1to126+Estop=128)
      if (spdstpm == 2) numspst = 28;   // speed step mode 28
      #ifdef LOG
      if (DEBUG == true) {
        Serial.print("new speed step mode: ");
        Serial.println(spdstpm);
      }
      #endif
      display.setFont(ArialMT_Plain_10);
      display.setColor(BLACK);
      display.fillRect(50, 18, 20, 10);
      display.setColor(WHITE);
      display.drawString(50, 18, String(numspst));
      display.display();
    }
  }
}
//------------------------------------------------------------------
void simlokbtn() {
  clrchgadr();
  clrrosnam();
  //lastdccadr = dccadr;
  btncnt = 0;
  setdccadr = true;
  adrbtnstate = true;
  dccadrrlsd = dccadr;  // force sendadr for mapping functions
  pshadr = true;
  vspeed = 0;
  vtemp = 0;
  vcnt = 0;
  dspeed = vcnt;
  lastvcnt = 0;
  dsp_speed();
  display.setColor(BLACK);
  display.fillRect(70, 13, 10, 15);  // (remove +)
  display.setColor(WHITE);
  display.drawString(80, 13, String(dspadr));
  display.drawString(120, 13, "?");
  display.display();
}
//------------------------------------------------------------------
void cfgthr() {
  bool thrcfgchg = false;
  display.displayOff();
  bool eoj = false;
  while (eoj == false) {
    if (Serial.available() > 0) {
      int indata = Serial.parseInt();  // receive byte as an integer
      Serial.print("You entered: ");
      Serial.println(indata);
      if (indata == 9) {
        eoj = true;
        Serial.println("throttle-config ended.");
        if (thrcfgchg == false) {
          display.displayOn();
        }
        else if (thrcfgchg == true) {
          if (svr == true) {  // send "Quit" to server
            client.print("Q\r\n");
            delay(fdelay);
            client.flush();
            Serial.println("Rebooting");
            ESP.restart();
          }
        }
      }
      if ((indata > 0) && (indata < 5) && (numthrottles != indata)) {
        numthrottles = indata;
        thrcfgchg = true;
        Serial.println("New number of throttles: " + String(numthrottles));
        EEPROM.write(19, numthrottles);
        EEPROM.commit();
        Serial.println("saved to EEPROM.");
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.drawString(0, 10, "To exit enter 9");
        display.drawString(0, 20, "or press stop.");
        display.displayOn();
        display.display();
      }
    }
    swt3state = digitalRead(switch3pin);
    if (swt3state != lastswt3state) {
      if (swt3state == LOW) eoj = true;
      lastswt3state = swt3state;
      Serial.println("throttle-config ended");
      display.displayOn();
      if (thrcfgchg == true) {
        Serial.println("Rebooting");
        ESP.restart();
      }
    }
  }
}
//---------------------------------------------------------------------------
void actdccadr() {
  setdccadr = false;
  if ((pshadr == true) || (ros_eAvail == true)) {  // address entered, or got a roster entry.
    if (ros_eAvail == true) {
      vcnt = 0;
      vtemp = 0;
      vspeed = 0;
      dccadr = locoadr.toInt();
      #ifdef LOG
      if (DEBUG == true) Serial.println("address taken from roster");
      #endif
      clrchgadr();
    } else {
      clrchgadr();
    }
    sprintf(dspadr, "%04d", dccadr);  // display with leading zeros
    display.setFont(ArialMT_Plain_16);
    display.setColor(BLACK);
    display.fillRect(50, 18, 17, 10);   // remove stack array #
    display.fillRect(120, 13, 10, 15);  // remove ?
    display.fillRect(80, 30, 70, 18);   // remove locoadr from loconame display
    display.setColor(WHITE);
    display.drawString(80, 13, String(dspadr));
    display.display();
    #ifdef LOG
    if (DEBUG == true) Serial.println("actdccadr #1: " + String(dspadr));
    #endif
    // avoid duplicate in stack array: -------------------------
    updstck = true;
    for (byte i = 0; i <= 5; i++) {
      if (dccstck[i] == dccadr) {
        updstck = false;
        break;
      }
    }
    // write to stack array: -----------------------------------
    if (updstck == true) {
      writestack();
      savestack();  // to eeprom
    }
    // also save last dcc address to eeprom:
    dccadrtmp = EEPROM.read(0);
    dccadrtmp = (dccadrtmp << 8) + EEPROM.read(1);
    if (dccadr != dccadrtmp) {
      EEPROM.write(0, (dccadr >> 8) & 0xff);
      EEPROM.write(1, dccadr & 0xff);
      EEPROM.commit();
    }
    // -------------------------------------------------
    pshadr = false;
    #ifdef LOG
    if (DEBUG == true) {
      Serial.print("adding dcc address ");
      Serial.println(dspadr);
    }
    #endif
    if (ros_eAvail == false) {
      rtvrosnam();
    }
    clrrosnam();
    if (ros_eAvail == true) dsprosnam();
    if ((lastdccadr != 0) && (lastdccadr != dccadr) && (setsdhadr == false)) releaseadr(lastdccadr);
    if (lastdccadr != dccadr) sendadr();
    if (MThrottle == 1) MT1active = true;
    if (MThrottle == 2) MT2active = true;
    if (MThrottle == 3) MT3active = true;
    if (MThrottle == 4) MT4active = true;
    #ifdef LOG
    if (DEBUG == true) {
      Serial.print("Active throttle(s): ");
      if (MT1active == true) Serial.print("1: " + String(dccadrT1));
      if (MT2active == true) Serial.print(", 2: " + String(dccadrT2));
      if (MT3active == true) Serial.print(", 3: " + String(dccadrT3));
      if (MT4active == true) Serial.print(", 4: " + String(dccadrT4));
      Serial.println();
    }
    #endif
  } else {
    #ifdef LOG
    if (DEBUG == true) {
      Serial.println("adr not changed ");
      Serial.println(dccadr);
    }
    #endif
    if (dccadrrlsd == dccadr) {
      sendadr();
      dccadrrlsd = 0;
    }
    display.setFont(ArialMT_Plain_16);
    display.setColor(BLACK);
    display.fillRect(80, 13, 50, 15);
    display.fillRect(80, 30, 50, 18);  // remove locoadr on display
    display.setColor(WHITE);
    sprintf(dspadr, "%04d", dccadr);
    display.drawString(80, 13, String(dspadr));
    display.display();
    #ifdef LOG
    if (DEBUG == true) Serial.println("actdccadr #2: " + String(dspadr));
    #endif
    vcnt = lastvcnt;
  }
}
//------------------------------------------------------
void keypad_check() {
  display.displayOff();
  if (svr == true) {  // send "Quit" to server
    client.print("Q\r\n");
    delay(fdelay);
    client.flush();
  }
  Serial.println("Keypad-Config: reading input from keypad 10 times");
  for (byte i = 0; i < 10; i++) {
    btnread = analogRead(keypadin);
    Serial.print("Keypad reading with no active button: ");
    Serial.println(btnread);
    if (btnread < btnprvl) {
      btnmin = btnread;
      btnprvl = btnread;
    }
    if (btnread > btnprvh) {
      btnmax = btnread;
      btnprvh = btnread;
    }
    delay(500);
  }
  btnprvl = 5000;  // reset
  Serial.println("If not all values are zero decrease resistor connecting keypad buttons to ground.");
  Serial.println("Current EEPROM content: ");
  byte j = 20;
  for (byte i = 0; i < 32; i++) {
    int v = EEPROM.read(j);
    j++;
    v = (v << 8) + EEPROM.read(j);
    j++;
    Serial.print(v);
    Serial.print(",");
  }
  Serial.println();
}
void keypad_config() {
  int btnmin = 0;      // for finding min value
  int btnmax = 0;      // for finding max value
  int btnprvl = 5000;  // previous low
  int btnprvh = 0;     // previous high
  int btnnum = 0;      // button number
  int btnsav = 0;
  bool kpreadON = false;
  byte offset[16];
  bool eoj = false;
  byte j = 20;
  for (byte i = 0; i < 16; i++) {
    offset[i] = j;
    j = j + 4;
  }
  Serial.println("Push a button for about 2 sec. Remember the button number");
  Serial.println("in case you want to save it.");
  Serial.println("For ending the keypad-config enter 99 or press the Brake button.");
  while (eoj == false) {
    btnread = analogRead(keypadin);
    if (btnread > 100) {  // minimum value indicating a button is pressed
      delay(dbcdly);
      kpreadON = true;
      Serial.print("Button reads: ");
      Serial.println(btnread);
      if (btnread < btnprvl) {
        btnmin = btnread;
        btnprvl = btnread;
      }
      if (btnread > btnprvh) {
        btnmax = btnread;
        btnprvh = btnread;
      }
      delay(100);
    } else if (btnread == 0) {
      if (kpreadON == true) {
        btnnum++;
        Serial.print(btnnum);
        Serial.print(". Button ");
        Serial.print(" min keypad reading ");
        Serial.println(btnmin);
        Serial.print(btnnum);
        Serial.print(". Button ");
        Serial.print(" max keypad reading ");
        Serial.println(btnmax);
        btnprvl = 5000;
        btnprvh = 0;
        kpreadON = false;
        Serial.println("To save the values: in SM enter the button number you pushed.");
      }
    }
    if (Serial.available() > 0) {
      int btnin = Serial.parseInt();  // receive byte as an integer
      Serial.print("You entered: ");
      Serial.println(btnin);
      if (btnin == 99) {
        eoj = true;
        Serial.println("keypad-config ended");
        display.displayOn();
        reconnect();
      }
      if ((btnin > 0) && (btnin < 17)) {  // is a button in keypad
        btnin--;
        btnsav = offset[btnin];
        if (btnmin > 5) btnmin = btnmin - 5;
        if (btnmax < 4090) btnmax = btnmax + 5;
        Serial.println("------------------------------------------");
        Serial.print("Saving values from ");
        Serial.print(btnnum);
        Serial.println(". reading.");
        Serial.print("Saving minimum value ");
        Serial.print(btnmin);
        Serial.print(" in elements ");
        Serial.print(btnsav);
        Serial.print(" & ");
        EEPROM.write(btnsav, (btnmin >> 8) & 0xff);
        btnsav++;
        Serial.println(btnsav);
        EEPROM.write(btnsav, (btnmin)&0xff);
        btnsav++;
        Serial.print("Saving maximum value ");
        Serial.print(btnmax);
        Serial.print(" in elements ");
        Serial.print(btnsav);
        Serial.print(" & ");
        EEPROM.write(btnsav, (btnmax >> 8) & 0xff);
        btnsav++;
        Serial.println(btnsav);
        EEPROM.write(btnsav, (btnmax)&0xff);
        EEPROM.commit();
        Serial.println("New EEPROM content: ");
        byte j = 20;
        for (byte i = 0; i < 32; i++) {
          int val = EEPROM.read(j);
          j++;
          val = (val << 8) + EEPROM.read(j);
          j++;
          Serial.print(val);
          Serial.print(",");
        }
        Serial.println();
      }
    }
    swt3state = digitalRead(switch3pin);
    if (swt3state != lastswt3state) {
      if (swt3state == LOW) eoj = true;
      lastswt3state = swt3state;
      Serial.println("keypad-config ended");
      display.displayOn();
      reconnect();
    }
  }
}
//-----------------------------------------------------
void reconnect() {
  #ifdef LOG
  if (DEBUG == true) {
    Serial.println("attempting to reconnect");
  }
  #endif
  svr = true;
  display.setFont(ArialMT_Plain_10);
  display.setColor(BLACK);
  display.fillRect(43, 0, 95, 12);
  display.setColor(WHITE);
  display.drawString(45, 0, "reconnecting");
  display.display();
  cntc = 6;
  cntt = 0;
  client.connect(host, port);
  while (!client.connected()) {
    delay(1000);
    cntc--;
    cntt++;
    if (cntc <= 0) {
      cntc = 6;
      client.connect(host, port);
    }
    if (cntt > 60) {
      #ifdef LOG
      if (DEBUG == true) {
        Serial.println();
        Serial.println("Main: server reconnect failed");
      }
      #endif
      client.stop();
      display.setColor(BLACK);
      display.fillRect(43, 0, 95, 12);
      display.setColor(WHITE);
      display.drawString(45, 0, "reconnect failed");
      display.display();
      WiFi.disconnect();
      #ifdef LOG
      if (DEBUG == true) {
        Serial.println("Throttle offline");
        Serial.println("WiFi disconnected");
      }
      #endif
      svr = false;
    }
  }
  if (svr == true) {
    //---Throttle ON: sends NAME,MAC to server and starts heart beat
    #ifdef LOG
    if (DEBUG == true) Serial.println("Sending HU" + MacString);
    #endif
    client.print("N" + tname + "\r\n");  // set Name of Throttle
    delay(fdelay);
    client.print("HU" + MacString + "\r\n");  // set identifier
    delay(ldelay);
    if (hbuse == true) {
      client.print("*+\r\n");  // starting heartbeat
      delay(ldelay);
      client.flush();
    }
    adrbtnstate = true;
    setdccadr = true;
    pshadr = true;
    dccadrrlsd = dccadr;  // force sendadr for mapping functions
    display.setColor(BLACK);
    display.fillRect(43, 0, 95, 12);
    display.setColor(WHITE);
    display.drawString(45, 0, "svr online");
    display.setFont(ArialMT_Plain_16);
    display.drawString(120, 13, "?");
    display.display();
  }
}
