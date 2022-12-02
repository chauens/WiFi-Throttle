/* "WiCAB1" WiFi Single Throttle with JMRI. Version V1.
   Christoph Hauenstein. Dec/01/2022.
   Updates: #1 removed interrupt statements. 
   The pin assignments are for EzSBC or Adafruit ESP32-Feather.
   Prerequisites:
   Install libraries: in Arduino IDE - select File, Preferences >
    after “Additional Boards Manager URLs:”  paste:
    https://dl.espressif.com/dl/package_esp32_index.json
    In the Board Manager select "ESP Dev Module".
   Install Additional lib via zip file: https://github.com/geekfactory/TimeLib
   For more details please refer to the document "WiFiThrottle Documentation".
   Turn on debug in JMRI/default.icf :
   log4j.category.jmri.jmrit.withrottle=DEBUG
*********************************************************************
***Before loading this program for the first time the keypad has to be***
***configured first:                                                  ***
***upload sketch_Keypad_Config-V1 and run it with Serial Monitor.     ***
***Follow the instructions in the Document "WiCab1_Documention.docx". ***
*************************************************************************
*/
#include <TimeLib.h>
#include <EEPROM.h>
#include <WiFi.h>
#include "SH1106Wire.h"
//#inlcude "SSD1306Wire.h"
// Initialize the OLED display:
SH1106Wire  display(0x3c, 23, 22);     // I2C addr, SDA, SCK
//SSD1306Wire  display(0x3c, 23, 22);    // I2C addr, SDA, SCK
String vrs = "Version 2.12.01.1";  // y.m.d.t#
WiFiClient client;
/////////////////////////////////////////////////////////////////////////////////
// compile options:
#define BATLEVEL   // Battery level sensor
#define CLOCK      // Display JMRI clock
//////////////////////////////////////////////////////////////////////////////////
// WiFi Server Definitions
const char* ssid = "RPi-JMRI";      // local wireless network id
const char* pass = "rpI-jmri";      // key
const char* host = "192.168.6.1";   // WiThrottle server IP Adr. (Device running JMRI-WiThrottle-server)
const int port = 12090;             // WiThrottle server Port
///////////////////////////////////////////////////////////////////////////////////////////
// JMRI-Roster mapping for dedicated function switches >>
char *fctname[] = {"Whistle-short", "Whistle-long", "Bell", "s1map", "Coupler", "Coupler-front", "Shunting", "Drive-Hold", "Fade"};
#define numswfcs 9 // nbr of switched functions = nbr of array elements
//---------------------------------------------------
// fctname-array elements dedicated switch assignments:
// element:  0   1   2   3   4   5   6   7    8
// switch:  SWs SWl SB  S1  S2f S2r F12 Hold Fade
// s=short,l=long,f=forward,r=reverse.
// idle=ESU Full Throttle Drive Hold Function. I mapped it
// to center position of direction switch.
//-----------------------------------------------------------------------------------------
byte fctswtmap[numswfcs]; // mapping dedicated switches to functions
byte fctswtmapdft[] {2, 3, 5, 99, 4, 99, 6, 99, 99}; // default mapping switches to functions
byte sdhswtmap[numswfcs]; // mapping switches to double header functions
#define numfcts 29      // number of functions (0-28)
byte dblhdrfct; // for the one function that is sent also to the consist (double header) address
byte blkdfks[numfcts];  // blocked function keys while using dedicated function switches.
// routes >>>
#define maxnbrrts 64     // max number of routes (routes defined in JMRI)
byte rsosa[maxnbrrts];   // route sysname offsets array
byte ruosa[maxnbrrts];   // route usrname offsets array
byte numrtos = 0;        // route offset in array
byte numrts = 0;         // number of routes found
bool setroute = false;   // setting route or accessory decoder address
String rsysname;         // route system name
String rusrname;         // route user name
bool rts_Avail = false;  // routes found in JMRI
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
String tname = "WiCab1";   // throttle name (don't make it too long otherwise it may overlap).
String dsptname;
int cd = 60;  // timeout for the screen saver (in seconds)
int cb = 180; // time in seconds between battery monitoring
#define numrose  100    // max number of JMRI roster entries to be processed
byte numspst = 28;      // current max speed steps (from DCC)
byte spdstpm = 2;       // speed step mode
byte chgspstm = 0;      // change speed step mode
bool pushshort = false; // switch E pushed short = brake (300ms)
bool pushlong = false;  // switch E pushed long = change speed step mode
byte dbcdly = 100;      // debounce delay
#define fdelay 10
#define ldelay 350
#define sdelay 500      // waiting for jmri replies to arrive in throttle.   
#define mdelay 200      // momentary functions - delay off after on.    
/////////////////////////////////////////////////////////////////////////////////////////////
bool noop = false;
#define voltagepin 35  // voltage sensor (internal #35 = A13)
int batin;          // calc battery voltage
float battmp, batvolt;
/////////////////////////////////////////////////////////////////////////////////////////////
String MacString;     // MAC Address
String unixtime;      //hold unix time stamp from jmri
String unixsub;       //holding time substring
String ratiosub;      //holding fast clock ratio substring
int cntc = 0;         // counter for connecting to WiFi
int cnth;             // counter for the heart beat timer
byte ch;              // for the heartbeat timer (received from jmri)
int hbitv;            // heart beat interval from JMRI
String hbres;         // receive heart beat interval from server
bool hbuse = false;   // heart beat off (on)
float cntf;           // calc throttle heart beat interval
int cntd;             // counter for the screen saver
int cntb;             // counter for the battery monitor timer
bool dsplon = true;   // screen saver tracking
bool DEBUG = false;   // turn on/off serial monitor debug info on the fly
bool DEBUGENC = false;  // turn on/off encoder debug info
/////////////////////////////////////////////////////////////////////
int btnmin = 0;     // for finding min value
int btnmax = 0;     // for finding max value
int btnprvl = 5000; // previous low
int btnprvh = 0;    // previous high
int btnnum = 0;     // button number
/////////////////////////////////////////////////////////////////
IPAddress ip;         // the IP address of this device
byte mac[6];          // MAC Address of the throttle sent to server for identification
//
unsigned long prvsMs = 0;
unsigned long crntMs;
unsigned long startTime;
unsigned long dspvprvsMs = 0;
unsigned long dspvcrntMs;
bool drivehold = false;
bool driveholdON = false;
//
int btnread = 0;        // holding keypad button values
byte btnnbr;
int btncnt = 0;         // button count for entering a dcc address (up to 4 digits)
int dccadr = 0;         // DCC Address
int sdhadr = 0;         // simple double header address
int dccadrrlsd = 0;     // DCC address that was released #3
bool sdhdct = true;     // double header direction - true is same as lead
int dccadrtmp;
int lastdccadr = 0;
int dccstck[6];         // stack array;
bool nostck = false;    // save or not to save to stack
int btnval = 0;         // the number of the keypad button
char dspadr[6];         // for displaying DCCadr with leading zeros

int dspmin;             // for displaying fast clock minute with leading zero
int dsphour;            // for displaying fast clock hour with leading zero
String dsptime;

bool adrbtnstate = false;
bool setdccadr = false;
bool setsdhadr = false;
bool kpbtnstate = false;
bool kparrRead = false;
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
#define EEPROM_SIZE 96  // 0+1=dccadr, 2-13=stack. 20-84=keypad, 95=debug.

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
bool fctbtnstate = false;   // function group button
byte fcgstate = 0;          // function group state (0,1,2)
bool dctstate = true;       // direction. true is forward
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
byte lastfnum;
byte sfnum;         // fct nbr for sound fade
byte t_sfnum;       // trailer fct nbr for sound fade
bool fading;        // fading active/inactive in loco
bool t_fading;      // fading for trailer
bool fade = false;  // use fade
bool t_fade = false;
bool faded = false;  // fade on/off
bool t_faded = false;
byte fdfctst = 0;   //fade function state
byte t_fdfctst = 0;
byte fnumts;        // function number for toggle switch which could be on while other functions are handled
byte fn;            // fct nbr array pointer
byte rosnamoffsets[numrose]; // array for loco names from roster
byte rosadroffsets[numrose]; // array for loco addresses from roster

// function buttons 0-28:
byte fctarray[numfcts];
byte t_fctarray[numfcts];
byte dspfos[] = {30, 40, 50, 60, 70, 80, 90, 100, 110, 120}; // display functions offsets
String dspfnum[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9"};
String fctsarray;        //buffer to hold functions JMRI reply
bool fctsavail = false;
byte fctbtnst;           // function state (1=on, 0=off)
byte fctstate;           // function state feedback from jmri
float vtemp = 0;
byte vspeed = 0;
byte dspeed = 0;
byte prvspeed = 0;
bool encoder_moved = false; //encoder moved flag
volatile byte ecnt = 0;  // encoder counter
volatile byte pecnt = 0; // previous ecnt
volatile byte vcnt = 0;  // encoder step counter for speed
volatile byte rcnt = 0;  // encoder step counter for roster
volatile byte rtcnt = 0; // encoder step counter for routes
byte hcnt = 0;           // drive hold counter for locked speed
byte lastvcnt = 0;
int rxn, rxnn, numre, numos;
byte fxi, fxn, fxm, fxs, fxe, x;

String xmits;            //send buffer
String jmri_res;         //buffer to hold incoming response
String jmri_sub;         //buffer to hold substring of incoming response
String roster;           //buffer to hold roster list
String routes;           //buffer to hold route list
String rosne;            //to hold nbr of roster entries
bool ros_eAvail = false; // roster entry available
String loconame;   // loco name from server. should not exceed about 10 char.
String locoadr;    // display dcc adr
String locoacts;   // to hold an action string
String sdhacts;    // hold actions for the consist address
String lkf;        // to hold search argument for functions
String lkr;        // to hold search argument for direction
String lks;        // hold search argument for speed step mode
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
// encoder subroutine:----------------------------
void IRAM_ATTR encisr() {
  crntAState = digitalRead(encoderA);
  if (crntAState != prvsAState) {
    if (digitalRead(encoderB) != crntAState) {  //rotating cw
      ecnt++;
      if (vcnt < numspst) vcnt ++;  // count up speed steps
      rcnt++;                       // count up roster entries
      if (rcnt > numos) rcnt = 0;   // end of roster, back to beginning
      rtcnt++;                      // count up route entries
      if (rtcnt > numrtos) rtcnt = 0; // end of route entries
    }
    else {                         // rotating ccw
      ecnt--;
      if (vcnt > 0) vcnt --;       // count down speed steps
      rcnt --;                     // count down roster entries
      if ((rcnt < 0) || (rcnt > numos)) rcnt = numos;  // begin of roster, return to end
      rtcnt--;                     // count down route entries
      if ((rtcnt < 0) || (rtcnt > numrtos)) rtcnt = numrtos; // begin of route entries
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
  display.setFont(ArialMT_Plain_10);    // available sizes: 10,16,24
  display.drawString(0, 0, vrs);
  display.display();
  delay(2000);  // start serial monitor to catch startup.
  // start setup---- 
  for (int i = 0; i <= numfcts; i++) {
    fctarray[i] = 0;
  }
  for (int i = 0; i <= numfcts; i++) {
    t_fctarray[i] = 0;
  }
  for (int i = 0; i <= numfcts; i++) {
    blkdfks[i] = 0;
  }
  // ------------------------------------
  pinMode(switchWpin, INPUT_PULLUP);
  pinMode(switch1pin, INPUT_PULLUP);
  pinMode(switch2pin, INPUT_PULLUP);
  pinMode(switchFpin, INPUT_PULLUP);
  pinMode(switchRpin, INPUT_PULLUP);
  pinMode(switchBpin, INPUT_PULLUP);
  pinMode(switch3pin, INPUT_PULLUP);
  pinMode (encoderA, INPUT_PULLUP);
  pinMode (encoderB, INPUT_PULLUP);
  pinMode(switchEpin, INPUT_PULLUP);
#ifdef BATLEVEL
  pinMode(2, OUTPUT);
  pinMode(voltagepin, INPUT);
  digitalWrite(2, LOW);   // inactive
#endif
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  EEPROM.begin(EEPROM_SIZE);
  dccadr = EEPROM.read(0);
  dccadr = (dccadr << 8) + EEPROM.read(1);
  if ((dccadr <= 0 ) || (dccadr > 9999)) {  // as supported by Lenz LZV100 V3.6 and later.
    dccadr = 3;
    EEPROM.write(0, 0);
    EEPROM.write(1, 3);
    EEPROM.commit();
  }
  sprintf(dspadr, "%04d", dccadr);      // print with leading zeros
  DEBUG = EEPROM.read(95);
  if (DEBUG == true) {
    Serial.println("-----------------------------");
    Serial.println(vrs);
    Serial.println("Running in DEBUG mode.");
  }
  // retrieve stack:
  retrievestack();
  // -keypad config:-----------------------------------
if (DEBUG == true) {
  Serial.println("Get keypad values in EEPROM:");
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
  byte j = 20;
  for (byte i = 0; i < 32; i++) {
    keypadarray[i] = EEPROM.read(j);
    j++;
    keypadarray[i] = (keypadarray[i] << 8) + EEPROM.read(j);
    j++;
  }
if (DEBUG == true) {
  Serial.println("Keypad values in array: ");
  for (byte i = 0; i < 32; i++) {
    int val = keypadarray[i];
    Serial.print(val);
    Serial.print(",");
  }
  Serial.println();
}
  display.setColor(BLACK);
  display.fillRect(0, 0, 132, 15);
  display.setColor(WHITE);
  if (DEBUG == true) dsptname = tname + "d";
  else dsptname = tname;
  display.drawString(0, 0, dsptname);
  /* connecting to a WiFi network **************************************************/
  display.drawString(45, 0, "connecting WiFi");
  display.display();
if (DEBUG == true) {
  Serial.print("Connecting to WiFi");
}
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
if (DEBUG == true) {
    Serial.print('.');
}
    delay(1000);
    cntc++;
    if (cntc > 60) {
if (DEBUG == true) {
      Serial.println();
      Serial.println("WiFi connection failed");
}
      svr = false;
      WiFi.disconnect();
      display.setColor(BLACK);
      display.fillRect(45, 0, 92, 15);
      display.setColor(WHITE);
      display.drawString(45, 0, "WiFi offline");
      display.display();
if (DEBUG == true) {
      Serial.println("WiFi disconnected");
}
      goto bailout;
    }
  }
  display.setColor(BLACK);
  display.fillRect(45, 0, 132, 15);
  display.setColor(WHITE);
  display.drawString(45, 0, "WiFi connected");
  display.display();
if (DEBUG == true) {
  Serial.println("WiFi connected ");
  ip = WiFi.localIP();
  Serial.print("Client IP address: ");
  Serial.println(ip);
}
  MacString = WiFi.macAddress();
  delay(2000);
  display.clear();
  display.drawString(0, 0, dsptname + "  Svr: " + host);
  display.display();
if (DEBUG == true) {
  Serial.println("Connecting to server");
}
  cntc = 0;
  client.connect(host, port);
  while (!client.connect(host, port)) {
if (DEBUG == true) {
    Serial.print('.');
}
    delay(1000);
    cntc++;
    if (cntc > 60) {
if (DEBUG == true) {
      Serial.println();
      Serial.println("server connection failed");
}
      display.setColor(BLACK);
      //display.fillRect(38, 0, 95, 12);
      display.fillRect(45, 0, 95, 12);
      display.setColor(WHITE);
      display.drawString(45, 0, "svr offline");
      display.display();
      WiFi.disconnect();
if (DEBUG == true) {
      Serial.println("Throttle offline");
      Serial.println("WiFi disconnected");
}
      svr = false;
      //return;
      goto bailout;      // continue without WiFi for testing
    }
  }
if (DEBUG == true) {
  Serial.println("Sending N" + tname);
}
  client.print("N" + tname + "\r\n");       // set Name of Throttle
  client.flush();
  delay(sdelay);
  while (client.available()) {
    jmri_res = client.readStringUntil('\n');
if (DEBUG == true) {
    Serial.print("response from server: ");
    Serial.println(jmri_res);
}
    if (jmri_res.startsWith("RL")) {
      roster = jmri_res;                // save roster
    }
    if (jmri_res.startsWith("PRL")) {
      routes = jmri_res;                // save route list
      rts_Avail = true;
    }
    if (jmri_res.startsWith("*")) {
if (DEBUG == true) {
      Serial.print("heart beat from server: ");
      Serial.println(jmri_res);
}
      hbres = jmri_res;                // save heartbeat value
      String hbs = hbres.substring(1, 3);   // get heart beat interval
      hbitv = hbs.toInt();
if (DEBUG == true) {
      Serial.print("heart beat itv response: ");
      Serial.println(hbitv);
}
      if (hbitv > 5) {
        cntf = hbitv / 1.5;
        ch = (int) cntf;
        hbuse = true;
      }
    }
  }
  if (hbuse == false) ch = 12;  //value needed for checking connection status.
  cnth = ch;
if (DEBUG == true) {
  Serial.println("heart beat interval: " + String(ch));
  Serial.println("Sending HU" + MacString);
}
  client.print("HU" + MacString + "\r\n");  // set identifier
  client.flush();
  delay(fdelay);
  /**end of WiFi connection startup ***********************************************/
if (DEBUG == true) {
  Serial.println("roster list: " + roster);
  Serial.println("route list: " + routes);
}
  //----build index of roster entries----:
  rxn = roster.indexOf('L');      // find first roster entry for nbr of entries
  rxn = rxn + 1;
  rxnn = roster.indexOf(']');
  rosne = roster.substring(rxn, rxnn);      // get nbr of roster entries
  numre = rosne.toInt();                    // convert to integer
  if (numre > 0) {                          // making sure we got a working value
    numos = numre - 1;                      // offsets start with 0
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
  }
  //----build index of route entries---->>>:
  if (rts_Avail == true) {
    rxnn = 0;
    rxn = 0;
    // PRL]\[IO...}|{usrname}|{
    for (byte i = 0; i < maxnbrrts; i++) {
      rxn = routes.indexOf("IO", rxnn); // count sysname entries
      if (rxn == -1) break;
      numrts++;
      rxn = rxn + 2;
      rxnn = rxn + 1;
    }
if (DEBUG == true) {
    Serial.print("Routes counted: ");
    Serial.println(numrts);
}
    if (numrts > 0) {
      rxnn = 0;
      numrtos = numrts - 1;
      for (byte i = 0; i < numrts; i++) {
        rxn = routes.indexOf("IO", rxnn);
        rsosa[i] = rxn;   // store index of sysname
        rxnn = rxn + 1;
        int rxa = routes.indexOf('{', rxnn);
        rxa++;
        ruosa[i] = rxa;   // store index of usrname
        rxnn = rxa + 1;
      }
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
    }
  }
  /*--------------------------------------------------------*/
  rtvrosnam();
  if (ros_eAvail == true) getrosnamen();
  display.setFont(ArialMT_Plain_10);
  display.setColor(BLACK);
  //display.fillRect(38, 0, 95, 12);
  display.fillRect(43, 0, 95, 12);
  display.setColor(WHITE);
  display.drawString(45, 0, "online");
  display.display();

bailout:   // continue without server connection for testing
  //display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(80, 13, String(dspadr));  // dccadr with leading zero's
  display.drawString(120, 13, "?");
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
if (DEBUG == true) {
    Serial.println("Starting *+ heart beat");
}
    client.print("*+\r\n");    // starting heartbeat
    client.flush();
  }
  cntd = cd;
  cntb = cb;
#ifdef BATLEVEL
  getbatlvl();
#endif
  //...............................................
  prvsAState = digitalRead(encoderA); // Read the "initial" state of encoderA
  attachInterrupt(encoderA, encisr, FALLING);
  //...............................
}
/*************************************************************************************/
void loop() {
  if (Serial.available() > 0) {
    int indata = Serial.read();
    if ((indata == 'C') || (indata == 'c')) {
      keypad_check();
      keypad_config();
    }
    if ((indata == 'D') || (indata == 'd')) {
      DEBUG = !DEBUG;
      if (DEBUG == true) {
        EEPROM.write(95,DEBUG); // keep beyond pwr off
        EEPROM.commit();
        Serial.println("Running in DEBUG mode.");
        dsptname = tname + "d";
        display.setColor(BLACK);
        display.fillRect(0, 0, 45, 15);
        display.setColor(WHITE);
        display.drawString(0, 0, dsptname);
        display.display();
      }
      if (DEBUG == false) {
        EEPROM.write(95,DEBUG); // keep beyond pwr off
        EEPROM.commit();
        Serial.println("DEBUG mode ended.");
        dsptname = tname;
        display.setColor(BLACK);
        display.fillRect(0, 0, 45, 15);
        display.setColor(WHITE);
        display.drawString(0, 0, dsptname);
        display.display();
      }
    }
    if ((indata == 'E') || (indata == 'e')) {
      DEBUGENC = !DEBUGENC;
      if (DEBUGENC == true) Serial.println("Running in Encoder DEBUG mode.");
      if (DEBUGENC == false) Serial.println("Encoder DEBUG mode ended.");
    }
  }
#ifdef CLOCK
  // look for fast clock time stamp from jmri
  if (svr == true) {
    if (client.available()) {
      jmri_res = client.readStringUntil('\n');
      if (jmri_res.startsWith("PFT")) {
        display.setColor(BLACK);
        //display.fillRect(43, 0, 95, 12);
        display.display();
        getjmritime();
      }
    }
  }
#endif
  // heartbeat:
  crntMs = millis();
  if (crntMs - prvsMs >= 1000) {     // 1 second interval timer
    prvsMs = crntMs;
    cnth--;
    cntd--;
    cntb--;                          // battery monitor
  }
  if (cnth <= 0) {                   // heartbeat time
    // still connected?:
    if ((!client.connected()) && (svr == true)) {
if (DEBUG == true) {
      Serial.println();
      Serial.println("Throttle offline.");
}
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
if (DEBUG == true) {
        Serial.print("Attempting to reconnect WiFi.");
}
        cntc = 0;
        recon = true;
        WiFi.begin(ssid, pass);
        while (WiFi.status() != WL_CONNECTED) {
if (DEBUG == true) {
          Serial.print('.');
}
          delay(1000);
          cntc++;
          if (cntc > 60) {
if (DEBUG == true) {
            Serial.println();
            Serial.println("WiFi connection failed");
}
            return;
          }
        }
if (DEBUG == true) {
        Serial.println();
        Serial.print("Connecting to server");
}
        display.setColor(BLACK);
        display.fillRect(43, 0, 95, 12);
        display.setColor(WHITE);
        display.drawString(45, 0, "reconnecting");
        display.display();
        cntc = 0;
        client.connect(host, port);
        while (!client.connect(host, port)) {
if (DEBUG == true) {
          Serial.print('.');
}
          delay(1000);
          cntc++;
          if (cntc > 120) {
if (DEBUG == true) {
            Serial.println();
            Serial.println("server reconnect failed");
}
            display.setColor(BLACK);
            display.fillRect(43, 0, 95, 12);
            display.setColor(WHITE);
            display.drawString(45, 0, "reconnect failed");
            display.display();
            WiFi.disconnect();
if (DEBUG == true) {
            Serial.println("Throttle offline");
            Serial.println("WiFi disconnected");
}
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
if (DEBUG == true) {
        Serial.println();
        Serial.println("Sending HU" + MacString);
}
        client.print("N" + tname + "\r\n");       // set Name of Throttle
        delay(fdelay);
        client.print("HU" + MacString + "\r\n");  // set identifier
        delay(ldelay);
        recon = false;
      }
      if (hbuse == true) {
        client.print("*+\r\n");    // starting heartbeat
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
      client.print("*\r\n");       // send heart beat
    }
    cnth = ch;
  }
  if ((cntd <= 0) && (dsplon == true)) {     // time for turning display dark
    display.displayOff();
    dsplon = false;
if (DEBUG == true) {
    Serial.println("display OFF");
}
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
      encoder_moved = false;
    }
    // reset estop in case it was -----------------------
    if (estop == true) {
      if (dctstate == true) {
        set_forward();
      }
      else {
        set_reverse();
      }
      estop = false;
      display.setFont(ArialMT_Plain_16);
      display.setColor(BLACK);
      display.fillRect(80, 30, 70, 18);    // remove STOP from display
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
}
    //speed:----------------------------
    if ((setdccadr == false) && (setaccadr == false)) {
      set_speed();
    }
    // ----roster--------------------------------------------
    if ((setdccadr == true) && (setaccadr == false)) {
      getrosnam(rcnt);
      dsprosnam();
      getrosadr(rcnt);
      dsprosadr();
      dspcnt(rcnt);
if (DEBUG == true) {
      Serial.print("roster offset: ");
      Serial.println(rcnt);
      Serial.print("name from roster: ");
      Serial.println(loconame);
      Serial.print("address from roster: ");
      Serial.println(locoadr);
}
    }
    //-----------routes---------------------
    if ((setdccadr == false) && (setaccadr == true) && (rts_Avail == true)) {
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
  }
  // dedicated switches: ////////////////////////////
  swtWstate = digitalRead(switchWpin);        // whistle
  if (swtWstate != lastswtWstate) {
    if (dsplon == false) {
      dspreset();
      delay(500);
      return;
    }
    else {
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
      fnum = fctswtmap[1];    // swtmap element 1 points to function "long whistle"
      if (fnum < numfcts) blkdfks[fnum] = 1;      // disable Function key
      fctbtnst = 1;
      sendfct(fnum);
if (DEBUG == true) {
      Serial.println("long whistle on");
}
    }
  }
  if ((swtWstate == HIGH) && (swtWon == true)) {
    swtWon = false;             // turn short whistle on & off, or long whistle off.
    if (shortwhstl == true) {
      fnum = fctswtmap[0];      // swtmap element 0 points to function "short whistle"
      if (fnum < numfcts) blkdfks[fnum] = 1;        // disable Function key
      fctbtnst = 1;
      shortwhstl = false;
if (DEBUG == true) {
      Serial.println("short whistle on");
}
      sendfct(fnum);
      delay(mdelay);
      fctbtnst = 0;
      sendfct(fnum);
      if (fnum < numfcts) blkdfks[fnum] = 0;
if (DEBUG == true) {
      Serial.println("short whistle off");
}
    }
    if (longwhstl == true) {
      fnum = fctswtmap[1];    // swtmap element 1 points to function "long whistle"
      if (fnum < numfcts) blkdfks[fnum] = 1;      // disable Function key
      fctbtnst = 0;
      sendfct(fnum);
      if (fnum < numfcts) blkdfks[fnum] = 0;
      longwhstl = false;
if (DEBUG == true) {
      Serial.println("long whistle off");
}
    }
  }
  //-------------------------------------------------------
  swtBstate = digitalRead(switchBpin);
  if (swtBstate != lastswtBstate) {
    if (dsplon == false) {
      lastswtBstate = swtBstate;
      dspreset();
      return;
    }
    else {
      cntd = cd;
    }
    if ((swtBstate == LOW) && (swtBon == false)) {
      swtBon = true;
if (DEBUG == true) {
      Serial.println("Switch B on");
}
      fnumts = fctswtmap[2];         // swtmap element 2 points to function "Bell"
      if (fctarray[fnumts] == 1) {
if (DEBUG == true) {
        Serial.println("Switch blocked, function is on");
}
        return;
      }
      if (fnum < numfcts) blkdfks[fnum] = 1;      // disable Function key
      fctbtnst = 1;
      sendfct(fnumts);
      delay(mdelay);
      fctbtnst = 0;         // simulate button
      sendfct(fnumts);
if (DEBUG == true) {
      Serial.println("Switch B fct on");
}
    }
    if ((swtBstate == HIGH) && (swtBon == true)) {
      swtBon = false;
if (DEBUG == true) {
      Serial.println("Switch B off");
}
      fctbtnst = 1;
      sendfct(fnumts);
      delay(mdelay);
      fctbtnst = 0;         // simulate momentary
      sendfct(fnumts);
      if (fnumts < numfcts) blkdfks[fnumts] = 0;
if (DEBUG == true) {
      Serial.println("Switch B fct off");
}
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
    }
    else {
      cntd = cd;
    }
    if ((swt2state == LOW) && (swt2on == false)) {
      swt2on = true;
      if ((dctstate == false) && (fctswtmap[5] != 99)) fnum = fctswtmap[5];  // map 5 = front coupler
      if ((dctstate == false) && (fctswtmap[5] == 99)) fnum = fctswtmap[4];  // map 4 = rear coupler
      if (dctstate == true) fnum = fctswtmap[4];  // swtmap element 4 points to function "Coupler" (rear)
      if (fnum < numfcts) blkdfks[fnum] = 1;      // disable Function key
      fctbtnst = 1;
if (DEBUG == true) {
      Serial.println("Switch 2 - coupler fct on");
}
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
      fctbtnst = 0;
if (DEBUG == true) {
      Serial.println("Switch 2 - coupler fct off");
}
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
    }
    else {
      cntd = cd;
    }
    //--------------------------------------
    if ((swt1state == LOW) && (swt1on == false)) {
      swt1on = true;
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
    if ((swt1state == HIGH) && (swt1on == true)) {
      swt1on = false;
      if (fctswtmap[3] != 99) {
        fnum = fctswtmap[3];
        fctbtnst = 0;
        sendfct(fnum);
      }
    }
    lastswt1state = swt1state;
  }
  //---Switch 3: Brake/Stop (not Estop) --------------------------------------------
  swt3state = digitalRead(switch3pin);
  if (swt3state != lastswt3state) {
    if (dsplon == false) {
      dspreset();
      //delay(500);
      return;
    }
    else {
      cntd = cd;
    }
    lastswt3state = swt3state;
  }
  if ((swt3state == LOW) && (pushlong == false)) {
    if (swt3on == false) startTime = millis();
    swt3on = true;
    pushshort = true;
    if (((millis() - startTime) >= 1000) && (pushlong == false)) {
      pushlong = true;
      pushshort = false;
if (DEBUG == true) {
      if (vspeed != 0) Serial.println("long push ignored, speed is not 0)");
}
      if ((svr == true) && (vspeed == 0)) {
        set_spstm();
if (DEBUG == true) {
        Serial.println("long push - change speed step mode");
}
      }
    }
  }
  if ((swt3state == HIGH) && (swt3on == true)) {
    swt3on = false;
    pushlong = false;
    if (pushshort == true) {
      vspeed = 0;
      vtemp = 0;
      vcnt = 0;
      dspeed = vcnt;
      lastvcnt = 0;
      set_speed();
      pushshort = false;
if (DEBUG == true) {
      Serial.println("short push - brake");
}
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
    }
    else {
      cntd = cd;
    }
    if ((swtFstate == LOW) && (swtRstate == HIGH) && (setdccadr == false)) {
      if (dctstate == false) {
        vcnt = 0;
        set_speed();
      }
      if (dctstate == false) set_forward();
      dctstate = true;
      if (driveholdON == true) {
        fnum = fctswtmap[7];
        fctbtnst = 0;
        sendfct(fnum);
        if (fnum < numfcts) blkdfks[fnum] = 0;
        driveholdON = false;
if (DEBUG == true) {
        Serial.println("Drive hold off");
}
        display.setFont(ArialMT_Plain_10);
        display.setColor(BLACK);
        display.fillRect(50, 18, 17, 10);
        display.display();
      }
      swtCon = false;
if (DEBUG == true) {
      Serial.println("Switch C off");
}
    }
    lastswtFstate = swtFstate;
  }
  if ((swtRstate != lastswtRstate)) {
    if (dsplon == false) {
      lastswtRstate = swtRstate;
      dspreset();
      return;
    }
    else {
      cntd = cd;
    }
    if ((swtRstate == LOW) && (swtFstate == HIGH) && (setdccadr == false)) {
      if (dctstate == true) {
        vcnt = 0;
        set_speed();
      }
      if (dctstate == true) set_reverse();
      dctstate = false;
      if (driveholdON == true) {
        fnum = fctswtmap[7];
        fctbtnst = 0;
        sendfct(fnum);
        if (fnum < numfcts) blkdfks[fnum] = 0;
        driveholdON = false;
if (DEBUG == true) {
        Serial.println("Drive hold off");
}
        display.setFont(ArialMT_Plain_10);
        display.setColor(BLACK);
        display.fillRect(50, 18, 17, 10);
        display.display();
      }
      swtCon = false;
if (DEBUG == true) {
      Serial.println("Switch C off");
}
    }
    lastswtRstate = swtRstate;
  }
  // Function for Drive Hold >
  if (drivehold == true) {
    if ((swtFstate == HIGH) && (swtRstate == HIGH) && (setdccadr == false) && (driveholdON == false)) {
      if (swtCon == false) {
        startTime = millis();
        swtCon = true;
if (DEBUG == true) {
        Serial.println("Switch C on");
}
      }
      if ((millis() - startTime) > 1000) {
        driveholdON = true;
        hcnt = vcnt;
        fnum = fctswtmap[7];
        if (fnum < numfcts) blkdfks[fnum] = 1; // disable Function key
        fctbtnst = 1;
        sendfct(fnum);
if (DEBUG == true) {
        Serial.println("Drive hold on");
}
        display.setFont(ArialMT_Plain_10);
        display.setColor(BLACK);
        display.fillRect(50, 18, 17, 10);
        display.setColor(WHITE);
        sprintf(dspadr, "%3d", hcnt);
        display.drawString(50, 18, String(dspadr));
        display.display();
      }
    }
  }
  //----Switch E: E-Stop----------------------------------------------
  swtEstate = digitalRead(switchEpin);
  if ((swtEstate != lastswtEstate)) {
    if (dsplon == false) {        //turn Display back on if off
      dspreset();
      return;
    } else {
      cntd = cd;
    }
    lastswtEstate = swtEstate;
  }
  if ((swtEstate == LOW) && (pushlong == false) && (setdccadr == false)) {
    if (swtEon == false) startTime = millis();
    swtEon = true;
    pushshort = false;
    if (estop == false) {
      vspeed = 0;
      vcnt = 0;
      lastvcnt = 0;
      dspeed = vcnt;
      set_speed();
if (DEBUG == true) {
      Serial.println(locoacts + "X");
}
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
    }
    if (((millis() - startTime) >= 1000) && (pushlong == false)) {
      pushlong = true;
      pushshort = false;
if (DEBUG == true) {
      if (vspeed != 0) Serial.println("long push ignored, speed is not 0)");
}
      if ((svr == true) && (vspeed == 0)) {
        set_spstm();
if (DEBUG == true) {
        Serial.println("long push - change speed step mode");
}
        display.setFont(ArialMT_Plain_16);
        display.setColor(BLACK);
        display.fillRect(80, 30, 70, 18);    // remove STOP from display
        display.display();
      }
    }
  }
  if ((swtEstate == LOW) && (setdccadr == true)) {
    actdccadr();
    adrbtnstate = !adrbtnstate;
  }
  if ((swtEstate == HIGH) && (swtEon == true)) {
    swtEon = false;
    pushlong = false;
    pushshort = false;
  }
  //---------------------------------------------------------------------
  // read keypad:
  btnread = analogRead(keypadin);
  if (btnread > 10) {
    delay(dbcdly);
    btnread = analogRead(keypadin);
    kparrRead = true;  //button pressed - read it
  } else {
    kparrRead = false;
  }
  // Button 1: ***********************************
  if (kparrRead == true) {  //find pressed button
    if ((btnread >= keypadarray[0]) && (btnread <= keypadarray[1])) {
      if (dsplon == false) {if ((btnread >= keypadarray[0]) && (btnread <= keypadarray[1]))
        kpbtnstate = true;
        dspreset();
        return;
      }
      else {
        cntd = cd;
      }
      btnnbr = 1;
      dspbtnread();
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
      // set function:
      if ((setdccadr == false) && (kpbtnstate == false)) {
        if (fc1btnstate == false) {
          fc1btnstate = true;
          fctbtnst = 1;
          if ((fading == true) && (vspeed > 0)) {
            fade = true;  // button used for fading
            sendfct(sfnum);  //send 'fade btn on' (F1xx)
            fdfctst = fctarray[sfnum]; // get fade function state
            if (fdfctst == 1) {
              faded = true;  // sound is faded
              display.setFont(ArialMT_Plain_16);
              display.setColor(BLACK);
              display.fillRect(70, 13, 10, 15); //clear +
              display.setFont(ArialMT_Plain_10);
              display.setColor(WHITE);
              display.drawString(70, 18, "F");
              display.display();
if (DEBUG == true) {
              Serial.println("Sound is faded");
}
            }
            else {
              faded = false;
              display.setFont(ArialMT_Plain_16);
              display.setColor(BLACK);
              display.fillRect(70, 13, 10, 15);
              if (setsdhadr == true) {
                display.setColor(WHITE);
                display.drawString(70, 13, "+");
              }
              display.display();
if (DEBUG == true) {
              Serial.println("Sound is not faded");
}
            }
          }
          if ((faded == true) && (vspeed == 0)) {
            sendfct(sfnum);  //send 'fade btn on' (F1xx)
            fdfctst = fctarray[sfnum]; // get fade function state
            if (fdfctst == 1) {
              faded = true;
if (DEBUG == true) {
              Serial.println("Sound faded turned on");
}
            }
            else {
              faded = false;
              display.setFont(ArialMT_Plain_16);
              display.setColor(BLACK);
              display.fillRect(70, 13, 10, 15);
              if (setsdhadr == true) {
                display.setColor(WHITE);
                display.drawString(70, 13, "+");
              }
              display.display();
if (DEBUG == true) {
              Serial.println("Sound faded turned off");
}
            }
          }
          if ((fdfctst == 0) && (fcgstate == 0)) {
            if (fade == false) {
              fnum = 1;
              sendfct(fnum);   //send F11
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
            t_fade = true;  // button used for fading
            sendsdhfct(t_sfnum);  //send 'fade btn on' (F1xx)
            if (setsdhadr == true) sendsdhfct(t_sfnum); //#1
            t_fdfctst = t_fctarray[t_sfnum]; // get fade function state
            if (t_fdfctst == 1) {
              t_faded = true;
            }
            else {
              t_faded = false;
            }
          }
          if ((t_faded == true) && (vspeed == 0)) {
            sendsdhfct(t_sfnum);  //send 'trailer fade btn on' (F1xx)
            if (setsdhadr == true) sendsdhfct(t_sfnum); //#1
            t_fdfctst = t_fctarray[t_sfnum]; // get fade function state
            if (t_fdfctst == 1) {
              t_faded = true;
            }
            else {
              t_faded = false;
            }
          }
          if (t_fdfctst == 0) {
            if (t_fade == false) {
              fnum = 1;
              //#1 sendsdhfct(fnum);   //send F11
              if (setsdhadr == true) sendsdhfct(t_sfnum); //#1
            }
          }
        }
        kpbtnstate = true;
      }
    }
    // Button 2: ***************************************
    else if ((btnread >= keypadarray[2]) && (btnread <= keypadarray[3])) {
      if (dsplon == false) {
        kpbtnstate = true;
        dspreset();
        return;
      }
      else {
        cntd = cd;
      }
      btnnbr = 2;
      dspbtnread();
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
      if ((setdccadr == false) && (kpbtnstate == false)) { // set funcion
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
      }
    }
    // Button 3: ***************************************
    else if ((btnread >= keypadarray[4]) && (btnread <= keypadarray[5])) {
      if (dsplon == false) {
        kpbtnstate = true;
        dspreset();
        return;
      }
      else {
        cntd = cd;
      }
      btnnbr = 3;
      dspbtnread();
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
      if ((setdccadr == false) && (kpbtnstate == false)) { // set funcion
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
      }
    }
    // Button 4: ***************************************
    else if ((btnread >= keypadarray[6]) && (btnread <= keypadarray[7])) {
      if (dsplon == false) {
        kpbtnstate = true;
        dspreset();
        return;
      }
      else {
        cntd = cd;
      }
      btnnbr = 4;
      dspbtnread();
      if ((setdccadr == true) || (accbtnstate == 1)) {
        if ((btncnt < 5) && (kpbtnstate == false)) {
          btncnt++;
          btnval = 4;
          if (setaccadr == false) putdgt();
          if (setaccadr == true) putaccdgt();
          kpbtnstate = true;
        }
      }
      if ((setdccadr == false) && (kpbtnstate == false)) { // set function
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
      }
    }
    // Button 5: ***************************************
    else if ((btnread >= keypadarray[8]) && (btnread <= keypadarray[9])) {
      if (dsplon == false) {
        kpbtnstate = true;
        dspreset();
        return;
      }
      else {
        cntd = cd;
      }
      btnnbr = 5;
      dspbtnread();
      if ((setdccadr == true) || (accbtnstate == 1)) {
        if ((btncnt < 5) && (kpbtnstate == false)) {
          btncnt++;
          btnval = 5;
          if (setaccadr == false) putdgt();
          if (setaccadr == true) putaccdgt();
          kpbtnstate = true;
        }
      }
      if ((setdccadr == false) && (kpbtnstate == false)) { // set function
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
      }
    }
    // Button 6: ***************************************
    else if ((btnread >= keypadarray[10]) && (btnread <= keypadarray[11])) {
      if (dsplon == false) {
        kpbtnstate = true;
        dspreset();
        return;
      }
      else {
        cntd = cd;
      }
      btnnbr = 6;
      dspbtnread();
      if ((setdccadr == true) || (accbtnstate == 1)) {
        if ((btncnt < 5) && (kpbtnstate == false)) {
          btncnt++;
          btnval = 6;
          if (setaccadr == false) putdgt();
          if (setaccadr == true) putaccdgt();
          kpbtnstate = true;
        }
      }
      if ((setdccadr == false) && (kpbtnstate == false)) { // set function
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
      }
    }
    // Button 7: ***************************************
    else if ((btnread >= keypadarray[12]) && (btnread <= keypadarray[13])) {
      if (dsplon == false) {
        kpbtnstate = true;
        dspreset();
        return;
      }
      else {
        cntd = cd;
      }
      btnnbr = 7;
      dspbtnread();
      if ((setdccadr == true) || (accbtnstate == 1)) {
        if ((btncnt < 5) && (kpbtnstate == false)) {
          btncnt++;
          btnval = 7;
          if (setaccadr == false) putdgt();
          if (setaccadr == true) putaccdgt();
          kpbtnstate = true;
        }
      }
      if ((setdccadr == false) && (kpbtnstate == false)) { // set function
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
      }
    }
    // Button 8: ***************************************
    else if ((btnread >= keypadarray[14]) && (btnread <= keypadarray[15])) {
      if (dsplon == false) {
        kpbtnstate = true;
        dspreset();
        return;
      }
      else {
        cntd = cd;
      }
      btnnbr = 8;
      dspbtnread();
      if ((setdccadr == true) || (accbtnstate == 1)) {
        if ((btncnt < 5) && (kpbtnstate == false)) {
          btncnt++;
          btnval = 8;
          if (setaccadr == false) putdgt();
          if (setaccadr == true) putaccdgt();
          kpbtnstate = true;
        }
      }
      if ((setdccadr == false) && (kpbtnstate == false)) { // set function
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
      }
    }
    // Button 9: ***************************************
    else if ((btnread >= keypadarray[16]) && (btnread <= keypadarray[17])) {
      if (dsplon == false) {
        kpbtnstate = true;
        dspreset();
        return;
      }
      else {
        cntd = cd;
      }
      btnnbr = 9;
      dspbtnread();
      if ((setdccadr == true) || (accbtnstate == 1)) {
        if ((btncnt < 5) && (kpbtnstate == false)) {
          btncnt++;
          btnval = 9;
          if (setaccadr == false) putdgt();
          if (setaccadr == true) putaccdgt();
          kpbtnstate = true;
        }
      }
      if ((setdccadr == false) && (kpbtnstate == false)) { // set function
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
      }
    }
    // Button 10 (function group/Acc dec close): *******
    else if ((btnread >= keypadarray[18]) && (btnread <= keypadarray[19])) {
      if (dsplon == false) {
        kpbtnstate = true;
        dspreset();
        return;
      }
      else {
        cntd = cd;
      }
      btnnbr = 10;
      dspbtnread();
      // send "close" to accessory decoder 12/13>------------------------
      if ((kpbtnstate == false) && (accbtnstate == 2) && (setaccadr == true)) {
        kpbtnstate = true;
        setacc = "C";
        display.setFont(ArialMT_Plain_16);
        display.setColor(BLACK);
        display.fillRect(15, 30, 60, 18);
        display.setColor(WHITE);
        display.drawString(15, 30, "-/cls");
        display.display();
        if (svr == true) sendaccadr();
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
      }
      else {
        cntd = cd;
      }
      btnnbr = 11;
      dspbtnread();
      // send "toggle" to accessory decoder >------------------------
      if ((kpbtnstate == false) && (accbtnstate == 2) && (setaccadr == true)) {
        kpbtnstate = true;
        setacc = "2";
        display.setFont(ArialMT_Plain_16);
        display.setColor(BLACK);
        display.fillRect(15, 30, 60, 18);
        display.setColor(WHITE);
        display.drawString(15, 30, "tgl");
        display.display();
        if (svr == true) sendaccadr();
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
      // set function: ------------------------------------------
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
      }
    }
    // Button 12 (shunting/switch direction of consist/acc dec throw): *****
    else if ((btnread >= keypadarray[22]) && (btnread < keypadarray[23])) {
      if (dsplon == false) {
        kpbtnstate = true;
        dspreset();
        return;
      }
      else {
        cntd = cd;
      }
      btnnbr = 12;
      dspbtnread();
      // send "thrown" to accessory decoder >------------------------
      if ((kpbtnstate == false) && (accbtnstate == 2) && (setaccadr == true)) {
        kpbtnstate = true;
        setacc = "T";
        display.setFont(ArialMT_Plain_16);
        display.setColor(BLACK);
        display.fillRect(15, 30, 60, 18);
        display.setColor(WHITE);
        display.drawString(15, 30, "+/thw");
        display.display();
        if (svr == true) sendaccadr();
      }
      // mapped function:
      if ((kpbtnstate == false) && (setsdhadr == false) && (setaccadr == false)) {
        kpbtnstate = true;
        fnum = fctswtmap[6];    // swtmap element 6 points to function "Shunting"
        fctbtnst = 1;
if (DEBUG == true) {
        Serial.println("Button 12 pushed for mapped function.");
}
        sendfct(fnum);
        delay(mdelay);
        fctbtnst = 0;
if (DEBUG == true) {
        Serial.println("Button 12 released for mapped function.");
}
        sendfct(fnum);
      }
      // change direction of consist:
      if ((kpbtnstate == false) && (setsdhadr == true) && (setaccadr == false)) {
        kpbtnstate = true;
        sdhdct = !sdhdct;
if (DEBUG == true) {
        if (sdhdct == true) Serial.println("sdhdct = true");
        if (sdhdct == false) Serial.println("sdhdct = false");
}
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
      dspbtnread();
      if (kpbtnstate == false) {
        //abort route selection>>
        if ((setroute == true) || (setaccadr == true)) {
          setroute = false;
          setaccadr = false;
          clrrosnam();
          dsprosnam();
          dsprosadr();
          accbtnstate = 0;
          display.setFont(ArialMT_Plain_10);
          display.setColor(BLACK);
          display.fillRect(50, 18, 17, 10);  //remove display of rtcnt.
          display.display();
        }
        adrbtnstate = !adrbtnstate;
        if ((adrbtnstate == true) && (vspeed != 0)) adrbtnstate = !adrbtnstate;
        if ((adrbtnstate == true) && (vspeed == 0)) {
          clrchgadr();
          btncnt = 0;
          setdccadr = true;
if (DEBUG == true) {
          Serial.print("lastdccadr ");
          Serial.println(lastdccadr);
          Serial.print("dccadr ");
          Serial.println(dccadr);
}
          lastdccadr = dccadr;
          display.setColor(WHITE);
          display.drawString(80, 13, String(dspadr));
          display.drawString(120, 13, "?");
          display.display();
        }
        else {
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
      dspbtnread();
      // accessory decoder
      if (kpbtnstate == false) {
        kpbtnstate = true;
        if ((setroute == false) && (setdccadr == false)) {
          accbtnstate++;   // 1 = address mode, 2 = action, 0 = off
if (DEBUG == true) {
          Serial.print("accbtnstate: ");
          Serial.println(accbtnstate);
}
          if (accbtnstate ==  1) { // set accessory decoder address mode
            btncnt = 0;
            setaccadr = true;     // ready to enter "C","2","T" via kp-buttons 10,11,12.
            display.setFont(ArialMT_Plain_16);
            display.setColor(BLACK);
            display.fillRect(0, 30, 132, 18);     // remove locoadr and loconame
            display.setColor(WHITE);
            display.drawString(0, 30, "T");
            sprintf(dspaccadr, "%04d", accadr);
            display.drawString(80, 30, String(dspaccadr));
            display.drawString(120, 30, "?");
            display.display();
if (DEBUG == true) {
            Serial.println("1:Set accessory decoder address mode");
}
          }
          if (accbtnstate == 2) {          // ready for setting turnout etc.
            if (accadr != 0) {  // set address
              display.setColor(BLACK);
              display.fillRect(120, 30, 10, 15);   // remove "?"
              display.display();
if (DEBUG == true) {
              Serial.print("2:Accessory decoder address: ");
              Serial.println(accadr);
}
            }
            if (accadr == 0) {
              dsprosnam();
              dsprosadr();
              display.setFont(ArialMT_Plain_10);
              display.setColor(BLACK);
              display.fillRect(50, 18, 17, 10);
              display.display();
if (DEBUG == true) {
              Serial.println("Nothing entered");
}
            }
          }
          if (accbtnstate == 3) {
            clrrosnam();
            dsprosnam();
            dsprosadr();
            accbtnstate = 0;
            setaccadr = false;
if (DEBUG == true) {
            Serial.println("3:Set accessory decoder address mode OFF");
}
          }
        }
        if (setroute == true) {  //------setroute------
          accbtnstate = 0;
if (DEBUG == true) {
          Serial.print("Setting route: ");
          Serial.println(rsysname);
}
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
      }
      else {
        cntd = cd;
      }
      btnnbr = 15;
      dspbtnread();
      // rls consist address: ----------------------------------------------
      if ((setsdhadr == true) && (setdccadr == false) && (kpbtnstate == false)) {
        kpbtnstate = true;
        if (sdhadr != 0) {
          releaseadr(sdhadr);
          sdhadr = 0;
          display.setFont(ArialMT_Plain_16);
          display.setColor(BLACK);
          display.fillRect(70, 13, 10, 15);
          display.display();
        }
        setsdhadr = false;
        t_fading = false; //2.7.19
      }
      // set consist address: -----------------------------------------------
      if ((setsdhadr == false) && (setdccadr == false) && (kpbtnstate == false)) {
        kpbtnstate = true;
        if (dccadr != 0) {
          setsdhadr = true;
          clrrosnam;
          rtvrosnam();
          sdhadr = dccadr;
          // copy fctswtmap to sdhswtmap:
          for (int i = 0; i < numswfcs; i++) {
            sdhswtmap[i] = fctswtmap[i];
          }
          // 2.7.19 copy fctarray to t_fctarray:
          for (int i = 0; i < numfcts; i++) {
            t_fctarray[i] = fctarray[i];
          }
          t_sfnum = sfnum;
if (DEBUG == true) {
          Serial.println("loconame is " + loconame);
          Serial.println("sdhadr is " + String(sdhadr));
}
          sendsdhadr();
          if (fading == true) t_fading = true; //2.7.19
          display.setFont(ArialMT_Plain_16);
          display.setColor(WHITE);
          display.drawString(70, 13, "+");
          display.display();
        }
      }
      // retrieve dcc address from stack array-------------------------
      if ((setdccadr == true) && (kpbtnstate == false)) {
        kpbtnstate = true;
        if (x > 5) x = 0;
        dspcnt(x);  // display counter
        dccadr = dccstck[x];
        x++;
        if ((x > 5) || (dccadr == 0)) x = 0;
if (DEBUG == true) {
        Serial.print("stack array@");
        Serial.print(x);
        Serial.print(" reads: ");
        Serial.println(dccadr);
}
        rtvrosnam();
        if (ros_eAvail == true) {     //display with name
          clrrosnam();
          dsprosnam();
          dsprosadr();
        } else {                      // display w/o name
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
      }
      else {
        cntd = cd;
      }
      btnnbr = 16;
      dspbtnread();
      if ((kpbtnstate == false) && (svr == true) && (noop == false)) {
        kpbtnstate = true;
        // Start the 'button press timer'
        startTime = millis();
        if (svr == true) {          // send "Quit" to server
          client.print("Q\r\n");
          delay(fdelay);
          client.flush();
          display.setColor(BLACK);
          display.fillRect(80, 30, 70, 18);
          display.setFont(ArialMT_Plain_16);
          display.setColor(WHITE);
          display.drawString(80, 30, "OFF");
          display.drawString(120, 13, "?");
          display.display();
          svr = false;
if (DEBUG == true) {
          Serial.println("Throttle disconnected");
}
        }
      }
      // reconnect
      if ((kpbtnstate == false) && (svr == false) && (noop == false)) {
        kpbtnstate = true;
        startTime = millis(); //restart button press timer
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
if (DEBUG == true) {
          Serial.println("restarting");
}
          display.clear();
          display.setFont(ArialMT_Plain_10);
          display.setColor(WHITE);
          display.drawString(40, 0, "Restarting");
          display.display();
          ESP.restart();
        }
      }
    }
  }  //close 'button pressed'
  /////////////////////////////////////////////////////////////
  //All buttons released - no button pushed:
  else if ((btnread == 0) && (kpbtnstate == true)) {
    kpbtnstate = false;
    noop = false;
    // function buttons off:
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
      if (fade == false) sendfct(fnum);
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
      if (fcgstate == 3) fcgstate = 0;
      if (fcgstate == 1) {
        display.setFont(ArialMT_Plain_10);
        display.setColor(BLACK);
        display.fillRect(15, 50, 10, 10);
        display.setColor(WHITE);
        display.drawString(15, 50, "1");
        display.display();
        dspfcts();
      }
      if (fcgstate == 0) {
        display.setFont(ArialMT_Plain_10);
        display.setColor(BLACK);
        display.fillRect(15, 50, 10, 10);
        display.setColor(WHITE);
        display.drawString(15, 50, "0");
        display.display();
        dspfcts();
      }
      if (fcgstate == 2) {
        display.setFont(ArialMT_Plain_10);
        display.setColor(BLACK);
        display.fillRect(15, 50, 10, 10);
        display.setColor(WHITE);
        display.drawString(15, 50, "2");
        display.display();
        dspfcts();
      }
      fctbtnstate = false;
    }
  }
}

//-----------------------------------------------------
void dspcnt(byte x) {
  byte y = x + 1;
  display.setFont(ArialMT_Plain_10);
  display.setColor(BLACK);
  display.fillRect(50, 18, 17, 10);
  display.setColor(WHITE);
  display.drawString(50, 18, String(y));
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
if (DEBUG == true) {
  Serial.println("display ON");
}
}
//------------------------------------------------------
void dspbtnread() {
  char dspbtnnbr[4];
  sprintf(dspbtnnbr, "% 4d", btnnbr);
  display.setFont(ArialMT_Plain_10);
  display.setColor(BLACK);
  display.fillRect(100, 0, 28, 10);
  display.setColor(WHITE);
  display.drawString(110, 0, String(dspbtnnbr));
  display.display();
  cntb = 3;   // reset and display battery level after 3 sec.
}
//------------------------------------------------------
void setfcts(byte fnum) {
if (DEBUG == true) {
  Serial.println("setfcts num " + String(fnum));
  Serial.println("setfcts state " + String(fctstate));
}
  fctarray[fnum] = fctstate;
}
//------------------------------------------------------
void dspfcts() {
  display.setFont(ArialMT_Plain_10);
  display.setColor(BLACK);
  display.fillRect(30, 50, 100, 15);
  display.display();
  if (fcgstate == 0) {
    for (int i = 0; i <= 9; i++) {            // 0-9 in fct grp 0
      if (fctarray[i] == 1) {
        display.setColor(WHITE);
        display.drawString(dspfos[i], 50, dspfnum[i]);
        display.display();
      }
    }
  }
  if (fcgstate == 1) {
    fn = -1;
    for (int i = 10; i <= 19; i++) {         // 0-9 in fct grp 1
      fn++;
      if (fctarray[i] == 1) {
        display.setColor(WHITE);
        display.drawString(dspfos[fn], 50, dspfnum[fn]);
        display.display();
      }
    }
  }
  if (fcgstate == 2) {
    fn = -1;
    for (int i = 20; i <= 28; i++) {         // 0-8 in fct grp 2
      fn++;
      if (fctarray[i] == 1) {
        display.setColor(WHITE);
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
  display.drawString(80, 30, String(dspaccadr));
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
if (DEBUG == true) {
  Serial.println("Get loco name from array element " + String(ptr));
}
  int rs = rosnamoffsets[ptr];             // begin of loco name
  int re = roster.indexOf('}', rs);        // end of loco name
  loconame = roster.substring(rs, re);
if (DEBUG == true) {
  Serial.println(loconame);
}
  ros_eAvail = true;
}
//--------------------------------------------------------------------------
void getrosadr(int ptr) {
if (DEBUG == true) {
  Serial.println("Get loco address from array element " + String(ptr));
}
  int rs = rosadroffsets[ptr];             // begin of loco address
  int re = roster.indexOf('}', rs);        // end of loco address
  locoadr = roster.substring(rs, re);
if (DEBUG == true) {
  Serial.println(locoadr);
}
}
//--------------------------------------------------------------------------
void getrosnamen() {
  for (int i = 0; i <= numos; i++) {
    rxn = roster.indexOf('[', rxn);    // first/next start of name
    rxn++;
    rxnn = rxn;
    rxnn = roster.indexOf('}', rxnn);   // first/next end of name
    if (roster.substring(rxn, rxnn) == loconame) {
if (DEBUG == true) {
      Serial.println("getrosnamen name offset: " + String(i));
}
      break;
    }
  }
}
//-------------------------------------------------------------------------
void rtvrosnam() {
  ros_eAvail = false;
if (DEBUG == true) {
  Serial.println("retr. loco name for " + String(dccadr));
}
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
if (DEBUG == true) {
  Serial.println("Get route sysname from array element: " + String(ptr));
}
  int rss = rsosa[ptr];
  int rse = routes.indexOf('}', rss);
  rsysname = routes.substring(rss, rse);
if (DEBUG == true) {
  Serial.println(rsysname);
  Serial.println("Get route usrname from array element " + String(ptr));
}
  int rus = ruosa[ptr];
  int rue = routes.indexOf('}', rus);
  rusrname = routes.substring(rus, rue);
if (DEBUG == true) {
  Serial.println(rusrname);
}
}
//--------------------------------------------------------
void set_speed() {
  lastvcnt = vcnt;
  vtemp = vcnt;
  dspeed = vcnt;
  if ((numspst == 28) && (numspst != 14)) {
    vtemp = vcnt * 4.5;
  }
  if (prvspeed != vtemp) {
    prvspeed = vtemp;
    vspeed = vtemp;
  }
  xmits = locoacts + "V" + String(vspeed);
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
  }
if (DEBUG == true) {
  Serial.print("set_speed: ");
  Serial.println(xmits);
}
  if (setsdhadr == true) {
    xmits = sdhacts + "V" + String(vspeed);
    if (svr == true) {
      client.print(xmits + "\r\n");
      client.flush();
    }
if (DEBUG == true) {
    Serial.print("set_speed consist: ");
    Serial.println(xmits);
}
  }
  display.setFont(ArialMT_Plain_16);
  display.setColor(BLACK);
  display.fillRect(15, 13, 27, 15);
  display.setColor(WHITE);
  display.drawString(15, 13, String(dspeed));
  display.display();
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
void sendsdhfct(byte fnum) {
  if (fnum != 99) {
    if (fctbtnst == 1) {
      xmits = sdhacts + "F1" + String(fnum);  // function button on
    }
    if (fctbtnst == 0) {
      xmits = sdhacts + "F0" + String(fnum);  // function button off
    }
if (DEBUG == true) {
    Serial.print("sendsdhfct: ");
    Serial.println(xmits);
}
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
      xmits = locoacts + "F1" + String(fnum);  // function button on
    }
    if (fctbtnst == 0) {
      xmits = locoacts + "F0" + String(fnum);  // function button off
    }
if (DEBUG == true) {
    Serial.print("sendfct: ");
    Serial.println(xmits);
}
    if (svr == true) {
      client.print(xmits + "\r\n");
      client.flush();
      delay(sdelay);
      while (client.available()) {
        jmri_res = client.readStringUntil('\n');
        if (jmri_res.startsWith(lkf)) {
if (DEBUG == true) {
          Serial.print("JMRI reply to sendfct: ");
          Serial.println(jmri_res);
}
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
  xmits = locoacts + "R1";
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
  }
if (DEBUG == true) {
  Serial.print("set_forward: ");
  Serial.println(xmits);
}
  delay(fdelay);
  xmits = locoacts + "V0";
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
  }
if (DEBUG == true) {
  Serial.println(xmits);
}
  display.setFont(ArialMT_Plain_16);
  display.setColor(BLACK);
  display.fillRect(0, 13, 10, 20);
  display.setColor(WHITE);
  display.drawString(0, 13, "F");
  display.display();
  //-----------------------
  if (setsdhadr == true) {
    if (sdhdct == true) xmits = sdhacts + "R1";
    if (sdhdct == false) xmits = sdhacts + "R0";
    if (svr == true) {
      client.print(xmits + "\r\n");
      client.flush();
      delay(fdelay);
    }
if (DEBUG == true) {
    Serial.print("set_forward: ");
    Serial.println(xmits);
}
    xmits = locoacts + "V0";
    if (svr == true) {
      client.print(xmits + "\r\n");
      client.flush();
    }
if (DEBUG == true) {
    Serial.println(xmits);
}
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
  xmits = locoacts + "R0";
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
  }
if (DEBUG == true) {
  Serial.print("set_reverse: ");
  Serial.println(xmits);
}
  xmits = locoacts + "V0";
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
  }
if (DEBUG == true) {
  Serial.println(xmits);
}
  display.setFont(ArialMT_Plain_16);
  display.setColor(BLACK);
  display.fillRect(0, 13, 10, 20);
  display.setColor(WHITE);
  display.drawString(0, 13, "R");
  display.display();
  //-----------------------
  if (setsdhadr == true) {
    if (sdhdct == true) xmits = sdhacts + "R0";
    if (sdhdct == false) xmits = sdhacts + "R1";
    if (svr == true) {
      client.print(xmits + "\r\n");
      client.flush();
    }
if (DEBUG == true) {
    Serial.print("set_reverse: ");
    Serial.println(xmits);
}
    delay(fdelay);
    xmits = locoacts + "V0";
    if (svr == true) {
      client.print(xmits + "\r\n");
      client.flush();
    }
if (DEBUG == true) {
    Serial.println(xmits);
}
  }
}
//-------------------------------------------------------------
void chgsdhdct() {
  if (sdhdct == true) xmits = sdhacts + "R1";
  if (sdhdct == false) xmits = sdhacts + "R0";
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
    delay(fdelay);
  }
if (DEBUG == true) {
  Serial.print("chgsdhdct: ");
  Serial.println(xmits);
}
  xmits = locoacts + "V0";
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
  }
if (DEBUG == true) {
  Serial.println(xmits);
}
}
/////////////////////////////////////////////////////////////////
void releaseadr(int rlsadr) {
  String xmitsi;
  if (rlsadr < 128) {
    xmits = "MT-S" + String(rlsadr) + "<;>r";
    xmitsi = "MTAS" + String(rlsadr) + "<;>I";
  }
  else {
    if (rlsadr > 127) {
      xmits = "MT-L" + String(rlsadr) + "<;>r";
      xmitsi = "MTAL" + String(rlsadr) + "<;>I";
    }
  }
if (DEBUG == true) {
  Serial.println(xmitsi);
}
  if (svr == true) {
    client.print(xmitsi + "\r\n");  //send "idle"
    client.flush();
  }
if (DEBUG == true) {
  Serial.print("Releasing address ");
  Serial.println(xmits);
}
  if (svr == true) {
    client.print(xmits + "\r\n");  //send "release"
    client.flush();
if (DEBUG == true) {
    delay(ldelay);
    Serial.println("JMRI-response: ");
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }
    Serial.println();
}
    dccadrrlsd = rlsadr;
  }
}
//-------------------------------------------------------------
void sendsdhadr() {
  if (sdhadr < 128) {
    xmits = "MT+S" + String(sdhadr) + "<;>S" + String(sdhadr);
    sdhacts = "MTAS" + String(sdhadr) + "<;>";    // set variable for actions
  }
  if (sdhadr > 127) {
    xmits = "MT+L" + String(sdhadr) + "<;>L" + String(sdhadr);
    sdhacts = "MTAL" + String(sdhadr) + "<;>";
  }
if (DEBUG == true) {
  Serial.println("sendsdhadr sends: " + xmits);
}
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
    delay(ldelay);
if (DEBUG == true) {
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }
}
  }
}
//---------------------------------------------------------------------
void sendadr() {
  // add a locomotive
  if (dccadr < 128) {
    if (ros_eAvail == false) {
      xmits = "MT+S" + String(dccadr) + "<;>S" + String(dccadr);
    } else {
      xmits = "MT+S" + String(dccadr) + "<;>E" + loconame;
    }
    locoacts = "MTAS" + String(dccadr) + "<;>";    // set variable for actions
    lkf = locoacts + "F";       // search argument to look for function states
    lkr = locoacts + "R";       // search argument to look for direction
    lks = locoacts + "s";       // search argument to look for speed step
  }
  else {
    if (dccadr > 127) {
      if (ros_eAvail == false) {
        xmits = "MT+L" + String(dccadr) + "<;>L" + String(dccadr);
      } else {
        xmits = "MT+L" + String(dccadr) + "<;>E" + loconame;
      }
      locoacts = "MTAL" + String(dccadr) + "<;>";
      lkf = locoacts + "F";       // search argument to look for function states
      lkr = locoacts + "R";       // search argument to look for direction
      lks = locoacts + "s";       // search argument to look for speed step
    }
  }
  ros_eAvail = false;
if (DEBUG == true) {
  Serial.println("sendadr sends " + xmits);
}
  fctsavail = false;
  display.setFont(ArialMT_Plain_10);
  display.setColor(BLACK);
  display.fillRect(30, 50, 100, 15);
  display.setColor(WHITE);
  display.drawString(30, 50, "mapping functions");
  display.display();
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
    delay(sdelay);
    while (client.available()) {
      jmri_res = client.readStringUntil('\n');
if (DEBUG == true) {
      Serial.println("JMRI add_loco resp " + jmri_res);
}
      if (jmri_res.startsWith("MTL")) {   // get functions array
        fctsarray = jmri_res;
        fctsavail = true;
if (DEBUG == true) {
        Serial.print("JMRI list of functions: ");
        Serial.println(jmri_res);
}
      }
      if (jmri_res.startsWith(lkf)) {
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
        if (fnum <= numfcts) {                   // limit to configured nbr of functions
          setfcts(fnum);;
        }
      }
      if (jmri_res.startsWith(lkr)) {
        int ix = jmri_res.indexOf('R');
        ix++;
        jmri_sub = jmri_res.substring(ix, ix + 1);
        int dct = jmri_sub.toInt();
        if (dct == 1) {
          dctstate = true;
if (DEBUG == true) {
          Serial.println("is forward");
}
          display.setFont(ArialMT_Plain_16);
          display.setColor(BLACK);
          display.fillRect(0, 13, 10, 20);
          display.setColor(WHITE);
          display.drawString(0, 13, "F");
          display.display();
        }
        if (dct == 0) {
          dctstate = false;
if (DEBUG == true) {
          Serial.println("is reverse");
}
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
        //if (spdstpm == 8) numspst = 14;
        if (spdstpm == 1) numspst = 126;    // speed step mode 128 (0=stop,1to126+Estop=128)
        if (spdstpm == 2) numspst = 28;     // speed step mode 28
if (DEBUG == true) {
        Serial.print("current speed step mode: ");
        Serial.println(numspst);
}
        display.setFont(ArialMT_Plain_10);
        display.setColor(BLACK);
        display.fillRect(50, 18, 20, 10);
        display.setColor(WHITE);
        display.drawString(50, 18, String(numspst));
        display.display();
      }
    }
  }
  // build dedicated function switch mapping array based on JMRI Roster:
  if (fctsavail == true) {
if (DEBUG == true) {
    Serial.println("mapping switches");
}
    for (int i = 0; i < numswfcs; i++) {  //reset all array elements to default
      fctswtmap[i] = 99;
    }
    fxi = 0;
    fxm = 0;
    for (byte fae = 0; fae < numswfcs; fae++) {
      fxn = 0;
      fxs = 0;
      fxe = 0;
      while (fxn < numfcts) {            // look up all elements of fctsarray for each switched function
        fxs = fctsarray.indexOf("[", fxs);
        fxe = fctsarray.indexOf("]", fxs);
        String lkn = fctname[fxi];
        String lk = fctsarray.substring(fxs + 1, fxe);
        fxs++;
        if (lk == lkn) {
if (DEBUG == true) {
          Serial.print("Match @ (fxn):");
          Serial.println(fxn);
}
          fctswtmap[fxm] = fxn;
          break;
        }
        fxn++;
      }
      fxi++;
      fxm++;
    }
if (DEBUG == true) {
    Serial.print("fctswtmap: ");
    for (byte x = 0; x < numswfcs; x++) {
      Serial.print(fctswtmap[x]);
      Serial.print(",");
    }
    Serial.println();
}
    dspfcts();
  }
  if (fctsavail == false) {
    for (int i = 0; i < numswfcs; i++) {
      fctswtmap[i] = fctswtmapdft[i];
    }
    dspfcts();
if (DEBUG == true) {
    Serial.println("Using default Function Switch map");
}
  }
  if (fctswtmap[7] == 99) {
    drivehold = false;
if (DEBUG == true) {
    Serial.println("drivehold OFF for this loco");
}
  } else {
    drivehold = true;
if (DEBUG == true) {
    Serial.println("drivehold ON for this loco");
}
  }
  sfnum = fctswtmap[8];
  if (sfnum != 99) {
    fading = true;
    if (fctarray[sfnum] == 1) {
      faded = true;
      display.setColor(WHITE);
      display.drawString(70, 18, "F");
      display.display();
if (DEBUG == true) {
      Serial.println("Sound of this loco is faded");
}
    }
if (DEBUG == true) {
    Serial.println("F1 + Speed > 0 = Sound Fading for this loco");
}
  } else {
    fading = false;
if (DEBUG == true) {
    Serial.println("Normal F1 for this loco");
}
  }
}
//---------------------------------------------------------------
void sendaccadr() {
  xmits = "PTA" + setacc + String(accadr);
if (DEBUG == true) {
  Serial.println("sendaccadr sends " + xmits);
}
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
    delay(sdelay);
  }
}
//-----------------------------------------------------------------
void sendroute() {
  xmits = "PRA2" + String(rsysname);
if (DEBUG == true) {
  Serial.println("sendroute sends " + xmits);
}
  if (svr == true) {
    client.print(xmits + "\r\n");
    client.flush();
    delay(ldelay);
if (DEBUG == true) {
    Serial.println("server response: ");
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }
}
  }
}
/////////////////////////////////////////////////////////////////////////
#ifdef CLOCK
void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600;
  pctime = unixsub.toInt();
  if ( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
    setTime(pctime); // Sync Arduino clock to the time received on the serial port
  }
}
void getjmritime() {
  unixtime = jmri_res;              // save fastclock time
  int j = unixtime.indexOf("<;>");
  if (j > 0) {           // some times a "<;>" isn't sent from jmri, ignore timestamp if it isn't.
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
      display.drawString(45, 0, String(dsphour) + ":0" + String(dspmin)  + "  r:" + ratiosub + "\r\n");
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
    if ((dccadrtmp >= 0 ) && (dccadrtmp < 9999)) dccstck[i] = dccadrtmp;
  }
if (DEBUG == true) {
  Serial.print("stack retrieved: ");
  for (byte i = 0; i <= 5; i++) {
    Serial.print(dccstck[i]);
    Serial.print(",");
  }
  Serial.println();
}
}
//--------------------------------------
void savestack() {
if (DEBUG == true) {
  Serial.print("saving stack: ");
  for (byte i = 0; i <= 5; i++) {
    Serial.print(dccstck[i]);
    Serial.print(",");
  }
  Serial.println();
}
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
  //batvolt = battmp * 3.63;        // 3.3 refV * 1.1 ADC refV = 3.63
  batvolt = battmp * 3.53;          // this gets us closer to a meter reading
  char dspbat[4];
  dtostrf(batvolt, 4, 1, dspbat);
  display.setFont(ArialMT_Plain_10);
  display.setColor(BLACK);
  display.fillRect(100, 0, 28, 10); //#1
  display.setColor(WHITE);
  display.drawString(100, 0, dspbat);
  display.drawString(120, 0, "V");
  display.display();
}
#endif
//--------------------------------------
void set_spstm() {
if (DEBUG == true) {
  Serial.print("current speed step mode: ");
  Serial.println(spdstpm);
}
  if (spdstpm == 1) chgspstm = 2;
  if (spdstpm == 2) chgspstm = 1;
  xmits = locoacts + "s" + String(chgspstm);
if (DEBUG == true) {
  Serial.print("throttle sends ");
  Serial.println(xmits);
}
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
      //if (spdstpm == 8) numspst = 14;
      if (spdstpm == 1) numspst = 126;    // speed step mode 128 (0=stop,1to126+Estop=128)
      if (spdstpm == 2) numspst = 28;     // speed step mode 28
if (DEBUG == true) {
      Serial.print("new speed step mode: ");
      Serial.println(spdstpm);
}
      display.setFont(ArialMT_Plain_10);
      display.setColor(BLACK);
      display.fillRect(50, 18, 20, 10);
      display.setColor(WHITE);
      display.drawString(50, 18, String(numspst));
      display.display();
    }
  }
}
void actdccadr() {
  setdccadr = false;
  if ((pshadr == true) || (ros_eAvail == true)) {     // address entered, or got a roster entry.
    if (ros_eAvail == true) {
      vcnt = 0;
      vtemp = 0;
      vspeed = 0;
      dccadr = locoadr.toInt();
if (DEBUG == true) {
      Serial.println("address taken from roster");
}
      clrchgadr();
    } else {
      clrchgadr();
    }
    sprintf(dspadr, "%04d", dccadr);      // display with leading zeros
    display.setFont(ArialMT_Plain_16);
    display.setColor(BLACK);
    display.fillRect(50, 18, 17, 10);     // remove stack array #
    display.fillRect(120, 13, 10, 15);    // remove ?
    display.fillRect(80, 30, 70, 18);     // remove locoadr from loconame display
    display.setColor(WHITE);
    display.drawString(80, 13, String(dspadr));
    display.display();
    // avoid duplicate in stack array: -------------------------
    nostck = false;
    for (byte i = 0; i <= 5; i++) {
      if (dccstck[i] == dccadr) {
        nostck = true;
        break;
      }
    }
    // save to stack array: -----------------------------------
    if (nostck == false) {  // all entries shift right
      for (byte i = 4; i >= 0; i--) {
        dccadrtmp = dccstck[i];
        byte j = i + 1;
        dccstck[j] = dccadrtmp;
        if (i == 0) {
          dccstck[0] = dccadr;  // add new entry as first
          break;
        }
      }
if (DEBUG == true) {
      Serial.print("stack array new: ");
      for (byte i = 0; i <= 5; i++) {
        Serial.print(dccstck[i]);
        Serial.print(",");
      }
      Serial.println();
}
      savestack();
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
if (DEBUG == true) {
    Serial.print("adding dcc address ");
    Serial.println(dspadr);
}
    if (ros_eAvail == false) {
      rtvrosnam();
    }
    clrrosnam();
    if (ros_eAvail == true) dsprosnam();
    if ((lastdccadr != 0) && (lastdccadr != dccadr) && (setsdhadr == false)) releaseadr(lastdccadr);
    if (lastdccadr != dccadr) sendadr();
  }
  else {
if (DEBUG == true) {
    Serial.println("adr not changed");
    Serial.println(dccadr);
}
    if (dccadrrlsd == dccadr) {
      sendadr();
      dccadrrlsd = 0;
    }
    display.setFont(ArialMT_Plain_16);
    display.setColor(BLACK);
    display.fillRect(80, 13, 50, 15);
    display.fillRect(80, 30, 50, 18);     // remove locoadr on display
    display.setColor(WHITE);
    sprintf(dspadr, "%04d", dccadr);
    display.drawString(80, 13, String(dspadr));
    display.display();
    vcnt = lastvcnt;
  }
}
//------------------------------------------------------
void keypad_check() {
  display.displayOff();
  if (svr == true) {          // send "Quit" to server
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
  btnprvl = 5000;   // reset
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
  int btnmin = 0;     // for finding min value
  int btnmax = 0;     // for finding max value
  int btnprvl = 5000; // previous low
  int btnprvh = 0;    // previous high
  int btnnum = 0;     // button number
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
    if (btnread > 100) {   // minimum value indicating a button is pressed
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
    }
    else if (btnread == 0) {
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
      int btnin = Serial.parseInt(); // receive byte as an integer
      Serial.print("You entered: ");
      Serial.println(btnin);
      if (btnin == 99) {
        eoj = true;
        Serial.println("keypad-config ended");
        display.displayOn();
        reconnect();
      }
      if ((btnin > 0) && (btnin < 17)) { // is a button in keypad
        btnin--;
        btnsav = offset[btnin];
        btnmin = btnmin - 5;
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
        EEPROM.write(btnsav, (btnmin) & 0xff);
        btnsav++;
        Serial.print("Saving maximum value ");
        Serial.print(btnmax);
        Serial.print(" in elements ");
        Serial.print(btnsav);
        Serial.print(" & ");
        EEPROM.write(btnsav, (btnmax >> 8) & 0xff);
        btnsav++;
        Serial.println(btnsav);
        EEPROM.write(btnsav, (btnmax) & 0xff);
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
if (DEBUG == true) {
  Serial.println("attempting to reconnect");
}
  svr = true;
  display.setFont(ArialMT_Plain_10);
  display.setColor(BLACK);
  display.fillRect(43, 0, 95, 12);
  display.setColor(WHITE);
  display.drawString(45, 0, "connecting");
  display.display();
  client.connect(host, port);
  while (!client.connect(host, port)) {
    display.setColor(BLACK);
    display.fillRect(43, 0, 95, 12);
    display.setColor(WHITE);
    display.drawString(45, 0, "svr offline");
    display.display();
    WiFi.disconnect();
if (DEBUG == true) {
    Serial.println("Throttle offline");
    Serial.println("WiFi disconnected");
}
    svr = false;
  }
  if (svr == true) {
    //---Throttle ON: sends NAME,MAC to server and starts heart beat
if (DEBUG == true) {
    Serial.println("Sending HU" + MacString);
}
    client.print("N" + tname + "\r\n");       // set Name of Throttle
    delay(fdelay);
    client.print("HU" + MacString + "\r\n");  // set identifier
    delay(ldelay);
    if (hbuse == true) {
      client.print("*+\r\n");    // starting heartbeat
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
