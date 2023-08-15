/* Store values for keypad buttons
    Christoph Hauenstein. June/13/2022. For privat use only.
    Start Serial Monitor at a baud rate of 115200.
*/
#include <EEPROM.h>
#define keypadin 34 // A2 analog pin connecting keypad 
int btnread = 0;    // holding keypad button values
int btnmin = 0;     // for finding min value
int btnmax = 0;     // for finding max value
int btnprvl = 5000; // previous low
int btnprvh = 0;    // previous high
int btnnum = 0;     // button number
int btnsav = 0;
bool kpreadON = false;
byte dbcdly = 50;      // debounce delay
#define EEPROM_SIZE 96  // 0+1=dccadr, 2-13=stack, 19=# throttles. 20-84=keypad, 95=debug flag.
byte offset[16];
void setup() {
  byte j = 20;
  for (byte i = 0; i < 16; i++) {
    offset[i] = j;
    j = j + 4;
  }
  EEPROM.begin(EEPROM_SIZE);
  Serial.begin(115200);
  delay(3000);
  Serial.println("reading input from keypad 10 times");
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
  Serial.println("For testing or saving button values: push a button for about 2 sec.");
  Serial.println("Current EEPROM keypad content: ");
  j = 20;
  for (byte i = 0; i < 32; i++) {
    int val = EEPROM.read(j);
    j++;
    val = (val << 8) + EEPROM.read(j);
    j++;
    Serial.print(val);
    Serial.print(",");
  }
  Serial.println();
  int val = EEPROM.read(19);  // get # of throttles
  if (val != 1) EEPROM.write(19, 1);  // init throttle for "single"
}
void loop() {
  btnread = analogRead(keypadin);
  if (btnread > 100) {   // minimum value indicating a button is pressed
    delay(dbcdly);
    //btnread = analogRead(keypadin);
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
      Serial.print("Button ");
      Serial.print(btnnum);
      Serial.print(" min keypad reading ");
      Serial.println(btnmin);
      Serial.print("Button ");
      Serial.print(btnnum);
      Serial.print(" max keypad reading ");
      Serial.println(btnmax);
      btnprvl = 5000;
      btnprvh = 0;
      kpreadON = false;
      Serial.println("In SM enter the button number for which to save values");
    }
  }
  if (Serial.available() > 0) {
    int btnin = Serial.parseInt(); // receive byte as an integer
    if ((btnin > 0) && (btnin < 17)) { // is a button in keypad
      btnin--;
      btnsav = offset[btnin];
      btnmin = btnmin - 5;
      if (btnmax < 4090) btnmax = btnmax + 5;
      Serial.println("------------------------------------------");
      Serial.print("Saving values for button ");
      Serial.println(btnnum);
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
    }
    Serial.println("Current content: ");
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
