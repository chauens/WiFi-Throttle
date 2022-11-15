# WiFi-Throttle for model railroading using DCC with JMRI.
Download WiCabFiles.zip and extract. It contains everything needed to build this Wireless throttle that works with JMRI.
The throttle uses an “ESP32” development board and some “dedicated” switches: a toggle Switch for Bell, a toggle Switch for Direction, real knob for the Speed. A Button for Whistle/Horn - short and long (short push/long push). And a button for Couplers – rear and front on separate functions, activated depending on direction, or on same function, or just for the “push/pull away-action”. “Dedicated” means: a switch called i.e. “Bell” calls the function “Bell” in a decoder no matter what function number it has configured for the Bell.
Because the throttle connects to the JMRI WiThrottle Server it is independent from the type of DCC system. 
The functions are configured in the JMRI roster.
The main components in my throttle are: 
DIY Keypad with 16 buttons in 4 rows and 4 columns. 
OLED 128x64 like SSD1306 type with I2C controller, 3-5V.
Rotary Encoder.
ESP32 based development board.
Toggle Switch ON/OFF/ON for direction (center off for Drive Hold if available).
Toggle Switch ON/OFF for Bell.
Push Buttons for Whistle, Coupler.
LiPo Battery.
Self-designed PCB using edging service.
Software included in package uploaded to the ESP32 board using the Arduino IDE.
Computer with WiFi connection running JMRI Panel Pro or Decoder Pro.
In my case I loaded the JMRI Panel Pro on a Raspberry Pi.
As mentioned above the throttle has “Dedicated” switches. They are named: Whistle-short, Whistle-long, Bell, Coupler-rear, Coupler-front, Shunting, Drive-hold, Fade. 
The JMRI WiThrottle Server Roster entries determine which function number is assigned to the extra Switches on the throttle. 
A matching function name in the JMRI-Roster selects the function number that will be mapped to the switch.
So all you need to do is go to the JMRI Roster and put the names of the switches to the function number of the locomotive. If a function is not available or not needed, just don’t define it in the Roster.
Features:
Select DCC addresses from the roster in JMRI by stepping through the roster turning the Speed Knob. 
Up to 100 Locos in your roster can be selected. The limit of 100 can be extended if needed. 
Enter DCC addresses via 3x4 Numeric-Keypad.
Operate Turnouts (stationary aka assessory decoders).
Select Speed Step Mode 28 or 128.
Display the Fast Clock from JMRI.
Display turns dark when not in use for 2 minutes. 
Utilizing heartbeat control in JMRI. 
Support for 28 functions as supported by JMRI.
The last 6 DCC addresses are kept in a stack beyond power off for quick access. 
Monitor the battery charge.
