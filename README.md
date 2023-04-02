# WiFi-Throttle for model railroading using DCC with JMRI.
Download WiCabFiles.zip and extract. It contains everything needed to build this Wireless throttle that works with JMRI.
The throttle uses an “ESP32” development board and some “dedicated” switches: a toggle Switch for Bell, a toggle Switch for Direction, real knob for the Speed. A Button for Whistle/Horn - short and long (short push/long push). And a button for Couplers – rear and front on separate functions, activated depending on direction, or on same function, or just for the “push/pull away-action”. “Dedicated” means: a switch called i.e. “Bell” calls the function “Bell” in a decoder no matter what function number it has configured for the Bell.
Because the throttle connects to the JMRI WiThrottle Server it is independent from the type of DCC system. 
The functions are configured in the JMRI roster.
<ol>
The main components in my throttle are:
</ol>
<ol>
  <li>DIY Keypad with 16 buttons in 4 rows and 4 columns.</li> 
  <li>OLED 128x64 like SSD1306 type with I2C controller, 3-5V.</li>
  <li>Rotary Encoder.</li>
  <li>ESP32 based development board.</li>
  <li>Toggle Switch ON/OFF/ON for direction (center off for Drive Hold if available).</li>
  <li>Toggle Switch ON/OFF for Bell.</.i>
  <li>Push Buttons for Whistle, Coupler.</li>
  <li>LiPo Battery.</li>
  <li>PCB using edging service.</li>
  <li>Software included in package uploaded to the ESP32 board using the Arduino IDE.</li>
  <li>Computer with WiFi connection running JMRI Panel Pro or Decoder Pro.</li>
  <li>In my case I loaded the JMRI Panel Pro on a Raspberry Pi.</li>
</ol>
<ol>
As mentioned above the throttle has “Dedicated” switches. They are named: Whistle-short, Whistle-long, Bell, Coupler-rear, Coupler-front, Shunting, Drive-hold, Fade.
</ol>
<ol>
The JMRI WiThrottle Server Roster entries determine which function number is assigned to the extra Switches on the throttle. 
A matching function name in the JMRI-Roster selects the function number that will be mapped to the switch.
So all you need to do is go to the JMRI Roster and put the names of the switches to the function number of the locomotive. If a function is not available or not needed, just don’t define it in the Roster.
</ol>
Features:
<li>Select DCC addresses from the roster in JMRI by stepping through the roster turning the Speed Knob.</li>
<li>Up to 100 Locos in your roster can be selected. The limit of 100 can be extended if needed.</li> 
<li>Enter DCC addresses via 3x4 Numeric-Keypad.</li>
<li>Operate Turnouts (stationary aka assessory decoders).</li>
<li>Select Function by Name</li>
<li>Select Speed Step Mode 28 or 128.</li>
<li>Display the Fast Clock from JMRI.</li>
<li>Display turns dark when not in use for 2 minutes.</li> 
<li>Utilizing heartbeat control in JMRI.</li> 
<li>upport for 28 functions as supported by JMRI.</li>
<li>The last 6 DCC addresses are kept in a stack beyond power off for quick access.</li> 
<li>Monitor the battery charge.</li>
