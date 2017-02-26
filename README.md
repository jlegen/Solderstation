# Solderstation
enhanced fork of the weller tip based solderstation from DebuggingLab
based on work from Matthias Wagner
https://debugginglab.wordpress.com/2014/10/30/soldering-station/

Including some experimenting with high res fonts and click encoder (replacing the pot)

Modified by jleg99@gmail.com:
* code cleanup
* new shutdown mode (default 5 minutes)
* proper RGB LED and fume fan support
* measure and show Vcc and Vin (simple voltage divider for Vin needed)
* icons to show standby and shutdown modes
* PWM is visualized using a bar graph
* other splash screen :)

# 3 versions available

## V1.7 - standard fonts
* this is more or less the original version with the enhancements/extensions mentioned above

## V1.7 - smooth fonts
* same as above, but using high res fonts from Adafruit TFT lib
* i regard this as a "try" - i wasn't able to eliminate flicker completely

## V2.0 - click encoder with normal fonts
* based on version with regular fonts, i replaced the pot with a standard cheapo click encoder
* a click switches to the next temperature preset
* a long click (hold) stores the current set value to the current preset 

# Hardware mods
I used the original PCB, available from the link above. For the extensions, i soldered some wires to the pro mini socket directly:
## Anti fume fan
* A0 - i used a cheap MOSFET module for Arduino, using a IRF520
## Voltage measurement
* Vcc is measured using the internal reference
* to optionally also measure Vin, i soldered 2 resistors to A6: Vin <-> 22k <-> A6 <-> 5.6k <-> GND
* this should be good for up to 24V Vin
## Click encoder
* i used D15, D16 for pahes A/B (or Data/Clock), and D17 for the button
* for some unknown reason, the original PCB which i used had D16 connected to Vcc, so i scratched a PCB trace
* as an alternative, the now unused Pot pin A5 could also be used to avoid the problem with the trace

