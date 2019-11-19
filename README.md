# XmotionFirmware
CNC Motion Control Firmware for XmotionCNC Motion Control Boards

# Hardware
Has been design to run on my Xmotion CNC Plasma Machines custom designed circuit board. If you wan't to use the firmware on
your DIY machine. Get yourself a Teensy 3.6 (Or port to virtually any other platform you would like) and a throw together a breakout board. See src/Machine.h for pin definitions.

# Firmware
I've designed this to be very portable. Uses the arduino wiring framework and platform io. For CNC plasma use 
(Full Floating Z axis with integrated ATHC and simultanious X & Y Linear interpolation) Requires one interupt timer with <= 10uS periods.
The shorter the period the higher the capability of the step train pulses and resulution.

# How it works
The motion control firmware is completely dumb and does not store any information on sd card or EEPROM. Configuration values are 
always written when the controller (See my XDesign repository which currently has the "Machine" Workbench built in to it) connects. 
The controller only handles rapid moves and linear moves at specified feedrates. This information is streamed to it in real time via USB Serial
and makes use of a CRC algorythm to ensure there's no data corruption in the serial stream that could cause incorrect movements. The machine interface keeps track of Work Offsets and is responsible for
making sure Gcode is inside the machine boundry. Automatic torch height control is built into the Firmware and the voltage setting can be
set via Gcode.

# Pros
Very simple 
Easy to expand
Highly Portable

# Cons
Only employs acceleration and decelleration with rapid moves. Line moves are handles in line segments at the specified feedrates and
the control will keep everything moving in constant velocity as per what the segments feedrates are set to. It's expected that constant
velocity path planning should be handled in the control since there is much more processing power availble for to employ a lookahead
planner
Doesn't yet have absolute Z axis control (Could be added very easily, just have not needed it yet on my end)
Doesn't yet have rotory axis code (This is coming soon, have a customer who needs it sooner than later)

# Post processors
There is a SheetCAM post processor in posts/Xmotion.scpost
