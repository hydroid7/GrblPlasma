# XmotionFirmware
CNC Motion Control Firmware for XmotionCNC Motion Control Boards. Focused on CNC Plasma Cutting

# Forked from GRBL
XmotionFirmware was forked from GRBL but the protocal has changed a lot and is not compatible with GRBL sender.

# Features
- All original GRBL features left intact
- Setup for Dual Y (Most plasma machines are dual Y gantry's)
- Cyclic redundancy check on Gcode stream to controller. This is absolutly critical for noisy environments, like plasma cutting. Without this
a dropped byte or framming issue during the gcode stream can cause your machine to go places it's not supposed to!
- Built in Arc Voltage Torch Height Control. Uses A0 to read arc voltage from the plasma cutter and makes adjustments on a 1ms intervol.
I've been working in the CNC plasma industry for over 6 years and around them for over 10 years and the AVTHC I've created here works better than any
other i've seen... But you try it and let me know what you think!

# Control Software
Right now there is only one option control software wise to utilize this firmware and it's called [ncPilot](https://github.com/UnfinishedBusiness/ncPilot)
It cross-platform (Windows, Linux, and MacOS) and the core of it (Xkernel) is written in C/C++. The UI is written in Javascript and new
features are easily added and it's highly customizable.


# Post processors
There is a SheetCAM post processor in posts/Xmotion.scpost
