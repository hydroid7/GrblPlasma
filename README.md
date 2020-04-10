# XmotionFirmware
CNC Motion Control Firmware for XmotionCNC Motion Control Boards or Arduino UNO DIY setup. Focused on CNC Plasma Cutting.

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

# Build using PlatformIO
If you want to build this from scratch, I recomend doing it via platformio. I personally like Microsoft's Vistual Code Studio with the PlatformIO plguin. Otherwise, there's a binary .hex file included in the ncPilot/Extra folder in the ncPilot Repository

# Preamble
I own [BadApple Machine & Fab](https://badappleproducts.com), our main business is manufacturing light-duty entry-level CNC plasma cutters. My goal in business is to lower costs for these types of machines without compromising quality or needed features. All of our machines have Teknic AC brushless servos, Rack and Pinion drives, Leadscrew Z, Floating Head, AVTHC and Waterpan. I've a great deal of time creating this control package (The circuit board we use in our pruduction machines, the software, the firmware, etc) because while Mach3, LinuxCNC, Flashcut, and a handfull of controls that already exists work great, they are all expensive and fairly complicated. We used to use a LinuxCNC based control on our productions machines with Messa hardware which is a great way to go but expensive and fairly difficult for us to add new features (We're always listening to what customers would like to see), also expensive and time consuming to replace (as opposed to now switching out a refurbished laptop and installing software). By making this group of software open source and available to hobbyist around the world who are instested in using DIY delopment boards like the Arduino UNO (ATMega328P) to build their CNC motion control project, I hope I can help you get your machine up and running with less time and money spent. Feel free to submit pull requests, I'd appreciate any help I can get. Thanks
Checkout our other projects
- [JetCAD](https://jetcad.io) (2D CAD in the cloud, geared for plasma design, offline version in the works)
- [Xkernel](https://github.com/UnfinishedBusiness/Xkernel) (Powerfull Cross-Platform Javascript VM the runs code that looks and feels like Arduino code)
- [XmotionFirmware](https://github.com/UnfinishedBusiness/XmotionFirmware) (GRBL based motion control for CNC plasma cutting, builtin arc voltage torch height control)
- [ncPilot](https://github.com/UnfinishedBusiness/ncPilot) (Front-End for XmotionFirmware. Runs on just about anything hardware or operating system wise. Has built-in Gcode Viewer, Live DRO, Click and Go way points, ctrl-click viewer program jump-ins)
