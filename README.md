# Plasma Grbl

GRBL focused on budget plasma cutting.

# Forked from GRBL
PlasmaGrbl was forked from [XmotionFirmware](https://github.com/UnfinishedBusiness/XmotionFirmware). It also was forked from GRBL but the protocal has changed a lot and is not compatible with GRBL sender and the firmware needs more modifications.

# Features
- All original GRBL features left intact
- Setup for Dual X (Most plasma machines are dual X gantry's)
- Cyclic redundancy check on Gcode stream to controller. This is absolutely critical for noisy environments, like plasma cutting. Without this
a dropped byte or framming issue during the gcode stream can cause your machine to go places it's not supposed to!
- Built in Arc Voltage Torch Height Control with 1ms update time.

# Post processors
There is a SheetCAM post processor in posts/Xmotion.scpost

# Build using PlatformIO
If you want to build this from scratch, I recommend doing it via platformio. I personally like Microsoft's Vistual Code Studio with the PlatformIO plugin. Otherwise, there's a binary .hex file included in the ncPilot/Extra folder in the ncPilot Repository
