UDKWiimote
==========

This project is for interfacing the Wiimote with the UDK on Windows. It is currently a work in progress.
As of September 2, 2012, the library supports Motion Plus, rumble, LEDs, and buttons.

It is built on WiiYourself, adding support for Wii remotes with embedded Motion Plus.
It implements Sebastian O.H. Madgwick's IMU orientation filter to estimate the Wiimote's orientation using Motion Plus.

The Visual C++ 2010 solution builds a DLL that can be used from UnrealScript through DLLBind.
