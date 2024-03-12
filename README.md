# TOSIM Simulation Program                                          

This program simulates the dynamics of a tethered aircraft. The tether line is connected to a moving platform on one end and another vehicle on the other end.

The code is written in the FORTRAN 95 programming language.

The program has gone through major revisions. They are listed below

Version 1.0 - 2013
Original TAPAS Code Developed by Dr. Mark Costello Oct 2013 (Tether Aircraft Parafoil and Ship Simulation) = TAPAS

Version 2.0 - 2014/2015
Dr. Carlos Montalvo added tether pay in / out capabilities for TERN 2014 (Tactical Exploitative Reconaissance Node).
Also added airwake model from the ship and the ability for the parafoil to capture an aircraft.
This model was used to determine lockout stability and a paper was written and published in 2015.

Verion 3.0 - 2015
A WingsX module was added in 2015 via Dan Keuhme at AREA-I
(WingsX was proprietary and is not included in this repo but remnants may still be in the code) 

Version 4.0 - 2015/2016
A quadcopter sim was originally created by Weston Barron in MATLAB 2015
This module was converted to FORTRAN and added in 2016 by Lisa Schibelius 

Version 5.0 - Fall 2017
The code was then majorly overhauled to have the driver be a quadcopter and the towed model become an aircraft
The routine was debugged and streamlined by Nghia Huynh in Fall 2017

Version 6.0 - Spring 2018
In order to utilize the quadcopter model in separate FORTRAN codes,
Dr. Carlos Montalvo created a separate module for the quadcopter and 
placed it in its own f90 file Spg 2018

Version 7.0 - Jan 2024
This version was also released to the public in Jan 2024. Due to the open source nature
of the code, most if not all of the TAPAS/TERN/AREAI code has been removed.
There may still be some remnants floating around but none of it is operational or endorsed as working.
Still the code has a quadcopter as the driver and an aircraft as the towed system 

Version 8.0 - Mar 2024
The following major changes were made for this version:
0.) The driver is now a truck and the towed system is now a quadcopter
1.) Nominal.CS file has been moved into each vehicles respective files - Read routines were updated as well
2.) There is now only 1 and only 1 control subroutine - This got rid of the handshake stuff
	Driver - Currentl contains a drive forward controller (The waypoint controller is in version 7.0 and will get copied later)xs
	Towed - Only inner loop control
	Tether - No change from Version 7.0
3.) All remnants of AoA sweeps have been removed
4.) Echo routine completely removed - Which means read routine is flag eq 1 and then compute is flag 2
5.) The DRIVER and TOWED functions are now passed the T variable rather than T%DRIVER or T%TOWED
6.) Made sure that tether reel and connection points are all properly aligned
7.) State vectors are properly aligned now since the towed structure now has 8 extra states (thrust of each motor) So we have
1-12 states are the driver, 13-33 are the quadcopter (13 states with quaternions and then 8 more for motors), and then the rest are bead states




