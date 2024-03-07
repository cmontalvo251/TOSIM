# TOSIM Simulation Program                                          

This program simulates the dynamics of a tethered aircraft. The tether line is connected to a moving platform on. One end and the aircraft on the other end.

The code is written in the FORTRAN 95 programming language.

The program has been edited by:                                     

Original TAPAS Code Developed by Dr. Mark Costello Oct 2013 (Tether Aircraft Parafoil and Ship Simulation) = TAPAS             

Dr. Carlos Montalvo Debugged Tether module for TERN 2014 (Tactical Exploitative Reconaissance Node).                          

A WingsX module was added in 2015 via Dan Keuhme at AREA-I (This is proprietary and is not included in this repo but remnants may still be in the code)

Quadcopter sim originally created by Weston Barron in MATLAB 2015

Quadcopter module in FORTRAN was added in 2016 by Lisa Schibelius 

Routine Debugged and streamlined by Nghia Huynh in Fall 2017     

Dr. Carlos Montalvo created a separate module for quadcopter.f90 Spg 2018

Dr. Carlos Montalvo moved this repo from Gitlab to Github and made the code open source. Due to the open source nature of the code, most if not all of the TAPAS/TERN/AREAI code has been removed. There may still be some remnants floating around but none of it is operational or endorsed as working. Still the code has a quadcopter as the driver and an aircraft as the towed system 

Version 7.0 Released Jan 2024

Things needed to do for Version 8.0

1.) Nominal.CS file needs to get moved into each vehicles respective files - Read routines need to get updated as well
2.) There needs to be 1 and only 1 control subroutine - This will get rid of the handshake stuff
	Driver - Needs a drive forward and the hooks (commented for now) for waypoint control
	Towed - Just needs inner loop control
	Tether - Can stay the same
3.) All remnants of AoA sweeps need to get removed 
4.) Remove echo routine completely - Which means read routine will be flag eq 1 and then compute will be flag 2
5.) The DRIVER and TOWED functions need to get passed T rather than T%DRIVER or T%TOWED
6.) Need to make sure that tether reel and connection points are all properly aligned
7.) Need to make sure the state vectors are properly aligned since the towed structure now has 8 extra states (thrust of each motor)


