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

Version 8.0 - Mar 2024 - In this version, the driver is a truck and the towed system is a hybrid quadcopter / airplane. This code was heavily modified and edited by Zach Miller

Version 9.0 - Oct 2025 - in this version the driver is a truck but you can really just make it any pulling vehicle. the towed system can be a quadcopter, airplane, hybrid or a ball edited by Carlos Montalvo (also merged Zach's Miller hybrid branch into main) 

