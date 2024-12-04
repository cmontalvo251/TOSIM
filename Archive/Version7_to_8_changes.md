Version 7 to Version 8 Changes

0.) The driver is now a truck and the towed system is now a hybrid vehicle
1.) Nominal.CS file has been moved into each vehicles respective files - Read routines were updated as well
2.) There is now only 1 control subroutine - This got rid of the handshake stuff
	Driver - Currently contains a drive forward controller (The waypoint controller is in version 7.0 and will get copied later)
	Towed - Only inner loop control
	Tether - No change from Version 7.0
3.) An attempt was made to remove all AoA sweeps
4.) Echo routine completely removed - Which means read routine is flag eq 1 and then compute is flag 2
5.) The DRIVER and TOWED functions are now passed the T variable rather than T%DRIVER or T%TOWED
6.) Made sure that tether reel and connection points are all properly aligned
7.) State vectors are properly aligned now since the towed structure now has 8 extra states (thrust of each motor) So we have
1-12 states are the driver, 13-33 are the hybrid vehicle (13 states with quaternions and then 8 more for motors (Thrust and Thrustdot)), and then the rest are bead states
8.) The towed system is a hybrid vehicle which means there are aerodynamics from an airplane and thrust from 4 motors

