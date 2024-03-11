!!!!!!!!!!!!!!! MODULE TOSIMDATATYPES!!!!!!!!!!!!!!!!!!!!!!!!!!
module TOSIMDATATYPES
IMPLICIT NONE
 integer,parameter :: MAXNBEADS = 100              ! Units: 'nd', Desc: 'Maximum Number of Tether Beads'
 integer,parameter :: MAXNALT = 200                ! Units: 'nd', Desc: 'Maximum Number of Atmosphere Altitude Table Points'
 integer,parameter :: MAXNLSE = 20                 ! Units: 'nd', Desc: 'Maximum Number of Lifting Surface Elements'
 integer,parameter :: MAXPROP = 20                 ! Units: 'nd', Desc: 'Maximum Number of Propellor Table size
 integer,parameter :: MAXNAOA = 100                ! Units: 'nd', Desc: 'Maximum Number of Aerodynamic Angle of Attack Table Points'
 integer,parameter :: MAXX = 1000                  ! Units: 'nd', Desc: 'Maximum Number of System States'
 integer,parameter :: NOACTUATORS = 9              ! Units: 'nd', Desc: Number of Actuators
 integer,parameter :: FT2M = 0.3048                ! Conversion from Feet to Meters
 integer,parameter :: M2FT = 3.28084               ! Conversion from Meters to Feet
 real*8,parameter  :: PI = 3.14159265358979323846  ! Units: 'nd', Desc: 'Pi'

!!!!!!!!!!!!!!!!!!!!!!ATMOSPHERE STRUCTURE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

type ATMOSPHERESTRUCTURE
    integer :: markX = 1                             ! X coordinate Marker in interpolation routine 
    integer :: markY = 1                             ! Y coordinate Marker in interpolation routine 
    integer :: markZ = 1                             ! Z coordinate Marker in interpolation routine 
    integer :: markT = 1                             ! T coordinate Marker in interpolation routine 
    integer :: markXT = 1                            ! X coordinate Marker in turbulence interpolation routine 
    integer :: markYT = 1                            ! Y coordinate Marker in turbulence interpolation routine 
    integer :: markZT = 1                            ! Z coordinate Marker in turbulenceinterpolation routine 
    integer :: bounds = 0                            ! Flag indicating wether or not you have gone out of WRF bounds
    integer :: boundflag = 1                         ! Flag indicating wether or not you have gone out of WRF bounds
    integer :: boundsT = 0                           ! Flag indicating wether or not you have gone out of TURB bounds
    integer :: boundflagT = 1                        ! Flag indicating wether or not you have gone out of TURB bounds
    integer :: parameters(5) = 0                     ! Vector of a few parameters 
    integer :: tcoord(601) = 0                       ! Vector of time
    integer :: dim = 40                              ! Dimension of Spatial wind data matrices
    integer :: dimT = 500                            ! Dimension of Turbulence wind data matrices
    integer :: tlength = 601                         ! Dimension of time vector
    integer :: OFFON = 0                             ! Units: 'nd', Desc: 'Off/On Switch'
    integer :: TABSIZE = 0                           ! Units: 'nd', Desc: 'Table Size'
    integer :: MODNO = 1                             ! Units: 'nd', Desc: 'Model Number'
    integer :: IP = 1                                ! Units: 'nd', Desc: 'Table Pointer'
    integer :: DQFLAG = 0                            ! Units: 'nd', Desc: 'Data Quality Flag (0=Data Not Loaded Successfully, 1=Data Loaded Successfully)'
    integer :: TIMEVARYING = 0                       ! Units: 'nd', Desc: 'Static Wind field or not
    real*8 :: FREQUENCY = 1                          ! Units: '1/s', Desc: 'frequency of sinusoidal winds
    real*8 :: TIMEON = 0                             ! Units: 'sec', Desc: 'Time to turn on winds'
    real*8 :: zcoord(40) = 0                         ! Z coordinates of wind data in z (ft)
    real*8 :: xcoord(40) = 0                         ! X coordinates of wind data in x (ft)
    real*8 :: ycoord(40) = 0                         ! Y coordinates of wind data in y (ft)
    real*8 :: xcoordT(500) = 0                       ! X coordinates of turbulence data in x (ft)
    real*8 :: ycoordT(500) = 0                       ! Y coordinates of turbulence data in y (ft)
    !real*8 :: terrain(40,40) = 0                     ! Terrain height in meters
    !!!REVISIT REVISIT REVIST - Always make sure this is back to normal
    ! real*8 :: U0(1,1,1),V0(1,1,1),W0(1,1,1)          ! U,V,W velocity at timestep t (ft/s)
    ! real*8 :: Udt(1,1,1),Vdt(1,1,1),Wdt(1,1,1)       ! U,V,W velocity at timestep t+dt (ft/s)
    ! real*8 :: UTURB(1,1),VTURB(1,1),WTURB(1,1)       ! U,V,W turbulence at timestep t (ft/s)
    real*8 :: U0(40,40,40),V0(40,40,40),W0(40,40,40) ! U,V,W velocity at timestep t (ft/s)
    real*8 :: Udt(40,40,40),Vdt(40,40,40),Wdt(40,40,40) ! U,V,W velocity at timestep t+dt (ft/s)
    real*8 :: UTURB(500,500),VTURB(500,500),WTURB(500,500) ! U,V,W turbulence at timestep t (ft/s)
    real*8 :: dx = 0                                 ! X resolution (ft)
    real*8 :: dxT = 1                                ! XY resolution of turbulence (ft)
    real*8 :: dyT = 1                                ! XY resolution of turbulence (ft)
    real*8 :: dy = 0                                 ! X resolution (ft)
    real*8 :: ztop = 0                               ! Highest altitude to sample from (ft)
    real*8 :: IWINDSCALE = 1                         ! Scale of WRF winds
    real*8 :: TURBLEVEL = 1                          ! Scale of Turbulence 
    real*8 :: Vtrim = 20                             ! Trim velocity of craft flying through winds
    real*8 :: TIMESTEP = 0.0001                      ! Timestep of Simulation
    real*8 :: WINDGUST(3) = 0                        ! Vector holding WINDGUST values at current timestep (ft/s)
    real*8 :: VWAKE(3) = 0                           ! Vector holding wake data values at current timestep (ft/s)
    real*8 :: WRFX = 0                               ! Wrf winds in x (ft/s)
    real*8 :: WRFY = 0                               ! Wrf winds in y (ft/s)
    real*8 :: WRFZ = 0                               ! Wrf winds in z (ft/s)
    real*8 :: ALT = 0.0                              ! Units: 'ft', Desc: 'Altitude'
    real*8 :: DEN = 0.002363                         ! Units: 'slug/ft^3', Desc: 'Density'
    real*8 :: SOS = 1086.336                         ! Units: 'slug/ft^3', Desc: 'Density'
    real*8 :: WINDSPEED = 0.0                        ! Units: 'ft/s', Desc: 'Wind Speed'
    real*8 :: WINDDIR = 0.0                          ! Units: 'rad', Desc: 'Wind Direction'
    real*8 :: WINDELEV = 0.0                         ! Units: 'rad', Desc: 'Wind Elevation'
    real*8 :: XI = 0.0                               ! Units: 'ft', Desc: 'Inertial X Position'
    real*8 :: YI = 0.0                               ! Units: 'ft', Desc: 'Inertial Y Position'
    real*8 :: ZI = 0.0                               ! Units: 'ft', Desc: 'Inertial Z Position'
    real*8 :: VXWIND = 0.0                           ! Units: 'ft/s', Desc: 'Atmospheric Wind Along Inertial I Axis'
    real*8 :: VYWIND = 0.0                           ! Units: 'ft/s', Desc: 'Atmospheric Wind Along Inertial J Axis'
    real*8 :: VZWIND = 0.0                           ! Units: 'ft/s', Desc: 'Atmospheric Wind Along Inertial K Axis'
    real*8 :: ALTTAB(MAXNALT) = 0.0                  ! Units: 'ft', Desc: 'Density Altitude Table'
    real*8 :: DENTAB(MAXNALT) = 0.0                  ! Units: 'slug/ft^3', Desc: 'Density Table'
    real*8 :: VXWINDTAB(MAXNALT) = 0.0               ! Units: 'ft/s', Desc: 'VXWIND Wind Table'
    real*8 :: VYWINDTAB(MAXNALT) = 0.0               ! Units: 'ft/s', Desc: 'VYWIND Wind Table'
    real*8 :: VZWINDTAB(MAXNALT) = 0.0               ! Units: 'ft/s', Desc: 'VZWIND Wind Table'
    real*8 :: xshift = 0                             ! Units: 'ft' , 'Desc: shift in x coordinate for interpolation
    real*8 :: yshift = 0                             ! Units: 'ft' , 'Desc: shift in y coordinate for interpolation
    real*8 :: zshift = 0                             ! Units: 'ft' , 'Desc: shift in y coordinate for interpolation
    real*8 :: xshiftT = 0                            ! Units: 'ft' , 'Desc: shift in x coordinate for interpolation
    real*8 :: yshiftT = 0                            ! Units: 'ft' , 'Desc: shift in y coordinate for interpolation
    real*8 :: zshiftT = 0                            ! Units: 'ft' , 'Desc: shift in y coordinate for interpolation
    real*8 :: PSIOFFSET = 0                          ! Units: 'rad' ,'Desc: used to rotate the dryden and WRF model'
    real*8 :: XOFFSET = 0                            ! Units: 'ft' ,'Desc: used to offset WRF model
    real*8 :: YOFFSET = 0                            ! Units: 'ft' ,'Desc: used to offset WRF model
    real*8 :: WAVESPEED(2) = 0                       ! Units: 'ft/s', Desc: Speed that the static waves propagate to simulate time varying component
    real*8 :: RANDOMIZE = 0                          ! Units: 'nd', Desc: randomization parameter for wind field. 0.1 = 10 percent gaussian noise
    character*128 PATH
    character*256 U0name,V0name,W0name,UTurbname,Vturbname,Wturbname
    character*256 Udtname,Vdtname,Wdtname
 end type ATMOSPHERESTRUCTURE

!!!!!!!!!!!!!!!!!!!!!!!!!! DRIVER STRUCTURE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  integer,parameter :: MAXNDRIVER = 10             ! Units: 'nd', Desc: 'Maximum Number of Driver Motion Table Points'
  integer,parameter :: MAXWP = 1000             ! Units: 'nd', Desc: 'Maximum Number of Driver Motion Table Points'
  integer,parameter :: IMAX = 1                    ! Units: 'nd', Desc: 'Maximum Airwake index in X
  integer,parameter :: JMAX = 1                    ! Units: 'nd', Desc: 'Maximum Airwake index in Y
  integer,parameter :: KMAX = 1                    ! Units: 'nd', Desc: 'Maximum Airwake index in Z
  integer,parameter :: NTIMES = 1                  ! Units: 'nd', Desc: 'Maximum Airwake index for t
  real*8,parameter  :: qPI = 3.14159265358979323846  ! Units: 'nd', Desc: 'Pi' - using qPI to not conflict with any other usage of PI
  type DRIVERSTRUCTURE
     integer :: OFFON = 0                             ! Units: 'nd', Desc: 'Off/On Switch'
     integer :: DYNOFFON = 0                             ! Units: 'nd', Desc: 'Off/On Switch'
     integer :: GRAVOFFON = 0                         ! Units: 'nd', Desc: 'Gravity Flag (0=Off, 1=On)'
     integer :: AEROOFFON = 0                         ! Units: 'nd', Desc: 'Aerodynamics Flag (0=Off, 1=On)'
     integer :: MODNO = 1                             ! Units: 'nd', Desc: 'Model Number Flag'
     integer :: IP = 1                                ! Units: 'nd', Desc: 'Time Table Look-Up Pointer'
     integer :: TABSIZE = 1                           ! Units: 'nd', Desc: 'Time Table Size'
     integer :: DQFLAG = 0                            ! Units: 'nd', Desc: 'Data Quality Flag (0=Data Not Loaded Successfully, 1=Data Loaded Successfully)'
     integer :: AIRWAKE = 0                           ! Units: 'nd', Desc: 'Air wake on or off'
     integer :: markX = 1                             ! Units: 'nd', X index placeholder
     integer :: markY = 1                             ! Units: 'nd', X index placeholder
     integer :: markZ = 1                             ! Units: 'nd', X index placeholder
     integer :: markT = 1                             ! Units: 'nd', X index placeholder
     integer :: DOWNWASHONOFF = 0                     ! Units: 'nd', X index placeholder
     integer :: CONTROLOFFON = 0
     integer :: WAYPOINT = 1                          ! Defaults to 1 but apparently can change in input file
     integer :: NUMWAYPOINTS = 0                         ! Defaults to 1 but apparently can change in input file
     integer :: COMMANDFLAG(MAXWP) = 1           ! 0 or 1 to advance waypoint command     real*8 :: COMMANDFLAG(MAXWP) =
     real*8 :: DXD = 0                                ! Driver aero parameter
     real*8 :: C_T = 0                                ! Driver aero parameter
     real*8 :: MINWAYPOINT = 1.0D0
     real*8 :: MS_MIN = 0.0                           ! Maximum PWM signal (microseconds)
     real*8 :: MS_MAX = 0.0                           ! Minimum PWM signal (microseconds)
     real*8 :: SLCG = 0.0                             ! Units: 'ft', Desc: 'Stationline of Mass Center of Driver'
     real*8 :: BLCG = 0.0                             ! Units: 'ft', Desc: 'Buttline of Mass Center of Driver'
     real*8 :: WLCG = 0.0                             ! Units: 'ft', Desc: 'Waterline of Mass Center of Driver'
     real*8 :: SLREEL = 0.0                           ! Units: 'ft', Desc: 'Stationline of Tether Reel Point on Driver'
     real*8 :: BLREEL = 0.0                           ! Units: 'ft', Desc: 'Buttline of Tether Reel Point on Driver'
     real*8 :: WLREEL = 0.0                           ! Units: 'ft', Desc: 'Waterline of Tether Reel Point on Driver'
     real*8 :: SLAIRWAKE = 0.0                        ! Units: 'ft', Desc: 'Stationline of Airwake grid start on Driver
     real*8 :: BLAIRWAKE = 0.0                        ! Units: 'ft', Desc: 'Buttline of Airwake grid start on Driver
     real*8 :: WLAIRWAKE = 0.0                        ! Units: 'ft', Desc: 'Waterline of Airwake grid start on Driver
     real*8 :: SPEED = 0.0                            ! Units: 'ft/s', Desc: 'Speed of Driver'
     real*8 :: FINALSPEED = 0.0                       ! Units: 'ft/s', Desc: 'Initial Speed of Driver'
     real*8 :: RESTARTSPEED = -999                    ! Units: 'ft/s', Desc: 'Restart speed of Driverdriver' 
     real*8 :: DIRECTION = 0.0                        ! Units: 'rad', Desc: 'Direction of Motion of Driver'
     real*8 :: XCGINITIAL = 0.0                       ! Units: 'ft', Desc: 'Initial XCG Inertial Position of Driver'
     real*8 :: YCGINITIAL = 0.0                       ! Units: 'ft', Desc: 'Initial YCG Inertial Position of Driver'
     real*8 :: ZCGINITIAL = 0.0                       ! Units: 'ft', Desc: 'Initial ZCG Inertial Position of Driver'
     real*8 :: TIME = 0.0                             ! Units: 's', Desc: 'Time'
     real*8 :: TIMEON = 0                             ! Units: 's', Desc: Time for Driver to come on
     real*8 :: XCG = 0.0                              ! Units: 'ft', Desc: 'XCG Inertial Position of Driver'
     real*8 :: YCG = 0.0                              ! Units: 'ft', Desc: 'YCG Inertial Position of Driver'
     real*8 :: ZCG = 0.0                              ! Units: 'ft', Desc: 'ZCG Inertial Position of Driver'
     real*8 :: PHI = 0.0                              ! Units: 'rad', Desc: 'Driver Euler Roll Angle'
     real*8 :: THETA = 0.0                            ! Units: 'rad', Desc: 'Driver Euler Pitch Angle'
     real*8 :: PSI = 0.0                              ! Units: 'rad', Desc: 'Driver Euler Yaw Angle'
     real*8 :: PSIPREV = 0.0                          ! Units: 'rad', Desc: 'Driver Euler Yaw Angle'
     real*8 :: UB = 0.0                               ! Units: 'ft/s', Desc: 'UB Body Frame Mass Center Velocity of Driver'
     real*8 :: VB = 0.0                               ! Units: 'ft/s', Desc: 'VB Body Frame Mass Center Velocity of Driver'
     real*8 :: WB = 0.0                               ! Units: 'ft/s', Desc: 'WB Body Frame Mass Center Velocity of Driver'
     real*8 :: PB = 0.0                               ! Units: 'rad/s', Desc: 'PB Roll Rate of Driver'
     real*8 :: QB = 0.0                               ! Units: 'rad/s', Desc: 'QB Pitch Rate of Driver'
     real*8 :: RB = 0.0                               ! Units: 'rad/s', Desc: 'RB Yaw Rate of Driver'
     real*8 :: XDOT = 0                               ! Units: 'ft/s', Desc: Inertial Frame X velocity of Driver
     real*8 :: YDOT = 0                               ! Units: 'ft/s', Desc: Inertial Frame X velocity of Driver
     real*8 :: ZDOT = 0                               ! Units: 'ft/s', Desc: Inertial Frame X velocity of Driver
     real*8 :: XREEL = 0.0                            ! Units: 'ft', Desc: 'XREEL Inertial Position of Reel'
     real*8 :: YREEL = 0.0                            ! Units: 'ft', Desc: 'YREEL Inertial Position of Reel'
     real*8 :: ZREEL = 0.0                            ! Units: 'ft', Desc: 'ZREEL Inertial Position of Reel'
     real*8 :: XREELDOT = 0.0                         ! Units: 'ft/s', Desc: 'XREEL DOT Inertial Velocity of Reel'
     real*8 :: YREELDOT = 0.0                         ! Units: 'ft/s', Desc: 'YREEL DOT Inertial Velocity of Reel'
     real*8 :: ZREELDOT = 0.0                         ! Units: 'ft/s', Desc: 'ZREEL DOT Inertial Velocity of Reel'
     real*8 :: UREEL = 0.0                            ! Units: 'ft', Desc: 'UREEL Inertial Velcity of Reel in Driver Reference Frame'
     real*8 :: VREEL = 0.0                            ! Units: 'ft', Desc: 'VREEL Inertial Velcity of Reel in Driver Reference Frame'
     real*8 :: WREEL = 0.0                            ! Units: 'ft', Desc: 'WREEL Inertial Velcity of Reel in Driver Reference Frame'
     !REVISIT REVISIT REVISIT
     ! real*8 :: UDRIVER(1,1,1),VDRIVER(1,1,1),WDRIVER(1,1,1) ! U,V,W Driver airwake velocity at timestep t (ft/s)
     ! real*8 :: UDRIVERDT(1,1,1),VDRIVERDT(1,1,1),WDRIVERDT(1,1,1) ! U,V,W airwake velocity at timestep t+dt (ft/s)
     ! real*8 :: XDRIVER(1,1,1),YDRIVER(1,1,1),ZDRIVER(1,1,1) ! U,V,W airwake velocity at timestep t+dt (ft/s)
     real*8 :: UDRIVER(IMAX,JMAX,KMAX),VDRIVER(IMAX,JMAX,KMAX),WDRIVER(IMAX,JMAX,KMAX) ! U,V,W Driver airwake velocity at timestep t (ft/s)
     real*8 :: UDRIVERDT(IMAX,JMAX,KMAX),VDRIVERDT(IMAX,JMAX,KMAX),WDRIVERDT(IMAX,JMAX,KMAX) ! U,V,W airwake velocity at timestep t+dt (ft/s)
     real*8 :: XDRIVER(IMAX,JMAX,KMAX),YDRIVER(IMAX,JMAX,KMAX),ZDRIVER(IMAX,JMAX,KMAX) ! U,V,W airwake velocity at timestep t+dt (ft/s)
     real*8 :: TCOORD(NTIMES)                            ! Units: 's' Desc: Airwake time intervals
     real*8 :: XCOORD(IMAX)                            ! Units: 's' Desc: Airwake time intervals
     real*8 :: YCOORD(JMAX)                            ! Units: 's' Desc: Airwake time intervals
     real*8 :: ZCOORD(KMAX)                            ! Units: 's' Desc: Airwake time intervals
     real*8 :: TIMETAB(MAXNDRIVER) = 0.0           ! Units: 's', Desc: 'Time Table'
     real*8 :: XCGTAB(MAXNDRIVER) = 0.0            ! Units: 'ft', Desc: 'XCG Inertial Position of Driver Table'
     real*8 :: YCGTAB(MAXNDRIVER) = 0.0            ! Units: 'ft', Desc: 'YCG Inertial Position of Driver Table'
     real*8 :: ZCGTAB(MAXNDRIVER) = 0.0            ! Units: 'ft', Desc: 'ZCG Inertial Position of Driver Table'
     real*8 :: PHITAB(MAXNDRIVER) = 0.0            ! Units: 'rad', Desc: 'Driver Euler Roll Angle Table'
     real*8 :: THETATAB(MAXNDRIVER) = 0.0          ! Units: 'rad', Desc: 'Driver Euler Pitch Angle Table'
     real*8 :: PSITAB(MAXNDRIVER) = 0.0            ! Units: 'rad', Desc: 'Driver Euler Yaw Angle Table'
     real*8 :: UBTAB(MAXNDRIVER) = 0.0             ! Units: 'ft/s', Desc: 'UB Body Frame Mass Center Velocity of Driver Table'
     real*8 :: VBTAB(MAXNDRIVER) = 0.0             ! Units: 'ft/s', Desc: 'VB Body Frame Mass Center Velocity of Driver Table'
     real*8 :: WBTAB(MAXNDRIVER) = 0.0             ! Units: 'ft/s', Desc: 'WB Body Frame Mass Center Velocity of Driver Table'
     real*8 :: PBTAB(MAXNDRIVER) = 0.0             ! Units: 'rad/s', Desc: 'PB Roll Rate of Driver Table'
     real*8 :: QBTAB(MAXNDRIVER) = 0.0             ! Units: 'rad/s', Desc: 'QB Pitch Rate of Driver Table'
     real*8 :: RBTAB(MAXNDRIVER) = 0.0             ! Units: 'rad/s', Desc: 'RB Yaw Rate of Driver Table'
     real*8 :: INITIALSTATE(20) = 0                    ! Units: 'varies', Desc: 'Initial State Vector'
     real*8 :: NOMINALSTATE(20) = 0                    ! Units: 'varies', Desc: 'Initial State Vector'
     real*8 :: STATE(20) = 0                           ! Units: 'varies', Desc: 'State Vector'
     real*8 :: STATEDOT(20) = 0                        ! Units: 'varies', Desc: 'State Dot Vector'
     real*8 :: KRKBODY(20,4) = 0.0                  ! Units: 'md', RK4 vector
     real*8 :: TIS(3,3) = 0.0                          ! Units: 'nd', Desc: 'Transformation from Driver to Inertial Reference Frame'
     real*8 :: TIC(3,3) = 0.0                          ! Units: 'nd', Desc: 'Transformation from Driver to Inertial Reference Frame'
     real*8 :: TCI(3,3) = 0.0                          ! Units: 'nd', Desc: 'Transformation from Driver to Inertial Reference Frame'
     real*8 :: XDDOTNOISE = 0.0                        ! Units: 'ft/s^2', Desc: 'Acceleration Noise of Driverdriver'
     real*8 :: YDDOTSCALE = 0.0                        ! Units: 'ft/s^2', Desc: 'Acceleration Noise of Driverdriver'
     real*8 :: YDDOTPERIOD = 0.0                        ! Units: 'ft/s^2', Desc: 'Acceleration Noise of Driverdriver'
     real*8 :: IXX = 0.0                              ! Units: 'kg m^2', Desc: 'Ixx of Inertia Matrix'
     real*8 :: IYY = 0.0                              ! Units: 'kg m^2', Desc: 'Iyy of Inertia Matrix'
     real*8 :: IZZ = 0.0                              ! Units: 'kg m^2', Desc: 'Izz of Inertia Matrix'
     real*8 :: IXY = 0.0                              ! Units: 'kg m^2', Desc: 'Ixy of Inertia Matrix'
     real*8 :: IXZ = 0.0                              ! Units: 'kg m^2', Desc: 'Ixz of Inertia Matrix'
     real*8 :: IYZ = 0.0                              ! Units: 'kg m^2', Desc: 'Iyz of Inertia Matrix'
     real*8 :: IXXI = 0.0                             ! Units: '1/(kg m^2)', Desc: 'Ixx Inverse of Inertia Matrix'
     real*8 :: IYYI = 0.0                             ! Units: '1/(kg m^2)', Desc: 'Iyy Inverse of Inertia Matrix'
     real*8 :: IZZI = 0.0                             ! Units: '1/(kg m^2)', Desc: 'Izz Inverse of Inertia Matrix'
     real*8 :: IXYI = 0.0                             ! Units: '1/(kg m^2)', Desc: 'Ixy Inverse of Inertia Matrix'
     real*8 :: IXZI = 0.0                             ! Units: '1/(kg m^2)', Desc: 'Ixz Inverse of Inertia Matrix'
     real*8 :: IYZI = 0.0                             ! Units: '1/(kg m^2)', Desc: 'Iyz Inverse of Inertia Matrix'
     real*8 :: MASS = 0.0                             ! Units: 'kg', Desc: 'Mass'
     real*8 :: WEIGHT = 0.0                           ! Units: 'lbf', Desc: 'Weight'
     real*8 :: TURNRADIUS = 0.0                       ! Units: 'ft', Desc: Minimum turn radius of driverdriver
     real*8 :: FXTOTAL = 0.0                          ! Units: 'lbf', Desc: 'X Total Applied Force in Body Frame'
     real*8 :: FYTOTAL = 0.0                          ! Units: 'lbf', Desc: 'Y Total Applied Force in Body Frame'
     real*8 :: FZTOTAL = 0.0                          ! Units: 'lbf', Desc: 'Z Total Applied Force in Body Frame'
     real*8 :: MXTOTAL = 0.0                          ! Units: 'N m', Desc: 'X Total Applied Moment About Mass Center in Body Frame'
     real*8 :: MYTOTAL = 0.0                          ! Units: 'N m', Desc: 'Y Total Applied Moment About Mass Center in Body Frame'
     real*8 :: MZTOTAL = 0.0                          ! Units: 'N m', Desc: 'Z Total Applied Moment About Mass Center in Body Frame'
     real*8 :: FXGRAV = 0.0                           ! Units: 'lbf', Desc: 'X Gravity Forces in Body Frame'
     real*8 :: FYGRAV = 0.0                           ! Units: 'lbf', Desc: 'Y Gravity Forces in Body Frame'
     real*8 :: FZGRAV = 0.0                           ! Units: 'lbf', Desc: 'Z Gravity Forces in Body Frame'
     real*8 :: MXGRAV = 0.0                           ! Units: 'N m', Desc: 'X Gravity Moment About CG in Body Frame'
     real*8 :: MYGRAV = 0.0                           ! Units: 'N m', Desc: 'Y Gravity Moment About CG in Body Frame'
     real*8 :: MZGRAV = 0.0                           ! Units: 'N m', Desc: 'Z Gravity Moment About CG in Body Frame'
     real*8 :: FXAERO = 0.0                           ! Units: 'lbf', Desc: 'X Aerodynamic Force in Body Frame'
     real*8 :: FYAERO = 0.0                           ! Units: 'lbf', Desc: 'Y Aerodynamic Force in Body Frame'
     real*8 :: FZAERO = 0.0                           ! Units: 'lbf', Desc: 'Z Aerodynamic Force in Body Frame'
     real*8 :: MXAERO = 0.0                           ! Units: 'N m', Desc: 'X Aerodynamic Moment About CG in Body Frame'
     real*8 :: MYAERO = 0.0                           ! Units: 'N m', Desc: 'Y Aerodynamic Moment About CG in Body Frame'
     real*8 :: MZAERO = 0.0                           ! Units: 'N m', Desc: 'Z Aerodynamic Moment About CG in Body Frame'
     real*8 :: FXCONT = 0.0                           ! Units: 'lbf', Desc: 'X Contact Force in Body Frame'
     real*8 :: FYCONT = 0.0                           ! Units: 'lbf', Desc: 'Y Contact Force in Body Frame'
     real*8 :: FZCONT = 0.0                           ! Units: 'lbf', Desc: 'Z Contact Force in Body Frame'
     real*8 :: MXCONT = 0.0                           ! Units: 'N m', Desc: 'X Contact Moment in Body Frame'
     real*8 :: MYCONT = 0.0                           ! Units: 'N m', Desc: 'Y Contact Moment in Body Frame'
     real*8 :: MZCONT = 0.0                           ! Units: 'N m', Desc: 'Z Contact Moment in Body Frame'
     real*8 :: VXWIND = 0.0                           ! Units: 'ft/s', Desc: 'Atmospheric Wind Along Inertial I Axis'
     real*8 :: VYWIND = 0.0                           ! Units: 'ft/s', Desc: 'Atmospheric Wind Along Inertial J Axis'
     real*8 :: VZWIND = 0.0                           ! Units: 'ft/s', Desc: 'Atmospheric Wind Along Inertial K Axis'
     real*8 :: DEN = 0.002363                         ! Units: 'slug/ft^3', Desc: 'Density'
     real*8 :: FTETHERX = 0.0
     real*8 :: FTETHERY = 0.0
     real*8 :: FTETHERZ = 0.0
     real*8 :: VWAKE(3) = 0.0
     real*8 :: AIRWAKEPOSITION(3) = 0.0
     real*8 :: GRAVITY = 32.2			      ! Units: 'ft/s this is the default but you can change it in the input file
     real*8 :: XCOMMAND = 0.0
     real*8 :: YCOMMAND = 0.0
     real*8 :: ZCOMMAND = 0.0
     real*8 :: PHICOMMAND = 0.0
     real*8 :: THETACOMMAND = 0.0
     real*8 :: PSICOMMAND = 0.0
     real*8 :: UCOMMAND = 0.0
     real*8 :: KPXDRIVE = 0                            ! Units: 'nd', Desc: driverdriver proportional gain on x-position
     real*8 :: KIXDRIVE = 0                            ! Units: 'nd', Desc: driverdriver integral gain on x-position
     real*8 :: KDXDRIVE = 0                            ! Units: 'nd', Desc: driverdriver derivative gain on x-position
     real*8 :: KPYDRIVE = 0                            ! Units: 'nd', Desc: driverdriver proportional gain on y-position
     real*8 :: KIYDRIVE = 0                            ! Units: 'nd', Desc: driverdriver integral gain on y-position
     real*8 :: KDYDRIVE = 0                            ! Units: 'nd', Desc: driverdriver derivative gain on y-position
     real*8 :: KPZDRIVE = 0                            ! Units: 'nd', Desc: driverdriver proportional gain on z-position
     real*8 :: KIZDRIVE = 0                            ! Units: 'nd', Desc: driverdriver integral gain on z-position 
     real*8 :: KDZDRIVE = 0                            ! Units: 'nd', Desc: driverdriver derivative gain on z-position
     real*8 :: KPPSI = 0                              ! Units: 'nd', Desc: driverdriver proportional gain on heading
     real*8 :: KIPSI = 0                              ! Units: 'nd', Desc: driverdriver integral gain on heading
     real*8 :: KDPSI = 0                              ! Units: 'nd', Desc: driverdriver derivative gain on heading
     real*8 :: KPPHI = 0                              ! Units: 'nd', Desc: driverdriver proportional gain on roll 
     real*8 :: KIPHI = 0                              ! Units: 'nd', Desc: driverdriver integral gain on roll 
     real*8 :: KDPHI = 0                              ! Units: 'nd', Desc: driverdriver derivative gain on roll
     real*8 :: KPTHETA = 0                            ! Units: 'nd', Desc: driverdriver proportional gain on pitch
     real*8 :: KITHETA = 0                            ! Units: 'nd', Desc: driverdriver integral gain on pitch
     real*8 :: KDTHETA = 0                            ! Units: 'nd', Desc: driverdriver derivative gain on pitch
     real*8 :: XINTEGRAL = 0
     real*8 :: YINTEGRAL = 0
     real*8 :: ZINTEGRAL = 0
     real*8 :: PHIINTEGRAL = 0.0                      ! Units: 'ft', Desc: 'PHI integral term in PID Controller'
     real*8 :: THETAINTEGRAL = 0.0                    ! Units: 'ft', Desc: 'THETA integral term in PID Controller'
     real*8 :: PSIINTEGRAL = 0.0                      ! Units: 'ft', Desc: 'PSI integral term in PID Controller'
     real*8 :: UINTEGRAL = 0.0
     real*8 :: MUTHROTTLE = 0                         ! Units: 'microseconds', Desc: 'PWM signal to motor'
     real*8 :: THRUSTVEC = 0                          ! Units: N, Desc: Thrust
     real*8 :: XCOM(MAXWP) = 500                         ! Units: 'm', X Waypoint command
     real*8 :: YCOM(MAXWP) = 500                         ! Units: 'm', Y Waypoint command
     real*8 :: ZCOM(MAXWP) = -200                        ! Units: 'm', Altitude command
     character*256 U0name,V0name,W0name
     character*256 Udtname,Vdtname,Wdtname,AIRWAKEPATH
     character(128) :: INPUTFILE = ' '                ! Units: 'nd', Desc: 'Driver Input File'     chara
  end type DRIVERSTRUCTURE

!!!!!!!!!!!!!!!!!!!!!!!!!!! TOWED STRUCTURE !!!!!!!!!!!!!!!!!!!!!!!!!!!!
 
 type TOWEDSTRUCTURE
  integer :: DYNOFFON = 0                          ! Units: 'nd', Desc: 'Dynamics Flag (0=Off, 1=On)'
  integer :: GRAVOFFON = 0                         ! Units: 'nd', Desc: 'Gravity Flag (0=Off, 1=On)'
  integer :: CONTROLOFFON = 0                      ! Units: 'nd', Desc: 'Control Flag (0=off, 1=On)'
  integer :: AEROFLAG = 0                          ! Units: 'nd', Desc: 'Aerodynamics Flag (0=Off, 1=On)'
  integer :: DQFLAG = 0                            ! Units: 'nd', Desc: 'Data Quality Flag (0=Data Not Loaded Successfully, 1=Data Loaded Successfully)'
  real*8 :: GRAVITY = 32.2                         ! Gravity constant on Earth (ft/s^2)
  real*8 :: ALC = 0                                ! Quadcopter aero parameter
  real*8 :: ALS = 0                                ! Quadcopter aero parameter
  real*8 :: DXD = 0                                ! Quadcopter aero parameter
  real*8 :: DYD = 0                                ! Quadcopter aero parameter
  real*8 :: RNEW = 0                               ! Quadcopter aero parameter
  real*8 :: C_T = 0                                ! Quadcopter aero parameter
  real*8 :: C_TAU = 0                              ! Quadcopter aero parameter
  real*8 :: LPHI12 = 0                             ! Quadcopter aero parameter
  real*8 :: LPHI34 = 0                             ! Quadcopter aero parameter
  real*8 :: LTHETA12 = 0                           ! Quadcopter aero parameter
  real*8 :: LTHETA34 = 0                           ! Quadcopter aero parameter
  real*8 :: OMEGAMAX = 0                           ! Quadcopter aero parameter
  real*8 :: OMEGAVEC(4,1) = 0                      ! Quadcopter aero parameter
  real*8 :: THRUSTVEC(4,1) = 0                     ! Quadcopter aero parameter
  real*8 :: MUVEC(4,1) = 0                         ! Quadcopter aero parameter
  real*8 :: AILERON = 0                            ! Adding an aileron parameter just in case we want an airplane being towed
  real*8 :: ELEVATOR = 0                           ! Adding an elevator parameter just in case we use the manta
  real*8 :: RUDDER = 0                             ! Adding a rudder parameter just in case we use the manta
  real*8 :: FLAPS = 0                              ! Adding flaps parameter in case we use an airplane later
  real*8 :: DELTHRUST = 0                          ! Adding incase we use an airplane later
  real*8 :: TURNRADIUS = 0
  real*8 :: V_T = 0                                ! Velocity (ft/s)
  real*8 :: KT = 0                                 ! Quadcopter Aero Parameter
  real*8 :: OMEGA0 = 0                             ! Quadcopter Aero Parameter
  real*8 :: IRR = 0                                ! Quadcopter Aero Parameter
  real*8 :: MS_MIN = 0.0                           ! Maximum PWM signal (microseconds)
  real*8 :: MS_MAX = 0.0                           ! Minimum PWM signal (microseconds)
  real*8 :: DOWNWASH = 0.0                         ! Units: 'nd', Magnitude of Downwash
  real*8 :: DIAMETER = 0.0                         ! Units: 'nd', Diameter of Downwash (ft)
  real*8 :: MASS = 0.0                             ! Units: 'kg', Desc: 'Mass'
  real*8 :: WEIGHT = 0.0                           ! Units: 'lbf', Desc: 'Weight'  
  real*8 :: SLCG = 0.0                             ! Units: 'ft', Desc: 'Stationline of Mass Center'
  real*8 :: BLCG = 0.0                             ! Units: 'ft', Desc: 'Buttline of Mass Center'
  real*8 :: WLCG = 0.0                             ! Units: 'ft', Desc: 'Waterline of Mass Center'
  real*8 :: IXX = 0.0                              ! Units: 'kg m^2', Desc: 'Ixx of Inertia Matrix'
  real*8 :: IYY = 0.0                              ! Units: 'kg m^2', Desc: 'Iyy of Inertia Matrix'
  real*8 :: IZZ = 0.0                              ! Units: 'kg m^2', Desc: 'Izz of Inertia Matrix'
  real*8 :: IXY = 0.0                              ! Units: 'kg m^2', Desc: 'Ixy of Inertia Matrix'
  real*8 :: IXZ = 0.0                              ! Units: 'kg m^2', Desc: 'Ixz of Inertia Matrix'
  real*8 :: IYZ = 0.0                              ! Units: 'kg m^2', Desc: 'Iyz of Inertia Matrix'
  real*8 :: IXXI = 0.0                             ! Units: '1/(kg m^2)', Desc: 'Ixx Inverse of Inertia Matrix'
  real*8 :: IYYI = 0.0                             ! Units: '1/(kg m^2)', Desc: 'Iyy Inverse of Inertia Matrix'
  real*8 :: IZZI = 0.0                             ! Units: '1/(kg m^2)', Desc: 'Izz Inverse of Inertia Matrix'
  real*8 :: IXYI = 0.0                             ! Units: '1/(kg m^2)', Desc: 'Ixy Inverse of Inertia Matrix'
  real*8 :: IXZI = 0.0                             ! Units: '1/(kg m^2)', Desc: 'Ixz Inverse of Inertia Matrix'
  real*8 :: IYZI = 0.0                             ! Units: '1/(kg m^2)', Desc: 'Iyz Inverse of Inertia Matrix'
  real*8 :: TIA(3,3) = 0.0                         ! Units: 'nd', Desc: 'Aircraft to Inertial Frame Transformation Matrix'
  real*8 :: TAI(3,3) = 0.0                         ! Units: 'nd', Desc: 'Inertial to Aircraft Frame Transformation Matrix'
  real*8 :: FXGRAV = 0.0                           ! Units: 'lbf', Desc: 'X Gravity Forces in Body Frame'
  real*8 :: FYGRAV = 0.0                           ! Units: 'lbf', Desc: 'Y Gravity Forces in Body Frame'
  real*8 :: FZGRAV = 0.0                           ! Units: 'lbf', Desc: 'Z Gravity Forces in Body Frame'
  real*8 :: MXGRAV = 0.0                           ! Units: 'N m', Desc: 'X Gravity Moment About CG in Body Frame'
  real*8 :: MYGRAV = 0.0                           ! Units: 'N m', Desc: 'Y Gravity Moment About CG in Body Frame'
  real*8 :: MZGRAV = 0.0                           ! Units: 'N m', Desc: 'Z Gravity Moment About CG in Body Frame'
  real*8 :: FXAERO = 0.0                           ! Units: 'lbf', Desc: 'X Aerodynamic Force in Body Frame'
  real*8 :: FYAERO = 0.0                           ! Units: 'lbf', Desc: 'Y Aerodynamic Force in Body Frame'
  real*8 :: FZAERO = 0.0                           ! Units: 'lbf', Desc: 'Z Aerodynamic Force in Body Frame'
  real*8 :: MXAERO = 0.0                           ! Units: 'N m', Desc: 'X Aerodynamic Moment About CG in Body Frame'
  real*8 :: MYAERO = 0.0                           ! Units: 'N m', Desc: 'Y Aerodynamic Moment About CG in Body Frame'
  real*8 :: MZAERO = 0.0                           ! Units: 'N m', Desc: 'Z Aerodynamic Moment About CG in Body Frame'
  real*8 :: FXCONT = 0.0                           ! Units: 'lbf', Desc: 'X Contact Force in Body Frame'
  real*8 :: FYCONT = 0.0                           ! Units: 'lbf', Desc: 'Y Contact Force in Body Frame'
  real*8 :: FZCONT = 0.0                           ! Units: 'lbf', Desc: 'Z Contact Force in Body Frame'
  real*8 :: MXCONT = 0.0                           ! Units: 'N m', Desc: 'X Contact Moment in Body Frame'
  real*8 :: MYCONT = 0.0                           ! Units: 'N m', Desc: 'Y Contact Moment in Body Frame'
  real*8 :: MZCONT = 0.0                           ! Units: 'N m', Desc: 'Z Contact Moment in Body Frame'
  real*8 :: FXTOTAL = 0.0                          ! Units: 'lbf', Desc: 'X Total Applied Force in Body Frame'
  real*8 :: FYTOTAL = 0.0                          ! Units: 'lbf', Desc: 'Y Total Applied Force in Body Frame'
  real*8 :: FZTOTAL = 0.0                          ! Units: 'lbf', Desc: 'Z Total Applied Force in Body Frame'
  real*8 :: MXTOTAL = 0.0                          ! Units: 'N m', Desc: 'X Total Applied Moment About Mass Center in Body Frame'
  real*8 :: MYTOTAL = 0.0                          ! Units: 'N m', Desc: 'Y Total Applied Moment About Mass Center in Body Frame'
  real*8 :: MZTOTAL = 0.0                          ! Units: 'N m', Desc: 'Z Total Applied Moment About Mass Center in Body Frame'
  real*8 :: INITIALPHI = 0.0                       ! Units: 'rad', Desc: 'Initial Euler Roll Angle of Aircraft'
  real*8 :: INITIALTHETA = 0.0                     ! Units: 'rad', Desc: 'Initial Euler Pitch Angle of Aircraft'
  real*8 :: INITIALPSI = 0.0                       ! Units: 'rad', Desc: 'Initial Euler Yaw Angle of Aircraft'
  real*8 :: INITIALSTATE(13) = 0.0                 ! Units: 'vd', Desc: 'Initial Aircraft State Vector'
  real*8 :: PHI = 0                                ! Units: 'rad', Desc: Aircraft roll angle
  real*8 :: THETA = 0                              ! Units: 'rad', Desc: Aircraft roll angle
  real*8 :: PSI = 0                                ! Units: 'rad', Desc: Aircraft roll angle
  real*8 :: STATE(13) = 0.0                        ! Units: 'vd', Desc: 'Aircraft State Vector'
  real*8 :: STATEPREV(13) = 0.0                    ! Units: 'vd', Desc: 'Aircraft State Vector before camera snapshot'
  real*8 :: STATEDOT(13) = 0.0                     ! Units: 'vd', Desc: 'Aircraft State Vector Derivative'
  real*8 :: VXWIND = 0.0                           ! Units: 'ft/s', Desc: 'Atmospheric Wind Along Inertial I Axis'
  real*8 :: VYWIND = 0.0                           ! Units: 'ft/s', Desc: 'Atmospheric Wind Along Inertial J Axis'
  real*8 :: VZWIND = 0.0                           ! Units: 'ft/s', Desc: 'Atmospheric Wind Along Inertial K Axis'
  real*8 :: XCOMMAND = 0.0
  real*8 :: YCOMMAND = 0.0
  real*8 :: ZCOMMAND = 0.0
  real*8 :: PHICOMMAND = 0.0
  real*8 :: THETACOMMAND = 0.0
  real*8 :: PSICOMMAND = 0.0
  real*8 :: UCOMMAND = 0.0
  real*8 :: KPXDRIVE = 0                            ! Units: 'nd', Desc: driverdriver proportional gain on x-position
  real*8 :: KIXDRIVE = 0                            ! Units: 'nd', Desc: driverdriver integral gain on x-position
  real*8 :: KDXDRIVE = 0                            ! Units: 'nd', Desc: driverdriver derivative gain on x-position
  real*8 :: KPYDRIVE = 0                            ! Units: 'nd', Desc: driverdriver proportional gain on y-position
  real*8 :: KIYDRIVE = 0                            ! Units: 'nd', Desc: driverdriver integral gain on y-position
  real*8 :: KDYDRIVE = 0                            ! Units: 'nd', Desc: driverdriver derivative gain on y-position
  real*8 :: KPZDRIVE = 0                            ! Units: 'nd', Desc: driverdriver proportional gain on z-position
  real*8 :: KIZDRIVE = 0                            ! Units: 'nd', Desc: driverdriver integral gain on z-position 
  real*8 :: KDZDRIVE = 0                            ! Units: 'nd', Desc: driverdriver derivative gain on z-position
  real*8 :: KPPSI = 0                              ! Units: 'nd', Desc: driverdriver proportional gain on heading
  real*8 :: KIPSI = 0                              ! Units: 'nd', Desc: driverdriver integral gain on heading
  real*8 :: KDPSI = 0                              ! Units: 'nd', Desc: driverdriver derivative gain on heading
  real*8 :: KPPHI = 0                              ! Units: 'nd', Desc: driverdriver proportional gain on roll 
  real*8 :: KIPHI = 0                              ! Units: 'nd', Desc: driverdriver integral gain on roll 
  real*8 :: KDPHI = 0                              ! Units: 'nd', Desc: driverdriver derivative gain on roll
  real*8 :: KPTHETA = 0                            ! Units: 'nd', Desc: driverdriver proportional gain on pitch
  real*8 :: KITHETA = 0                            ! Units: 'nd', Desc: driverdriver integral gain on pitch
  real*8 :: KDTHETA = 0                            ! Units: 'nd', Desc: driverdriver derivative gain on pitch
  real*8 :: XINTEGRAL = 0
  real*8 :: YINTEGRAL = 0
  real*8 :: ZINTEGRAL = 0
  real*8 :: PHIINTEGRAL = 0.0                      ! Units: 'ft', Desc: 'PHI integral term in PID Controller'
  real*8 :: THETAINTEGRAL = 0.0                    ! Units: 'ft', Desc: 'THETA integral term in PID Controller'
  real*8 :: PSIINTEGRAL = 0.0                      ! Units: 'ft', Desc: 'PSI integral term in PID Controller'
  real*8 :: UINTEGRAL = 0.0
  real*8 :: MS_0 = 0.0                             ! Units: 'us', Desc: Microsecond impulse to keep driver hovering // REVISIT can take this out later
  real*8 :: MS_ROLL = 0.0                          ! Units: 'us', Desc: Microsecond impulse to roll  // REVISIT can take this out later
  real*8 :: MS_PITCH = 0.0                         ! Units: 'us', Desc: Microsecond impulse to pitch // REVISIT can take this out later
  real*8 :: MS_YAW = 0.0                           ! Units: 'us', Desc: Microsecond impulse to yaw   // REVISIT can take this out later
  real*8 :: PWM2F(4,1) = 0.0                       ! Units: 'lbf',Desc: Force converted as a function of input microsecond pulse
  real*8 :: SLTETHER = 0.0                         ! Units: 'ft', Desc: Stationline distance from center of mass to tether connection point on towed system
  real*8 :: BLTETHER = 0.0                         ! Units: 'ft', Desc: buttline distance from center of mass to tether connection point on towed system
  real*8 :: WLTETHER = 0.0                         ! Units: 'ft', Desc: waterline distance from center of mass to tether connection point on towed system
 end type TOWEDSTRUCTURE
 
!!!!!!!!!!!!!!!!!!!!!!!!!!!! TETHER STRUCTURE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 
 type TETHERSTRUCTURE
  integer :: DYNOFFON = 0                          ! Units: 'nd', Desc: 'Dynamics Flag (0=Off, 1=On)'
  integer :: GRAVOFFON = 0                         ! Units: 'nd', Desc: 'Gravity Flag (0=Off, 1=On)'
  integer :: AEROFLAG = 0                          ! Units: 'nd', Desc: 'Aerodynamics Flag (0=Off, 1=On)'
  integer :: ADDBEAD = 0                           ! Units: 'nd' Desc: 'Whether or not to add a bead
  integer :: REMOVEBEAD = 0                        ! Units: 'nd' Desc: 'Whether or not to remove a bead or not
  integer :: ELASOFFON = 0                         ! Units: 'nd', Desc: 'Elasticity Flag (0=Off, 1=On)'
  integer :: NBEADS = 0                            ! Units: 'nd', Desc: 'Number of Tether Beads'                    
  integer :: DQFLAG = 0                            ! Units: 'nd', Desc: 'Data Quality Flag (0=Data Not Loaded Successfully, 1=Data Loaded Successfully)'
  integer :: NONLINEAR = 0                         ! Units: 'nd', Desc: 'Nonlinearity of Tether'
  real*8 :: XTETHER = 0.0                          ! Units: 'ft', Desc: 'X Inertial Position of Tether Attachment Point'
  real*8 :: YTETHER = 0.0                          ! Units: 'ft', Desc: 'Y Inertial Position of Tether Attachment Point'
  real*8 :: ZTETHER = 0.0                          ! Units: 'ft', Desc: 'Z Inertial Position of Tether Attachment Point'
  real*8 :: FXTETHER = 0.0                         ! Units: 'ft', Desc: 'X Tether force
  real*8 :: FYTETHER = 0.0                         ! Units: 'ft', Desc: 'Y Tether force
  real*8 :: FZTETHER = 0.0                         ! Units: 'ft', Desc: 'Z Tether force
  real*8 :: FXPLATFORM = 0.0                         ! Units: 'ft', Desc: 'X Tether force
  real*8 :: FYPLATFORM = 0.0                         ! Units: 'ft', Desc: 'Y Tether force
  real*8 :: FZPLATFORM = 0.0                         ! Units: 'ft', Desc: 'Z Tether force
  real*8 :: XTETHERDOT = 0.0                       ! Units: 'ft', Desc: 'X Inertial Velocity of Tether Attachment Point
  real*8 :: YTETHERDOT = 0.0                       ! Units: 'ft', Desc: 'Y Inertial Velocity of Tether Attachment Point
  real*8 :: ZTETHERDOT = 0.0                       ! Units: 'ft', Desc: 'Z Inertial Velocity of Tether Attachment Point
  real*8 :: MASSPUL = 0.0                          ! Units: 'slug/ft', Desc: 'Mass Per Unit Length of Total Tether Line'   
  real*8 :: AREA = 0.0                             ! Units: 'ft^2', Desc: 'Cross sectional area of tether line'
  real*8 :: LEN = 0.0                              ! Units: 'ft', Desc: 'Unstretched Total Length of Tether Line'   
  real*8 :: NOMLEN = 0.0                           ! Units: 'ft', Desc: 'Nominal Length of Tether at Initial Condition'
  real*8 :: LENMAX = 0.0                           ! Units: 'ft', Desc: 'Maximum Length of Tether Line'   
  real*8 :: LENMIN = 0.0                           ! Units: 'ft', Desc: 'Minimum Length of Tether Line'   
  real*8 :: DIA = 0.0                              ! Units: 'ft', Desc: 'Unstretched Total Diameter of Tether Line' 
  real*8 :: GP = 0.0                               ! Units: 'lbf/(rad-ft^2)', Desc: 'Gp Torsional Modulus of Total Tether Line'   
  real*8 :: GD = 0.0                               ! Units: 'lbf-s/(rad-ft^2)', Desc: 'Gd Torsional Damping Modulus of Total Tether Line'     
  real*8 :: KE = 0.0                               ! Units: 'lbf/ft^2', Desc: 'Ke Stiffness of Total Tether Line'   
  real*8 :: KV = 0.0                               ! Units: 'lbf/ft^2', Desc: 'Kv Stiffness of Total Tether Line'   
  real*8 :: CV = 0.0                               ! Units: 'lbf/(ft^2/s)', Desc: 'Cv Damping of Total Tether Line'   
  real*8 :: EMASS = 0.0                            ! Units: 'kg', Desc: 'Mass of Tether Line Element'   
  real*8 :: ELEN = 0.0                             ! Units: 'ft', Desc: 'Length of Tether Line Element'   
  real*8 :: EKE = 0.0                              ! Units: 'lbf/ft', Desc: 'Ke Stiffness of Tether Line Element'   
  real*8 :: EKV = 0.0                              ! Units: 'lbf/ft', Desc: 'Kv Stiffness of Tether Line Element'   
  real*8 :: KP = 0.0                               ! Units: 'lbf-ft/rad', Desc: 'Gp Torsional Stiffness of Total Tether Line'   
  real*8 :: KD = 0.0                               ! Units: 'lbf-ft-s/rad', Desc: 'Gd Torsional Damping of Total Tether Line'   
  real*8 :: ECV = 0.0                              ! Units: 'lbf/(ft/s)', Desc: 'Cv Damping of Tether Line Element'   
  real*8 :: SIGMA = 0.0                            ! Units: 'rad/s', Desc: 'Tension Filter Root'   
  real*8 :: KU = 0.0                               ! Units: 'lbf/ft', Desc: 'Tension Filter Stiffness Input'   
  real*8 :: CU = 0.0                               ! Units: 'lbf/(ft/s)', Desc: 'Tension Filter Damping Input'   
  real*8 :: NU = 0.0                               ! Units: 'nd', Desc: 'Tether Material Poisson's Ratio'   
  real*8 :: CD_AXIAL = 0.0                         ! Units: 'nd', Desc: 'Skin Friction Drag Coefficient'   
  real*8 :: CD_NORMAL = 0.0                        ! Units: 'nd', Desc: 'Flat Plate Drag Coefficient'   
  real*8 :: XS = 0.0                               ! Units: 'ft', Desc: 'X Inertial Position of Tether Start Edge Point'
  real*8 :: YS = 0.0                               ! Units: 'ft', Desc: 'Y Inertial Position of Tether Start Edge Point'
  real*8 :: ZS = 0.0                               ! Units: 'ft', Desc: 'Z Inertial Position of Tether Start Edge Point'
  real*8 :: XSDOT = 0.0                            ! Units: 'ft/s', Desc: 'X Inertial Velocity of Tether Start Edge Point'
  real*8 :: YSDOT = 0.0                            ! Units: 'ft/s', Desc: 'Y Inertial Velocity of Tether Start Edge Point'
  real*8 :: ZSDOT = 0.0                            ! Units: 'ft/s', Desc: 'Z Inertial Velocity of Tether Start Edge Point'
  real*8 :: XF = 0.0                               ! Units: 'ft', Desc: 'X Inertial Position of Tether Finish Edge Point'
  real*8 :: YF = 0.0                               ! Units: 'ft', Desc: 'Y Inertial Position of Tether Finish Edge Point'
  real*8 :: ZF = 0.0                               ! Units: 'ft', Desc: 'Z Inertial Position of Tether Finish Edge Point'
  real*8 :: XFDOT = 0.0                            ! Units: 'ft/s', Desc: 'X Inertial Velocity of Tether Finish Edge Point'
  real*8 :: YFDOT = 0.0                            ! Units: 'ft/s', Desc: 'Y Inertial Velocity of Tether Finish Edge Point'
  real*8 :: ZFDOT = 0.0                            ! Units: 'ft/s', Desc: 'Z Inertial Velocity of Tether Finish Edge Point'
  real*8 :: FXGRAV(MAXNBEADS) = 0.0                ! Units: 'lbf', Desc: 'X Gravity Force on Tether Bead in Inertial Frame'
  real*8 :: FYGRAV(MAXNBEADS) = 0.0                ! Units: 'lbf', Desc: 'Y Gravity Force on Tether Bead in Inertial Frame'
  real*8 :: FZGRAV(MAXNBEADS) = 0.0                ! Units: 'lbf', Desc: 'Z Gravity Force on Tether Bead in Inertial Frame'
  real*8 :: FXAERO(MAXNBEADS) = 0.0                ! Units: 'lbf', Desc: 'X Aerodynamic Force on Tether Bead in Inertial Frame'
  real*8 :: FYAERO(MAXNBEADS) = 0.0                ! Units: 'lbf', Desc: 'Y Aerodynamic Force on Tether Bead in Inertial Frame'
  real*8 :: FZAERO(MAXNBEADS) = 0.0                ! Units: 'lbf', Desc: 'Z Aerodynamic Force on Tether Bead in Inertial Frame'
  real*8 :: FXELAS(MAXNBEADS) = 0.0                ! Units: 'lbf', Desc: 'X Elastic Force on Tether Bead in Inertial Frame'
  real*8 :: FYELAS(MAXNBEADS) = 0.0                ! Units: 'lbf', Desc: 'Y Elastic Force on Tether Bead in Inertial Frame'
  real*8 :: FZELAS(MAXNBEADS) = 0.0                ! Units: 'lbf', Desc: 'Z Elastic Force on Tether Bead in Inertial Frame'
  real*8 :: TCOM(200) = 0.0                        ! Time indices for reel trajectory
  real*8 :: REELTRAJ(200) = 0.0                    ! Unstretched length values for reel trajectory
  real*8 :: FELASBEADMINUS(MAXNBEADS,3) = 0.0      ! Units: 'lbf', Desc: 'Tether Bead Elastic Force Computation Matrix'
  real*8 :: FELASBEADPLUS(MAXNBEADS,3) = 0.0       ! Units: 'lbf', Desc: 'Tether Bead Elastic Force Computation Matrix'
  real*8 :: VELOCITYMAG(MAXNBEADS) = 0.0           ! Units: 'ft/s', Desc: 'Tether Bead Aerodynamic Velocity'
  real*8 :: INITIALSTATE(7*MAXNBEADS+1) = 0.0      ! Units: 'vd', Desc: 'Initial Tether State Vector'
  real*8 :: STATE(7*MAXNBEADS+1) = 0.0             ! Units: 'vd', Desc: 'Tether State Vector'
  real*8 :: STATEDOT(7*MAXNBEADS+1) = 0.0          ! Units: 'vd', Desc: 'Tether State Vector Derivative' 
  real*8 :: THETAINT = 0.0                         ! Units: 'rad', Desc: Integral of Theta
  real*8 :: THETACOMMAND = 0.0                     ! Units: 'rad', Desc: Theta Command
  real*8 :: THETAREEL = 0.0                        ! Units: 'rad', Desc: Theta of reel
  real*8 :: THETADOTREEL = 0.0                     ! Units: 'rad/s', Desc: Thetadot of reel
  real*8 :: TORQUE = 0.0                           ! Units: 'lbf-ft', Desc: Torque applied at reel
  real*8 :: NOMTORQUE = 0.0                        ! Units: 'lbf-ft', Desc: Nominal Torque applied at reel
  real*8 :: TENSION = 0.0                          ! Units: 'lbf', Desc: Tension at Reel
  real*8 :: TORQUECOMMAND = 0.0                    ! Units: 'lbf-ft', Desc: Torque Command
  real*8 :: IREEL = 0.0                            ! Units: 'slug-ft^2', Desc: Rotational Inertia of Reel
  real*8 :: RREEL = 0.0                            ! Units: 'ft', Desc: radius of reel
  real*8 :: TAU = 0.0                              ! Units: 'ft', Desc: Time Constant of Torque at Reel
  real*8 :: STRETCHLEN = 0.0                       ! Units: 'ft', Desc: Stretched length of tether
  real*8 :: PREVLEN = 0.0                          ! Units: 'ft', Desc: Previous Length command
  real*8 :: PREVLENCOMMAND = 0.0                   ! Units: 'ft', Desc: Previous Length command
  real*8 :: LDOTCOMMAND = 0.0                      ! Units: 'ft/s', Desc: Pay Out Rate Command
  real*8 :: LDOTNOMINAL = 0.0                      ! Units: 'ft/s', Desc: Nominal Pay Out Rate
  real*8 :: LDOTFACTOR = 1.0                       ! Units: 'ft/s', Desc: Nominal Pay Out Rate
 end type TETHERSTRUCTURE

!!!!!!!!!!!!!!!!!!!!!!!!!! SIMULATION STRUCTURE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 
 type SIMULATIONSTRUCTURE
  character(128) :: SIMINPUTFILE = ' '             ! Units: 'nd', Desc: 'Simulation Input File'
  character(128) :: RESTARTFILE = ' '              ! Units: 'nd', Desc: 'Simulation Input File'
  character(128) :: STATEDES(MAXX) = ' '           ! Units: 'nd', Desc: 'State Description'
  integer :: NOSTATES = 1                          ! Units: 'nd', Desc: 'Number of States'
  integer :: IOUTSKIP = 1                          ! Units: 'nd', Desc: 'Output Skip Parameter'
  integer :: ICS = 0                               ! Units: 'nd', Desc: 'Control System Off On Flag'
  integer :: IDEBUG = 0                            ! Units: 'nd', Desc: 'Debug Flag'
  integer :: IDX = 0                               ! Units: 'nd', Desc: 'Integration Index'
  integer :: IDXOUT = 0                            ! Units: 'nd', Desc: ''
  integer :: IECHOOFFON = 1                        ! Units: 'nd', Desc: 'Input Data Echo Flag'
  integer :: DQFLAG = 0                            ! Units: 'nd', Desc: 'Data Quality Flag (0=Data Not Loaded Successfully, 1=Data Loaded Successfully)'
  integer :: CREATERESTART = 0                     ! Units: 'nd', Desc: 'Create Restart Point'
  integer :: RESTART = 0                           ! Units: 'nd', Desc: 'Use a restart file'
  integer :: ACTUATORONOFF = 0                     ! Units: 'nd', Desc: Turn on actuator dynamics or not
  real*8 :: TIME = 0.0                             ! Units: 's', Desc: 'Time'
  real*8 :: RESTARTTIME = 0                        ! Units: 's', Desc: 'Time to create restart time
  real*8 :: DELTATIME = 0.0                        ! Units: 's', Desc: 'Delta Time'
  real*8 :: INITIALTIME = 0.0                      ! Units: 's', Desc: 'Initial Time'
  real*8 :: CPUTIMEUSER = 0.0                      ! Units: 's', Desc: 'Elapsed CPU Time by User Process'
  real*8 :: CPUTIMESYSTEM = 0.0                    ! Units: 's', Desc: 'Elapsed CPU Time by System Process'
  real*8 :: CPUTIMETOTAL = 0.0                     ! Units: 's', Desc: 'Elapsed CPU Time by User and System Processes'
  real*8 :: FINALTIME = 0.0                        ! Units: 's', Desc: 'Final Time'
  real*8 :: STATE(MAXX) = 0.0                      ! Units: 'vd', Desc: 'State'
  real*8 :: STATEPREV(MAXX) = 0.0                  ! Units: 'vd', Desc: 'State before camera snapshot'
  real*8 :: INITIALSTATE(MAXX) = 0.0               ! Units: 'vd', Desc: 'Initial State'
  real*8 :: STATEDOT(MAXX) = 0.0                   ! Units: 'vd', Desc: 'State Dot'
  real*8 :: INITIALSTATEDOT(MAXX) = 0.0            ! Units: 'vd', Desc: 'Initial State Dot'
  real*8 :: ACTUATOR(NOACTUATORS) = 0.0            ! Units: 'vd', Desc: 'Vector of Actuators
  real*8 :: ACTUATORDOT(NOACTUATORS) = 0.0         ! Units: 'vd', Desc: 'Vector of Actuators Derivatives
end type SIMULATIONSTRUCTURE

!!!!!!!!!!!!!!!!!!!!!!!!!! TOSIM STRUCTURE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

 type TOSIMSTRUCTURE
  character(128) :: FILEINPUTFILE = ' '            ! Units: 'nd', Desc: 'File of Files Input File'
  character(128) :: TOSIMINPUTFILE = ' '           ! Units: 'nd', Desc: 'Tapas Input File'
  character(128) :: ATMOSPHEREINPUTFILE = ' '      ! Units: 'nd', Desc: 'Atmosphere Input File'
  character(128) :: TOWEDINPUTFILE = ' '           ! Units: 'nd', Desc: 'Aircraft Input File'
  character(128) :: TETHERINPUTFILE = ' '          ! Units: 'nd', Desc: 'Tether Input File'
  character(256) :: WINGSXINPUTDIR = ' '           ! Units: 'nd', Desc: 'WingsX Input Directory'  
  character(256) :: WINGSXINPUTFILE = ' '          ! Units: 'nd', Desc: 'WingsX Input File'
  character(128) :: TCOMINPUTFILE = ' '            ! Units: 'nd', Desc: 'Tether control system input file'
  character(128) :: SIMINPUTFILE = ' '             ! Units: 'nd', Desc: 'Simulation Input File'
  character(128) :: RUNLOGFILE = ' '               ! Units: 'nd', Desc: 'Run Log File'
  character(128) :: STATEOUTPUTFILE = ' '          ! Units: 'nd', Desc: 'State Output File'
  character(128) :: MISCOUTPUTFILE = ' '           ! Units: 'nd', Desc: 'Miscellaneous Output File'
  character(128) :: CONTROLOUTPUTFILE = ' '        ! Units: 'nd', Desc: 'Control Output File'
  character(128) :: FORCEOUTPUTFILE = ' '          ! Units: 'nd', Desc: 'Force Output File'
  character(128) :: ERROROUTPUTFILE = ' '          ! Units: 'nd', Desc: 'Error Output File'
  character(24) :: DATEANDTIMEOFRUN = ' '          ! Units: 'nd', Desc: 'Date and Time of Run'
  real*8 :: GRAVITY = 32.2                         ! Units: 'ft/s^2', Desc: 'Gravity'
  type(ATMOSPHERESTRUCTURE) :: ATM
  type(TOWEDSTRUCTURE) :: TOW
  type(TETHERSTRUCTURE) :: THR
  type(DRIVERSTRUCTURE) :: DRIVER
  type(SIMULATIONSTRUCTURE) :: SIM
 end type TOSIMSTRUCTURE

end module TOSIMDATATYPES

!!!!!!!!!!!!!!!!!!!!!!!! PROGRAM TOSIM !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

PROGRAM TOSIM
 use TOSIMDATATYPES
 !USE Math_Module, ONLY: Math_Init - Archaic from AREA-I days
 implicit none
 integer openflag,readflag,LENGTH
 character(128) inputfilename
 character(12) inputfiletype
 type(TOSIMSTRUCTURE) T
 !CHARACTER(len=256)::AircraftDataFile !Aircraft-data file name (Moved to main Data structure - CM 8/16/2015
 CHARACTER(len=128)::rec,default
 !INTEGER:: numNodesNLL = 100 (Moved to TOWED Data structure - CM 8/16/2015)
 !REAL::Rde,CL0,Sw,bw,cw,Vflt,Ixx,Iyy,Izz,Ixy,Ixz,Iyz,hx,hy,hz

 readflag = 0
 openflag = 0

 !call Math_Init !Initialize module - Functionality removed as of 1/2/2017
 
 ! Input File of Files

 call getarg(1,T%FILEINPUTFILE)
 if (len(trim(T%FILEINPUTFILE)) .lt. 1) then
    default = 'Input_Files/Hovering/TOSIM.ifiles'
    write(*,'(a)') 'Enter input file otherwise will use the following file: ',default
    write(*,*) 'Hit enter if you just wish to continue....'
    read(5,'(a)')rec
        if(rec.eq.'q')STOP
        if(rec.eq.'quit')STOP
        if(rec.ne.' ')then
            T%FILEINPUTFILE = rec
        else 
            T%FILEINPUTFILE = default
        endif
 endif
 
 open(unit=93,file=T%FILEINPUTFILE,status='old',iostat=openflag)
 if (openflag .ne. 0) then
  write(*,*) 'Error Opening TOSIM Input File: ',T%FILEINPUTFILE; PAUSE; STOP
 end if
 rewind(93)  
 
 do while (readflag .eq. 0)
  inputfiletype = ' '; inputfilename = ' '
  read(unit=93,fmt=*,iostat=readflag) inputfiletype,inputfilename
  write(*,*) 'Input file: ', inputfilename
  !I added a line in the ifiles file that reads in a file with extension WINGS - CM 8/16/2015
    if ((inputfiletype.eq.'WINGSDIR') .or. (inputfiletype.eq.'wingsdir') .or. (inputfiletype.eq.'Wingsdir')) then
        T%WINGSXINPUTDIR = inputfilename;  
    end if
    if ((inputfiletype.eq.'WINGS') .or. (inputfiletype.eq.'wings') .or. (inputfiletype.eq.'Wings')) then
        T%WINGSXINPUTFILE = inputfilename;  
    end if
    if ((inputfiletype.eq.'ATM') .or. (inputfiletype.eq.'atm') .or. (inputfiletype.eq.'Atm')) then
        T%ATMOSPHEREINPUTFILE = inputfilename;  
    end if
    if ((inputfiletype.eq.'DRIVER') .or. (inputfiletype.eq.'Driver') .or. (inputfiletype.eq.'Driver')) then
        T%DRIVER%INPUTFILE = inputfilename;  
    end if
    if ((inputfiletype.eq.'THR') .or. (inputfiletype.eq.'thr') .or. (inputfiletype.eq.'Thr')) then
        T%TETHERINPUTFILE = inputfilename;  
    end if
    if ((inputfiletype.eq.'TOW') .or. (inputfiletype.eq.'tow') .or. (inputfiletype.eq.'Tow')) then
        T%TOWEDINPUTFILE = inputfilename;  
    end if
    if ((inputfiletype.eq.'TPS') .or. (inputfiletype.eq.'tps') .or. (inputfiletype.eq.'Tps')) then
        T%TOSIMINPUTFILE = inputfilename;  
    end if
    if ((inputfiletype.eq.'SIM') .or. (inputfiletype.eq.'sim') .or. (inputfiletype.eq.'Sim')) then
        T%SIMINPUTFILE = inputfilename;  
    end if
    if ((inputfiletype.eq.'TCOM') .or. (inputfiletype.eq.'tcom') .or. (inputfiletype.eq.'Tcom')) then
        T%TCOMINPUTFILE = inputfilename;  
    end if
    if ((inputfiletype.eq.'LOG') .or. (inputfiletype.eq.'log') .or. (inputfiletype.eq.'log')) then
        T%RUNLOGFILE = inputfilename;  
    end if
    if ((inputfiletype.eq.'SOUT') .or. (inputfiletype.eq.'sout') .or. (inputfiletype.eq.'Sout')) then
        T%STATEOUTPUTFILE = inputfilename;  
    end if
    if ((inputfiletype.eq.'MOUT') .or. (inputfiletype.eq.'mout') .or. (inputfiletype.eq.'Mout')) then
        T%MISCOUTPUTFILE = inputfilename;  
    end if
    if ((inputfiletype.eq.'COUT') .or. (inputfiletype.eq.'cout') .or. (inputfiletype.eq.'Cout')) then
        T%CONTROLOUTPUTFILE = inputfilename;  
    end if
    if ((inputfiletype.eq.'FOUT') .or. (inputfiletype.eq.'fout') .or. (inputfiletype.eq.'Fout')) then
        T%FORCEOUTPUTFILE = inputfilename;  
    end if
    if ((inputfiletype.eq.'EOUT') .or. (inputfiletype.eq.'eout') .or. (inputfiletype.eq.'Eout')) then
        T%ERROROUTPUTFILE = inputfilename;  
    end if
 end do

 close(93)

!!!!!!!!!!!!!!!!!!11! Load Data !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

 call ATMOSPHERE(T,1)
 call DRIVER(T,1) !1 = load data , 2 = print data , 3 = compute for the different models
 call TETHER(T,1)
 call TOWED(T,1)
 call SIMULATION(T,1)

!!!!!!!!!!!!!!!!!!! Compute Simulation !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

 call SIMULATION(T,2)

!!!!!!!!!!!!!!!!! Close Output Files !!!!!!!!!!!!!!!!!!!!!!!

 close(25)
 
 STOP
END PROGRAM TOSIM

!!!!!!!!!!!!!!!!!!!! SUBROUTINE SIMULATION !!!!!!!!!!!!!!!!!!!!!

SUBROUTINE SIMULATION(T,iflag)
 use TOSIMDATATYPES
 implicit none
 integer iflag,openflag,readflag
 integer i,j,k,npts,stateindex
 real*8 readreal,xslope,yslope,zslope,idx
 real*8 sum,nominaltime,nominalstate(MAXX),rkalfa(4),krkbody(MAXX,4)
 real*4 tictotal,ticuser,ticsystem,toctotal,tocuser,tocsystem,elapsed(2)
 real*4 etime,aoa,unominal,wnominal,ALFAMAX
 real*8 krkactuator(NOACTUATORS,4),nomactuator(NOACTUATORS)
 real*8 zint
 type(TOSIMSTRUCTURE) T

 !!!!!!!!!!!!!!! COMPUTE iflag = 3 !!!!!!!!!!!!!!!!!!!!!!1

 if (iflag .eq. 2) then  

  ! Define Constants

  rkalfa(1) = 1.0; rkalfa(2) = 2.0; rkalfa(3) = 2.0; rkalfa(4) = 1.0

  ! Initial State Vector

  !T%SIM%TIME = T%SIM%INITIALTIME
  !T%SIM%STATE(1:T%SIM%NOSTATES) = T%SIM%INITIALSTATE(1:T%SIM%NOSTATES)
  call SYSTEMDERIVATIVES(T,2)

  ! Compute Nominal Torque Value
  if (T%SIM%ACTUATORONOFF .eq. 1) then
     T%THR%NOMTORQUE = -T%THR%TENSION*T%THR%RREEL
     ! Set initial Torque to Nominal Torque
     T%SIM%ACTUATOR(5) = T%THR%NOMTORQUE
     T%THR%TORQUE = T%THR%NOMTORQUE
  end if
   
  if (T%SIM%ICS .eq. 1) then
     call CONTROL(T,2)
  end if

  ! Open Time Simulation Output Files

  open(unit=92,file=T%STATEOUTPUTFILE)
  write(25,*) ' '
  write(25,*) 'STATE VECTOR OUTPUT FILE CREATED: ',trim(T%STATEOUTPUTFILE)
  write(25,*) ' '
  rewind(92)
  open(unit=96,file=T%MISCOUTPUTFILE)
  write(25,*) ' '
  write(25,*) 'MISCELLANEOUS VECTOR OUTPUT FILE CREATED: ',trim(T%MISCOUTPUTFILE)
  write(25,*) ' '
  rewind(96)
  open(unit=91,file=T%CONTROLOUTPUTFILE)
  write(25,*) ' '
  write(25,*) 'CONTROL VECTOR OUTPUT FILE CREATED: ',trim(T%CONTROLOUTPUTFILE)
  write(25,*) ' '
  rewind(91)
  open(unit=83,file=T%FORCEOUTPUTFILE)
  write(25,*) ' '
  write(25,*) 'FORCE VECTOR OUTPUT FILE CREATED: ',trim(T%FORCEOUTPUTFILE)
  write(25,*) ' '
  rewind(83)
 
  ! Integrate Equations of Motion

  tictotal = ETIME(elapsed)
  ticuser = elapsed(1)
  ticsystem = elapsed(2)
  npts = nint((T%SIM%FINALTIME-T%SIM%INITIALTIME)/T%SIM%DELTATIME)

  write(*,*) 'Simulation Start'

  ! THIS IS THE MAIN SIMULATION LOOP
  do i=1,npts

   T%SIM%IDX = i
  
   ! Output Data to File
   !!OUTPUTS OUTPUTFILES STANDARD OUTS FILE OUTPUTS OUTPUT DATA
   !!! Note that the code only compiles if you print out variables that belongs to a data structure (See below).
   !!OUTPUTS OUTPUTFILES STANDARD OUTS FILE OUTPUTS
   if (mod(i-1,T%SIM%IOUTSKIP) .eq. 0) then 
      !State.OUT File
      ! REVISIT subtracted 8 to get rid of thrust states
    write(92,fmt='(1000F30.10)') T%SIM%TIME,T%SIM%STATE(1:T%SIM%NOSTATES-7),T%SIM%STATEDOT(1:T%SIM%NOSTATES-7)
      !Misc.OUT File
    write(96,fmt='(1000F30.10)') T%SIM%TIME,T%TOW%VXWIND,T%TOW%VYWIND,T%TOW%VZWIND,T%THR%NBEADS*1.0D0,T%THR%STATE(7*T%THR%NBEADS+1)
    ! T%SIM%TIME,T%TOW%VXWIND,T%TOW%VYWIND,T%TOW%VZWIND,T%THR%NBEADS*1.0D0,T%TOW%FXCONT,T%TOW%FYCONT,T%TOW%FZCONT,T%TOW%MXCONT,T%TOW%MYCONT,T%TOW%MZCONT
      ! T%SIM%STATE(T%SIM%NOSTATES-7:T%SIM%NOSTATES),T%SIM%STATEDOT(T%SIM%NOSTATES-7:T%SIM%NOSTATES)
      !Control OUT File
    write(91,fmt='(1000F30.10)') T%SIM%TIME,T%TOW%AILERON,T%TOW%ELEVATOR,T%TOW%RUDDER,T%TOW%FLAPS,T%TOW%OMEGAVEC(1:4,1), T%DRIVER%FYGRAV, T%DRIVER%FYAERO, T%DRIVER%FYCONT
      !Force Vector File
    write(83,fmt='(1000F30.10)') T%SIM%TIME,T%THR%FXGRAV(1:T%THR%NBEADS),T%THR%FYGRAV(1:T%THR%NBEADS),T%THR%FZGRAV(1:T%THR%NBEADS),T%THR%FXELAS(1:T%THR%NBEADS),T%THR%FYELAS(1:T%THR%NBEADS),T%THR%FZELAS(1:T%THR%NBEADS),T%THR%FXAERO(1:T%THR%NBEADS),T%THR%FYAERO(1:T%THR%NBEADS),T%THR%FZAERO(1:T%THR%NBEADS)
   end if 

   
   if (npts .gt. 100) then
      if (mod(i,npts/100) .eq. 1) then 
         idx = i
         write(*,*) 'Time Simulation: ',int(100*idx/npts)+1, ' Percent Complete'
      end if
   else
      write(*,*) 'Time Simulation: ',i, ' out of ',npts
   end if

   ! Store Nominal State Values

   nominaltime = T%SIM%TIME
   nominalstate(1:T%SIM%NOSTATES) = T%SIM%STATE(1:T%SIM%NOSTATES)

   !Store Nominal Actuators
   if (T%SIM%ACTUATORONOFF .eq. 1) then
      nomactuator(1) = T%SIM%ACTUATOR(1) !right brake
      nomactuator(2) = T%SIM%ACTUATOR(2) !left brake
      nomactuator(3) = T%SIM%ACTUATOR(3) !thetareel
      nomactuator(4) = T%SIM%ACTUATOR(4) !thetadotreel
      nomactuator(5) = T%SIM%ACTUATOR(5) !torque at reel
      nomactuator(6) = T%SIM%ACTUATOR(6) !theta int
      nomactuator(7) = T%SIM%ACTUATOR(7) !incidence
      nomactuator(8) = T%SIM%ACTUATOR(8) !integral of vertical angle error
      nomactuator(9) = T%SIM%ACTUATOR(9) !integral of lateral offset
   end if
   
   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   ! Numerical Integration of Equations of Motion

   do j=1,4

    ! State Values to Evaluate Derivatives

    if (j .ne. 1) then
       T%SIM%TIME = nominaltime + T%SIM%DELTATIME/rkalfa(j)
       do k=1,T%SIM%NOSTATES
          T%SIM%STATE(k) = nominalstate(k) + krkbody(k,j-1)/rkalfa(j)
       end do
       !Actuator Dynamics
       if (T%SIM%ACTUATORONOFF .eq. 1) then
          do k = 1,NOACTUATORS
             T%SIM%ACTUATOR(k) = nomactuator(k) + krkactuator(k,j-1)/rkalfa(j)
          end do
       end if
    end if


    if (j .ne. 1) then
       T%SIM%TIME = nominaltime + T%SIM%DELTATIME/rkalfa(j)
       do k=1,T%SIM%NOSTATES
          T%SIM%STATE(k) = nominalstate(k) + krkbody(k,j-1)/rkalfa(j)
       end do
       !Actuator Dynamics
       if (T%SIM%ACTUATORONOFF .eq. 1) then
          do k = 1,NOACTUATORS
             T%SIM%ACTUATOR(k) = nomactuator(k) + krkactuator(k,j-1)/rkalfa(j)
          end do
       end if
    end if

    ! Compute Derivatives

    call SYSTEMDERIVATIVES(T,2)

    ! write(*,*) 'Fx,Fy,Fz - Grav,Aero,Cont,Total'
    ! write(*,*) T%TOW%FXGRAV,T%TOW%FYGRAV,T%TOW%FZGRAV
    ! write(*,*) T%TOW%FXAERO,T%TOW%FYAERO,T%TOW%FZAERO
    ! write(*,*) T%TOW%FXCONT,T%TOW%FYCONT,T%TOW%FZCONT
    ! write(*,*) T%TOW%FXTOTAL,T%TOW%FYTOTAL,T%TOW%FZTOTAL
    ! write(*,*) 'Mx,My,Mz - Grav,Aero,Cont,Total'
    ! write(*,*) T%TOW%MXGRAV,T%TOW%MYGRAV,T%TOW%MZGRAV
    ! write(*,*) T%TOW%MXAERO,T%TOW%MYAERO,T%TOW%MZAERO
    ! write(*,*) T%TOW%MXCONT,T%TOW%MYCONT,T%TOW%MZCONT
    ! write(*,*) T%TOW%MXTOTAL,T%TOW%MYTOTAL,T%TOW%MZTOTAL

    ! Runge-Kutta Constants

    do k=1,T%SIM%NOSTATES
     krkbody(k,j) = T%SIM%DELTATIME*T%SIM%STATEDOT(k)
    end do
    
    if (T%SIM%ACTUATORONOFF .eq. 1) then
       do k=1,NOACTUATORS
          krkactuator(k,j) = T%SIM%DELTATIME*T%SIM%ACTUATORDOT(k)
       end do
    end if

   end do

   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

   ! Step Time

   T%SIM%TIME = nominaltime + T%SIM%DELTATIME

   ! Step States

   do j=1,T%SIM%NOSTATES
      sum = 0.0
      do k=1,4
         sum = sum + rkalfa(k)*krkbody(j,k)
      end do
      T%SIM%STATE(j) = nominalstate(j) + sum/6.0
   end do

   ! Step Actuators
   if (T%SIM%ACTUATORONOFF .eq. 1) then
      do j=1,NOACTUATORS
         sum = 0.0
         do k=1,4
            sum = sum + rkalfa(k)*krkactuator(j,k)
         end do
         T%SIM%ACTUATOR(j) = nomactuator(j) + sum/6.0
      end do
   end if

   ! write(*,*) '--------------------'
   ! write(*,*) 'Nominal = ',nomactuator(1:2)
   ! write(*,*) 'Actuator = ',T%SIM%ACTUATOR(1:2)
   ! write(*,*) 'Derivative = ',T%SIM%ACTUATORDOT(1:2)

   ! Step State Derivatives
   ! rk4
   call SYSTEMDERIVATIVES(T,2)

   ! Step Control System

   if (T%SIM%ICS .eq. 1) then
      call CONTROL(T,2)
   end if

  ! State Limits

   call STATELIMITS(T)

  !Check for Creating a Restart point

   if ((T%SIM%CREATERESTART .eq. 1) .and. (T%SIM%RESTARTTIME .le. T%SIM%TIME+T%SIM%DELTATIME)) then
      write(98,*) T%SIM%TIME-T%DRIVER%TIMEON, ' !Restart Time offset'
      write(98,*) T%THR%NBEADS, ' !Number of Beads'
      do k=1,T%SIM%NOSTATES
         if (k .lt. 14) then
            write(98,*) T%SIM%STATE(k),' !Towed States'
         else if (k .lt. 26) then
            write(98,*) T%SIM%STATE(k),' !Driver States'
         else if (k .ge. 26) then
            write(98,*) T%SIM%STATE(k),' !Tether States'
         end if
      end do
      write(98,*) T%TOW%DELTHRUST, ' !Thrust'
      write(98,*) T%TOW%ELEVATOR, ' !Elevator'
      write(98,*) T%TOW%AILERON, ' !Aileron'
      write(98,*) T%TOW%RUDDER, ' !Rudder'
      write(98,*) T%TOW%INITIALSTATE(8), ' !Velocity Command'
      !close(98)
      write(*,*) 'Restart point created'
      T%SIM%CREATERESTART = 0
   end if


  end do

  toctotal = ETIME(elapsed)
  tocuser = elapsed(1)
  tocsystem = elapsed(2)

  T%SIM%CPUTIMEUSER = tocuser - ticuser
  T%SIM%CPUTIMESYSTEM = tocsystem - ticsystem
  T%SIM%CPUTIMETOTAL = toctotal - tictotal

  write(*,*) 'Driver Integral States'
  write(*,*) T%DRIVER%XINTEGRAL,T%DRIVER%YINTEGRAL,T%DRIVER%ZINTEGRAL,T%DRIVER%PHIINTEGRAL,T%DRIVER%THETAINTEGRAL,T%DRIVER%PSIINTEGRAL,T%DRIVER%UINTEGRAL

  write(25,*) ' '
  write(25,*) 'SIMULATION CPU TIME USER (sec): ',T%SIM%CPUTIMEUSER,tocuser,ticuser
  write(25,*) 'SIMULATION CPU TIME SYSTEM (sec): ',T%SIM%CPUTIMESYSTEM,tocsystem,ticsystem
  write(25,*) 'SIMULATION CPU TIME TOTAL (sec): ',T%SIM%CPUTIMETOTAL,toctotal,tictotal
  write(25,*) ' '
 
  write(*,*) ' '
  write(*,*) 'TIME SIMULATION COMPLETE'
  write(*,*) ' '
  write(25,*) ' '
  write(25,*) 'TIME SIMULATION COMPLETE'
  write(25,*) ' '

  ! Close Output Files

  close(92)
  close(96)
  close(91)
  close(83)

  RETURN
  
 end if
 
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! LOAD DATA iflag = 1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 
 if (iflag .eq. 1) then

  !!!13 = aircraft,12 = Driver,7*T%TH%NBEADS + 1 = tether,2 = actuator dynamics
  !!! REVISIT Nghia : Added 8 states for thrust and thrustdot for each rotor
  
  T%SIM%NOSTATES = 13 + 12 + 7*T%THR%NBEADS + 1 + 8
  
  open(unit=90,file=T%SIMINPUTFILE,status='old',iostat=openflag)
  if (openflag .ne. 0) then
   write(*,*) 'Error Opening Simulation Input File: ',T%SIMINPUTFILE; PAUSE; STOP
  end if
  rewind(90)
  
  read(unit=90,fmt=*,iostat=readflag) T%SIM%INITIALTIME
  read(unit=90,fmt=*,iostat=readflag) T%SIM%FINALTIME
  read(unit=90,fmt=*,iostat=readflag) T%SIM%DELTATIME
  read(unit=90,fmt=*,iostat=readflag) readreal; T%SIM%IOUTSKIP = int(readreal)
  read(unit=90,fmt=*,iostat=readflag) readreal; T%SIM%ICS = int(readreal)
  read(unit=90,fmt=*,iostat=readflag) readreal; T%SIM%ACTUATORONOFF = int(readreal)
  read(unit=90,fmt=*,iostat=readflag) readreal; T%SIM%IDEBUG = int(readreal)
  read(unit=90,fmt=*,iostat=readflag) readreal; T%SIM%IDXOUT = int(readreal)
  ! read(unit=90,fmt=*,iostat=readflag) readreal; T%SIM%CREATERESTART = int(readreal)
  ! read(unit=90,fmt=*,iostat=readflag) T%SIM%RESTARTTIME
  read(unit=90,fmt=*,iostat=readflag) readreal; T%SIM%RESTART = int(readreal)
  read(unit=90,fmt=*,iostat=readflag) T%SIM%RESTARTFILE
  
  T%SIM%CREATERESTART = 1
  T%SIM%RESTARTTIME = T%SIM%FINALTIME

  !Always make a restart point at the end of the simulation
  open(unit=98,file='Output_Files/Default.RESTART',iostat=openflag)
  if (openflag .ne. 0) then
     open(unit=98,file='Default.RESTART',iostat=openflag)
     if (openflag .ne. 0) then
        write(*,*) 'Error Opening Restart Input File: ','Default.RESTART'; PAUSE; STOP
     end if
  end if

  if (T%SIM%RESTART .eq. 1)  then
     close(90)

     !!Open RESTARTFILE 

     open(unit=90,file=T%SIM%RESTARTFILE,status='old',iostat=openflag)
     if (openflag .ne. 0) then
        write(*,*) 'Error Opening Restart Input File: ',T%SIM%RESTARTFILE; PAUSE; STOP
     end if
     rewind(90)
     
     !Read restart time and shift Driver time

     read(unit=90,fmt=*,iostat=readflag) readreal;
     T%DRIVER%TIMEON = -readreal

     !Read number of beads

     read(unit=90,fmt=*,iostat=readflag) readreal
     if (readreal .ne. T%THR%NBEADS) then
        write(*,*) 'Error Restart file has a different number of beads then Tether Input file'; PAUSE; STOP  
     end if

     !Read state

     do i = 1,T%SIM%NOSTATES
        read(unit=90,fmt=*,iostat=readflag) T%SIM%INITIALSTATE(i)
     end do

     !Set initial location of Driver (x,y,z,p,t,p,u,v,w,p,q,r)
     T%DRIVER%INITIALSTATE(1:12) = T%SIM%INITIALSTATE(14:25)
     ! Added thrust initial conditions
     T%DRIVER%INITIALSTATE(13:20) = T%SIM%INITIALSTATE((T%SIM%NOSTATES-7):T%SIM%NOSTATES)
     T%DRIVER%XCGINITIAL = T%SIM%INITIALSTATE(14)
     T%DRIVER%YCGINITIAL = T%SIM%INITIALSTATE(15)
     T%DRIVER%ZCGINITIAL = T%SIM%INITIALSTATE(16)
     !Set restart speed of Driver
     T%DRIVER%RESTARTSPEED = T%SIM%INITIALSTATE(20)

     !Read controls

     read(unit=90,fmt=*,iostat=readflag) T%TOW%DELTHRUST
     read(unit=90,fmt=*,iostat=readflag) T%TOW%ELEVATOR
     read(unit=90,fmt=*,iostat=readflag) T%TOW%AILERON
     read(unit=90,fmt=*,iostat=readflag) T%TOW%RUDDER

     !Read Towed Velocity Command

     read(unit=90,fmt=*,iostat=readflag) T%TOW%INITIALSTATE(8) 
     write(*,*) '********Restart point loaded*******'
  else

     !Set restart speed to 0
     T%DRIVER%RESTARTSPEED = 0

     !Read Driver initial states
        ! write(*,*) 'Driver States'
     do i=1,12
        ! write(*,*) T%DRIVER%INITIALSTATE(i)
        read(unit=90,fmt=*,iostat=readflag) T%DRIVER%INITIALSTATE(i)
     end do
     !Put initial state vector into the state vector
     T%DRIVER%STATE(1:12) = T%DRIVER%INITIALSTATE(1:12)

     !!!!Read Towed Initial States!!!

        ! write(*,*) 'Aircraft States'
     do i=1,12
        ! write(*,*) T%TOW%INITIALSTATE(i)
        read(unit=90,fmt=*,iostat=readflag) T%TOW%INITIALSTATE(i)
     end do

     !Read Driver Thrust initial states
        ! write(*,*) 'Thrust States'
     do i=1,8
        ! write(*,*) T%DRIVER%INITIALSTATE(i)
        read(unit=90,fmt=*,iostat=readflag) T%DRIVER%INITIALSTATE(i+12)
     end do
     T%DRIVER%STATE(13:20) = T%DRIVER%INITIALSTATE(13:20)
     !!!Convert Phi,theta,psi to quaternions!!!

     T%TOW%INITIALPHI = T%TOW%INITIALSTATE(4)
     T%TOW%INITIALTHETA = T%TOW%INITIALSTATE(5)
     T%TOW%INITIALPSI = T%TOW%INITIALSTATE(6)
     T%TOW%INITIALSTATE(13) = T%TOW%INITIALSTATE(12) ! rb 
     T%TOW%INITIALSTATE(12) = T%TOW%INITIALSTATE(11) ! qb
     T%TOW%INITIALSTATE(11) = T%TOW%INITIALSTATE(10) ! pb
     T%TOW%INITIALSTATE(10) = T%TOW%INITIALSTATE(9)  ! wb
     T%TOW%INITIALSTATE(9)  = T%TOW%INITIALSTATE(8)  ! vb
     T%TOW%INITIALSTATE(8)  = T%TOW%INITIALSTATE(7)  ! ub
     T%TOW%INITIALSTATE(7) = sin(T%TOW%INITIALPSI/2.0)*cos(T%TOW%INITIALTHETA/2.0)*cos(T%TOW%INITIALPHI/2.0) - cos(T%TOW%INITIALPSI/2.0)*sin(T%TOW%INITIALTHETA/2.0)*sin(T%TOW%INITIALPHI/2.0)  ! q3
     T%TOW%INITIALSTATE(6) = cos(T%TOW%INITIALPSI/2.0)*sin(T%TOW%INITIALTHETA/2.0)*cos(T%TOW%INITIALPHI/2.0) + sin(T%TOW%INITIALPSI/2.0)*cos(T%TOW%INITIALTHETA/2.0)*sin(T%TOW%INITIALPHI/2.0)  ! q2
     T%TOW%INITIALSTATE(5) = cos(T%TOW%INITIALPSI/2.0)*cos(T%TOW%INITIALTHETA/2.0)*sin(T%TOW%INITIALPHI/2.0) - sin(T%TOW%INITIALPSI/2.0)*sin(T%TOW%INITIALTHETA/2.0)*cos(T%TOW%INITIALPHI/2.0)  ! q1
     T%TOW%INITIALSTATE(4) = cos(T%TOW%INITIALPSI/2.0)*cos(T%TOW%INITIALTHETA/2.0)*cos(T%TOW%INITIALPHI/2.0) + sin(T%TOW%INITIALPSI/2.0)*sin(T%TOW%INITIALTHETA/2.0)*sin(T%TOW%INITIALPHI/2.0)  ! q0
     T%TOW%INITIALSTATE(3) = T%TOW%INITIALSTATE(3) ! zcg 
     !T%TOW%INITIALSTATE(3) = T%THR%LEN ! zcg 
     T%TOW%INITIALSTATE(2) = T%TOW%INITIALSTATE(2) ! ycg
     T%TOW%INITIALSTATE(1) = T%TOW%INITIALSTATE(1) ! xcg

     !!!!!! Compute initial Tether points !!!!!!!!!!

     call DRIVER(T,2) ! Compute location of reel !!!!!Reel Location = T%DRIVER%(XYZ)REEL

     !!Place Driver states in initial state vector
     T%DRIVER%INITIALSTATE(1:20) = T%DRIVER%STATE(1:20)

     !pass aircraft state to global state

     T%TOW%STATE(1:13) = T%TOW%INITIALSTATE(1:13)

     ! Compute location of connection point on Towed Platform

     call TOWED(T,2) !Cradle tether connection point Location = T%THR%(XYZ)TETHER

     stateindex = 0
     xslope = (T%THR%XTETHER - T%DRIVER%XREEL)/(T%THR%NBEADS+1)
     yslope = (T%THR%YTETHER - T%DRIVER%YREEL)/(T%THR%NBEADS+1)
     zslope = (T%THR%ZTETHER - T%DRIVER%ZREEL)/(T%THR%NBEADS+1)
     do i=1,T%THR%NBEADS
        T%THR%INITIALSTATE(stateindex+1) = T%DRIVER%XREEL + xslope*(i);
        T%THR%INITIALSTATE(stateindex+2) = T%DRIVER%YREEL + yslope*(i);
        T%THR%INITIALSTATE(stateindex+3) = T%DRIVER%ZREEL + zslope*(i);
        T%THR%INITIALSTATE(stateindex+4) = 0 !Initialize derivatives to zero
        T%THR%INITIALSTATE(stateindex+5) = 0
        T%THR%INITIALSTATE(stateindex+6) = 0
        stateindex = stateindex + 6
     end do
     do i=1,T%THR%NBEADS+1
        !T%THR%INITIALSTATE(stateindex+1) = 0 !Initialize tensions to the weight of the towed system or just zero
        T%THR%INITIALSTATE(stateindex+1) = T%TOW%WEIGHT !Initialize tensions to the weight of the towed system or just zero
        stateindex = stateindex + 1
     end do

     !!!!PLACE LOCAL INITIAL STATES INTO GLOBAL INITIAL STATES
     !!TOWED STATES
     T%SIM%INITIALSTATE(1:13) = T%TOW%INITIALSTATE(1:13)
     !!DRIVER STATES(14:25)
     T%SIM%INITIALSTATE(14:25) = T%DRIVER%INITIALSTATE(1:12)
     !!!TETHER STATES
     T%SIM%INITIALSTATE(26:T%SIM%NOSTATES-8) = T%THR%INITIALSTATE(1:7*T%THR%NBEADS+1)
     !!!REVISIT THRUST STATES
     T%SIM%INITIALSTATE(T%SIM%NOSTATES-7:T%SIM%NOSTATES) = T%DRIVER%INITIALSTATE(13:20)
  end if

  !Initialize Time Vector
  T%SIM%TIME = T%SIM%INITIALTIME
  T%SIM%STATE(1:T%SIM%NOSTATES) = T%SIM%INITIALSTATE(1:T%SIM%NOSTATES)

  ! State Limits
  
  call STATELIMITS(T)

  T%SIM%DQFLAG = 1

  write(*,*) 'SIMULATION Load Complete'


  RETURN
 
 end if
 
 RETURN
END SUBROUTINE SIMULATION

!!!!!!!!!!!!!!!!!!!!!!!! SUBROUTINE SYSTEMDERIVATIVES !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

SUBROUTINE SYSTEMDERIVATIVES(T,iflag)
 use TOSIMDATATYPES
 implicit none
 integer stateindex,iflag,i
 type(TOSIMSTRUCTURE) T

!!!!!!!!!!!!!!!!!!!!!!!! COMPUTE iflag = 3!!!!!!!!!!!!!!!!!

if (iflag .eq. 2) then
   
    stateindex = 0 
    
    ! Towed State Derivatives

    T%TOW%STATEDOT = 0.0
    T%TOW%STATE(1:13) = T%SIM%STATE(stateindex+1:stateindex+13)
    if (T%TOW%DYNOFFON .eq. 1) then
       call TOWED(T,2)
       T%SIM%STATEDOT(stateindex+1:stateindex+13) = T%TOW%STATEDOT(1:13)
    end if
    stateindex = stateindex + 13

    ! Driver Motion or Derivation
    if (T%DRIVER%OFFON .eq. 1) then
       if ((T%DRIVER%MODNO .eq. 0) .or. (T%DRIVER%MODNO .eq. 3)) then !0 and 3 are our integration models
          T%DRIVER%STATE(1:12) = T%SIM%STATE(stateindex+1:stateindex+12)
          T%DRIVER%STATE(13:20) = T%SIM%STATE((T%SIM%NOSTATES-7):T%SIM%NOSTATES)
       end if
       call DRIVER(T,2)
       if ((T%DRIVER%MODNO .eq. 0) .or. (T%DRIVER%MODNO .eq. 3)) then
          T%SIM%STATEDOT(stateindex+1:stateindex+12) = T%DRIVER%STATEDOT(1:12)
          T%SIM%STATEDOT((T%SIM%NOSTATES-7):T%SIM%NOSTATES) = T%DRIVER%STATEDOT(13:20)
       else
          T%SIM%STATE(stateindex+1:stateindex+12) = T%DRIVER%STATE(1:12)
       end if
    else
       T%DRIVER%TIMEON = T%SIM%TIME
    end if
    stateindex = stateindex + 12

    ! Tether State Derivatives
    
    T%THR%STATEDOT = 0.0
    T%THR%STATE(1:7*T%THR%NBEADS+1) = T%SIM%STATE(stateindex+1:stateindex+7*T%THR%NBEADS+1)
    if (T%THR%DYNOFFON .eq. 1) then
       call TETHER(T,2)
       T%SIM%STATEDOT(stateindex+1:stateindex+7*T%THR%NBEADS+1) = T%THR%STATEDOT(1:7*T%THR%NBEADS+1)
    end if
    stateindex = stateindex + 7*T%THR%NBEADS + 1

    RETURN
    
 end if

 RETURN
END SUBROUTINE SYSTEMDERIVATIVES

!!!!!!!!!!!!!!!!!!!1!!! SUBROUTINE STATELIMITS!!!!!!!!!!!!!!!!!!!!!!!!!!!!

SUBROUTINE STATELIMITS(T)
 use TOSIMDATATYPES
 implicit none
 integer i,stateindex,counter
 real*8 q0,q1,q2,q3,norm
 type(TOSIMSTRUCTURE) T

 ! Normalize Towed System Quaternions

 if (T%TOW%DYNOFFON .eq. 1) then
  stateindex = 0.0
  T%TOW%STATE(1:13) = T%SIM%STATE(stateindex+1:stateindex+13)
  q0 = T%TOW%STATE(4)
  q1 = T%TOW%STATE(5)
  q2 = T%TOW%STATE(6)
  q3 = T%TOW%STATE(7)
  norm = sqrt(q0**2+q1**2+q2**2+q3**2)
  T%TOW%STATE(4) = q0/norm
  T%TOW%STATE(5) = q1/norm
  T%TOW%STATE(6) = q2/norm
  T%TOW%STATE(7) = q3/norm  
  T%SIM%STATE(stateindex+1:stateindex+13) = T%TOW%STATE(1:13)
 end if

 RETURN
END SUBROUTINE STATELIMITS

!!!!!!!!!!!!!!!!!!!!!!!!!!! SUBROUTINE CONTROL !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

SUBROUTINE CONTROL(T,iflag)
 use TOSIMDATATYPES
 implicit none
 integer iflag,openflag,readflag,i,stateindex,now(3),ctr,j
 integer , parameter :: nwp = 1
 real*8 readreal,phicommand,deltheta,delpsi,dely,delydot,rAS_I(3,1),vAS_I(3,1),thetacommand, xcommand, ycommand, zcommand
 real*8 rAS_A(3,1),vAS_A(3,1),lencommand,rGS_I(3,1),vGS_I(3,1),psicp,rGS_P(3,1),vGS_P(3,1),tension,len
 real*8 angles(2,1),delphi,delphidot,ldotnom,ldot,n1,q0,q1,q2,q3,pb,qb,rb,vb,xcg,ycg,zcg,vaero,wb
 real*8 delx,delz,phi,theta,psi,p,q,r,xdot,ydot,zdot,omegaNot,addpitch,addroll,addyaw
 real*8 domegaLeft,omegaRight,domegaFront,omegaBack,domegaDiag,omegaOpp,omegaDiag,omegaFront
 real*8 xwaypoint(nwp,1),ywaypoint(nwp,1),zwaypoint(nwp,1),Dwaypoint
 real*8 delmu(4),munominal,MAXANGLE
 type(TOSIMSTRUCTURE) T
 LOGICAL :: DOUBLET = .FALSE.
 
!!!!!!!!!!!!!!!!!!!!!!! COMPUTE iflag = 3 !!!!!!!!!!!!!!!!!!!!!!!!!!
 if (iflag .eq. 2) then  

    ! Tether Line Length Control
    lencommand = T%THR%NOMLEN !Default value is to keep tether at nominal length

    if (T%THR%TETHERCONTROLOFFON .eq. 1) then
       ! Extension according to precomputed Bezier curve profile
       lencommand = 0.0 
       if (T%SIM%TIME .le. T%THR%TCOM(1)) then
          lencommand = T%THR%REELTRAJ(1) 
       else if (T%SIM%TIME .ge. T%THR%TCOM(200)) then
          lencommand = T%THR%REELTRAJ(200)
       else
          do i=2,200
             if ((T%SIM%TIME .ge. T%THR%TCOM(i-1)) .and. (T%SIM%TIME .le. T%THR%TCOM(i))) then
                lencommand = T%THR%REELTRAJ(i-1) + ((T%SIM%TIME-T%THR%TCOM(i-1))/(T%THR%TCOM(i)-T%THR%TCOM(i-1)))*(T%THR%REELTRAJ(i)-T%THR%REELTRAJ(i-1))
             end if
          end do
       end if
    end if

    !!Constant tension controller
    if (T%THR%TETHERCONTROLOFFON .eq. 2) then
       ldotnom = 0
       T%THR%LDOTNOMINAL = ldotnom
       tension = T%THR%TENSIONWINCH
       !!!Now edit pay out rate based on tension
       ldot = 0.004*(tension-1000) + ldotnom
       !Check for minimum
       if (ldotnom .gt. 0) then
          if (ldot .lt. ldotnom) then
             ldot = ldotnom
          end if
       end if
       !Check for maximum
       if (ldot .gt. 10) then
          ldot = 10
       end if
       if (ldot .lt. -10) then
          ldot = -10
       end if
       T%THR%LDOTCOMMAND = ldot
       !!Now compute new length command based on ldot
       lencommand = T%SIM%DELTATIME*ldot + T%THR%PREVLEN
       T%THR%PREVLEN = lencommand
    end if

    if (T%THR%TETHERCONTROLOFFON .eq. 3) then
       ! Extension according to precomputed Bezier curve profile
       if (T%THR%LDOTFACTOR .eq. 1) then
          lencommand = 0.0 
          if (T%SIM%TIME .le. T%THR%TCOM(1)) then
             lencommand = T%THR%REELTRAJ(1) 
          else if (T%SIM%TIME .ge. T%THR%TCOM(200)) then
             lencommand = T%THR%REELTRAJ(200)
           else
             do i=2,200
                if ((T%SIM%TIME .ge. T%THR%TCOM(i-1)) .and. (T%SIM%TIME .le. T%THR%TCOM(i))) then
                   lencommand = T%THR%REELTRAJ(i-1) + ((T%SIM%TIME-T%THR%TCOM(i-1))/(T%THR%TCOM(i)-T%THR%TCOM(i-1)))*(T%THR%REELTRAJ(i)-T%THR%REELTRAJ(i-1))
                end if
             end do
          end if
          !Compute LdotNom based on previous length command
          ldotnom = (lencommand-T%THR%PREVLENCOMMAND)/T%SIM%DELTATIME
       else
          !Once tether fully paid out no need to compute
          ldotnom = 0
       end if
       T%THR%LDOTNOMINAL = ldotnom
       T%THR%PREVLENCOMMAND = lencommand
       !Now get tension estimate
       tension = T%THR%TENSIONWINCH
       !!!Now edit pay out rate based on tension
       ldot = 0.004*(tension-1000) + ldotnom
       !Check for minimum
       if (ldotnom .gt. 0) then
          if (ldot .lt. ldotnom) then
             ldot = ldotnom
          end if
       end if
       !Check for maximum
       if (ldot .gt. 10) then
          ldot = 10
       end if
       if (ldot .lt. -10) then
          ldot = -10
       end if
       T%THR%LDOTCOMMAND = ldot
       !!Now compute new length command based on ldot
       lencommand = T%SIM%DELTATIME*ldot + T%THR%PREVLEN
       T%THR%PREVLEN = lencommand
    end if

    if (T%THR%LEN .gt. 478) then
       T%THR%LDOTFACTOR = 0
    end if

    !! Controllers from above set the lencommand of the tether
    !! It is the job of the Torque Controller at the reel to set the 
    !! pay in and pay out the line
    if (T%SIM%ACTUATORONOFF .eq. 1) then
       tension = T%THR%TENSIONWINCH
       T%THR%THETACOMMAND = (lencommand - T%THR%NOMLEN)/T%THR%RREEL
       T%THR%THETAREEL = T%SIM%ACTUATOR(3)
       T%THR%THETADOTREEL = T%SIM%ACTUATOR(4)
       T%THR%THETAINT = T%SIM%ACTUATOR(6)
       T%THR%TORQUECOMMAND = T%THR%NOMTORQUE + T%THR%IREEL*(T%THR%KTETHER*(T%THR%THETACOMMAND-T%THR%THETAREEL)+T%THR%KDTETHER*(0-T%THR%THETADOTREEL)+T%THR%KITETHER*(T%THR%THETAINT))

       ! T%THR%TORQUECOMMAND = -194

       !!Compute Tether Length based on theta
       ! T%THR%LEN = T%THR%THETAREEL*T%THR%RREEL + T%THR%NOMLEN
       T%THR%LEN = lencommand
    else
       T%THR%LEN = lencommand
    end if

    !Check for stops on LENCOMMAND
    if (T%THR%LEN .lt. T%THR%LENMIN) then
       T%THR%LEN = T%THR%LENMIN
    end if
    if (T%THR%LEN .gt. T%THR%LENMAX) then
       T%THR%LEN = T%THR%LENMAX
    end if

    call TETHERPROPERTIES(T)

    !! Controller for TOW
    if (T%TOW%CONTROLOFFON .eq. 1) then
       !Quadcopter control
       T%TOW%OMEGAVEC = 0
       T%TOW%THRUSTVEC = 0
       T%TOW%MUVEC = 0
       
       xdot = T%TOW%STATEDOT(1)
       ydot = T%TOW%STATEDOT(2)
       zdot = T%TOW%STATEDOT(3)
       phi = T%TOW%STATE(4)
       theta = T%TOW%STATE(5)
       psi = T%TOW%STATE(6)
       p = T%TOW%STATE(10)
       q = T%TOW%STATE(11)
       r = T%TOW%STATE(12)

       ! gotocontrols
       ! T%TOW%XCOMMAND =  xwaypoint(T%TOW%WAYPOINT,1)
       ! T%TOW%YCOMMAND =  ywaypoint(T%TOW%WAYPOINT,1)
       ! T%TOW%ZCOMMAND =  zwaypoint(T%TOW%WAYPOINT,1)
       
       !T%TOW%ZCOMMAND = -30.0 -- these are now set in the input file 
       !T%TOW%UCOMMAND = 28.93
       !T%TOW%YCOMMAND = 0.0
       
       !delx = (T%TOW%XCOMMAND - T%TOW%STATE(1))*-1.00D0
       dely = (T%TOW%YCOMMAND - T%TOW%STATE(2))
       !write(*,*) 'Zstuff = ',T%TOW%ZCOMMAND,T%TOW%STATE(3)
       delz = (T%TOW%ZCOMMAND - T%TOW%STATE(3))*-1.00D0

       ! Dwaypoint = sqrt((delx)**2 + (dely)**2 + (delz)**2)

       ! Change this to 4(square) or 8(octagon) depending on the shape you want to simulate
       ! if (Dwaypoint .lt. 1.0D0) then
       !   T%TOW%WAYPOINT= T%TOW%WAYPOINT+1
       !   if (T%TOW%WAYPOINT .gt. nwp) then
       !     T%TOW%WAYPOINT=1
       !   end if
       ! end if

       !!! Attitude Controller
       T%TOW%PHICOMMAND = 0
       !T%TOW%PHICOMMAND = 20.0*qPI/180.0
     
       ! REVISIT hardcoded xdotcommand = 40 ft/s
       ! For now the KP and KD gains for 'x' is controlling the 'u' velocity at which the quad is travelling
       ! Ok shit so the 'u' velocity couldn't really be controlled so -24.5 deg is what 
       ! the quad needs to be to achieve this velocity
       ! T%TOW%THETACOMMAND = -24.5*qPI/180!T%TOW%KPXDRIVE*(T%TOW%STATE(7) - 40.0) + T%TOW%KIXDRIVE*T%TOW%XINTEGRAL + T%TOW%KDXDRIVE*(T%TOW%STATEDOT(7))
       ! T%TOW%THETACOMMAND = T%TOW%KPXDRIVE*delx + T%TOW%KIXDRIVE*T%TOW%XINTEGRAL - T%TOW%KDXDRIVE*xdot
       T%TOW%THETACOMMAND = 0
       !write(*,*) 'integral = ',T%TOW%YINTEGRAL,T%TOW%XINTEGRAL
       !T%TOW%THETACOMMAND = 0.0
       T%TOW%PSICOMMAND = 0.0

       !write(*,*) 'Xcontrol = ',delx,T%TOW%XINTEGRAL,xdot
       !write(*,*) 'ptpcom = ',T%TOW%PHICOMMAND,T%TOW%THETACOMMAND,T%TOW%PSICOMMAND
       
       if (abs(T%TOW%THETACOMMAND) .gt. 30*qPI/180) then
          T%TOW%THETACOMMAND = sign(30*qPI/180,T%TOW%THETACOMMAND)
       end if
       if (abs(T%TOW%PHICOMMAND) .gt. 30*qPI/180) then
          T%TOW%PHICOMMAND = sign(30*qPI/180,T%TOW%PHICOMMAND)
       end if
       if (abs(T%TOW%PSICOMMAND) .gt. 30*qPI/180) then
          T%TOW%PSICOMMAND = sign(30*qPI/180,T%TOW%PSICOMMAND)
       end if

       ! Hovering microseconds and altitude control
       munominal = 1614.855 + T%TOW%KPZDRIVE*(delz) + T%TOW%KIZDRIVE*T%TOW%ZINTEGRAL +T%TOW%KDZDRIVE*zdot  ! Nominal microsecond pulse for hover

       T%TOW%MS_ROLL = T%TOW%KPPHI*(T%TOW%PHICOMMAND-phi) + T%TOW%KIPHI*T%TOW%PHIINTEGRAL - T%TOW%KDPHI*p
       T%TOW%MS_PITCH = T%TOW%KPTHETA*(T%TOW%THETACOMMAND - theta) + T%TOW%KITHETA*T%TOW%THETAINTEGRAL- T%TOW%KDTHETA*q
       T%TOW%MS_YAW = T%TOW%KPPSI*(T%TOW%PSICOMMAND-psi) + T%TOW%KIPSI*T%TOW%PSIINTEGRAL - T%TOW%KDPSI*r
       
       !write(*,*) 'Att = ',phi,theta,psi,p,q,r

       T%TOW%MUVEC(1,1) = munominal + T%TOW%MS_ROLL + T%TOW%MS_PITCH + T%TOW%MS_YAW
       T%TOW%MUVEC(2,1) = munominal - T%TOW%MS_ROLL + T%TOW%MS_PITCH - T%TOW%MS_YAW
       T%TOW%MUVEC(3,1) = munominal - T%TOW%MS_ROLL - T%TOW%MS_PITCH + T%TOW%MS_YAW
       T%TOW%MUVEC(4,1) = munominal + T%TOW%MS_ROLL - T%TOW%MS_PITCH - T%TOW%MS_YAW

       !write(*,*) 'muvec = ',T%TOW%MUVEC
       
       ! Now we saturate the microseconds so that it doesn't go over 1900 or under 1100
       do j = 1,4
          if (T%TOW%MUVEC(j,1) .gt. 1900.00D0) then
             T%TOW%OMEGAVEC(j,1) = 1900.00D0
          end if
          if (T%TOW%MUVEC(j,1) .lt. 1100.00D0) then
             T%TOW%MUVEC(j,1) = 1100.00D0
          end if
       end do
    end if !Quad control off / on
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  RETURN
end if

END SUBROUTINE CONTROL

!!!!!!!!!!!!!!!!!!!1!!! SUBROUTINE ATMOSPHERE !!!!!!!!!!!!!!!!!!!!!!!!!!!!


SUBROUTINE ATMOSPHERE(T,iflag)
 use TOSIMDATATYPES
 implicit none
 integer i,iflag,ifind,openflag,readflag,ii,jj,ierr,now(3)
 real*8 m,readreal,wind,dirnominal,windnominal,winddir
 real*8 dim,dummy(40),rx,test(3),ramp,n1,freq
 character*11 HeightFile
 character*14 ParametersFile
 character*1 letter
 character*10 ZcoordFile,number
 character*15 TimesFile
 character*256 inParameters,inZcoord,inTimes,inHeight,temp
 type(TOSIMSTRUCTURE) T

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! COMPUTE iflag = 3 !!!!!!!!!!!!!!!!!!!!!!!!!!!!

 if (iflag .eq. 2) then  

    !!Reset all winds to zero
    T%ATM%VXWIND = 0
    T%ATM%VYWIND = 0
    T%ATM%VZWIND = 0
    T%ATM%WRFX = 0
    T%ATM%WRFY = 0
    T%ATM%WRFZ = 0
    T%ATM%WINDGUST(1) = 0
    T%ATM%WINDGUST(2) = 0
    T%ATM%WINDGUST(3) = 0
    T%ATM%VWAKE(1) = 0
    T%ATM%VWAKE(2) = 0
    T%ATM%VWAKE(3) = 0

    T%ATM%ALT = -T%ATM%ZI

  ! Constant Model

    dirnominal = T%ATM%WINDDIR
    windnominal = 0.0
    if ((T%ATM%WINDDIR .ne. dirnominal) .or. (T%ATM%WINDSPEED .ne. windnominal)) then
       !if (T%ATM%WINDSPEED .ne. windnominal) then
       if (T%SIM%TIME .gt. 1) then
          winddir = T%ATM%WINDDIR
          wind = T%ATM%WINDSPEED
       else
          wind = windnominal + (T%SIM%TIME/1)*(T%ATM%WINDSPEED-windnominal)
          winddir = dirnominal + (T%SIM%TIME/1)*(T%ATM%WINDDIR-dirnominal)     
       endif
    else
       wind = windnominal
       winddir = dirnominal
    end if
    
    if (T%ATM%MODNO .eq. 1) then
       T%ATM%VXWIND = wind*cos(winddir)*cos(T%ATM%WINDELEV)
       !!!!REVISIT REVISIT REVISIT
       freq = 1
       if (T%ATM%FREQUENCY .ne. 0) then
          freq = sin((2*PI)/(T%ATM%FREQUENCY)*T%SIM%TIME)
       end if
       T%ATM%VYWIND = wind*sin(winddir)*cos(T%ATM%WINDELEV)*freq
       T%ATM%VZWIND = -wind*sin(T%ATM%WINDELEV)
       !T%ATM%DEN = 1.22566; This got moved to the towed body file
    end if

  ! Equation Density and Constant Wind Model

  if (T%ATM%MODNO .eq. 2) then
     T%ATM%DEN = 1.22566578494891*(1.00000000-0.0000225696709*T%ATM%ALT)**4.258
     T%ATM%VXWIND = T%ATM%WINDSPEED*cos(T%ATM%WINDDIR)
     T%ATM%VYWIND = T%ATM%WINDSPEED*sin(T%ATM%WINDDIR)
     T%ATM%VZWIND = 0.0
  end if

  ! Table Look-Up Model 

  if (T%ATM%MODNO .eq. 3) then

   ifind = 0

   ! Position Pointer  

   if (T%ATM%ALT .le. T%ATM%ALTTAB(T%ATM%IP)) then 
    ifind = -1 
    do while ((ifind.ne.0) .and. (T%ATM%IP.gt.1))
     T%ATM%IP = T%ATM%IP - 1 
     if (T%ATM%ALTTAB(T%ATM%IP)   .le. T%ATM%ALT) then 
     if (T%ATM%ALTTAB(T%ATM%IP+1) .gt. T%ATM%ALT) then 
      ifind = 0
     end if
     end if 
    end do 
   end if
   if (T%ATM%ALT .gt. T%ATM%ALTTAB(T%ATM%IP+1)) then 
    ifind = 1
    do while ((ifind.ne.0) .and. (T%ATM%IP.lt.T%ATM%TABSIZE-1))
     T%ATM%IP = T%ATM%IP + 1 
     if (T%ATM%ALTTAB(T%ATM%IP)   .le. T%ATM%ALT) then 
     if (T%ATM%ALTTAB(T%ATM%IP+1) .gt. T%ATM%ALT) then 
      ifind = 0 
     end if 
     end if 
    end do 
   end if
   if (ifind .eq. 0) then
    m = (T%ATM%ALT-T%ATM%ALTTAB(T%ATM%IP))/(T%ATM%ALTTAB(T%ATM%IP+1)-T%ATM%ALTTAB(T%ATM%IP))
   else if (ifind .eq. -1) then
    m = 0.0
   else if (ifind .eq. 1) then
    m = 1.0
   end if

   ! Interpolate

   T%ATM%DEN    = T%ATM%DENTAB(T%ATM%IP)    + m*(T%ATM%DENTAB(T%ATM%IP+1)-T%ATM%DENTAB(T%ATM%IP))
   T%ATM%VXWIND = T%ATM%VXWINDTAB(T%ATM%IP) + m*(T%ATM%VXWINDTAB(T%ATM%IP+1)-T%ATM%VXWINDTAB(T%ATM%IP))
   T%ATM%VYWIND = T%ATM%VYWINDTAB(T%ATM%IP) + m*(T%ATM%VYWINDTAB(T%ATM%IP+1)-T%ATM%VYWINDTAB(T%ATM%IP))
   T%ATM%VZWIND = T%ATM%VZWINDTAB(T%ATM%IP) + m*(T%ATM%VZWINDTAB(T%ATM%IP+1)-T%ATM%VZWINDTAB(T%ATM%IP))

  end if

  ! write(*,*) 'xi,YI,zi = ',T%ATM%XI,T%ATM%YI,T%ATM%ZI

  if (T%ATM%MODNO .eq. 4) then
     !T%ATM%DEN = 1.22566; !Keep density constant at least for now this got moved to the towed body file
     call WRFMODEL(T) !Call WRF model. All velocities needed are set inside this subroutine
     !//Add in Full Field Dryden Gust model
     !write(*,*) 'XI,yi,zi = ',T%ATM%XI,T%ATM%YI,T%ATM%ZI
     call TURBULENCE(T)
  end if

  ! write(*,*) 'XI,YI,zi = ',T%ATM%XI,T%ATM%YI,T%ATM%ZI

  ! T%ATM%XI = -119.88765+0-77
  ! T%ATM%YI = 0
  ! T%ATM%ZI = -31.19
  ! T%SIM%TIME = 6.8
  if (T%DRIVER%AIRWAKE .eq. 1) then
     !Add in Driver airwake velocity
     T%DRIVER%TIME = T%SIM%TIME
     call AIRWAKE(T%DRIVER,T%ATM%XI,T%ATM%YI,T%ATM%ZI)
     T%ATM%VWAKE = T%DRIVER%VWAKE
  end if
  ! PAUSE; STOP;

  !!Add all winds
  T%ATM%VXWIND = T%ATM%VXWIND + T%ATM%VWAKE(1) + T%ATM%WINDGUST(1) + T%ATM%WRFX
  T%ATM%VYWIND = T%ATM%VYWIND + T%ATM%VWAKE(2) + T%ATM%WINDGUST(2) + T%ATM%WRFY
  T%ATM%VZWIND = T%ATM%VZWIND + T%ATM%VWAKE(3) + T%ATM%WINDGUST(3) + T%ATM%WRFZ

  if (T%SIM%TIME .lt. T%ATM%TIMEON) then
     T%ATM%VXWIND = 0
     T%ATM%VYWIND = 0
     T%ATM%VZWIND = 0
  else
     ramp = 0
     if (T%SIM%TIME .lt. 10+T%ATM%TIMEON) then
        ramp = (T%SIM%TIME-T%ATM%TIMEON)/10
     else
        ramp = 1
     end if
     T%ATM%VXWIND = ramp*T%ATM%VXWIND
     T%ATM%VYWIND = ramp*(T%ATM%VYWIND-0*10.5) !!!!REVISIT REVISIT REVISIT - HARDCODE GUST FIELD
     T%ATM%VZWIND = ramp*T%ATM%VZWIND
  end if

  RETURN

end if

!!!!!!!!!!!!!!!!!!!!!!!!!!!! LOAD DATA iflag = 1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 
 if (iflag .eq. 1) then

    write(*,*) 'Loading Atmosphere File'
    
  open(unit=94,file=T%ATMOSPHEREINPUTFILE,status='old',iostat=openflag)
  if (openflag .ne. 0) then
   write(*,*) 'Error Opening Atmosphere Input File: ',T%ATMOSPHEREINPUTFILE; PAUSE; STOP
  else
     write(*,*) 'Atmosphere File opened'
  end if
  rewind(94)
  
  read(unit=94,fmt=*,iostat=readflag) readreal; T%ATM%MODNO = readreal
  read(unit=94,fmt=*,iostat=readflag) readreal; T%ATM%TIMEON = readreal

  if (T%ATM%MODNO .eq. 1) then
   !read(unit=94,fmt=*,iostat=readflag) T%ATM%DEN this got moved to the towed body file
   read(unit=94,fmt=*,iostat=readflag) T%ATM%WINDSPEED
   read(unit=94,fmt=*,iostat=readflag) T%ATM%WINDDIR
   read(unit=94,fmt=*,iostat=readflag) T%ATM%WINDELEV
   read(unit=94,fmt=*,iostat=readflag) T%ATM%FREQUENCY
  end if
  if (T%ATM%MODNO .eq. 2) then
   read(unit=94,fmt=*,iostat=readflag) T%ATM%WINDSPEED
   read(unit=94,fmt=*,iostat=readflag) T%ATM%WINDDIR
   read(unit=94,fmt=*,iostat=readflag) T%ATM%WINDELEV
  end if
  if (T%ATM%MODNO .eq. 3) then
   read(unit=94,fmt=*,iostat=readflag) readreal; T%ATM%TABSIZE = readreal
   do i=1,T%ATM%TABSIZE  
    read(unit=94,fmt=*,iostat=readflag) T%ATM%ALTTAB(i),T%ATM%DENTAB(i),T%ATM%VXWINDTAB(i),T%ATM%VYWINDTAB(i),T%ATM%VZWINDTAB(i)
   end do
  end if
  if (T%ATM%MODNO .eq. 4) then
     !write(*,*) 'Module Number 4'
     !Atmospheric Density
     !read(unit=94,fmt=*,iostat=readflag) T%ATM%DEN this got moved to the towed body file
     !read(unit=94,fmt=*,iostat=readflag) readreal;T%ATM%TIMEVARYING = int(readreal)
     read(unit=94,fmt=*,iostat=readflag) T%ATM%IWINDSCALE
     read(unit=94,fmt=*,iostat=readflag) T%ATM%TURBLEVEL
     read(unit=94,fmt=*,iostat=readflag) T%ATM%PSIOFFSET
     read(unit=94,fmt=*,iostat=readflag) T%ATM%WAVESPEED(1)
     read(unit=94,fmt=*,iostat=readflag) T%ATM%WAVESPEED(2)

     !Read PATH
     read(unit=94,fmt=*,iostat=readflag) T%ATM%PATH
     read(unit=94,fmt=*,iostat=readflag) T%ATM%RANDOMIZE

     !%%%%%%%%Import Extra Parameters File%%%%%%%%% */

     ParametersFile = 'Parameters.txt';
     inParameters = trim(T%ATM%PATH)//ParametersFile
     open(unit=78,file=inParameters,status='old',iostat=ierr)
     if (ierr .ne. 0) then
        write(*,*) 'Parameters.txt File defined incorrectly. Check to make sure your path is set correctly'
        write(*,*) 'Looking in Path: ',T%ATM%PATH
        PAUSE;
        STOP;
     endif
     read(unit=78,fmt=*) T%ATM%parameters
     close(78)
  
     ZcoordFile = 'Zcoord.txt';
     inZcoord = trim(T%ATM%PATH)//ZcoordFile
     open(unit=78,file=inZcoord,status='old',iostat=ierr)
     if (ierr .ne. 0) then
        write(*,*) 'Zcoord.txt File defined incorrectly. Check to make sure your path is set correctly'
        write(*,*) 'Looking in Path: ',T%ATM%PATH
        PAUSE; STOP;
     endif
     read(78,*) T%ATM%zcoord
     close(78)
  
     TimesFile = 'SampleTimes.txt';
     inTimes = trim(T%ATM%PATH)//TimesFile
     open(unit=78,file=inTimes,status='old',iostat=ierr)
     if (ierr .ne. 0) then
        write(*,*) 'SampleTimes.txt File defined incorrectly. Check to make sure your path is set correctly'
        write(*,*) 'Looking in Path: ',T%ATM%PATH
        PAUSE; STOP;
     endif
     read(78,*) T%ATM%tcoord
     close(78)

     ! HeightFile = 'THeight.txt';
     ! inHeight = trim(T%ATM%PATH)//HeightFile
     ! open(unit=78,file=inHeight,status='old',iostat=ierr)
     ! if (ierr .ne. 0) then
     !    write(*,*) 'THeight.txt File defined incorrectly. Check to make sure your path is set correctly'
     !    write(*,*) 'Looking in Path: ',T%ATM%PATH
     !    PAUSE; STOP;
     ! endif
     ! do ii = 1,40
     !    read(78,*) dummy
     !    do jj = 1,40
     !       T%ATM%terrain(ii,jj) = dummy(jj)
     !    enddo
     ! enddo
     ! close(78)

     !%%%%%%%%%%Unwrap Parameters%%%%%%%%%%%%%%% */

     T%ATM%dx = T%ATM%parameters(1);
     T%ATM%dy = T%ATM%parameters(2);
     T%ATM%ztop = T%ATM%parameters(3);

     !%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

     !%%%%%%%%%%%Create xcoord and ycoord%%%%%%% */
     dim = T%ATM%dim
     do ii = 1,dim
        T%ATM%xcoord(ii) = -T%ATM%dx*(dim-1)/2 + (ii-1)*T%ATM%dx
        T%ATM%ycoord(ii) = -T%ATM%dy*(dim-1)/2 + (ii-1)*T%ATM%dy
     enddo

     ! call itime(now)

     ! do i = 1,now(1)*3600+now(2)*60+now(3)
     !    call RandUniform(n1)  
     ! end do

     ! !Randomly offset the wind field
     ! call RandGaussian(T%ATM%XOFFSET)
     ! call RandGaussian(T%ATM%YOFFSET)

     T%ATM%XOFFSET = 0*T%ATM%XOFFSET*dim*T%ATM%dx/2.0D0
     T%ATM%YOFFSET = 0*T%ATM%YOFFSET*dim*T%ATM%dx/2.0D0

     ! write(*,*) T%ATM%XOFFSET
     ! PAUSE

     !%%%%%%%Import Initial UVW matrices%%%%%%%%% */
  
     write(number, '(i1)' )  T%ATM%tcoord(1)
     letter = trim('U')
     T%ATM%U0name = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
     letter = trim('V')
     T%ATM%V0name = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
     letter = trim('W')
     T%ATM%W0name = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
     write(number, '(i1)' )  T%ATM%tcoord(2)
     T%ATM%Wdtname = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
     letter = trim('U')
     T%ATM%Udtname = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
     letter = trim('V')
     T%ATM%Vdtname = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'

     call IMPORTWIND(T%ATM%U0,T%ATM%U0name,40);
     call IMPORTWIND(T%ATM%Udt,T%ATM%Udtname,40);
     call IMPORTWIND(T%ATM%V0,T%ATM%V0name,40);
     call IMPORTWIND(T%ATM%Vdt,T%ATM%Vdtname,40);
     call IMPORTWIND(T%ATM%W0,T%ATM%W0name,40);
     call IMPORTWIND(T%ATM%Wdt,T%ATM%Wdtname,40);
  
     !write(*,*) 'UVW Initialization Complete'

     write(*,*) 'Dryden Initialization '

     !%%%%%%%%%%%Initialize Variables%%%%%%%%%%%%% */

     !Allocate memory for WRF interpolation
     T%ATM%boundsT = 0;
     T%ATM%boundflagT = 1;

     !%%%%%%%%%%Unwrap Parameters%%%%%%%%%%%%%%% 

     !%%%%%%%%%%%Create xcoord and ycoord%%%%%%%
     do ii = 1,T%ATM%dimT
        T%ATM%xcoordT(ii) = -T%ATM%dxT * (T%ATM%dimT - 1) / 2 + ii * T%ATM%dxT;
        T%ATM%ycoordT(ii) = -T%ATM%dyT * (T%ATM%dimT - 1) / 2 + ii * T%ATM%dyT;
     end do
     !%%%%%%%Import Initial UVW matrices%%%%%%%%%

     T%ATM%Uturbname = trim(T%ATM%PATH)//trim('Uturb.txt')
     T%ATM%Vturbname = trim(T%ATM%PATH)//trim('Vturb.txt')
     T%ATM%Wturbname = trim(T%ATM%PATH)//trim('Wturb.txt')

     call IMPORTTURB(T%ATM%UTURB, T%ATM%UTurbname,500); !REVISIT HARDCODED DIMENSIONS, MAKE A GLOBAL OR SOMETHING -DK 8/10/2015 
     !This parameter (500) will never change. Doesn't necessarily need to be a global or anything fancy. - CM 8/16/2015
     call IMPORTTURB(T%ATM%VTURB, T%ATM%VTurbname,500);
     call IMPORTTURB(T%ATM%WTURB, T%ATM%WTurbname,500);
     
     write(*,*) 'Turbulence Initialization Complete'

  end if

  close(94) 
  write(*,*) 'ATMOSPHERE Load Complete'

  T%ATM%DQFLAG = 1
  
  RETURN
end if
 
RETURN
END SUBROUTINE ATMOSPHERE

!!!!!!!!!!!!!!!!!!!!!!!!!!!SUBROUTINE DRIVER !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

SUBROUTINE DRIVER(T,iflag)
 use TOSIMDATATYPES
 implicit none
 integer i,iflag,ifind,openflag,readflag
 real*8 m,readreal,ctheta,stheta,sphi,cphi,spsi,cpsi,ttheta,dspeed,psiprev,omegar
 real*8 tend,slope,accel,noise,freq,vATM_I(3,1),vATM_A(3,1),uaero,vaero,waero,V_A,sumomega
 real*8 xcg,ycg,zcg,phi,theta,psi,ub,vb,wb,pb,qb,rb,rCF_B(3,1),forcevec(4,1),thrust
 real*8 Gammavec(3,1),bquad,C_Ftether_I(3,1),C_Ftether_B(3,1),S_rCF_B(3,3),C_Mtether_B(3,1)
 real*8 xcgdot,ycgdot,zcgdot,phidot,thetadot,psidot,ubdot,vbdot,wbdot,c1,c2,c3,pbdot,qbdot,rbdot
 real*8 rReel_I(3,1),rCG_I(3,1),v_CG_I(3,1),S_wt_B(3,3),v_Reel_I(3,1),deti
 real*8 TVEC(4),TDOTVEC(4),TDBLDOTVEC(4),S,q_inf_S,q_inf
 real*8 sigmaF,omegaF,zetaF,C1F(4),C2F(4),C3F(4),idx,W2Tpwm(4,1),W0,j
 character*256 xgridname,ygridname,zgridname
 character*1 letter
 character*10 number
 type(TOSIMSTRUCTURE) T

!!!!!!!!!!!!!!!!!!!!!!!!!! COMPUTE FLAG iflag = 3 !!!!!!!!!!!!!!!!!!!!!!!!!
  
 if (iflag .eq. 2) then  
    
    !Extract everything from state vector
    xcg = T%DRIVER%STATE(1)
    ycg = T%DRIVER%STATE(2)
    zcg = T%DRIVER%STATE(3)
    phi = T%DRIVER%STATE(4) 
    theta = T%DRIVER%STATE(5)
    psi = T%DRIVER%STATE(6)
    ub = T%DRIVER%STATE(7)
    vb = T%DRIVER%STATE(8)
    wb = T%DRIVER%STATE(9)
    pb = T%DRIVER%STATE(10)
    qb = T%DRIVER%STATE(11) 
    rb = T%DRIVER%STATE(12)
    TVEC(1)    = T%DRIVER%STATE(13)
    TDOTVEC(1) = T%DRIVER%STATE(14)
    TVEC(2)    = T%DRIVER%STATE(15)
    TDOTVEC(2) = T%DRIVER%STATE(16)
    TVEC(3)    = T%DRIVER%STATE(17)
    TDOTVEC(3) = T%DRIVER%STATE(18)
    TVEC(4)    = T%DRIVER%STATE(19)
    TDOTVEC(4) = T%DRIVER%STATE(20)
    ! TVEC(1) = T%DRIVER%STATE(13)
    ! TVEC(2) = T%DRIVER%STATE(14)
    ! TVEC(3) = T%DRIVER%STATE(15)
    ! TVEC(4) = T%DRIVER%STATE(16)

    !Set some things for other models
    T%DRIVER%XCG = T%DRIVER%STATE(1) 
    T%DRIVER%YCG = T%DRIVER%STATE(2)
    T%DRIVER%ZCG = T%DRIVER%STATE(3)
    T%DRIVER%PSI = T%DRIVER%STATE(6) !psi
    T%DRIVER%SPEED = T%DRIVER%STATE(7) !ub

    !Distance from CG to tether attachment point
    rCF_B(1,1) = T%DRIVER%SLREEL
    rCF_B(2,1) = T%DRIVER%BLREEL 
    rCF_B(3,1) = T%DRIVER%WLREEL

    !- This is where we're going to hi-jack the model and compute the derivatives using Lisa Schibelius' quadcopter MODEL
    if (T%DRIVER%MODNO .eq. 3) then

       ! Unwrap State Vector - This is the same for the DRIVEDRIVER

       ! Aircraft to Inertial Transformation Matrix - Same for DRIVEDRIVER
       ctheta = cos(theta);
       stheta = sin(theta);
       ttheta = stheta / ctheta;
       cphi = cos(phi);
       sphi = sin(phi);
       spsi = sin(psi);
       cpsi = cos(psi);
       T%DRIVER%TIC(1,1) = ctheta * cpsi;
       T%DRIVER%TIC(2,1) = ctheta * spsi;
       T%DRIVER%TIC(3,1) = -stheta;
       T%DRIVER%TIC(1,2) = sphi * stheta * cpsi - cphi * spsi;
       T%DRIVER%TIC(2,2) = sphi * stheta * spsi + cphi * cpsi;
       T%DRIVER%TIC(3,2) = sphi * ctheta;
       T%DRIVER%TIC(1,3) = cphi * stheta * cpsi + sphi * spsi;
       T%DRIVER%TIC(2,3) = cphi * stheta * spsi - sphi * cpsi;
       T%DRIVER%TIC(3,3) = cphi * ctheta;

       ! Inertial to copter Transformation Matrix
       
       T%DRIVER%TCI = transpose(T%DRIVER%TIC)
  
       ! Gravity Forces and Moments - Same for DRIVER
  
       T%DRIVER%FXGRAV = 0.0; T%DRIVER%FYGRAV = 0.0; T%DRIVER%FZGRAV = 0.0;
       T%DRIVER%MXGRAV = 0.0; T%DRIVER%MYGRAV = 0.0; T%DRIVER%MZGRAV = 0.0;
       if (T%DRIVER%GRAVOFFON .eq. 1) then
          T%DRIVER%FXGRAV = T%DRIVER%TIC(3,1)*T%DRIVER%WEIGHT
          T%DRIVER%FYGRAV = T%DRIVER%TIC(3,2)*T%DRIVER%WEIGHT
          T%DRIVER%FZGRAV = T%DRIVER%TIC(3,3)*T%DRIVER%WEIGHT
       end if

       ! Aerodynamic Forces and Moments - Still Same for DRIVER
       
       T%DRIVER%FXAERO = 0.0; T%DRIVER%FYAERO = 0.0; T%DRIVER%FZAERO = 0.0;
       T%DRIVER%MXAERO = 0.0; T%DRIVER%MYAERO = 0.0; T%DRIVER%MZAERO = 0.0;

       T%DRIVER%FXAERO = T%DRIVER%C_T*T%DRIVER%MUTHROTTLE
       
       if (T%DRIVER%AEROOFFON .eq. 1) then
          vATM_I(1,1) = T%DRIVER%VXWIND
          vATM_I(2,1) = T%DRIVER%VYWIND
          vATM_I(3,1) = T%DRIVER%VZWIND
          vATM_A = matmul(T%DRIVER%TCI,vATM_I)

          !Add in atmospheric winds

          uaero = ub - vATM_A(1,1)
          vaero = vb - vATM_A(2,1)
          waero = wb - vATM_A(3,1)
          
          !Compute total velocity

          V_A = sqrt(uaero**2 + vaero**2 + waero**2)
          q_inf = 0.5*T%DRIVER%DEN*(V_A**2) !This assumes the reference area is 1
          S = 1.0
          q_inf_S = q_inf*S

          T%DRIVER%FXAERO = T%DRIVER%FXAERO - q_inf_S*T%DRIVER%DXD

       end if !Aerodynamic forces

       !At this point we should have F(XYZ)AERO and M(XYZ)AERO populated
       T%DRIVER%FXCONT = 0.0; T%DRIVER%FYCONT = 0.0; T%DRIVER%FZCONT = 0.0;          
       T%DRIVER%MXCONT = 0.0; T%DRIVER%MYCONT = 0.0; T%DRIVER%MZCONT = 0.0;

       !Don't forget to add Tether forces
       if ((T%THR%DYNOFFON .eq. 1) .and. (T%THR%ELASOFFON .eq. 1)) then
          if (isnan(T%DRIVER%FTETHERX) .or. isnan(T%DRIVER%FTETHERY)) then
             write(*,*) 'Sorry dude Nans in tether model detected - I suggest lowering your timestep'
             write(*,*) 'Current Time = ',T%SIM%TIME
             write(*,*) 'Tether Forces = ',T%DRIVER%FTETHERX,T%DRIVER%FTETHERY
             STOP
          else
             C_Ftether_I(1,1) = T%DRIVER%FTETHERX
             C_Ftether_I(2,1) = T%DRIVER%FTETHERY
             C_Ftether_I(3,1) = T%DRIVER%FTETHERZ
             C_Ftether_B = matmul(T%DRIVER%TCI,C_Ftether_I)
             T%DRIVER%FXCONT = C_Ftether_B(1,1)
             T%DRIVER%FYCONT = C_Ftether_B(2,1)
             T%DRIVER%FZCONT = C_Ftether_B(3,1)
                
             !Skew symmetric operator on cradle cg to tether connection point
    
             S_rCF_B(1,1) = 0.0
             S_rCF_B(1,2) = -rCF_B(3,1)
             S_rCF_B(1,3) = rCF_B(2,1)
             S_rCF_B(2,1) = rCF_B(3,1)
             S_rCF_B(2,2) = 0.0
             S_rCF_B(2,3) = -rCF_B(1,1)
             S_rCF_B(3,1) = -rCF_B(2,1)
             S_rCF_B(3,2) = rCF_B(1,1)
             S_rCF_B(3,3) = 0.0

             C_Mtether_B = matmul(S_rCF_B,C_Ftether_B)

             T%DRIVER%MXCONT = C_Mtether_B(1,1)
             T%DRIVER%MYCONT = C_Mtether_B(2,1)
             T%DRIVER%MZCONT = C_Mtether_B(3,1)
          end if
       end if

       ! Total Forces and Moments
       
       T%DRIVER%FXTOTAL = T%DRIVER%FXGRAV + T%DRIVER%FXAERO + T%DRIVER%FXCONT
       T%DRIVER%FYTOTAL = T%DRIVER%FYGRAV + T%DRIVER%FYAERO + T%DRIVER%FYCONT
       !write(*,*) 'Z Force = ',T%DRIVER%FZGRAV,T%DRIVER%FZAERO,T%DRIVER%FZCONT
       T%DRIVER%FZTOTAL = T%DRIVER%FZGRAV + T%DRIVER%FZAERO + T%DRIVER%FZCONT
       T%DRIVER%MXTOTAL = T%DRIVER%MXGRAV + T%DRIVER%MXAERO + T%DRIVER%MXCONT
       T%DRIVER%MYTOTAL = T%DRIVER%MYGRAV + T%DRIVER%MYAERO + T%DRIVER%MYCONT
       T%DRIVER%MZTOTAL = T%DRIVER%MZGRAV + T%DRIVER%MZAERO + T%DRIVER%MZCONT

       ! State Derivatives

       xcgdot = T%DRIVER%TIC(1,1)*ub + T%DRIVER%TIC(1,2)*vb + T%DRIVER%TIC(1,3)*wb
       ycgdot = T%DRIVER%TIC(2,1)*ub + T%DRIVER%TIC(2,2)*vb + T%DRIVER%TIC(2,3)*wb
       zcgdot = T%DRIVER%TIC(3,1)*ub + T%DRIVER%TIC(3,2)*vb + T%DRIVER%TIC(3,3)*wb  

       phidot = pb + sphi * ttheta * qb + cphi * ttheta * rb;
       thetadot = cphi * qb - sphi * rb;
       psidot = (sphi / ctheta) * qb + (cphi / ctheta) * rb;
       ubdot = T%DRIVER%FXTOTAL/T%DRIVER%MASS + rb*vb - qb*wb
       vbdot = T%DRIVER%FYTOTAL/T%DRIVER%MASS + pb*wb - rb*ub
       !write(*,*) 'Weight and Z =',T%DRIVER%FZTOTAL,T%DRIVER%MASS
       wbdot = T%DRIVER%FZTOTAL/T%DRIVER%MASS + qb*ub - pb*vb

       c1 = T%DRIVER%MXTOTAL - pb*(qb*T%DRIVER%IXZ-rb*T%DRIVER%IXY) - qb*(qb*T%DRIVER%IYZ-rb*T%DRIVER%IYY) - rb*(qb*T%DRIVER%IZZ-rb*T%DRIVER%IYZ)
       c2 = T%DRIVER%MYTOTAL - pb*(rb*T%DRIVER%IXX-pb*T%DRIVER%IXZ) - qb*(rb*T%DRIVER%IXY-pb*T%DRIVER%IYZ) - rb*(rb*T%DRIVER%IXZ-pb*T%DRIVER%IZZ)
       c3 = T%DRIVER%MZTOTAL - pb*(pb*T%DRIVER%IXY-qb*T%DRIVER%IXX) - qb*(pb*T%DRIVER%IYY-qb*T%DRIVER%IXY) - rb*(pb*T%DRIVER%IYZ-qb*T%DRIVER%IXZ)
       pbdot = T%DRIVER%IXXI*c1 + T%DRIVER%IXYI*c2 + T%DRIVER%IXZI*c3
       qbdot = T%DRIVER%IXYI*c1 + T%DRIVER%IYYI*c2 + T%DRIVER%IYZI*c3
       rbdot = T%DRIVER%IXZI*c1 + T%DRIVER%IYZI*c2 + T%DRIVER%IZZI*c3

       ! Wrap State Derivatives

       T%DRIVER%STATEDOT(1) = xcgdot
       T%DRIVER%STATEDOT(2) = ycgdot
       T%DRIVER%STATEDOT(2) = ycgdot
       T%DRIVER%STATEDOT(3) = zcgdot
       T%DRIVER%STATEDOT(4) = phidot
       T%DRIVER%STATEDOT(5) = thetadot
       T%DRIVER%STATEDOT(6) = psidot
       T%DRIVER%STATEDOT(7) = ubdot
       T%DRIVER%STATEDOT(8) = vbdot
       T%DRIVER%STATEDOT(9) = wbdot
       T%DRIVER%STATEDOT(10) = pbdot 
       T%DRIVER%STATEDOT(11) = qbdot 
       T%DRIVER%STATEDOT(12) = rbdot
       T%DRIVER%STATEDOT(13) = TDOTVEC(1)
       T%DRIVER%STATEDOT(14) = TDBLDOTVEC(1)
       T%DRIVER%STATEDOT(15) = TDOTVEC(2)
       T%DRIVER%STATEDOT(16) = TDBLDOTVEC(2)
       T%DRIVER%STATEDOT(17) = TDOTVEC(3)
       T%DRIVER%STATEDOT(18) = TDBLDOTVEC(3)
       T%DRIVER%STATEDOT(19) = TDOTVEC(4)
       T%DRIVER%STATEDOT(20) = TDBLDOTVEC(4)

       !write(*,*) 'Statedot = ',T%DRIVER%STATEDOT

       !!Save some stuff for Tether model
       T%DRIVER%XDOT = xcgdot
       T%DRIVER%YDOT = ycgdot
       T%DRIVER%ZDOT = zcgdot

       !Compute Reel Locations - r_reel = r_cg + TIB*r_body
       rCG_I(1,1) = xcg 
       rCG_I(2,1) = ycg
       rCG_I(3,1) = zcg ! + (1/3) !REVISIT
       rReel_I = rCG_I + matmul(T%DRIVER%TIC,rCF_B)
       T%DRIVER%XREEL = rReel_I(1,1)
       T%DRIVER%YREEL = rReel_I(2,1)
       T%DRIVER%ZREEL = rReel_I(3,1)
       !Compute Reel Dot - v_reel = v_cg + TIB*(omega x r_body)
       v_CG_I(1,1) = xcgdot
       v_CG_I(2,1) = ycgdot
       v_CG_I(3,1) = zcgdot
       S_wt_B(1,1) = 0.0
       S_wt_B(1,2) = -rb
       S_wt_B(1,3) = qb
       S_wt_B(2,1) = rb
       S_wt_B(2,2) = 0.0
       S_wt_B(2,3) = -pb
       S_wt_B(3,1) = -qb
       S_wt_B(3,2) = pb
       S_wt_B(3,3) = 0.0
       v_Reel_I = v_CG_I + matmul(T%DRIVER%TIC,matmul(S_wt_B,rCF_B))
       T%DRIVER%XREELDOT = v_Reel_I(1,1)
       T%DRIVER%YREELDOT = v_Reel_I(2,1)
       T%DRIVER%ZREELDOT = v_Reel_I(3,1)
    end if !End MODNO
    ! Integration Model 
    if (T%DRIVER%MODNO .eq. 0) then
       !!!Compute Driver speed 
       if (T%DRIVER%RESTARTSPEED .eq. -999) then
          !T%DRIVER%SPEED = T%DRIVER%FINALSPEED
          accel = 0
       else
          !Ramp in speed - assume constant decel/acceleration of 2 ft/s
          dspeed = 3
          tend = abs(T%DRIVER%RESTARTSPEED-T%DRIVER%FINALSPEED)/dspeed
          if (T%SIM%TIME .gt. tend) then
             !T%DRIVER%SPEED = T%DRIVER%FINALSPEED
             accel = 0
          else
             !T%DRIVER%SPEED = T%DRIVER%RESTARTSPEED + sign(1.0,T%DRIVER%FINALSPEED-T%DRIVER%RESTARTSPEED)*dspeed*(T%SIM%TIME)
             accel = sign(1.0,T%DRIVER%FINALSPEED-T%DRIVER%RESTARTSPEED)*dspeed
          end if
       end if

       !!Compute Derivatives
       T%DRIVER%XDOT = T%DRIVER%SPEED !!This assumes you are flying straight - REVISIT REVISIT
       T%DRIVER%YDOT = vb
       T%DRIVER%ZDOT = wb
       T%DRIVER%STATEDOT(1) = T%DRIVER%XDOT
       T%DRIVER%STATEDOT(2) = T%DRIVER%YDOT
       T%DRIVER%STATEDOT(3) = T%DRIVER%ZDOT
       T%DRIVER%STATEDOT(4:6) = 0 !Phi theta psi
       call RandUniform(noise)
       T%DRIVER%STATEDOT(7) = accel + (1.0D0-2*noise)*T%DRIVER%XDDOTNOISE !udot
       call RandUniform(noise)
       freq = 0
       if (T%DRIVER%YDDOTPERIOD .ne. 0) then
          freq = (2*qPI)/(T%DRIVER%YDDOTPERIOD)*cos((2*qPI)/(T%DRIVER%YDDOTPERIOD)*T%SIM%TIME)
       end if
       T%DRIVER%STATEDOT(8) = 0 + (1.0D0-2*noise)*T%DRIVER%XDDOTNOISE + T%DRIVER%YDDOTSCALE*freq !vdot
       call RandUniform(noise)
       T%DRIVER%STATEDOT(9) = (1.0D0-2*noise)*T%DRIVER%XDDOTNOISE !wdot
       T%DRIVER%STATEDOT(10:12) = 0 !p,q,rdot

       !Compute Reel Locations
       T%DRIVER%XREEL = T%DRIVER%XCG + cos(T%DRIVER%PSI)*(T%DRIVER%SLREEL-T%DRIVER%SLCG) - sin(T%DRIVER%PSI)*(T%DRIVER%BLREEL-T%DRIVER%BLCG) 
       T%DRIVER%YREEL = T%DRIVER%YCG + sin(T%DRIVER%PSI)*(T%DRIVER%SLREEL-T%DRIVER%SLCG) + cos(T%DRIVER%PSI)*(T%DRIVER%BLREEL-T%DRIVER%BLCG) 
       T%DRIVER%ZREEL = T%DRIVER%ZCG + T%DRIVER%WLREEL - T%DRIVER%WLCG 
       T%DRIVER%XREELDOT = T%DRIVER%XDOT
       T%DRIVER%YREELDOT = T%DRIVER%YDOT
       T%DRIVER%ZREELDOT = T%DRIVER%ZDOT
       
    end if

 ! Constant Model

    if (T%DRIVER%MODNO .eq. 1) then

     T%DRIVER%SPEED = T%DRIVER%FINALSPEED
        
     ! if (T%SIM%TIME .gt. 1000) then
     !    T%DRIVER%XCG = T%DRIVER%XCGINITIAL + T%DRIVER%SPEED*1000 + (T%DRIVER%SPEED+5)*(T%SIM%TIME-1000)
     !    speed = T%DRIVER%SPEED+5
     ! end if
     T%DRIVER%XDOT = T%DRIVER%SPEED*cos(T%DRIVER%PSI)
     T%DRIVER%YDOT = T%DRIVER%SPEED*sin(T%DRIVER%PSI) !!!DO NOT FORGET TO FIX THIS TOO

     !!Compute CG location -- This assumes that speed is constant

     T%DRIVER%XCG = T%DRIVER%XCGINITIAL + T%DRIVER%SPEED*cos(T%DRIVER%PSI)*T%SIM%TIME
     T%DRIVER%YCG = T%DRIVER%YCGINITIAL + T%DRIVER%SPEED*sin(T%DRIVER%PSI)*T%SIM%TIME
     T%DRIVER%ZCG = T%DRIVER%ZCGINITIAL

     !Compute CG speed

     T%DRIVER%ZDOT = 0.0
     T%DRIVER%XREEL = T%DRIVER%XCG + cos(T%DRIVER%PSI)*(T%DRIVER%SLREEL-T%DRIVER%SLCG) - sin(T%DRIVER%PSI)*(T%DRIVER%BLREEL-T%DRIVER%BLCG) 
     T%DRIVER%YREEL = T%DRIVER%YCG + sin(T%DRIVER%PSI)*(T%DRIVER%SLREEL-T%DRIVER%SLCG) + cos(T%DRIVER%PSI)*(T%DRIVER%BLREEL-T%DRIVER%BLCG) 
     T%DRIVER%ZREEL = T%DRIVER%ZCG + T%DRIVER%WLREEL - T%DRIVER%WLCG 
     T%DRIVER%XREELDOT = T%DRIVER%XDOT
     T%DRIVER%YREELDOT = T%DRIVER%YDOT
     T%DRIVER%ZREELDOT = T%DRIVER%ZDOT

     !Place everything in state vector

     T%DRIVER%STATE(1) = T%DRIVER%XCG
     T%DRIVER%STATE(2) = T%DRIVER%YCG
     T%DRIVER%STATE(3) = T%DRIVER%ZCG
     T%DRIVER%STATE(4) = 0 !phi 
     T%DRIVER%STATE(5) = 0 !theta
     T%DRIVER%STATE(6) = T%DRIVER%PSI !psi
     T%DRIVER%STATE(7) = T%DRIVER%SPEED
     T%DRIVER%STATE(8:12) = 0

   ! Helpful variables
     
     T%DRIVER%THETA = 0
     T%DRIVER%PHI = 0
     ctheta = cos(T%DRIVER%THETA)
     stheta = sin(T%DRIVER%THETA)
     cphi = cos(T%DRIVER%PHI)
     sphi = sin(T%DRIVER%PHI)
     cpsi = cos(T%DRIVER%PSI)
     spsi = sin(T%DRIVER%PSI)

     ! Reel Position and Velocity
     
     T%DRIVER%TIS(1,1) = ctheta*cpsi
     T%DRIVER%TIS(2,1) = ctheta*spsi
     T%DRIVER%TIS(3,1) = -stheta
     T%DRIVER%TIS(1,2) = sphi*stheta*cpsi - cphi*spsi
     T%DRIVER%TIS(2,2) = sphi*stheta*spsi + cphi*cpsi
     T%DRIVER%TIS(3,2) = sphi*ctheta
     T%DRIVER%TIS(1,3) = cphi*stheta*cpsi + sphi*spsi
     T%DRIVER%TIS(2,3) = cphi*stheta*spsi - sphi*cpsi
     T%DRIVER%TIS(3,3) = cphi*ctheta
  end if
  
  ! Table Look-Up Model 

  if (T%DRIVER%MODNO .eq. 2) then
    
   ifind = 0
  
   ! Position Pointer  

   if (T%SIM%TIME .le. T%DRIVER%TIMETAB(T%DRIVER%IP)) then 
    ifind = -1 
    do while ((ifind.ne.0) .and. (T%DRIVER%IP.gt.1))
     T%DRIVER%IP = T%DRIVER%IP - 1 
     if (T%DRIVER%TIMETAB(T%DRIVER%IP)   .le. T%SIM%TIME) then 
     if (T%DRIVER%TIMETAB(T%DRIVER%IP+1) .gt. T%SIM%TIME) then 
      ifind = 0
     end if
     end if 
    end do 
   end if
   if (T%SIM%TIME .gt. T%DRIVER%TIMETAB(T%DRIVER%IP+1)) then 
    ifind = 1
    do while ((ifind.ne.0) .and. (T%DRIVER%IP.lt.T%DRIVER%TABSIZE-1))
     T%DRIVER%IP = T%DRIVER%IP + 1 
     if (T%DRIVER%TIMETAB(T%DRIVER%IP)   .le. T%SIM%TIME) then 
     if (T%DRIVER%TIMETAB(T%DRIVER%IP+1) .gt. T%SIM%TIME) then 
      ifind = 0 
     end if 
     end if 
    end do 
   end if
   if (ifind .eq. 0) then
    m = (T%SIM%TIME-T%DRIVER%TIMETAB(T%DRIVER%IP))/(T%DRIVER%TIMETAB(T%DRIVER%IP+1)-T%DRIVER%TIMETAB(T%DRIVER%IP))
   else if (ifind .eq. -1) then
    m = 0.0
   else if (ifind .eq. 1) then
    m = 1.0
   end if
  
   ! Interpolate
    
   T%DRIVER%XCG   = T%DRIVER%XCGINITIAL + T%DRIVER%XCGTAB(T%DRIVER%IP)   + m*(T%DRIVER%XCGTAB(T%DRIVER%IP+1)-T%DRIVER%XCGTAB(T%DRIVER%IP))
   T%DRIVER%YCG   = T%DRIVER%YCGINITIAL + T%DRIVER%YCGTAB(T%DRIVER%IP)   + m*(T%DRIVER%YCGTAB(T%DRIVER%IP+1)-T%DRIVER%YCGTAB(T%DRIVER%IP))
   T%DRIVER%ZCG   = T%DRIVER%ZCGINITIAL + T%DRIVER%ZCGTAB(T%DRIVER%IP)   + m*(T%DRIVER%ZCGTAB(T%DRIVER%IP+1)-T%DRIVER%ZCGTAB(T%DRIVER%IP))
   T%DRIVER%PHI   = T%DRIVER%PHITAB(T%DRIVER%IP)   + m*(T%DRIVER%PHITAB(T%DRIVER%IP+1)-T%DRIVER%PHITAB(T%DRIVER%IP))
   T%DRIVER%THETA = T%DRIVER%THETATAB(T%DRIVER%IP) + m*(T%DRIVER%THETATAB(T%DRIVER%IP+1)-T%DRIVER%THETATAB(T%DRIVER%IP))
   T%DRIVER%PSI   = T%DRIVER%PSITAB(T%DRIVER%IP)   + m*(T%DRIVER%PSITAB(T%DRIVER%IP+1)-T%DRIVER%PSITAB(T%DRIVER%IP))
   T%DRIVER%UB    = T%DRIVER%UBTAB(T%DRIVER%IP)    + m*(T%DRIVER%UBTAB(T%DRIVER%IP+1)-T%DRIVER%UBTAB(T%DRIVER%IP))
   T%DRIVER%VB    = T%DRIVER%VBTAB(T%DRIVER%IP)    + m*(T%DRIVER%VBTAB(T%DRIVER%IP+1)-T%DRIVER%VBTAB(T%DRIVER%IP))
   T%DRIVER%WB    = T%DRIVER%WBTAB(T%DRIVER%IP)    + m*(T%DRIVER%WBTAB(T%DRIVER%IP+1)-T%DRIVER%WBTAB(T%DRIVER%IP))
   T%DRIVER%PB    = T%DRIVER%PBTAB(T%DRIVER%IP)    + m*(T%DRIVER%PBTAB(T%DRIVER%IP+1)-T%DRIVER%PBTAB(T%DRIVER%IP))
   T%DRIVER%QB    = T%DRIVER%QBTAB(T%DRIVER%IP)    + m*(T%DRIVER%QBTAB(T%DRIVER%IP+1)-T%DRIVER%QBTAB(T%DRIVER%IP))
   T%DRIVER%RB    = T%DRIVER%RBTAB(T%DRIVER%IP)    + m*(T%DRIVER%RBTAB(T%DRIVER%IP+1)-T%DRIVER%RBTAB(T%DRIVER%IP))

   !Place state in statevector

   T%DRIVER%STATE(1) = T%DRIVER%XCG
   T%DRIVER%STATE(2) = T%DRIVER%YCG
   T%DRIVER%STATE(3) = T%DRIVER%ZCG
   T%DRIVER%STATE(4) = T%DRIVER%PHI
   T%DRIVER%STATE(5) = T%DRIVER%THETA
   T%DRIVER%STATE(6) = T%DRIVER%PSI
   T%DRIVER%STATE(7) = T%DRIVER%UB
   T%DRIVER%STATE(8) = T%DRIVER%VB
   T%DRIVER%STATE(9) = T%DRIVER%WB
   T%DRIVER%STATE(10) = T%DRIVER%PB
   T%DRIVER%STATE(11) = T%DRIVER%QB
   T%DRIVER%STATE(12) = T%DRIVER%RB

   ! Helpful variables

   ctheta = cos(T%DRIVER%THETA)
   stheta = sin(T%DRIVER%THETA)
   cphi = cos(T%DRIVER%PHI)
   sphi = sin(T%DRIVER%PHI)
   cpsi = cos(T%DRIVER%PSI)
   spsi = sin(T%DRIVER%PSI)

   ! Reel Position and Velocity
    
   T%DRIVER%TIS(1,1) = ctheta*cpsi
   T%DRIVER%TIS(2,1) = ctheta*spsi
   T%DRIVER%TIS(3,1) = -stheta
   T%DRIVER%TIS(1,2) = sphi*stheta*cpsi - cphi*spsi
   T%DRIVER%TIS(2,2) = sphi*stheta*spsi + cphi*cpsi
   T%DRIVER%TIS(3,2) = sphi*ctheta
   T%DRIVER%TIS(1,3) = cphi*stheta*cpsi + sphi*spsi
   T%DRIVER%TIS(2,3) = cphi*stheta*spsi - sphi*cpsi
   T%DRIVER%TIS(3,3) = cphi*ctheta

   !!Compute Driver speed in inertial coordinates

   T%DRIVER%XDOT = T%DRIVER%TIS(1,1)*T%DRIVER%UB + T%DRIVER%TIS(1,2)*T%DRIVER%VB + T%DRIVER%TIS(1,3)*T%DRIVER%WB  
   T%DRIVER%YDOT = T%DRIVER%TIS(2,1)*T%DRIVER%UB + T%DRIVER%TIS(2,2)*T%DRIVER%VB + T%DRIVER%TIS(2,3)*T%DRIVER%WB  
   T%DRIVER%ZDOT = T%DRIVER%TIS(3,1)*T%DRIVER%UB + T%DRIVER%TIS(3,2)*T%DRIVER%VB + T%DRIVER%TIS(3,3)*T%DRIVER%WB  

   !!!Compute Reel velocity in Driver frame

   T%DRIVER%UREEL = T%DRIVER%UB + T%DRIVER%QB*(T%DRIVER%WLREEL-T%DRIVER%WLCG) - T%DRIVER%RB*(T%DRIVER%BLREEL-T%DRIVER%BLCG) 
   T%DRIVER%VREEL = T%DRIVER%VB + T%DRIVER%RB*(T%DRIVER%SLREEL-T%DRIVER%SLCG) - T%DRIVER%PB*(T%DRIVER%WLREEL-T%DRIVER%WLCG)
   T%DRIVER%WREEL = T%DRIVER%WB + T%DRIVER%PB*(T%DRIVER%BLREEL-T%DRIVER%BLCG) - T%DRIVER%QB*(T%DRIVER%SLREEL-T%DRIVER%SLCG)

   T%DRIVER%XREEL = T%DRIVER%XCG + T%DRIVER%TIS(1,1)*(T%DRIVER%SLREEL-T%DRIVER%SLCG) + T%DRIVER%TIS(1,2)*(T%DRIVER%BLREEL-T%DRIVER%BLCG) + T%DRIVER%TIS(1,3)*(T%DRIVER%WLREEL-T%DRIVER%WLCG)  
   T%DRIVER%YREEL = T%DRIVER%YCG + T%DRIVER%TIS(2,1)*(T%DRIVER%SLREEL-T%DRIVER%SLCG) + T%DRIVER%TIS(2,2)*(T%DRIVER%BLREEL-T%DRIVER%BLCG) + T%DRIVER%TIS(2,3)*(T%DRIVER%WLREEL-T%DRIVER%WLCG)  
   T%DRIVER%ZREEL = T%DRIVER%ZCG + T%DRIVER%TIS(3,1)*(T%DRIVER%SLREEL-T%DRIVER%SLCG) + T%DRIVER%TIS(3,2)*(T%DRIVER%BLREEL-T%DRIVER%BLCG) + T%DRIVER%TIS(3,3)*(T%DRIVER%WLREEL-T%DRIVER%WLCG)  

   T%DRIVER%XREELDOT = T%DRIVER%TIS(1,1)*T%DRIVER%UREEL + T%DRIVER%TIS(1,2)*T%DRIVER%VREEL + T%DRIVER%TIS(1,3)*T%DRIVER%WREEL  
   T%DRIVER%YREELDOT = T%DRIVER%TIS(2,1)*T%DRIVER%UREEL + T%DRIVER%TIS(2,2)*T%DRIVER%VREEL + T%DRIVER%TIS(2,3)*T%DRIVER%WREEL  
   T%DRIVER%ZREELDOT = T%DRIVER%TIS(3,1)*T%DRIVER%UREEL + T%DRIVER%TIS(3,2)*T%DRIVER%VREEL + T%DRIVER%TIS(3,3)*T%DRIVER%WREEL  
    
  end if !MODNO = 2

  ! This is pretty sloppy. Nghia wants this to run only once per rk4 function call
  ! should probably just add these as states but this will do for now
  ! What we will do is just divide everything out by 4 so when this runs 4 times we should
  ! be good.
  ! Using trapezoidal rule(ish) to compute integral error term
  ! REVISIT Replaced xintegral with u velocity term!!!!
  ! The 1/4 is used because this is called once every rk4 call
  ! and since we're using trap(ish) we need to divide by 4. Untested CJM - 2/13/2018
  T%DRIVER%XINTEGRAL     = T%DRIVER%XINTEGRAL     + -(1.0/4.0)*((T%DRIVER%XCOMMAND     - T%DRIVER%STATE(1))/2)*T%SIM%DELTATIME
  T%DRIVER%YINTEGRAL     = T%DRIVER%YINTEGRAL     + (1.0/4.0)*((T%DRIVER%YCOMMAND     - T%DRIVER%STATE(2))/2)*T%SIM%DELTATIME
  T%DRIVER%ZINTEGRAL     = T%DRIVER%ZINTEGRAL     + -(1.0/4.0)*((T%DRIVER%ZCOMMAND     - T%DRIVER%STATE(3))/2)*T%SIM%DELTATIME
  T%DRIVER%PHIINTEGRAL   = T%DRIVER%PHIINTEGRAL   +    (1.0/4.0)*((T%DRIVER%PHICOMMAND   - T%DRIVER%STATE(4))/2)*T%SIM%DELTATIME
  T%DRIVER%THETAINTEGRAL = T%DRIVER%THETAINTEGRAL +    (1.0/4.0)*((T%DRIVER%THETACOMMAND - T%DRIVER%STATE(5))/2)*T%SIM%DELTATIME
  T%DRIVER%PSIINTEGRAL   = T%DRIVER%PSIINTEGRAL   +    (1.0/4.0)*((T%DRIVER%PHICOMMAND   - T%DRIVER%STATE(6))/2)*T%SIM%DELTATIME
  T%DRIVER%UINTEGRAL     = T%DRIVER%UINTEGRAL     + -(1.0/4.0)*((T%DRIVER%UCOMMAND     - T%DRIVER%STATE(7))/2)*T%SIM%DELTATIME
    
  RETURN
  
end if
  
!!!!!!!!!!!!!!!!!!!!!!!!!!!!! LOAD DATA iflag = 1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 
 if (iflag .eq. 1) then
   
  open(unit=94,file=T%DRIVER%INPUTFILE,status='old',iostat=openflag)
  if (openflag .ne. 0) then
     write(*,*) T%DRIVER%INPUTFILE
   write(*,*) 'Error Opening Driver Input File => ',T%DRIVER%INPUTFILE,' <= ';PAUSE; STOP
  end if
  rewind(94)

  read(unit=94,fmt=*,iostat=readflag) readreal; T%DRIVER%OFFON = readreal
  read(unit=94,fmt=*,iostat=readflag) readreal; T%DRIVER%MODNO = readreal

  if (T%DRIVER%MODNO .eq. 3) then
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%GRAVOFFON
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%AEROOFFON
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%WEIGHT
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%GRAVITY
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%SLCG
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%BLCG
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%WLCG
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%SLREEL
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%BLREEL
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%WLREEL
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%IXX
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%IYY
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%IZZ
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%IXY
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%IXZ
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%IYZ
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%TURNRADIUS
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%DXD
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%C_T
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%MS_MIN
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%MS_MAX
     read(unit=94,fmt=*,iostat=readflag) readreal; T%DRIVER%CONTROLOFFON = int(readreal)
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%KPXDRIVE
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%KIXDRIVE
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%KDXDRIVE
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%XINTEGRAL 
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%THETAINTEGRAL
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%PSIINTEGRAL
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%UCOMMAND !!Changed to UCOMMAND for forward flight 
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%YCOMMAND
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%ZCOMMAND

     T%DRIVER%WAYPOINT = 1 !This is always defaulted to 1

     !!!DO SOME CALCULATIONS ON driver
     T%DRIVER%MASS = T%DRIVER%WEIGHT/T%DRIVER%GRAVITY 
     deti = + T%DRIVER%IXX*(T%DRIVER%IYY*T%DRIVER%IZZ-T%DRIVER%IYZ*T%DRIVER%IYZ) - T%DRIVER%IXY*(T%DRIVER%IXY*T%DRIVER%IZZ-T%DRIVER%IYZ*T%DRIVER%IXZ) + T%DRIVER%IXZ*(T%DRIVER%IXY*T%DRIVER%IYZ-T%DRIVER%IYY*T%DRIVER%IXZ)
     T%DRIVER%IXXI = (T%DRIVER%IYY*T%DRIVER%IZZ-T%DRIVER%IYZ*T%DRIVER%IYZ)/deti
     T%DRIVER%IXYI = (T%DRIVER%IYZ*T%DRIVER%IXZ-T%DRIVER%IXY*T%DRIVER%IZZ)/deti
     T%DRIVER%IXZI = (T%DRIVER%IXY*T%DRIVER%IYZ-T%DRIVER%IYY*T%DRIVER%IXZ)/deti
     T%DRIVER%IYYI = (T%DRIVER%IXX*T%DRIVER%IZZ-T%DRIVER%IXZ*T%DRIVER%IXZ)/deti
     T%DRIVER%IYZI = (T%DRIVER%IXY*T%DRIVER%IXZ-T%DRIVER%IXX*T%DRIVER%IYZ)/deti
     T%DRIVER%IZZI = (T%DRIVER%IXX*T%DRIVER%IYY-T%DRIVER%IXY*T%DRIVER%IXY)/deti
  else     
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%SLREEL
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%BLREEL
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%WLREEL
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%AIRWAKE
     !Read location of airwake data
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%AIRWAKEPATH
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%SLAIRWAKE
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%BLAIRWAKE
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%WLAIRWAKE

     if (T%DRIVER%AIRWAKE .eq. 1) then

        !Set TCOORD
        do i = 1,NTIMES
           T%DRIVER%TCOORD(i) = 0 + (i-1)*1.0D0
        end do

        !%%%%%%%Import Initial UVW matrices%%%%%%%%% */

        write(number, '(i1)' )  1
        letter = trim('U')
        T%DRIVER%U0name = trim(T%DRIVER%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        letter = trim('V')
        T%DRIVER%V0name = trim(T%DRIVER%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        letter = trim('W')
        T%DRIVER%W0name = trim(T%DRIVER%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        write(number, '(i1)' )  2
        letter = trim('U')
        T%DRIVER%Udtname = trim(T%DRIVER%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        letter = trim('V')
        T%DRIVER%Vdtname = trim(T%DRIVER%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        letter = trim('W')
        T%DRIVER%Wdtname = trim(T%DRIVER%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'

        call IMPORTWAKE(T%DRIVER%UDRIVER,T%DRIVER%U0name);
        call IMPORTWAKE(T%DRIVER%UDRIVERDT,T%DRIVER%Udtname);
        call IMPORTWAKE(T%DRIVER%VDRIVER,T%DRIVER%V0name);
        call IMPORTWAKE(T%DRIVER%VDRIVERDT,T%DRIVER%Vdtname);
        call IMPORTWAKE(T%DRIVER%WDRIVER,T%DRIVER%W0name);
        call IMPORTWAKE(T%DRIVER%WDRIVERDT,T%DRIVER%Wdtname);

        !!Import X,Y,Z Grid data
        xgridname = trim(T%DRIVER%AIRWAKEPATH)//'X_Grid.txt'
        ygridname = trim(T%DRIVER%AIRWAKEPATH)//'Y_Grid.txt'
        zgridname = trim(T%DRIVER%AIRWAKEPATH)//'Z_Grid.txt'

        call IMPORTWAKE(T%DRIVER%XDRIVER,xgridname);
        call IMPORTWAKE(T%DRIVER%YDRIVER,ygridname);
        call IMPORTWAKE(T%DRIVER%ZDRIVER,zgridname);

        do i = 1,IMAX
           T%DRIVER%XCOORD(i) = T%DRIVER%XDRIVER(i,1,1);
        end do

        write(*,*) 'Wake Data Load Complete'
     end if

  end if
  

  if ((T%DRIVER%MODNO .eq. 1) .or. (T%DRIVER%MODNO .eq. 0)) then
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%XCGINITIAL
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%YCGINITIAL
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%ZCGINITIAL
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%FINALSPEED
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%PSI
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%XDDOTNOISE
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%YDDOTSCALE
     read(unit=94,fmt=*,iostat=readflag) T%DRIVER%YDDOTPERIOD
  end if
  if (T%DRIVER%MODNO .eq. 2) then
   read(unit=94,fmt=*,iostat=readflag) readreal; T%DRIVER%TABSIZE = int(readreal)
   do i=1,T%DRIVER%TABSIZE  
    read(unit=94,fmt=*,iostat=readflag) T%DRIVER%TIMETAB(i),T%DRIVER%XCGTAB(i),T%DRIVER%YCGTAB(i),T%DRIVER%ZCGTAB(i),T%DRIVER%PHITAB(i),T%DRIVER%THETATAB(i),T%DRIVER%PSITAB(i), & 
                                        T%DRIVER%UBTAB(i),T%DRIVER%VBTAB(i),T%DRIVER%WBTAB(i),T%DRIVER%PBTAB(i),T%DRIVER%QBTAB(i),T%DRIVER%RBTAB(i)
   end do
  end if

  close(94) 
  write(*,*) 'DRIVER Load Complete'

  T%DRIVER%DQFLAG = 1
  
  RETURN
   
 end if
   
 RETURN
END SUBROUTINE DRIVER

!!Import routine to load data files placed in text files
SUBROUTINE IMPORTWAKE(mat,filename)
  use TOSIMDATATYPES
  implicit none
  integer uvw,time
  integer ii,jj,kk,nii,njj,ierr;
  real*8 mat(IMAX,JMAX,KMAX),tempmat(77) !! mat is 55x77x61
  character*256 filename

  write(*,*) 'Importing: ',filename
  
  open(unit=78,file=filename,status='old',iostat=ierr)
  if (ierr .ne. 0) then 
     write(*,*) 'Driver Airwake File defined incorrectly'
     write(*,*) filename
     PAUSE;
     STOP;
  endif
  do kk=1,KMAX
     do ii=1,IMAX
        read(78,*) tempmat
        !write(*,*) tempmat
        do jj=1,JMAX
           mat(ii,jj,kk) = tempmat(jj)
        enddo
     enddo
  enddo
  close(78)

END SUBROUTINE IMPORTWAKE

SUBROUTINE AIRWAKE(T,XI,YI,ZI)
  use TOSIMDATATYPES
  implicit none
  integer stepX,stepY,stepZ,stepT,extrapX,extrapY,extrapZ,extrapT,cord2(2)
  integer markX,markY,markZ,markT,gust,body,i,j,k
  integer tinterp,x1,x2,y1,y2,z1,z2,tt,ii,coord1(4),coord2(4),cord1(2)
  real*8 uvw(3,2),xpts2(2),ypts2(2),zpts2(2),zpts1,xpts1,ypts1,rx;
  real*8 u8(8),v8(8),w8(8),u4(4),v4(4),w4(4),uslope,vslope,wslope;
  real*8 u2(2),v2(2),w2(2),u,v,w,tpts(2),Lu,Lv,Lw,sigw,sigu,sigv,tstar,tshift;
  real*8 ugo,vgo,wgo,vatm(3),xstar,ystar,zstar,xtemp,vtemp,counter,xwidth,ywidth
  real*8 rI_I(3,1),rS_I(3,1),rAIRWAKE_S(3,1),rAwP_S(3,1),XI,YI,ZI
  character*1 letter
  character*10 number
  type(TOSIMSTRUCTURE) T

  !   /*   %%This function will take in x,y,z(ft),t(sec) and location and  */
  !   /*   %return u,v,w(m/s). This uses a fast quad-linear interpolation */
  ! using matrices XDRIVER,YDRIVER,ZDRIVER to interpolate against
  !   /*   %so many globals must be defined. location is a string that */
  !   /*   %contains the location of the data to be interpolated. */
  
  T%DRIVER%VWAKE(1) = 0
  T%DRIVER%VWAKE(2) = 0
  T%DRIVER%VWAKE(3) = 0

  rI_I(1,1) = XI
  rI_I(2,1) = YI
  rI_I(3,1) = ZI
  
  rS_I(1,1) = T%DRIVER%XCG
  rS_I(2,1) = T%DRIVER%YCG
  rS_I(3,1) = T%DRIVER%ZCG

  rAIRWAKE_S(1,1) = T%DRIVER%SLAIRWAKE
  rAIRWAKE_S(2,1) = T%DRIVER%BLAIRWAKE
  rAIRWAKE_S(3,1) = T%DRIVER%WLAIRWAKE

  rAwP_S = matmul(transpose(T%DRIVER%TIS),rI_I-rS_I)-rAIRWAKE_S

  xstar = -rAwP_S(1,1)
  ystar = rAwP_S(2,1)
  zstar = -rAwP_S(3,1)

  !!Loop tstar
  tstar = T%SIM%TIME
  tshift = 0
  if (tstar .gt. T%DRIVER%TCOORD(NTIMES)) then
     tshift = -floor(abs(tstar-T%DRIVER%TCOORD(1))/T%DRIVER%TCOORD(NTIMES))
  end if
  tstar = tstar + tshift*T%DRIVER%TCOORD(NTIMES)
  ! write(*,*) 'Tstar = ',tstar, 'tshift = ',tshift

  stepX = 1;stepY = 1;stepZ = 1;stepT = 1;
  extrapX = 0;extrapY = 0;extrapZ = 0;extrapT = 0;
  if (zstar .lt. 0) then
     zstar = -zstar
  endif
  tinterp = 2;

  uvw(1,1)=0;uvw(2,1)=0;uvw(3,1)=0;
  uvw(1,2)=0;uvw(2,2)=0;uvw(3,2)=0;

  markX = T%DRIVER%markX
  markY = T%DRIVER%markY
  markZ = T%DRIVER%markZ
  markT = T%DRIVER%markT

  !%%Check X
  if (markX .eq. IMAX) then
     markX = markX - 1;
  end if
  if ((xstar .ge. T%DRIVER%XCOORD(markX)) .and. (xstar .le. T%DRIVER%XCOORD(markX+1))) then
     !%%You're in between the markers so keep going
  else
     if (xstar .gt. T%DRIVER%XCOORD(IMAX)) then
        markX = IMAX;
        stepX = -1;
        extrapX = 1;
        ! write(*,*) 'Out of bounds on x'
     elseif (xstar .lt. T%DRIVER%XCOORD(1)) then
        markX = 1;
        stepX = 1;
        extrapX = 1;
     else
        call FINDGE(T%DRIVER%XCOORD,IMAX,xstar,markX)
        if (markX .eq. IMAX) then
           markX = markX - 1;
        else if (markX .le. 0) then
           markX = 1;
        end if
     end if
  end if

  ! write(*,*) 'markX = ',markX

  !%%%Now that you have markX you can grab the y and z planes
  ! write(*,*) 'Ycoord'
  do j = 1,JMAX
     T%DRIVER%YCOORD(j) = T%DRIVER%YDRIVER(markX,j,1); 
     ! write(*,*) T%DRIVER%YCOORD(j)
  end do
  ! write(*,*) '------------'
  do k = 1,KMAX
     T%DRIVER%ZCOORD(k) = T%DRIVER%ZDRIVER(markX,1,k);    
     ! write(*,*) T%DRIVER%ZCOORD(k)
  end do

  !%%Check Y
  if (markY .eq. JMAX) then
     markY = markY - 1;
  end if
  if ((ystar .ge. T%DRIVER%YCOORD(markY)) .and. (ystar .le. T%DRIVER%YCOORD(markY+1))) then
     !%%You're in between the markers so keep going
  else
     if (ystar .gt. T%DRIVER%YCOORD(JMAX)) then
        markY = JMAX;
        stepY = -1;
        extrapY = 1;
     elseif (ystar .lt. T%DRIVER%YCOORD(1)) then
        markY = 1;
        stepY = 1;
        extrapY = 1;
     else
        call FINDGE(T%DRIVER%YCOORD,JMAX,ystar,markY)
        if (markY .eq. JMAX) then
           markY = markY - 1;
        else if (markY .le. 0) then
           markY = 1;
        end if
     end if
  end if

  ! write(*,*) 'markY = ',markY

  !%%Check Z
  if (markZ .eq. KMAX) then
     markZ = markZ - 1;
  end if

  if ((zstar .ge. T%DRIVER%ZCOORD(markZ)) .and. (zstar .le. T%DRIVER%ZCOORD(markZ+1))) then
     !%%You're in between the markers so keep going
  else
     !%Find markZ
     if (zstar .gt. T%DRIVER%ZCOORD(KMAX)) then
        !%use endpt
        markZ = KMAX;
        stepZ = -1;
        extrapZ = 1;
        T%DRIVER%VWAKE(1) = 0
        T%DRIVER%VWAKE(2) = 0
        T%DRIVER%VWAKE(3) = 0
        RETURN
     else if (zstar .lt. T%DRIVER%ZCOORD(1)) then
        markZ = 1;
        stepZ = 1;
        extrapZ = 1;
     else
        call FINDGE(T%DRIVER%ZCOORD,KMAX,zstar,markZ)
        if (markZ .eq. KMAX) then
           markZ = markZ - 1;
        else if (markZ .eq. 0) then
           markZ = 1;
        end if
     end if
  end if

  ! write(*,*) 'markZ = ',markZ

  !%%Check T
  if (markT .eq. NTIMES) then
     markT = markT - 1;
  end if
  !write(*,*) tstar,markT,T%DRIVER%TCOORD(markT)
  if ((tstar .ge. T%DRIVER%TCOORD(markT)) .and. (tstar .le. T%DRIVER%TCOORD(markT+1))) then
     !%%You're in between the markers so keep going
  else
     !%Find markT
     if (tstar .gt. T%DRIVER%TCOORD(NTIMES)) then
        !%use endpt
        markT = NTIMES;
        extrapT = 1;
     else if (tstar .lt. T%DRIVER%TCOORD(1)) then
        !%use start pt
        markT = 1;
        extrapT = 1;
     else
        call FINDGE(T%DRIVER%TCOORD,NTIMES,tstar,markT)
        if (markT .eq. NTIMES) then
           markT = markT - 1;
        else if (markT .eq. 0) then
           markT = 1;
        end if
     end if

     !%%Import U,V,W maps since markT changed
     if (markT .lt. 10) then
        write(number, '(i1)' )  markT
     else
        if (markT .le. 99) then
           write(number, '(i2)' )  markT
        else
           write(number, '(i3)' )  markT
        endif
     endif
     letter = trim('U')
     T%DRIVER%U0name = trim(T%DRIVER%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
     letter = trim('V')
     T%DRIVER%V0name = trim(T%DRIVER%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
     letter = trim('W')
     T%DRIVER%W0name = trim(T%DRIVER%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
     !%%only import at markT
     call IMPORTWAKE(T%DRIVER%UDRIVER,T%DRIVER%U0name);
     call IMPORTWAKE(T%DRIVER%VDRIVER,T%DRIVER%V0name);
     call IMPORTWAKE(T%DRIVER%WDRIVER,T%DRIVER%W0name);
     if (extrapT .eq. 1) then
        tinterp = 1;
     else
        !%%import markT + 1
        if (markT+1 .lt. 10) then
           write(number, '(i1)' )  markT+1
        else if (markT+1 .le. 99) then
           write(number, '(i2)' )  markT+1
        else
           write(number, '(i3)' )  markT+1
        endif
        letter = trim('U')
        T%DRIVER%Udtname = trim(T%DRIVER%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        letter = trim('V')
        T%DRIVER%Vdtname = trim(T%DRIVER%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        letter = trim('W')
        T%DRIVER%Wdtname = trim(T%DRIVER%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        !%%only import at markT
        call IMPORTWAKE(T%DRIVER%UDRIVERDT,T%DRIVER%Udtname);
        call IMPORTWAKE(T%DRIVER%VDRIVERDT,T%DRIVER%Vdtname);
        call IMPORTWAKE(T%DRIVER%WDRIVERDT,T%DRIVER%Wdtname);
     end if !(extrapT eq. 1 ) 
  end if

  ! write(*,*) 'markT = ',markT
  ! write(*,*) 'tinterp = ',tinterp

  !%%Interpolation Scheme
  do tt = 1,tinterp 
     !%Interpolate Spatially

     !%%To start we have 8 discrete point (8 corners of a cube)
     xpts2(1) = T%DRIVER%XCOORD(markX)
     xpts2(2) = T%DRIVER%XCOORD(markX+stepX);
     ypts2(1) = T%DRIVER%YCOORD(markY)
     ypts2(2) = T%DRIVER%YCOORD(markY+stepY);
     zpts2(1) = T%DRIVER%ZCOORD(markZ)
     zpts2(2) = T%DRIVER%ZCOORD(markZ+stepZ);
     x1 = markX;x2 = markX+stepX;
     y1 = markY;y2 = (markY+stepY);
     z1 = markZ;z2 = markZ+stepZ;
     ! write(*,*) 'x1,x2,y1,y2,z1,z2 = ',x1,x2,y1,y2,z1,z2
     if (tt .eq. 1) then
        !%%Use UDRIVER,VDRIVER,WDRIVER
        u8(1) = T%DRIVER%UDRIVER(x1,y1,z1);
        u8(2) = T%DRIVER%UDRIVER(x2,y1,z1);
        u8(3) = T%DRIVER%UDRIVER(x2,y2,z1);
        u8(4) = T%DRIVER%UDRIVER(x1,y2,z1);
        u8(5) = T%DRIVER%UDRIVER(x1,y1,z2);
        u8(6) = T%DRIVER%UDRIVER(x2,y1,z2);
        u8(7) = T%DRIVER%UDRIVER(x2,y2,z2);
        u8(8) = T%DRIVER%UDRIVER(x1,y2,z2);
        v8(1) = T%DRIVER%VDRIVER(x1,y1,z1);
        v8(2) = T%DRIVER%VDRIVER(x2,y1,z1);
        v8(3) = T%DRIVER%VDRIVER(x2,y2,z1);
        v8(4) = T%DRIVER%VDRIVER(x1,y2,z1);
        v8(5) = T%DRIVER%VDRIVER(x1,y1,z2);
        v8(6) = T%DRIVER%VDRIVER(x2,y1,z2);
        v8(7) = T%DRIVER%VDRIVER(x2,y2,z2);
        v8(8) = T%DRIVER%VDRIVER(x1,y2,z2);
        w8(1) = T%DRIVER%WDRIVER(x1,y1,z1);
        w8(2) = T%DRIVER%WDRIVER(x2,y1,z1);
        w8(3) = T%DRIVER%WDRIVER(x2,y2,z1);
        w8(4) = T%DRIVER%WDRIVER(x1,y2,z1);
        w8(5) = T%DRIVER%WDRIVER(x1,y1,z2);
        w8(6) = T%DRIVER%WDRIVER(x2,y1,z2);
        w8(7) = T%DRIVER%WDRIVER(x2,y2,z2);
        w8(8) = T%DRIVER%WDRIVER(x1,y2,z2);
        ! do i = 1,8
        !    write(*,*) 'u8 = ',u8(i)
        ! end do
     else
        !%%Use Udt,Vdt,Wdt
        u8(1) = T%DRIVER%UDRIVERDT(x1,y1,z1);
        u8(2) = T%DRIVER%UDRIVERDT(x2,y1,z1);
        u8(3) = T%DRIVER%UDRIVERDT(x2,y2,z1);
        u8(4) = T%DRIVER%UDRIVERDT(x1,y2,z1);
        u8(5) = T%DRIVER%UDRIVERDT(x1,y1,z2);
        u8(6) = T%DRIVER%UDRIVERDT(x2,y1,z2);
        u8(7) = T%DRIVER%UDRIVERDT(x2,y2,z2);
        u8(8) = T%DRIVER%UDRIVERDT(x1,y2,z2);
        v8(1) = T%DRIVER%VDRIVERDT(x1,y1,z1);
        v8(2) = T%DRIVER%VDRIVERDT(x2,y1,z1);
        v8(3) = T%DRIVER%VDRIVERDT(x2,y2,z1);
        v8(4) = T%DRIVER%VDRIVERDT(x1,y2,z1);
        v8(5) = T%DRIVER%VDRIVERDT(x1,y1,z2);
        v8(6) = T%DRIVER%VDRIVERDT(x2,y1,z2);
        v8(7) = T%DRIVER%VDRIVERDT(x2,y2,z2);
        v8(8) = T%DRIVER%VDRIVERDT(x1,y2,z2);
        w8(1) = T%DRIVER%WDRIVERDT(x1,y1,z1);
        w8(2) = T%DRIVER%WDRIVERDT(x2,y1,z1);
        w8(3) = T%DRIVER%WDRIVERDT(x2,y2,z1);
        w8(4) = T%DRIVER%WDRIVERDT(x1,y2,z1);
        w8(5) = T%DRIVER%WDRIVERDT(x1,y1,z2);
        w8(6) = T%DRIVER%WDRIVERDT(x2,y1,z2);
        w8(7) = T%DRIVER%WDRIVERDT(x2,y2,z2);
        w8(8) = T%DRIVER%WDRIVERDT(x1,y2,z2);
     end if


     !%%%%%interpZ%%%%%%%%%%%%

     if (extrapZ .eq. 1) then
        !%%You don't need to interpolate on z and you can just use
        !%%the values at markZ or z1
        zpts1 = zpts2(1);
        u4(1) = u8(1);
        u4(2) = u8(2);
        u4(3) = u8(3);
        u4(4) = u8(4);
        v4(1) = v8(1);
        v4(2) = v8(2);
        v4(3) = v8(3);
        v4(4) = v8(4);
        w4(1) = w8(1);
        w4(2) = w8(2);
        w4(3) = w8(3);
        w4(4) = w8(4);
     else
        !%%Interpolate Between Z points(interpolate pts 1-4 and 5-8)
        !%Pts 1,5 : 2,6 : 3,7 : 4,8
        coord1 = (/1,2,3,4/);
        coord2 = (/5,6,7,8/);
        do ii = 1,4
           uslope = (u8(coord2(ii))-u8(coord1(ii)))/(zpts2(2)-zpts2(1));
           vslope = (v8(coord2(ii))-v8(coord1(ii)))/(zpts2(2)-zpts2(1));
           wslope = (w8(coord2(ii))-w8(coord1(ii)))/(zpts2(2)-zpts2(1));
           u4(ii) = uslope*(zstar-zpts2(1))+u8(coord1(ii));
           v4(ii) = vslope*(zstar-zpts2(1))+v8(coord1(ii));
           w4(ii) = wslope*(zstar-zpts2(1))+w8(coord1(ii));
        end do
        zpts1 = zstar;
     end if

     !%%%%%interpY%%%%%%%%%%%

     if (extrapY .eq. 1) then
        !%%You don't need to interpolate on y
        ypts1 = ypts2(1);
        u2(1) = u4(1);
        u2(2) = u4(2);
        v2(1) = v4(1);
        v2(2) = v4(2);
        w2(1) = w4(1);
        w2(2) = w4(2);
     else
        !%%Interpolate between Y points(interpolate pts 1-2 and 3-4)
        !%%Pts 1,4 : 2,3
        cord1 = (/1,2/);
        cord2 = (/4,3/);
        do ii = 1,2
           uslope = (u4(cord2(ii))-u4(cord1(ii)))/(ypts2(2)-ypts2(1));
           vslope = (v4(cord2(ii))-v4(cord1(ii)))/(ypts2(2)-ypts2(1));
           wslope = (w4(cord2(ii))-w4(cord1(ii)))/(ypts2(2)-ypts2(1));
           u2(ii) = uslope*(ystar-ypts2(1))+u4(cord1(ii));
           v2(ii) = vslope*(ystar-ypts2(1))+v4(cord1(ii));
           w2(ii) = wslope*(ystar-ypts2(1))+w4(cord1(ii));
        end do
        ypts1 = ystar;
     end if

     !%%%%interpX%%%%%%%%%%%%
     if (extrapX .eq. 1) then
        !%%You don't need to interpolate on x
        xpts1 = xpts2(1);
        u = u2(1);
        v = v2(1);
        w = w2(1);
     else
        !%%Interpolate between X points
        uslope = (u2(2)-u2(1))/(xpts2(2)-xpts2(1));
        vslope = (v2(2)-v2(1))/(xpts2(2)-xpts2(1));
        wslope = (w2(2)-w2(1))/(xpts2(2)-xpts2(1));
        u = uslope*(xstar-xpts2(1))+u2(1);
        v = vslope*(xstar-xpts2(1))+v2(1);
        w = wslope*(xstar-xpts2(1))+w2(1);
        xpts1 = xstar;
     end if

     !%%%%Save wind values%%%%%

     uvw(1,tt) = u;
     uvw(2,tt) = v;
     uvw(3,tt) = w;

     ! write(*,*) 'tt = ',tt,' u,v,w = ',u,v,w

  end do

  if (extrapT .eq. 1) then
     !//Answer is just first entry of uvw
     vatm(1) = uvw(1,1);
     vatm(2) = uvw(2,1);
     vatm(3) = uvw(3,1);
  else
     !%%Interpolate on T
     tpts(1) = T%DRIVER%TCOORD(markT)
     tpts(2) = T%DRIVER%TCOORD(markT+1);
     u2(1) = uvw(1,1);
     u2(2) = uvw(1,2);
     v2(1) = uvw(2,1);
     v2(2) = uvw(2,2);
     w2(1) = uvw(3,1);
     w2(2) = uvw(3,2);
     uslope = (u2(2)-u2(1))/(tpts(2)-tpts(1));
     vslope = (v2(2)-v2(1))/(tpts(2)-tpts(1));
     wslope = (w2(2)-w2(1))/(tpts(2)-tpts(1));  
     u = uslope*(tstar-tpts(1))+u2(1);
     v = vslope*(tstar-tpts(1))+v2(1);
     w = wslope*(tstar-tpts(1))+w2(1);
     vatm(1) = u;
     vatm(2) = v;
     vatm(3) = w;
  end if

  T%DRIVER%VWAKE(1) = -(vatm(1) - T%DRIVER%UDRIVER(markX,markY,KMAX))
  T%DRIVER%VWAKE(2) = vatm(2)
  T%DRIVER%VWAKE(3) = -vatm(3)

  T%DRIVER%markX = markX
  T%DRIVER%markY = markY
  T%DRIVER%markZ = markZ
  T%DRIVER%markT = markT

END SUBROUTINE AIRWAKE

SUBROUTINE TOWED_CONTROL(TOW)
  use TOSIMDATATYPES
  implicit none
  type(TOWEDSTRUCTURE) TOW
  integer iflag,openflag,readflag,i,stateindex,now(3),ctr,j
  integer , parameter :: nwp = 1
  real*8 readreal,phicommand,deltheta,delpsi,dely,delydot,rAS_I(3,1),vAS_I(3,1),thetacommand, xcommand, ycommand, zcommand
  ! real*8 rAS_A(3,1),vAS_A(3,1),lencommand,rGS_I(3,1),vGS_I(3,1),psicp,rGS_P(3,1),vGS_P(3,1),tension,len
  real*8 angles(2,1),delphi,delphidot,ldotnom,ldot,n1,q0,q1,q2,q3,pb,qb,rb,vb,xcg,ycg,zcg,vaero,wb
  real*8 delx,delz,phi,theta,psi,p,q,r,xdot,ydot,zdot,omegaNot,addpitch,addroll,addyaw
  real*8 domegaLeft,omegaRight,domegaFront,omegaBack,domegaDiag,omegaOpp,omegaDiag,omegaFront
  real*8 xwaypoint(nwp,1),ywaypoint(nwp,1),zwaypoint(nwp,1),Dwaypoint
  real*8 delmu(4), munominal
  LOGICAL :: DOUBLET = .FALSE.

END SUBROUTINE TOWED_CONTROL

!!!!!!!!!!!!!!!!!!!!!!!!!! SUBROUTINE TOWED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

SUBROUTINE TOWED(T,iflag)
  use TOSIMDATATYPES
  ! This will only work if you have the right WingsX module
  !USE WingsXAero_Module, ONLY: WingsX_ForceMoment
  !USE SimTable_Module,   ONLY: Read_SimTables, SimTable_ForceMoment 
  !USE Table_Module,      ONLY: Coef_table1
 implicit none
 integer i,iflag,ifind,openflag,readflag
 INTEGER:: ierror
 CHARACTER*100::emsge
 real*8 m,readreal,deti,c1,c2,c3
 real*8 xcg,ycg,zcg,q0,q1,q2,q3,ub,vb,wb,pb,qb,rb
 real*8 xcgdot,ycgdot,zcgdot,q0dot,q1dot,q2dot,q3dot,ubdot,vbdot,wbdot,pbdot,qbdot,rbdot
 real*8 sphi,cphi,stheta,ctheta,spsi,cpsi,tphi(3,3),ttheta(3,3),tpsi(3,3),tib(3,3),vATM_A(3,1),vATM_I(3,1)
 real*8 rxlse,rylse,rzlse,xpt,ypt,zpt,upt,vpt,wpt,uptaero,vptaero,wptaero,C_roll,q_inf,q_inf_S,salfa,calfa
 real*8 sphilse,cphilse,sgamlse,cgamlse,ulse,vlse,wlse,cllse,cdlse,liftlse,draglse,sangle,cangle
 real*8 fxloc,fyloc,fzloc,ct,J,T_0,T_A,Q_A,V_A,omega,rps,alfa,beta,C_Y,C_n,C_m,uaero,vaero,waero
 real*8 C_Ftether_I(3,1),C_Ftether_B(3,1),alfahat,alfadot,MACH,C_Mtether_B(3,1),rCF_B(3,1),S_rCF_B(3,3)
 real*8 vF_I(3,1),vC_I(3,1),rC_I(3,1),rF_I(3,1),S_wt_B(3,3),phat,rhat,qhat,uhat,AR
 real*8 xcgcp, ycgcp, zcgcp,TVEC(4),TDOTVEC(4),TDBLDOTVEC(4),Gammavec(3,1),zetaF
 real*8 thrust,sumomega,sigmaF,omegar,omegaF,C3F(4),C2F(4),C1F(4),bquad,forcevec(4,1)
 integer idx
 type(TOSIMSTRUCTURE) T
 REAL:: y(6), cntrl(0:10)
 REAL:: CXb,CYb,CZb,CL,CD,Cll,Cm,Cn
 REAL:: dummy, dummy2(2)

!!!!!!!!!!!!!!!!!!!!!!!!!!!!! COMPUTE iflag = 3 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  
 if (iflag .eq. 2) then  
 
  ! Unwrap State Vector
  
  xcg = T%TOW%STATE(1)
  ycg = T%TOW%STATE(2)
  zcg = T%TOW%STATE(3)
  q0 = T%TOW%STATE(4) 
  q1 = T%TOW%STATE(5)
  q2 = T%TOW%STATE(6)
  q3 = T%TOW%STATE(7)
  ub = T%TOW%STATE(8)
  vb = T%TOW%STATE(9)
  wb = T%TOW%STATE(10)
  pb = T%TOW%STATE(11) 
  qb = T%TOW%STATE(12)
  rb = T%TOW%STATE(13)
  TVEC(1)    = T%TOW%STATE(13)
  TDOTVEC(1) = T%TOW%STATE(14)
  TVEC(2)    = T%TOW%STATE(15)
  TDOTVEC(2) = T%TOW%STATE(16)
  TVEC(3)    = T%TOW%STATE(17)
  TDOTVEC(3) = T%TOW%STATE(18)
  TVEC(4)    = T%TOW%STATE(19)
  TDOTVEC(4) = T%TOW%STATE(20)

  !Compute phi,theta,psi
  !REVISIT - I've got a function for that -DK 8/18/2015 - It even checks for gimbal lock state
  !USE Math_Module, ONLY: QuatEulTrans
  !REAL quatVec(4)
  !quatVec = {q0,q1,q2,q3)
  !call QuatEulTrans(quatVec,T%TOW%PHI,T%TOW%THETA,T%TOW%PSI)
  
  T%TOW%PHI   = atan2(2.*(q0*q1 + q2*q3),1.-2.*(q1**2 + q2**2));
  T%TOW%THETA = asin (2.*(q0*q2 - q3*q1));
  T%TOW%PSI   = atan2(2.*(q0*q3 + q1*q2),1.-2.*(q2**2 + q3**2));
  
  ! Towed to Inertial Transformation Matrix
  
  T%TOW%TIA(1,1) = q0**2. + q1**2. - q2**2. - q3**2. 
  T%TOW%TIA(2,1) = 2.0*(q1*q2+q0*q3)
  T%TOW%TIA(3,1) = 2.0*(q1*q3-q0*q2)
  T%TOW%TIA(1,2) = 2.0*(q1*q2-q0*q3)
  T%TOW%TIA(2,2) = q0**2. - q1**2. + q2**2. - q3**2.
  T%TOW%TIA(3,2) = 2.0*(q2*q3+q0*q1)
  T%TOW%TIA(1,3) = 2.0*(q1*q3+q0*q2)
  T%TOW%TIA(2,3) = 2.0*(q2*q3-q0*q1)
  T%TOW%TIA(3,3) = q0**2. - q1**2. - q2**2. + q3**2.

  ! Inertial to Towed Transformation Matrix
  
  T%TOW%TAI = transpose(T%TOW%TIA)

  !Inertial Location of Tether Point

  !First get location of CG
  rC_I(1,1) = xcg 
  rC_I(2,1) = ycg
  rC_I(3,1) = zcg !!! REVISIT Nghia did -1 to make the connection point realistic

  !Then get location of tether in body frame
  rCF_B(1,1) = T%TOW%SLTETHER 
  rCF_B(2,1) = T%TOW%BLTETHER
  rCF_B(3,1) = T%TOW%WLTETHER

  !Then compute inertial location of tether connection point
  rF_I = rC_I + matmul(T%TOW%TIA,rCF_B)  !REVISIT DK 8/18/2015 - I've got a function for this too BodyToEarthTrans(quatVec,vb,vf)

  T%THR%XTETHER = rF_I(1,1)
  T%THR%YTETHER = rF_I(2,1)
  T%THR%ZTETHER = rF_I(3,1) -(1/2) !Why is this 1/2 here???? 1/19/2024 - CJM
  
  ! Gravity Forces and Moments
  
  T%TOW%FXGRAV = 0.0; T%TOW%FYGRAV = 0.0; T%TOW%FZGRAV = 0.0;
  T%TOW%MXGRAV = 0.0; T%TOW%MYGRAV = 0.0; T%TOW%MZGRAV = 0.0;
  
  if (T%TOW%GRAVOFFON .eq. 1) then
    T%TOW%FXGRAV = T%TOW%TAI(1,3)*T%TOW%WEIGHT
    T%TOW%FYGRAV = T%TOW%TAI(2,3)*T%TOW%WEIGHT
    T%TOW%FZGRAV = T%TOW%TAI(3,3)*T%TOW%WEIGHT
  end if
  
  ! Aerodynamic Forces and Moments
  
  T%TOW%FXAERO = 0.0; T%TOW%FYAERO = 0.0; T%TOW%FZAERO = 0.0;
  T%TOW%MXAERO = 0.0; T%TOW%MYAERO = 0.0; T%TOW%MZAERO = 0.0;

  !Compute Atmopsheric density and winds

  T%ATM%XI = xcg
  T%ATM%YI = ycg
  T%ATM%ZI = zcg

  ! write(*,*) T%SIM%TIME,T%ATM%XI,T%ATM%YI,T%ATM%ZI

  call ATMOSPHERE(T,2) !T%ATM%DEN

  ! write(*,*) T%ATM%VXWIND,T%ATM%VYWIND,T%ATM%VZWIND

  vATM_I(1,1) = T%ATM%VXWIND
  vATM_I(2,1) = T%ATM%VYWIND
  vATM_I(3,1) = T%ATM%VZWIND

  T%TOW%VXWIND = T%ATM%VXWIND
  T%TOW%VYWIND = T%ATM%VYWIND
  T%TOW%VZWIND = T%ATM%VZWIND

  vATM_A = matmul(T%TOW%TAI,vATM_I)

  !Add in atmospheric winds

  uaero = ub - vATM_A(1,1)
  vaero = vb - vATM_A(2,1)
  waero = wb - vATM_A(3,1)
  
  !Compute total velocity
  
  V_A = sqrt(uaero**2 + vaero**2 + waero**2)

  if (T%TOW%AEROFLAG .gt. 0) then !If AEROFLAG is greater than 0 it is either a 1 or a 2 - CM 8/16/2015
    !Compute Atmopsheric density and winds

    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!TOWED AERODYNAMIC MODEL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    if (V_A .eq. 0) then
        V_A = uaero
    end if

    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Quadcopter Aerodynamic Model written by Lisa Schibelius - 12/2016!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    !Recompute KT
    T%TOW%KT = T%TOW%C_T*((T%ATM%DEN*qPI*(T%TOW%RNEW**4)/4))
    
    !Compute Thrust using a 2nd order function
    sigmaF = 0.000437554764978899 !0.000437554764978899
    omegaF = 45.42   !18.65
    zetaF  = 0.942     !0.8533

    !write(*,*) 'MUvec = ',T%TOW%MUVEC

    !!! Second order filter
    do idx = 1,4
       C1F(idx) = -2*zetaF*TDOTVEC(idx)
       C2F(idx) = -(omegaF)*TVEC(idx)
       C3F(idx) = T%TOW%MUVEC(idx,1)*sigmaF*omegaF   ! replaced sigma with force
       TDBLDOTVEC(idx) = omegaF*(C1F(idx) + C2F(idx) + C3F(idx))
       T%TOW%THRUSTVEC(idx,1) = TVEC(idx)
    end do

    T%TOW%OMEGAVEC = sqrt(T%TOW%THRUSTVEC/T%TOW%KT)
    sumomega = sum(T%TOW%OMEGAVEC)

    !!! Make sure angular velocities of rotor does not go beyond the limit
    IF (sumomega .ge. T%TOW%OMEGAMAX*4) then
       do j = 1,4
          if (T%TOW%OMEGAVEC(j,1) .gt. T%TOW%OMEGAMAX) then
             T%TOW%OMEGAVEC(j,1) = T%TOW%OMEGAMAX
          end if
          if (T%TOW%OMEGAVEC(j,1) .lt. 0.00D0) then
             T%TOW%OMEGAVEC(j,1) = 0.00D0
          end if
       end do
       sumomega = sum(T%TOW%OMEGAVEC)
       T%TOW%THRUSTVEC = T%TOW%KT*T%TOW%OMEGAVEC**2
       do j = 1,4
          TVEC(idx) = T%TOW%THRUSTVEC(idx,1)
       end do
    ENDIF
    forcevec = T%TOW%THRUSTVEC
    thrust = sum(T%TOW%THRUSTVEC)

    !write(*,*) 'Rotor Thrust = ',forcevec,thrust

    !!! Adding constraint to run Monte Carlo
    !!! This constraint actually messed up my altitude controller
    ! if (thrust .gt. T%TOW%WEIGHT/cos(30.0*qPI/180)) then !You need to put in theta of the quad 
    !   thrust = T%TOW%WEIGHT/cos(30.0*qPI/180) !!Not just a static 30 degrees. That's why this didn't work.
    ! end if
    ! I was thinking this would be better.
    ! if (thrust .lt. T%TOW%WEIGHT/cos(theta)) then
    ! Increase thrust by the difference/4
    ! Make sense?

    !Aerodynamic Forces
    if (sumomega .gt. 1e-2) then
       T%TOW%FXAERO = -thrust*(((T%TOW%ALC/(sumomega*T%TOW%RNEW))+T%TOW%DXD)*uaero - ((T%TOW%ALS)/(sumomega*T%TOW%RNEW))*vaero)
       T%TOW%FYAERO = -thrust*(((T%TOW%ALS)/(sumomega*T%TOW%RNEW))*uaero + (((T%TOW%ALC)/(sumomega*T%TOW%RNEW))+T%TOW%DYD)*vaero)
       T%TOW%FZAERO = -thrust
    end if

    omegar = T%TOW%OMEGAVEC(1,1) - T%TOW%OMEGAVEC(2,1) + T%TOW%OMEGAVEC(3,1) - T%TOW%OMEGAVEC(4,1)
    Gammavec(1,1) = T%TOW%IRR * omegar * qb
    Gammavec(2,1) = -T%TOW%IRR * omegar * pb
    Gammavec(3,1) = 0
    !gotodynamics

    !!!!!!!!! Aerodynamics
    bquad = T%TOW%C_TAU*((T%ATM%DEN*qPI*(T%TOW%RNEW**5)/4))

    !!! According to dynamic equations, a positive roll will have rotors 1,4 > 2,3. This was previously 2,3>1,4
    ! Since T3 = T1*Ltheta_front/Ltheta_back we're just going to do this for simplicity
    T%TOW%MXAERO = Gammavec(1,1) + (T%TOW%LPHI12*(TVEC(1) - TVEC(2)) + T%TOW%LPHI34*(TVEC(4) - TVEC(3)))
    T%TOW%MYAERO = Gammavec(2,1) + T%TOW%LTHETA12*(TVEC(1) +TVEC(2) - TVEC(3) -TVEC(4))
    T%TOW%MZAERO = Gammavec(3,1) + bquad*(T%TOW%OMEGAVEC(1,1)**2 - T%TOW%OMEGAVEC(2,1)**2 + T%TOW%OMEGAVEC(3,1)**2 - T%TOW%OMEGAVEC(4,1)**2)
    ! T%TOW%MZAERO = Gammavec(3,1) + bquad*(TVEC(1)**2 - TVEC(2)**2 + TVEC(3)**2 - TVEC(4)**2)

   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!END OF QUADCOPTER AERODYNAMIC MODEL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   
  end if !AEROFORCES

  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!END OF AIRCRAFT AERODYNAMIC FORCES AND MOMENTS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  ! Tether Forces and Moments

  T%TOW%MXCONT = 0.0; T%TOW%MYCONT = 0.0; T%TOW%MZCONT = 0.0;
  T%TOW%FXCONT = 0.0; T%TOW%FYCONT = 0.0; T%TOW%FZCONT = 0.0;          
  if ((T%THR%DYNOFFON .eq. 1) .and. (T%THR%ELASOFFON .eq. 1)) then
     if (isnan(T%THR%FXTETHER) .or. isnan(T%THR%FZTETHER)) then
        T%TOW%FXCONT = 0.0; T%TOW%FYCONT = 0.0; T%TOW%FZCONT = 0.0;          
        T%TOW%MXCONT = 0.0; T%TOW%MYCONT = 0.0; T%TOW%MZCONT = 0.0;
     else
        C_Ftether_I(1,1) = T%THR%FXTETHER
        C_Ftether_I(2,1) = T%THR%FYTETHER
        C_Ftether_I(3,1) = T%THR%FZTETHER
        C_Ftether_B = matmul(T%TOW%TAI,C_Ftether_I)
        T%TOW%FXCONT = C_Ftether_B(1,1)
        T%TOW%FYCONT = C_Ftether_B(2,1)
        T%TOW%FZCONT = C_Ftether_B(3,1)
        
        !Skew symmetric operator on cradle cg to tether connection point
    
        S_rCF_B(1,1) = 0.0
        S_rCF_B(1,2) = -rCF_B(3,1)
        S_rCF_B(1,3) = rCF_B(2,1)
        S_rCF_B(2,1) = rCF_B(3,1)
        S_rCF_B(2,2) = 0.0
        S_rCF_B(2,3) = -rCF_B(1,1)
        S_rCF_B(3,1) = -rCF_B(2,1)
        S_rCF_B(3,2) = rCF_B(1,1)
        S_rCF_B(3,3) = 0.0

        C_Mtether_B = matmul(S_rCF_B,C_Ftether_B)
        
        T%TOW%MXCONT = C_Mtether_B(1,1)
        T%TOW%MYCONT = C_Mtether_B(2,1)
        T%TOW%MZCONT = C_Mtether_B(3,1)

        !Add Torsional Stiffness of Tether
        T%TOW%MXCONT = T%TOW%MXCONT - T%THR%KP*(T%TOW%PHI) - T%THR%KD*pb

     end if
  endif

  ! write(*,*) T%TOW%PHI,pb,T%TOW%MXCONT,T%TOW%MYCONT,T%TOW%MZCONT
  ! write(*,*) T%TOW%FXAERO,T%TOW%FYAERO,T%TOW%FZAERO

  ! Total Forces and Moments

  T%TOW%FXTOTAL = T%TOW%FXGRAV + T%TOW%FXAERO + T%TOW%FXCONT
  T%TOW%FYTOTAL = T%TOW%FYGRAV + T%TOW%FYAERO + T%TOW%FYCONT 
  T%TOW%FZTOTAL = T%TOW%FZGRAV + T%TOW%FZAERO + T%TOW%FZCONT
  T%TOW%MXTOTAL = T%TOW%MXGRAV + T%TOW%MXAERO + T%TOW%MXCONT
  T%TOW%MYTOTAL = T%TOW%MYGRAV + T%TOW%MYAERO + T%TOW%MYCONT
  T%TOW%MZTOTAL = T%TOW%MZGRAV + T%TOW%MZAERO + T%TOW%MZCONT
  
  ! State Derivatives
  
  xcgdot = T%TOW%TIA(1,1)*ub + T%TOW%TIA(1,2)*vb + T%TOW%TIA(1,3)*wb
  ycgdot = T%TOW%TIA(2,1)*ub + T%TOW%TIA(2,2)*vb + T%TOW%TIA(2,3)*wb
  zcgdot = T%TOW%TIA(3,1)*ub + T%TOW%TIA(3,2)*vb + T%TOW%TIA(3,3)*wb  

  !Velocity of Tether Connection point
  vC_I(1,1) = xcgdot
  vC_I(2,1) = ycgdot
  vC_I(3,1) = zcgdot

  ! Skew symmetric of towed angular velocity

  S_wt_B(1,1) = 0.0
  S_wt_B(1,2) = -rb
  S_wt_B(1,3) = qb
  S_wt_B(2,1) = rb
  S_wt_B(2,2) = 0.0
  S_wt_B(2,3) = -pb
  S_wt_B(3,1) = -qb
  S_wt_B(3,2) = pb
  S_wt_B(3,3) = 0.0

  vF_I = vC_I + matmul(T%TOW%TIA,matmul(S_wt_B,rCF_B))

  T%THR%XTETHERDOT = vF_I(1,1)
  T%THR%YTETHERDOT = vF_I(2,1)
  T%THR%ZTETHERDOT = vF_I(3,1)

  q0dot = (- q1*pb - q2*qb - q3*rb)/2.0
  q1dot = (  q0*pb - q3*qb + q2*rb)/2.0
  q2dot = (  q3*pb + q0*qb - q1*rb)/2.0
  q3dot = (- q2*pb + q1*qb + q0*rb)/2.0
  ubdot = T%TOW%FXTOTAL/T%TOW%MASS + rb*vb - qb*wb
  vbdot = T%TOW%FYTOTAL/T%TOW%MASS + pb*wb - rb*ub 
  wbdot = T%TOW%FZTOTAL/T%TOW%MASS + qb*ub - pb*vb 
  c1 = T%TOW%MXTOTAL - pb*(qb*T%TOW%IXZ-rb*T%TOW%IXY) - qb*(qb*T%TOW%IYZ-rb*T%TOW%IYY) - rb*(qb*T%TOW%IZZ-rb*T%TOW%IYZ)
  c2 = T%TOW%MYTOTAL - pb*(rb*T%TOW%IXX-pb*T%TOW%IXZ) - qb*(rb*T%TOW%IXY-pb*T%TOW%IYZ) - rb*(rb*T%TOW%IXZ-pb*T%TOW%IZZ)
  c3 = T%TOW%MZTOTAL - pb*(pb*T%TOW%IXY-qb*T%TOW%IXX) - qb*(pb*T%TOW%IYY-qb*T%TOW%IXY) - rb*(pb*T%TOW%IYZ-qb*T%TOW%IXZ)
  pbdot = T%TOW%IXXI*c1 + T%TOW%IXYI*c2 + T%TOW%IXZI*c3
  qbdot = T%TOW%IXYI*c1 + T%TOW%IYYI*c2 + T%TOW%IYZI*c3
  rbdot = T%TOW%IXZI*c1 + T%TOW%IYZI*c2 + T%TOW%IZZI*c3
  
  ! Wrap State Derivatives
  
  T%TOW%STATEDOT(1) = xcgdot
  T%TOW%STATEDOT(2) = ycgdot
  T%TOW%STATEDOT(2) = ycgdot
  T%TOW%STATEDOT(3) = zcgdot
  T%TOW%STATEDOT(4) = q0dot 
  T%TOW%STATEDOT(5) = q1dot
  T%TOW%STATEDOT(6) = q2dot
  T%TOW%STATEDOT(7) = q3dot
  T%TOW%STATEDOT(8) = ubdot
  T%TOW%STATEDOT(9) = vbdot
  T%TOW%STATEDOT(10) = wbdot
  T%TOW%STATEDOT(11) = pbdot 
  T%TOW%STATEDOT(12) = qbdot
  T%TOW%STATEDOT(13) = rbdot
   
  RETURN
  
 end if !!For the compute flag
  
!!!!!!!!!!!!!!!!!!!!!!!!!! LOAD DATA iflag = 1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 
 if (iflag .eq. 1) then
 
  open(unit=94,file=T%TOWEDINPUTFILE,status='old',iostat=openflag)
  if (openflag .ne. 0) then
   write(*,*) 'Error Opening Towed Input File: ',T%TOWEDINPUTFILE; PAUSE; STOP
  end if
  rewind(94)

  read(unit=94,fmt=*,iostat=readflag) T%TOW%DYNOFFON
  read(unit=94,fmt=*,iostat=readflag) T%TOW%GRAVOFFON
  read(unit=94,fmt=*,iostat=readflag) T%TOW%AEROFLAG
  read(unit=94,fmt=*,iostat=readflag) T%TOW%WEIGHT
  read(unit=94,fmt=*,iostat=readflag) T%TOW%GRAVITY
  read(unit=94,fmt=*,iostat=readflag) T%TOW%SLCG
  read(unit=94,fmt=*,iostat=readflag) T%TOW%BLCG
  read(unit=94,fmt=*,iostat=readflag) T%TOW%WLCG
  read(unit=94,fmt=*,iostat=readflag) T%TOW%SLTETHER
  read(unit=94,fmt=*,iostat=readflag) T%TOW%BLTETHER
  read(unit=94,fmt=*,iostat=readflag) T%TOW%WLTETHER
  read(unit=94,fmt=*,iostat=readflag) T%TOW%IXX
  read(unit=94,fmt=*,iostat=readflag) T%TOW%IYY
  read(unit=94,fmt=*,iostat=readflag) T%TOW%IZZ
  read(unit=94,fmt=*,iostat=readflag) T%TOW%IXY
  read(unit=94,fmt=*,iostat=readflag) T%TOW%IXZ
  read(unit=94,fmt=*,iostat=readflag) T%TOW%IYZ
  read(unit=94,fmt=*,iostat=readflag) T%TOW%TURNRADIUS
  read(unit=94,fmt=*,iostat=readflag) T%TOW%ALC
  read(unit=94,fmt=*,iostat=readflag) T%TOW%ALS
  read(unit=94,fmt=*,iostat=readflag) T%TOW%DXD
  read(unit=94,fmt=*,iostat=readflag) T%TOW%DYD
  read(unit=94,fmt=*,iostat=readflag) T%TOW%RNEW
  read(unit=94,fmt=*,iostat=readflag) T%TOW%C_T
  read(unit=94,fmt=*,iostat=readflag) T%TOW%C_TAU
  read(unit=94,fmt=*,iostat=readflag) T%TOW%LPHI12
  read(unit=94,fmt=*,iostat=readflag) T%TOW%LPHI34
  read(unit=94,fmt=*,iostat=readflag) T%TOW%LTHETA12
  read(unit=94,fmt=*,iostat=readflag) T%TOW%LTHETA34
  read(unit=94,fmt=*,iostat=readflag) T%TOW%OMEGAMAX
  read(unit=94,fmt=*,iostat=readflag) T%TOW%IRR
  read(unit=94,fmt=*,iostat=readflag) T%TOW%MS_MIN
  read(unit=94,fmt=*,iostat=readflag) T%TOW%MS_MAX
  read(unit=94,fmt=*,iostat=readflag) readreal; T%TOW%CONTROLOFFON = int(readreal)
  read(unit=94,fmt=*,iostat=readflag) T%TOW%KPXDRIVE
  read(unit=94,fmt=*,iostat=readflag) T%TOW%KIXDRIVE
  read(unit=94,fmt=*,iostat=readflag) T%TOW%KDXDRIVE
  read(unit=94,fmt=*,iostat=readflag) T%TOW%KPYDRIVE
  read(unit=94,fmt=*,iostat=readflag) T%TOW%KIYDRIVE
  read(unit=94,fmt=*,iostat=readflag) T%TOW%KDYDRIVE
  read(unit=94,fmt=*,iostat=readflag) T%TOW%KPZDRIVE
  read(unit=94,fmt=*,iostat=readflag) T%TOW%KIZDRIVE
  read(unit=94,fmt=*,iostat=readflag) T%TOW%KDZDRIVE
  read(unit=94,fmt=*,iostat=readflag) T%TOW%KPPHI
  read(unit=94,fmt=*,iostat=readflag) T%TOW%KIPHI
  read(unit=94,fmt=*,iostat=readflag) T%TOW%KDPHI
  read(unit=94,fmt=*,iostat=readflag) T%TOW%KPTHETA
  read(unit=94,fmt=*,iostat=readflag) T%TOW%KITHETA
  read(unit=94,fmt=*,iostat=readflag) T%TOW%KDTHETA
  read(unit=94,fmt=*,iostat=readflag) T%TOW%KPPSI
  read(unit=94,fmt=*,iostat=readflag) T%TOW%KIPSI
  read(unit=94,fmt=*,iostat=readflag) T%TOW%KDPSI
  read(unit=94,fmt=*,iostat=readflag) T%TOW%XINTEGRAL 
  read(unit=94,fmt=*,iostat=readflag) T%TOW%YINTEGRAL 
  read(unit=94,fmt=*,iostat=readflag) T%TOW%ZINTEGRAL
  read(unit=94,fmt=*,iostat=readflag) T%TOW%PHIINTEGRAL 
  read(unit=94,fmt=*,iostat=readflag) T%TOW%THETAINTEGRAL
  read(unit=94,fmt=*,iostat=readflag) T%TOW%PSIINTEGRAL
  read(unit=94,fmt=*,iostat=readflag) T%TOW%UCOMMAND !!Changed to UCOMMAND for forward flight 
  read(unit=94,fmt=*,iostat=readflag) T%TOW%YCOMMAND
  read(unit=94,fmt=*,iostat=readflag) T%TOW%ZCOMMAND
  close(94) 
   
  T%TOW%MASS = T%TOW%WEIGHT/T%GRAVITY
  deti = + T%TOW%IXX*(T%TOW%IYY*T%TOW%IZZ-T%TOW%IYZ*T%TOW%IYZ) - T%TOW%IXY*(T%TOW%IXY*T%TOW%IZZ-T%TOW%IYZ*T%TOW%IXZ) + T%TOW%IXZ*(T%TOW%IXY*T%TOW%IYZ-T%TOW%IYY*T%TOW%IXZ)
  T%TOW%IXXI = (T%TOW%IYY*T%TOW%IZZ-T%TOW%IYZ*T%TOW%IYZ)/deti
  T%TOW%IXYI = (T%TOW%IYZ*T%TOW%IXZ-T%TOW%IXY*T%TOW%IZZ)/deti
  T%TOW%IXZI = (T%TOW%IXY*T%TOW%IYZ-T%TOW%IYY*T%TOW%IXZ)/deti
  T%TOW%IYYI = (T%TOW%IXX*T%TOW%IZZ-T%TOW%IXZ*T%TOW%IXZ)/deti
  T%TOW%IYZI = (T%TOW%IXY*T%TOW%IXZ-T%TOW%IXX*T%TOW%IYZ)/deti
  T%TOW%IZZI = (T%TOW%IXX*T%TOW%IYY-T%TOW%IXY*T%TOW%IXY)/deti

  write(*,*) 'TOWED Load Complete'
 
  T%TOW%DQFLAG = 1

  RETURN
 
 end if !!End of load flag
 
 RETURN
END SUBROUTINE TOWED

!!!!!!!!!!!!!!!!!!!!! SUBROUTINE TETHER !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

SUBROUTINE TETHER(T,iflag)
 use TOSIMDATATYPES
 implicit none
 integer i,iflag,ifind,openflag,readflag,stateindex
 real*8 xcg(MAXNBEADS),ycg(MAXNBEADS),zcg(MAXNBEADS)
 real*8 xcgdot(MAXNBEADS),ycgdot(MAXNBEADS),zcgdot(MAXNBEADS)
 real*8 xcgdotdot(MAXNBEADS),ycgdotdot(MAXNBEADS),zcgdotdot(MAXNBEADS)
 real*8 tension(MAXNBEADS+1),tensiondot(MAXNBEADS+1),ELEN
 real*8 r(MAXNBEADS+2,3),dr(MAXNBEADS+1,3),dl(MAXNBEADS+1),v(MAXNBEADS+2,3),dv(MAXNBEADS+1,3),dldot(MAXNBEADS+1)
 real*8 dn(MAXNBEADS+1,3),tiner(MAXNBEADS+1,3),raero(MAXNBEADS+1,3),diameter
 real*8 vaero(MAXNBEADS+1,3),vaerosf(MAXNBEADS+1),vaerofp(MAXNBEADS+1,3),vaerofpmag(MAXNBEADS+1),thetadot,thetaddot
 real*8 rhoaero(MAXNBEADS+1),sfarea(MAXNBEADS+1),swarea(MAXNBEADS+1),dragsf(MAXNBEADS+1),dragfp(MAXNBEADS+1)
 real*8 fxsf(MAXNBEADS+1),fysf(MAXNBEADS+1),fzsf(MAXNBEADS+1),fxfp(MAXNBEADS+1),fyfp(MAXNBEADS+1),fzfp(MAXNBEADS+1)
 type(TOSIMSTRUCTURE) T

!!!!!!!!!!!!!!!!!! COMPUTE iflag = 3 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  
 if ((iflag .eq. 2) .or. (iflag .eq. 3)) then  !3 is a debug flag I added when adding beads (C. Montalvo 11/1/15)
 
  ! Unwrap State Vector
  
  stateindex = 0
  do i=1,T%THR%NBEADS
  ! This assigns the translational states to the ith tether bead
   xcg(i) = T%THR%STATE(stateindex+1) ! X 
   ycg(i) = T%THR%STATE(stateindex+2) ! Y 
   zcg(i) = T%THR%STATE(stateindex+3) ! Z 
   xcgdot(i) = T%THR%STATE(stateindex+4) ! X Dot 
   ycgdot(i) = T%THR%STATE(stateindex+5) ! Y Dot
   zcgdot(i) = T%THR%STATE(stateindex+6) ! Z Dot
   stateindex = stateindex + 6
  end do
  do i=1,T%THR%NBEADS+1
   tension(i) = T%THR%STATE(stateindex+1) ! Tension 
   stateindex = stateindex + 1
  end do

  ! Construct Tether Line Position and Velocity Matrices
  
  !Position of 1st truss element

  r(1,1) = T%DRIVER%XREEL; 
  r(1,2) = T%DRIVER%YREEL; 
  r(1,3) = T%DRIVER%ZREEL;

  !Position of every truss element in between is just equal to the bead location

  r(2:T%THR%NBEADS+1,1) = xcg(1:T%THR%NBEADS); 
  r(2:T%THR%NBEADS+1,2) = ycg(1:T%THR%NBEADS);
  r(2:T%THR%NBEADS+1,3) = zcg(1:T%THR%NBEADS);
  
  !Position of the last truss element is at the towed connection point

  r(T%THR%NBEADS+2,1) = T%THR%XTETHER; 
  r(T%THR%NBEADS+2,2) = T%THR%YTETHER; 
  r(T%THR%NBEADS+2,3) = T%THR%ZTETHER;

  !Debug Output
  ! if (iflag .eq. 3) then
  !    write(*,*) 'Tension = ',tension(1:T%THR%NBEADS+1)
  !    write(*,*) 'X Position = ',r(1:T%THR%NBEADS+2,1)
  !    write(*,*) 'Y Position = ',r(1:T%THR%NBEADS+2,2)
  !    write(*,*) 'Z Position = ',r(1:T%THR%NBEADS+2,3)
  ! end if

  !The difference between all truss elements is

  dr = 0.0
  dr(1:T%THR%NBEADS+1,1:3) = r(2:T%THR%NBEADS+2,1:3) - r(1:T%THR%NBEADS+1,1:3)

  !write(*,*) dr(:,1)
  
  ! And the norm

  dl(1:T%THR%NBEADS+1) = sqrt(dr(:,1)**2+dr(:,2)**2+dr(:,3)**2)

  T%THR%STRETCHLEN = sum(dl(1:T%THR%NBEADS+1))
     
  !The velocity of the first point is the velocity of the reel

  v(1,1) = T%DRIVER%XREELDOT; 
  v(1,2) = T%DRIVER%YREELDOT; 
  v(1,3) = T%DRIVER%ZREELDOT;
  
  !The velocity of all other points just comes from the state vector

  v(2:T%THR%NBEADS+1,1) = xcgdot(1:T%THR%NBEADS);
  v(2:T%THR%NBEADS+1,2) = ycgdot(1:T%THR%NBEADS);
  v(2:T%THR%NBEADS+1,3) = zcgdot(1:T%THR%NBEADS);

  !The last point is the velocity of the connection point on the cradle

  v(T%THR%NBEADS+2,1) = T%THR%XTETHERDOT; 
  v(T%THR%NBEADS+2,2) = T%THR%YTETHERDOT; 
  v(T%THR%NBEADS+2,3) = T%THR%ZTETHERDOT;
  
  !Difference in velocity

  dv(1:T%THR%NBEADS+1,1:3) = v(2:T%THR%NBEADS+2,1:3) - v(1:T%THR%NBEADS+1,1:3)

  !The normalized position vector

  do i = 1,T%THR%NBEADS+1
      if (abs(dl(i)) .gt. 0) then
          dn(i,1:3) = dr(i,1:3)/dl(i)
      else
          dn(i,1:3) = 0 
      end if
      !To compute dldot we need the component of dv along dn
      dldot(i) = dv(i,1)*dn(i,1) + dv(i,2)*dn(i,2) + dv(i,3)*dn(i,3)
  end do

  !Tension force time normal vector

  tiner(1:T%THR%NBEADS+1,1) = dn(1:T%THR%NBEADS+1,1)*tension(1:T%THR%NBEADS+1)
  tiner(1:T%THR%NBEADS+1,2) = dn(1:T%THR%NBEADS+1,2)*tension(1:T%THR%NBEADS+1)
  tiner(1:T%THR%NBEADS+1,3) = dn(1:T%THR%NBEADS+1,3)*tension(1:T%THR%NBEADS+1)
   
  ! Gravity Forces
  
  T%THR%FXGRAV(1:T%THR%NBEADS) = 0.0; 
  T%THR%FYGRAV(1:T%THR%NBEADS) = 0.0; 
  T%THR%FZGRAV(1:T%THR%NBEADS) = 0.0;
  if (T%THR%GRAVOFFON .eq. 1) then
   T%THR%FXGRAV(1:T%THR%NBEADS) = 0.0
   T%THR%FYGRAV(1:T%THR%NBEADS) = 0.0
   T%THR%FZGRAV(1:T%THR%NBEADS) = T%THR%EMASS*T%GRAVITY
  end if
  
  ! Elastic Forces
  
  T%THR%FXELAS(1:T%THR%NBEADS) = 0.0;
  T%THR%FYELAS(1:T%THR%NBEADS) = 0.0; 
  T%THR%FZELAS(1:T%THR%NBEADS) = 0.0;
  if (T%THR%ELASOFFON .eq. 1) then
   T%THR%FXELAS(1:T%THR%NBEADS) = -tiner(1:T%THR%NBEADS,1) + tiner(2:T%THR%NBEADS+1,1)
   T%THR%FYELAS(1:T%THR%NBEADS) = -tiner(1:T%THR%NBEADS,2) + tiner(2:T%THR%NBEADS+1,2)
   T%THR%FZELAS(1:T%THR%NBEADS) = -tiner(1:T%THR%NBEADS,3) + tiner(2:T%THR%NBEADS+1,3)
  end if

  ! Aerodynamic Forces
    
  T%THR%FXAERO(1:T%THR%NBEADS) = 0.0;
  T%THR%FYAERO(1:T%THR%NBEADS) = 0.0; 
  T%THR%FZAERO(1:T%THR%NBEADS) = 0.0;
  if (T%THR%AEROFLAG .gt. 0) then
     !Average position
     raero(1:T%THR%NBEADS+1,1:3) = (r(2:T%THR%NBEADS+2,1:3)+r(1:T%THR%NBEADS+1,1:3))/2.0
     !Average velocity
     vaero(1:T%THR%NBEADS+1,1:3) = (v(2:T%THR%NBEADS+2,1:3)+v(1:T%THR%NBEADS+1,1:3))/2.0
     
     do i=1,T%THR%NBEADS+1
        !Add in atmospheric winds

        T%ATM%XI = raero(i,1); 
        T%ATM%YI = raero(i,2);
        T%ATM%ZI = raero(i,3);
        call ATMOSPHERE(T,2)

        vaero(i,1) = vaero(i,1) - T%ATM%VXWIND;
        vaero(i,2) = vaero(i,2) - T%ATM%VYWIND;
        vaero(i,3) = vaero(i,3) - T%ATM%VZWIND;

        !!Aero velocity for skin friction

        vaerosf(i) = vaero(i,1)*dn(i,1) + vaero(i,2)*dn(i,2) + vaero(i,3)*dn(i,3)

        !!Aero velocity for flat plate drag

        vaerofp(i,1) = vaero(i,1) - vaerosf(i)*dn(i,1);
        vaerofp(i,2) = vaero(i,2) - vaerosf(i)*dn(i,2);
        vaerofp(i,3) = vaero(i,3) - vaerosf(i)*dn(i,3); 

        !!Magnitude of aerodynamic velocity along perpendicular

        vaerofpmag(i) = sqrt(vaerofp(i,1)**2+vaerofp(i,2)**2+vaerofp(i,3)**2)

        !!Density

        rhoaero(i) = T%ATM%DEN  

        !Stretched area of bead

        !Linear Model

        !sfarea(i) = T%THR%DIA*dl(i)*(1.0-T%THR%NU*(dl(i)-T%THR%ELEN)/T%THR%ELEN)

        !Non-Linear Model
        
        if (T%THR%NU .eq. 0) then
           diameter = T%THR%DIA
        else
           diameter = T%THR%DIA - T%THR%DIA*(1-(1+(dl(i)-T%THR%ELEN)/T%THR%ELEN)**(-T%THR%NU))
        end if
        !write(*,*) diameter,dl(i),T%THR%ELEN,sfarea(i),abs(dl(i)-T%THR%ELEN)

        if (diameter .lt. 0) then
           write(*,*) 'Negative Line Area Detected'
        end if

        !Cross Sectional Area

        sfarea(i) = diameter*abs(dl(i)) !(ft^2)

        !Wetted area

        swarea(i) = PI*sfarea(i) !(ft^2)

        !Drag of bead (Axial or Skin Friction Drag)

        dragsf(i) = - 0.5*rhoaero(i)*vaerosf(i)*abs(vaerosf(i))*swarea(i)*T%THR%CD_AXIAL 

        !Drag of bead (Normal or Flat Plat Drag)

        dragfp(i) = - 0.5*rhoaero(i)*vaerofpmag(i)**2*sfarea(i)*T%THR%CD_NORMAL 

        !Skin friction force

        fxsf(i) = dragsf(i)*dn(i,1);
        fysf(i) = dragsf(i)*dn(i,2); 
        fzsf(i) = dragsf(i)*dn(i,3); 

        !Flat plate force

        if (abs(vaerofpmag(i)) .gt. 0) then
            fxfp(i) = dragfp(i)*vaerofp(i,1)/vaerofpmag(i); 
            fyfp(i) = dragfp(i)*vaerofp(i,2)/vaerofpmag(i);
            fzfp(i) = dragfp(i)*vaerofp(i,3)/vaerofpmag(i);
        else
            fxfp(i) = 0
            fyfp(i) = 0
            fzfp(i) = 0
        end if
     end do

   !!Average the forces on the right and left

   T%THR%FXAERO(1:T%THR%NBEADS) = (fxsf(1:T%THR%NBEADS)+fxsf(2:T%THR%NBEADS+1)+fxfp(1:T%THR%NBEADS)+fxfp(2:T%THR%NBEADS+1))/2.0  
   T%THR%FYAERO(1:T%THR%NBEADS) = (fysf(1:T%THR%NBEADS)+fysf(2:T%THR%NBEADS+1)+fyfp(1:T%THR%NBEADS)+fyfp(2:T%THR%NBEADS+1))/2.0
   T%THR%FZAERO(1:T%THR%NBEADS) = (fzsf(1:T%THR%NBEADS)+fzsf(2:T%THR%NBEADS+1)+fzfp(1:T%THR%NBEADS)+fzfp(2:T%THR%NBEADS+1))/2.0

!   if (T%SIM%TIME .gt. 150) then
!      writead(*,*) '----------------------------'
!      write(*,*) dragfp(1:T%THR%NBEADS+1),vaerofp(1:T%THR%NBEADS+1,3)
!      write(*,*) fzsf(1:T%THR%NBEADS+1),fzfp(1:T%THR%NBEADS+1)
!      write(*,*) T%THR%FZAERO(1:T%THR%NBEADS)
!   end if

  end if

  !Save Tether Force connected to towed system
  !Tether force applied to towed system is a combination of tension force plus
  !The aerodynamics of the tether between the last bead and the connection point
  !
  T%THR%FXTETHER = -tiner(T%THR%NBEADS+1,1)
  T%THR%FYTETHER = -tiner(T%THR%NBEADS+1,2)
  T%THR%FZTETHER = -tiner(T%THR%NBEADS+1,3)

  T%THR%FXPLATFORM = -tiner(1,1)
  T%THR%FYPLATFORM = -tiner(2,1)
  T%THR%FZPLATFORM = -tiner(3,1)
      
  ! Compute Bead State Derivatives
  
  xcgdotdot(1:T%THR%NBEADS) = (T%THR%FXGRAV(1:T%THR%NBEADS)+T%THR%FXELAS(1:T%THR%NBEADS)+T%THR%FXAERO(1:T%THR%NBEADS) )/T%THR%EMASS
  ycgdotdot(1:T%THR%NBEADS) = (T%THR%FYGRAV(1:T%THR%NBEADS)+T%THR%FYELAS(1:T%THR%NBEADS)+T%THR%FYAERO(1:T%THR%NBEADS) )/T%THR%EMASS
  zcgdotdot(1:T%THR%NBEADS) = (T%THR%FZGRAV(1:T%THR%NBEADS)+T%THR%FZELAS(1:T%THR%NBEADS)+T%THR%FZAERO(1:T%THR%NBEADS) )/T%THR%EMASS

  ! Compute Tension State Derivatives

  ! T%THR%KU = 10000
  ! T%THR%CU = 50000
  ! T%THR%SIGMA = 30

  ! write(*,*) 'Tether Parameters = ',T%THR%KU,T%THR%CU,T%THR%ELEN,T%THR%SIGMA
! write(*,*) 'ELEN ===',ELEN

! write(*,*) 'T%THR%ELEN ===',T%THR%ELEN
  do i=1,T%THR%NBEADS+1
     if (dl(i) .gt. T%THR%ELEN) then
      !!! REVISIT REVISIT REVISIT - C Montalvo - 10/28/2015 - Experimenting with adding beads during pay out
        if (i .eq. 1) then
           ELEN = T%THR%LEN - T%THR%NBEADS*(T%THR%ELEN)
           if (ELEN .ge. 2*T%THR%ELEN) then
              T%THR%ADDBEAD = 1
           end if
           if (ELEN .lt. 0.99*T%THR%ELEN) then
              T%THR%REMOVEBEAD = 1
           end if
        else
           ELEN = T%THR%ELEN
           ! T%THR%SIGMA = T%THR%EKV/T%THR%ECV
           ! T%THR%KU = T%THR%EKV*T%THR%EKE/T%THR%ECV
           ! T%THR%CU = T%THR%EKE + T%THR%EKV
        end if
        tensiondot(i) = T%THR%KU*(dl(i)-ELEN) + T%THR%CU*dldot(i) - T%THR%SIGMA*tension(i)
        if ((tension(i) .le. 0 ) .and. (tensiondot(i) .lt. 0)) then
          tensiondot(i) = 0
        end if
        ! write(*,*) 'Bead number',i
        ! write(*,*) 'tensiondot(i)=',tensiondot(i)
        ! write(*,*) 'KU, CU, SI term===',T%THR%KU*(dl(i)-ELEN),T%THR%CU*dldot(i),-T%THR%SIGMA*tension(i)
        ! write(*,*) 'Tdot, T, dl, dldot = ',tensiondot(i),tension(i),dl(i),dldot(i)
        ! write(*,*) 'Tdot, T, dl, dldot = ',tensiondot(i),tension(i),dl(i),dldot(i)
     else
        ! This is the slack condition
        tensiondot(i) = -T%THR%SIGMA*tension(i) 
     end if
  end do
  
  ! PAUSE

  ! Wrap State Vector
    
  stateindex = 0
  do i=1,T%THR%NBEADS
   T%THR%STATEDOT(stateindex+1) = xcgdot(i) ! X Dot 
   T%THR%STATEDOT(stateindex+2) = ycgdot(i) ! Y Dot
   T%THR%STATEDOT(stateindex+3) = zcgdot(i) ! Z Dot
   T%THR%STATEDOT(stateindex+4) = xcgdotdot(i) ! X Dot Dot
   T%THR%STATEDOT(stateindex+5) = ycgdotdot(i) ! Y Dot Dot
   T%THR%STATEDOT(stateindex+6) = zcgdotdot(i) ! Z Dot Dot
   stateindex = stateindex + 6
  end do
  do i=1,T%THR%NBEADS+1
   T%THR%STATEDOT(stateindex+1) = tensiondot(i)! Tension Dot 
   stateindex = stateindex + 1
  end do

  !!Reel Dynamics

  if (T%SIM%ACTUATORONOFF .eq. 1) then
     T%THR%THETAREEL = T%SIM%ACTUATOR(3)
     T%THR%THETADOTREEL = T%SIM%ACTUATOR(4)
     T%THR%TORQUE = T%SIM%ACTUATOR(5)

     thetadot = T%THR%THETADOTREEL
     thetaddot = (1/T%THR%IREEL)*(T%THR%TORQUE+tension(1)*T%THR%RREEL)
     T%THR%TENSION = tension(1)

     T%SIM%ACTUATORDOT(3) = thetadot
     T%SIM%ACTUATORDOT(4) = thetaddot
     T%SIM%ACTUATORDOT(6) = (T%THR%THETACOMMAND-T%THR%THETAREEL)

     !Torque Dynamics
     T%SIM%ACTUATORDOT(5) = T%THR%TAU*(T%THR%TORQUECOMMAND-T%THR%TORQUE)

  end if

  RETURN
  
 end if
  
!!!!!!!!!!!!!!!!!!!!!!!!!!!!! LOAD DATA iflag = 1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 
 if (iflag .eq. 1) then
 
  open(unit=94,file=T%TETHERINPUTFILE,status='old',iostat=openflag)
  if (openflag .ne. 0) then
   write(*,*) 'Error Opening Tether Input File: ',T%TETHERINPUTFILE; PAUSE; STOP
  end if
  rewind(94)
  
  read(unit=94,fmt=*,iostat=readflag) T%THR%DYNOFFON
  read(unit=94,fmt=*,iostat=readflag) T%THR%GRAVOFFON
  read(unit=94,fmt=*,iostat=readflag) T%THR%AEROFLAG
  read(unit=94,fmt=*,iostat=readflag) T%THR%ELASOFFON
  read(unit=94,fmt=*,iostat=readflag) T%THR%NBEADS
  read(unit=94,fmt=*,iostat=readflag) T%THR%MASSPUL
  read(unit=94,fmt=*,iostat=readflag) T%THR%LEN
  read(unit=94,fmt=*,iostat=readflag) T%THR%LENMAX
  read(unit=94,fmt=*,iostat=readflag) T%THR%LENMIN
  read(unit=94,fmt=*,iostat=readflag) T%THR%DIA
  read(unit=94,fmt=*,iostat=readflag) T%THR%KE
  read(unit=94,fmt=*,iostat=readflag) T%THR%KV
  read(unit=94,fmt=*,iostat=readflag) T%THR%CV  
  read(unit=94,fmt=*,iostat=readflag) T%THR%GP
  read(unit=94,fmt=*,iostat=readflag) T%THR%GD
  read(unit=94,fmt=*,iostat=readflag) T%THR%NONLINEAR
  read(unit=94,fmt=*,iostat=readflag) T%THR%NU
  read(unit=94,fmt=*,iostat=readflag) T%THR%CD_AXIAL
  read(unit=94,fmt=*,iostat=readflag) T%THR%CD_NORMAL
  read(unit=94,fmt=*,iostat=readflag) T%THR%IREEL
  read(unit=94,fmt=*,iostat=readflag) T%THR%RREEL
  read(unit=94,fmt=*,iostat=readflag) T%THR%TAU

  close(94) 

  ! Compute Bead Properties

  T%THR%NOMLEN = T%THR%LEN
  T%THR%PREVLEN = T%THR%LEN
  T%THR%PREVLENCOMMAND = T%THR%LEN
  call TETHERPROPERTIES(T)
  
  ! write(*,*) T%THR%KU,T%THR%CU,T%THR%SIGMA
  ! PAUSE
  
  write(*,*) 'TETHER Load Complete'

  T%THR%DQFLAG = 1

  RETURN

 end if

 RETURN
END SUBROUTINE TETHER

SUBROUTINE TETHERPROPERTIES(T)
 use TOSIMDATATYPES
 implicit none
 type(TOSIMSTRUCTURE) T
 integer i
 real*8 J,stateindex,old_first_bead_state(6),old_first_bead_tension

  !Add a bead if it needs it
  if (T%THR%ADDBEAD .eq. 1) then

     !Call TETHER model to make sure tether is initialized properly
     call TETHER(T,3) 
     
     !Make sure to set ADDBEAD to 0 so that you don't add an extra bead again
     T%THR%ADDBEAD = 0
     !First Pass State of Tether from SIM%STATE to THR%STATE
     stateindex = 25
     T%THR%STATE(1:7*T%THR%NBEADS+1) = T%SIM%STATE(stateindex+1:stateindex+7*T%THR%NBEADS+1)

     !Current Tether State
     ! write(*,*) 'Current Tether State = ',T%THR%STATE(1:7*T%THR%NBEADS+1)

     !Grab entire first bead state
     old_first_bead_state(1:6) = T%THR%STATE(1:6) !this includes all 6 states
     old_first_bead_tension = T%THR%STATE(6*T%THR%NBEADS+1)

     ! write(*,*) 'Entire First Bead = ',old_first_bead_state(1:6)
     ! write(*,*) 'Tension of First Bead = ',old_first_bead_tension
     ! write(*,*) 'Reel Location = ',T%DRIVER%XREEL,T%DRIVER%YREEL,T%DRIVER%ZREEL,T%DRIVER%XREELDOT,T%DRIVER%YREELDOT,T%DRIVER%ZREELDOT

     !The weird thing about the tether model is that all states are in the first part of the vector and 
     !the tensions are all at the end so you need to shift the tensions down first
     T%THR%STATE(6*(T%THR%NBEADS+1)+2:7*(T%THR%NBEADS+1)+1) = T%THR%STATE(6*T%THR%NBEADS+1:7*T%THR%NBEADS+1)

     !For debugging set what you just shifted to zero
     T%THR%STATE(6*T%THR%NBEADS+1:7*T%THR%NBEADS+1) = 0.0
     ! write(*,*) 'After Tension Shift = ',T%THR%STATE(1:7*(T%THR%NBEADS+1)+1)

     !Then shift all states down by 1 bead. You have to do this second because you end up overwriting alot of the vector
     T%THR%STATE(7:6*(T%THR%NBEADS+1)) = T%THR%STATE(1:6*T%THR%NBEADS)

     !For debugging zero out the first bead
     T%THR%STATE(1:6) = 0.0
     ! write(*,*) 'After State Shift = ',T%THR%STATE(1:7*(T%THR%NBEADS+1)+1)

     !Now we need to initialize the new first beads by averaging the state of the now second bead and the state of the connection point on the Driver
     T%THR%STATE(1) = 0.5*(old_first_bead_state(1) + T%DRIVER%XREEL)
     T%THR%STATE(2) = 0.5*(old_first_bead_state(2) + T%DRIVER%YREEL)
     T%THR%STATE(3) = 0.5*(old_first_bead_state(3) + T%DRIVER%ZREEL)
     !The velocity of the bead should be the same as the first bead
     T%THR%STATE(4) = old_first_bead_state(4)
     T%THR%STATE(5) = old_first_bead_state(5)
     T%THR%STATE(6) = old_first_bead_state(6)

     !Because we can't average the tension state we're just going to have to save the one from the first bead (Hopefully this will work)
     T%THR%STATE(6*(T%THR%NBEADS+1)+1) = old_first_bead_tension

     !Finally increment the number of beads by 1
     T%THR%NBEADS = T%THR%NBEADS + 1

     !Debugging Purposes
     !New Tether State
     ! write(*,*) 'New Tether State = ',T%THR%STATE(1:7*T%THR%NBEADS+1)
     ! write(*,*) 'Current Time = ',T%SIM%TIME
     ! write(*,*) 'Length of Tether = ',T%THR%LEN
     write(*,*) 'Number of Beads = ',T%THR%NBEADS

     !Then pass the tether state back to the sim state
     T%SIM%STATE(stateindex+1:stateindex+7*T%THR%NBEADS+1) = T%THR%STATE(1:7*T%THR%NBEADS+1)

     !Increase the number of states so the simulation can run correctly
     T%SIM%NOSTATES = 13 + 12 + 7*T%THR%NBEADS + 1

     !Re-Run the TETHER and then SYSTEMDERIVATIVES routine to get the derivatives shifted correctly
     call TETHER(T,3)
     call SYSTEMDERIVATIVES(T,2)
  end if

  !Remove a bead if it needs it
  if (T%THR%REMOVEBEAD .eq. 1) then
     !Call TETHER model to make sure tether is initialized properly
     call TETHER(T,3) 
     
     !Make sure to set REMOVEBEAD to 0 so that you don't remove an extra bead again
     T%THR%REMOVEBEAD = 0
     !First Pass State of Tether from SIM%STATE to THR%STATE
     stateindex = 25
     T%THR%STATE(1:7*T%THR%NBEADS+1) = T%SIM%STATE(stateindex+1:stateindex+7*T%THR%NBEADS+1)

     !Decrement the number of beads by 1
     T%THR%NBEADS = T%THR%NBEADS - 1 

     !The weird thing about the tether model is that all states are in the first part of the vector and 
     !the tensions are all at the end so you need to shift the states up first.
     T%THR%STATE(1:6*T%THR%NBEADS) = T%THR%STATE(7:6*(T%THR%NBEADS+1))

     !Then shift all the tensions up by 1 bead
     T%THR%STATE(6*T%THR%NBEADS+1:7*T%THR%NBEADS+1) = T%THR%STATE(6*(T%THR%NBEADS+1)+2:7*(T%THR%NBEADS+1)+1)

     write(*,*) 'Number of Beads = ',T%THR%NBEADS

     !Then pass the tether state back to the sim state
     T%SIM%STATE(stateindex+1:stateindex+7*T%THR%NBEADS+1) = T%THR%STATE(1:7*T%THR%NBEADS+1)

     !Decrease the number of states so the simulation can run correctly
     T%SIM%NOSTATES = 13 + 12 + 7*T%THR%NBEADS + 1

     !Re-Run the TETHER and then SYSTEMDERIVATIVES routine to get the derivatives shifted correctly
     call TETHER(T,3)
     call SYSTEMDERIVATIVES(T,2)
  end if
  
  T%THR%AREA = PI*(T%THR%DIA/2)**2
  T%THR%EMASS = T%THR%MASSPUL*T%THR%LEN/(T%THR%NBEADS)
  T%THR%ELEN = T%THR%LEN/(T%THR%NBEADS+1)
  
  ! REVISIT REVISIT REVISIT - C Montalvo - 10/28/2015 Testing ELEN being Constant and adding beads
  ! T%THR%ELEN = 12.25 ! Keep ELEN constant and only pay out 1st bead
  ! T%THR%EMASS = 0.000740618666666667 !Keep EMASS CONSTANT as well

  T%THR%EKE = T%THR%KE*T%THR%AREA/T%THR%ELEN 
  T%THR%EKV = T%THR%KV*T%THR%AREA/T%THR%ELEN 
  T%THR%ECV = T%THR%CV*T%THR%AREA/T%THR%ELEN   

  T%THR%SIGMA = T%THR%EKV/T%THR%ECV
  T%THR%KU = T%THR%EKV*T%THR%EKE/T%THR%ECV
  T%THR%CU = T%THR%EKE + T%THR%EKV

  !write(*,*) T%THR%EKE,T%THR%EKV,T%THR%ECV,T%THR%SIGMA,T%THR%KU,T%THR%CU
  !STOP

  !Torsional Stiffness
  J = (PI*(T%THR%DIA/2)**4)/2 !!ft^4
  T%THR%KP = 0.0
  T%THR%KD = 0.0
  if (T%THR%NONLINEAR .ge. 0) then !if it's -1 the numbers are zero
     if (T%THR%NONLINEAR .eq. 0) then
        T%THR%KP = T%THR%GP*J/T%THR%LEN !! lbf-ft/rad = ? * ft^4 / ft = ? * ft^3
        T%THR%KD = T%THR%GD*J/T%THR%LEN !! lbf-ft-s/rad = ? * ft^4 / ft = ? * ft^3
     else
        T%THR%KP = 0.0005671*T%THR%GP*(J/T%THR%LEN)**0.59 !! lbf-ft/rad = ? * ft^4 / ft = ? * ft^3
        T%THR%KD = 0.05344*T%THR%GD*(J/T%THR%LEN)**0.5989 !! lbf-ft/rad = ? * ft^4 / ft = ? * ft^3
     end if
  end if

  !GP => lbf/(ft^2-rad)
  !GD => lbf-s/(ft^2-rad)

END SUBROUTINE TETHERPROPERTIES

SUBROUTINE RandGaussian(xg)
  implicit none
  real*8 u1,u2,r,fac
  real*8 xu1,xu2,xg
  
  !c.... Convert Uniform Random Numbers to Gaussian Random Numbers

  r = 2.00000000
  do while (r .ge. 1.00000000)

     !c..... Obtain Uniform Random Numbers Between Zero and One

     call RandUniform(xu1)
     call RandUniform(xu2)

     !c..... Generate Uniform Random Numbers Between Minus One and Plus One
     
     u1 = 2.00000000*xu1 - 1.00000000
     u2 = 2.00000000*xu2 - 1.00000000

     !.. Check if Numbers are in Unit Circle

     r = u1**2 + u2**2
  enddo

  !. Convert Uniform Random Numbers to Gaussian Random Numbers

  fac = sqrt(-2.00000000*log(r)/r)
  xg = u1*fac

  RETURN
END SUBROUTINE

!. SUBROUTINE RandUniform

SUBROUTINE RandUniform(xu)
  implicit none
  integer i,j,iff,m1,m2,m3,ia1,ia2,ia3,ic1,ic2,ic3,ix1,ix2,ix3
  real*8 rm1,rm2,xu
  real*8 r(97)
  save

  !. Basic Algorithm Data

  parameter(m1=259200,ia1=7141,ic1=54773,rm1=1./m1)
  parameter(m2=134456,ia2=8121,ic2=28411,rm2=1./m2)
  parameter(m3=243000,ia3=4561,ic3=51349)
  data iff /0/

  !. Initialization

  if (iff .eq. 0) then
     iff = 1
     ix1 = mod(ic1-1,m1)
     ix1 = mod(ia1*ix1+ic1,m1)
     ix2 = mod(ix1,m2)
     ix1 = mod(ia1*ix1+ic1,m1)
     ix3 = mod(ix1,m3)
     do j=1,97
        ix1 = mod(ia1*ix1+ic1,m1)
        ix2 = mod(ia2*ix2+ic2,m2)
        r(j) = (float(ix1)+float(ix2)*rm2)*rm1
     enddo
  endif

  !. Generate Uniform Random Number

  do i=1,2
     ix1 = mod(ia1*ix1+ic1,m1)
     ix2 = mod(ia2*ix2+ic2,m2)
     ix3 = mod(ia3*ix3+ic3,m3)
     j = 1 + (97*ix3)/m3
     if ((j.gt.97) .or. (j.lt.1)) then
        j = 43
        write(*,*) ' '
        write(*,*) 'ERROR IN RANDUNIFORM'
        write(*,*) ' '
     endif
     xu = r(j)
     r(j) = (float(ix1)+float(ix2)*rm2)*rm1
  enddo

  RETURN
END SUBROUTINE RandUniform

!!Import routine to load data files placed in text files
SUBROUTINE IMPORTWIND(mat,filename,dim)
  implicit none
  integer ii,jj,kk,nii,njj,ierr
  integer, intent (in) :: dim               ! DK 8/10/2015 
  !CM 8/16/2015 - Same thing. These text files will never change. They will always have a dimension of 40
  !DK 8/17/2015 - Had to do this for Intel Fortran compiler, it's useful anyways because it makes your subroutine more reuseable for the future. 
  real*8 mat(dim,dim,dim),tempmat(dim)
  character*256 filename

  write(*,*) 'Importing: ',filename
  
  open(unit=78,file=filename,status='old',iostat=ierr)
  if (ierr .ne. 0) then 
     write(*,*) 'Wind File defined incorrectly'
     write(*,*) filename
     PAUSE;
     STOP;
  endif
  do jj=1,dim
     do ii=1,dim
        read(78,*) tempmat
        do kk=1,dim
           mat(ii,kk,jj) = tempmat(kk)
        enddo
     enddo
  enddo
  close(78)

END SUBROUTINE IMPORTWIND

SUBROUTINE IMPORTTURB(outmat,filename,dim) 
  integer ii, jj, kk, nii, njj
  integer, intent(in)::dim
  character*256 filename
  real*8 outmat(dim,dim),temp(dim)

  open(unit=78,file=filename,status='old',iostat=ierr)
  if (ierr .ne. 0) then
     write(*,*) 'Turbulence Data File defined incorrectly'
     write(*,*) filename
     PAUSE; STOP;
  end if
  do ii = 1,dim
     read(78,*) temp
     do jj = 1,dim
        outmat(ii,jj) = temp(jj)
     end do
  end do
  close(78)

END SUBROUTINE IMPORTTURB

!!!FIND FUNCTIONS FOR ATMOSPHERE MODEL

SUBROUTINE FINDGE(invec,row,val,counter)
  implicit none
  integer idx,counter,row;
  real*8 invec(row),val
  idx = 1;
  counter = row;
  do while (idx .lt. row)
     if ((invec(idx) .ge. val)) then
        counter = idx-1;
        idx = row + 1;
     endif
     idx = idx + 1
  enddo
END SUBROUTINE FINDGE

SUBROUTINE FINDGT(invec,row,val,counter)
  implicit none
  integer, intent(in):: row
  real*8,  intent(in)::invec(row)
  integer :: idx,counter
  real*8  :: val
  idx = 1;
  counter = row;
  do while (idx .lt. row)
     if ((invec(idx) .gt. val)) then
        counter = idx-1;
        idx = row + 1;
     endif
     idx = idx + 1
  enddo
END SUBROUTINE FINDGT

SUBROUTINE FIND2(invec,row,val,counter)
  implicit none
  integer, intent(in):: row, invec(row)
  integer idx,counter
  real*8 val
  idx = 1;
  counter = row;
  do while (idx .lt. row)
     if ((invec(idx) .gt. val)) then
        counter = idx-1;
        idx = row + 1;
     endif
     idx = idx + 1
  enddo
END SUBROUTINE FIND2

SUBROUTINE WRFMODEL(T)
  use TOSIMDATATYPES
  !USE Math_Module, ONLY: M2FT, FT2M - Archaic from AREA-I days
  implicit none
  integer stepX,stepY,stepZ,stepT,extrapX,extrapY,extrapZ,extrapT,cord2(2)
  integer markX,markY,markZ,markT,gust,body
  integer tinterp,x1,x2,y1,y2,z1,z2,tt,ii,coord1(4),coord2(4),cord1(2)
  real*8 uvw(3,2),xpts2(2),ypts2(2),zpts2(2),zpts1,xpts1,ypts1,rx;
  real*8 u8(8),v8(8),w8(8),u4(4),v4(4),w4(4),uslope,vslope,wslope;
  real*8 u2(2),v2(2),w2(2),u,v,w,tpts(2),Lu,Lv,Lw,sigw,sigu,sigv,tstar;
  real*8 ugo,vgo,wgo,vatm(3),xstar,ystar,zstar,xtemp,vtemp,counter,xwidth,ywidth
  real*8 xi,yi,zi,xi_ft,yi_ft,zi_ft,noise
  character*1 letter
  character*10 number
  type(TOSIMSTRUCTURE) T

  !   /*   %%This function will take in x,y,z(ft),t(sec) and location and  */
  !   /*   %return u,v,w(ft/s). This uses a fast quad-linear interpolation */
  !   /*   %so many globals must be defined. location is a string that */
  !   /*   %contains the location of the data to be interpolated. */

  xi_ft = T%ATM%XI 
  yi_ft = T%ATM%YI
  zi_ft = 656-T%ATM%ZI !REVISIT REVISIT REVISIT - Assume that the Driver is at 200 meters

  xtemp =  xi_ft
  xi_ft =  xtemp*cos(T%ATM%PSIOFFSET) + yi_ft*sin(T%ATM%PSIOFFSET);
  yi_ft = -xtemp*sin(T%ATM%PSIOFFSET) + yi_ft*cos(T%ATM%PSIOFFSET);

  !!Extra term is from phase offset to add time-varying component
  xi_ft = xi_ft - T%ATM%WAVESPEED(1)*(T%SIM%TIME-T%SIM%INITIALTIME) + T%ATM%XOFFSET
  yi_ft = yi_ft - T%ATM%WAVESPEED(2)*(T%SIM%TIME-T%SIM%INITIALTIME) + T%ATM%YOFFSET

  xi = xi_ft*FT2M   !Convert from feet to meters
  yi = yi_ft*FT2M
  zi = zi_ft*FT2M

  !Now shift the WRF Grid so that XI,YI and ZI fall inside the cube
  T%ATM%xshift = 0
  T%ATM%yshift = 0

  xwidth = T%ATM%xcoord(T%ATM%dim)-T%ATM%xcoord(1)
  ywidth = T%ATM%ycoord(T%ATM%dim)-T%ATM%ycoord(1)

  !%Find markX
  if (xi .gt. T%ATM%xcoord(T%ATM%dim)) then
     T%ATM%xshift = -floor(abs(xi-T%ATM%xcoord(1))/xwidth)
  else if (xi .lt. T%ATM%xcoord(1)) then 
     T%ATM%xshift = floor(abs(xi-T%ATM%xcoord(T%ATM%dim))/xwidth)
  end if

  !%Find markY
  if (yi .gt. T%ATM%ycoord(T%ATM%dim)) then
     T%ATM%yshift = -floor(abs(yi-T%ATM%ycoord(1))/ywidth)
  else if (yi .lt. T%ATM%ycoord(1)) then 
     T%ATM%yshift = floor(abs(yi-T%ATM%ycoord(T%ATM%dim))/ywidth)
  end if

  xstar = xi + T%ATM%xshift*xwidth
  ystar = yi + T%ATM%yshift*ywidth
  zstar = -zi

  ! if (abs(xstar) .gt. T%ATM%xcoord(T%ATM%dim)) then
  !    write(*,*) 'x = ',xi,xstar
  !    PAUSE
  ! end if
  ! if (abs(ystar) .gt. T%ATM%ycoord(T%ATM%dim)) then
  !    write(*,*) 'y = ',yi,ystar
  !    PAUSE
  ! end if

  !write(*,*) xi,xstar

  if (T%ATM%IWINDSCALE .gt. 0) then
     tstar = T%SIM%TIME
     stepX = 1;stepY = 1;stepZ = 1;stepT = 1;
     extrapX = 0;extrapY = 0;extrapZ = 0;extrapT = 0;
     if (zstar .lt. 0) then
        zstar = -zstar
     endif
     tinterp = 2;

     uvw(1,1)=0;uvw(2,1)=0;uvw(3,1)=0;
     uvw(1,2)=0;uvw(2,2)=0;uvw(3,2)=0;

     markX = T%ATM%markX
     markY = T%ATM%markY
     markZ = T%ATM%markZ
     markT = T%ATM%markT

     !%%Check X
     if (markX .eq. T%ATM%dim) then
        markX = markX - 1;
     end if
     if ((xstar .ge. T%ATM%xcoord(markX)) .and. (xstar .le. T%ATM%xcoord(markX+1))) then
        !%%You're in between the markers so keep going
     else
        call FINDGT(T%ATM%xcoord,T%ATM%dim,xstar,markX)
        if (markX .eq. T%ATM%dim) then
           markX = markX - 1;
        else if (markX .le. 0) then
           markX = 1;
        end if
     end if
     !%%Check Y
     if (markY .eq. T%ATM%dim) then
        markY = markY - 1;
     end if
     if ((ystar .ge. T%ATM%ycoord(markY)) .and. (ystar .le. T%ATM%ycoord(markY+1))) then
        !%%You're in between the markers so keep going
     else
        call FINDGT(T%ATM%ycoord,T%ATM%dim,ystar,markY)
        if (markY .eq. T%ATM%dim) then
           markY = markY - 1;
        else if (markY .le. 0) then
           markY = 1;
        end if
     end if
     !%%Check Z
     if (markZ .eq. T%ATM%dim) then
        markZ = markZ - 1;
     end if

     if ((zstar .ge. T%ATM%zcoord(markZ)) .and. (zstar .le. T%ATM%zcoord(markZ+1))) then
        !%%You're in between the markers so keep going
     else
        !%Find markZ
        if (zstar .gt. T%ATM%zcoord(T%ATM%dim)) then
           !%use endpt
           markZ = T%ATM%dim;
           stepZ = -1;
           extrapZ = 1;
        else if (zstar .lt. T%ATM%zcoord(1)) then
           markZ = 1;
           stepZ = 1;
           extrapZ = 1;
        else
           call FINDGT(T%ATM%zcoord,T%ATM%dim,zstar,markZ)
           if (markZ .eq. T%ATM%dim) then
              markZ = markZ - 1;
           else if (markZ .eq. 0) then
              markZ = 1;
           end if
        end if
     end if
     !%%Check T
     if (markT .eq. T%ATM%tlength) then
        markT = markT - 1;
     end if
     if ((tstar .ge. T%ATM%tcoord(markT)) .and. (tstar .le. T%ATM%tcoord(markT+1))) then
        !%%You're in between the markers so keep going
     else
        if (T%ATM%TIMEVARYING .eq. 1) then
           !%Find markT
           if (tstar .gt. T%ATM%tcoord(T%ATM%tlength)) then
              !%use endpt
              markT = T%ATM%tlength;
              extrapT = 1;
           else if (tstar .lt. T%ATM%tcoord(1)) then
              !%use start pt
              markT = 1;
              extrapT = 1;
           else
              call FIND2(T%ATM%tcoord,T%ATM%tlength,tstar,markT)
              if (markT .eq. T%ATM%tlength) then
                 markT = markT - 1;
              else if (markT .eq. 0) then
                 markT = 1;
              end if
           end if

           !%%Import U,V,W maps since markT changed
           if (T%ATM%tcoord(markT) .lt. 10) then
              write(number, '(i1)' )  T%ATM%tcoord(markT)
           else
              if (T%ATM%tcoord(markT) .le. 99) then
                 write(number, '(i2)' )  T%ATM%tcoord(markT)
              else
                 write(number, '(i3)' )  T%ATM%tcoord(markT)
              endif
           endif
           letter = trim('U')
           T%ATM%U0name = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
           letter = trim('V')
           T%ATM%V0name = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
           letter = trim('W')
           T%ATM%W0name = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
           !%%only import at markT
           call IMPORTWIND(T%ATM%U0,T%ATM%U0name,40); !REVISIT HARDCODED DIMENSIONS, MAKE A GLOBAL OR SOMETHING TO DEFINE SIZES PROPERLY - DK 8/10/2015
           call IMPORTWIND(T%ATM%V0,T%ATM%V0name,40); !CM 8/16/2015 - Same thing. These text files will never change. They will always have a dimension of 40
           call IMPORTWIND(T%ATM%W0,T%ATM%W0name,40);
           !%U0 = U0(end:-1:1,end:-1:1,:);
           !%V0 = V0(end:-1:1,end:-1:1,:);
           !%W0 = W0(end:-1:1,end:-1:1,:);
           if (extrapT .eq. 1) then
              tinterp = 1;
           else
              !%%import markT + 1
              if (T%ATM%tcoord(markT+1) .lt. 10) then
                 write(number, '(i1)' )  T%ATM%tcoord(markT+1)
              else if (T%ATM%tcoord(markT+1) .le. 99) then
                 write(number, '(i2)' )  T%ATM%tcoord(markT+1)
              else
                 write(number, '(i3)' )  T%ATM%tcoord(markT+1)
              endif
              letter = trim('U')
              T%ATM%Udtname = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
              letter = trim('V')
              T%ATM%Vdtname = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
              letter = trim('W')
              T%ATM%Wdtname = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
              call IMPORTWIND(T%ATM%Udt,T%ATM%Udtname,40);
              call IMPORTWIND(T%ATM%Vdt,T%ATM%Vdtname,40);
              call IMPORTWIND(T%ATM%Wdt,T%ATM%Wdtname,40);
           end if !(extrapT eq. 1 ) 
        else !Not time varyine
           tinterp = 1
        end if
     end if

     !%%Interpolation Scheme
     do tt = 1,tinterp 
        !%Interpolate Spatially

        !%%To start we have 8 discrete point (8 corners of a cube)
        xpts2(1) = T%ATM%xcoord(markX)
        xpts2(2) = T%ATM%xcoord(markX+stepX);
        ypts2(1) = T%ATM%ycoord(markY)
        ypts2(2) = T%ATM%ycoord(markY+stepY);
        zpts2(1) = T%ATM%zcoord(markZ)
        zpts2(2) = T%ATM%zcoord(markZ+stepZ);
        x1 = markX;x2 = markX+stepX;
        y1 = markY;y2 = (markY+stepY);
        z1 = markZ;z2 = markZ+stepZ;
        if (tt .eq. 1) then
           !%%Use U0,V0,W0
           u8(1) = T%ATM%U0(y1,x1,z1);
           u8(2) = T%ATM%U0(y1,x2,z1);
           u8(3) = T%ATM%U0(y2,x2,z1);
           u8(4) = T%ATM%U0(y2,x1,z1);
           u8(5) = T%ATM%U0(y1,x1,z2);
           u8(6) = T%ATM%U0(y1,x2,z2);
           u8(7) = T%ATM%U0(y2,x2,z2);
           u8(8) = T%ATM%U0(y2,x1,z2);
           v8(1) = T%ATM%V0(y1,x1,z1);
           v8(2) = T%ATM%V0(y1,x2,z1);
           v8(3) = T%ATM%V0(y2,x2,z1);
           v8(4) = T%ATM%V0(y2,x1,z1);
           v8(5) = T%ATM%V0(y1,x1,z2);
           v8(6) = T%ATM%V0(y1,x2,z2);
           v8(7) = T%ATM%V0(y2,x2,z2);
           v8(8) = T%ATM%V0(y2,x1,z2);
           w8(1) = T%ATM%W0(y1,x1,z1);
           w8(2) = T%ATM%W0(y1,x2,z1);
           w8(3) = T%ATM%W0(y2,x2,z1);
           w8(4) = T%ATM%W0(y2,x1,z1);
           w8(5) = T%ATM%W0(y1,x1,z2);
           w8(6) = T%ATM%W0(y1,x2,z2);
           w8(7) = T%ATM%W0(y2,x2,z2);
           w8(8) = T%ATM%W0(y2,x1,z2);
        else
           !%%Use Udt,Vdt,Wdt
           u8(1) = T%ATM%Udt(y1,x1,z1);
           u8(2) = T%ATM%Udt(y1,x2,z1);
           u8(3) = T%ATM%Udt(y2,x2,z1);
           u8(4) = T%ATM%Udt(y2,x1,z1);
           u8(5) = T%ATM%Udt(y1,x1,z2);
           u8(6) = T%ATM%Udt(y1,x2,z2);
           u8(7) = T%ATM%Udt(y2,x2,z2);
           u8(8) = T%ATM%Udt(y2,x1,z2);
           v8(1) = T%ATM%Vdt(y1,x1,z1);
           v8(2) = T%ATM%Vdt(y1,x2,z1);
           v8(3) = T%ATM%Vdt(y2,x2,z1);
           v8(4) = T%ATM%Vdt(y2,x1,z1);
           v8(5) = T%ATM%Vdt(y1,x1,z2);
           v8(6) = T%ATM%Vdt(y1,x2,z2);
           v8(7) = T%ATM%Vdt(y2,x2,z2);
           v8(8) = T%ATM%Vdt(y2,x1,z2);
           w8(1) = T%ATM%Wdt(y1,x1,z1);
           w8(2) = T%ATM%Wdt(y1,x2,z1);
           w8(3) = T%ATM%Wdt(y2,x2,z1);
           w8(4) = T%ATM%Wdt(y2,x1,z1);
           w8(5) = T%ATM%Wdt(y1,x1,z2);
           w8(6) = T%ATM%Wdt(y1,x2,z2);
           w8(7) = T%ATM%Wdt(y2,x2,z2);
           w8(8) = T%ATM%Wdt(y2,x1,z2);
        end if


        !%%%%%interpZ%%%%%%%%%%%%

        if (extrapZ .eq. 1) then
           !%%You don't need to interpolate on z and you can just use
           !%%the values at markZ or z1
           zpts1 = zpts2(1);
           u4(1) = u8(1);
           u4(2) = u8(2);
           u4(3) = u8(3);
           u4(4) = u8(4);
           v4(1) = v8(1);
           v4(2) = v8(2);
           v4(3) = v8(3);
           v4(4) = v8(4);
           w4(1) = w8(1);
           w4(2) = w8(2);
           w4(3) = w8(3);
           w4(4) = w8(4);
           T%ATM%bounds = 1;
        else
           !%%Interpolate Between Z points(interpolate pts 1-4 and 5-8)
           !%Pts 1,5 : 2,6 : 3,7 : 4,8
           coord1 = (/1,2,3,4/);
           coord2 = (/5,6,7,8/);
           do ii = 1,4
              uslope = (u8(coord2(ii))-u8(coord1(ii)))/(zpts2(2)-zpts2(1));
              vslope = (v8(coord2(ii))-v8(coord1(ii)))/(zpts2(2)-zpts2(1));
              wslope = (w8(coord2(ii))-w8(coord1(ii)))/(zpts2(2)-zpts2(1));
              u4(ii) = uslope*(zstar-zpts2(1))+u8(coord1(ii));
              v4(ii) = vslope*(zstar-zpts2(1))+v8(coord1(ii));
              w4(ii) = wslope*(zstar-zpts2(1))+w8(coord1(ii));
           end do
           zpts1 = zstar;
        end if

        !%%%%%interpY%%%%%%%%%%%

        if (extrapY .eq. 1) then
           !%%You don't need to interpolate on y
           ypts1 = ypts2(1);
           u2(1) = u4(1);
           u2(2) = u4(2);
           v2(1) = v4(1);
           v2(2) = v4(2);
           w2(1) = w4(1);
           w2(2) = w4(2);
           T%ATM%bounds = 1;
        else
           !%%Interpolate between Y points(interpolate pts 1-2 and 3-4)
           !%%Pts 1,4 : 2,3
           cord1 = (/1,2/);
           cord2 = (/4,3/);
           do ii = 1,2
              uslope = (u4(cord2(ii))-u4(cord1(ii)))/(ypts2(2)-ypts2(1));
              vslope = (v4(cord2(ii))-v4(cord1(ii)))/(ypts2(2)-ypts2(1));
              wslope = (w4(cord2(ii))-w4(cord1(ii)))/(ypts2(2)-ypts2(1));
              u2(ii) = uslope*(ystar-ypts2(1))+u4(cord1(ii));
              v2(ii) = vslope*(ystar-ypts2(1))+v4(cord1(ii));
              w2(ii) = wslope*(ystar-ypts2(1))+w4(cord1(ii));
           end do
           ypts1 = ystar;
        end if

        !%%%%interpX%%%%%%%%%%%%
        if (extrapX .eq. 1) then
           !%%You don't need to interpolate on x
           xpts1 = xpts2(1);
           u = u2(1);
           v = v2(1);
           w = w2(1);
           T%ATM%bounds = 1;
        else
           !%%Interpolate between X points
           uslope = (u2(2)-u2(1))/(xpts2(2)-xpts2(1));
           vslope = (v2(2)-v2(1))/(xpts2(2)-xpts2(1));
           wslope = (w2(2)-w2(1))/(xpts2(2)-xpts2(1));
           u = uslope*(xstar-xpts2(1))+u2(1);
           v = vslope*(xstar-xpts2(1))+v2(1);
           w = wslope*(xstar-xpts2(1))+w2(1);
           xpts1 = xstar;
        end if

        !%%%%Save wind values%%%%%

        uvw(1,tt) = u;
        uvw(2,tt) = v;
        uvw(3,tt) = w;

     end do

     if (T%ATM%TIMEVARYING .eq. 1) then
        if (extrapT .eq. 1) then
           !//Answer is just first entry of uvw
           vatm(1) = uvw(1,1);
           vatm(2) = uvw(2,1);
           vatm(3) = uvw(3,1);
        else
           !%%Interpolate on T
           tpts(1) = T%ATM%tcoord(markT)
           tpts(2) = T%ATM%tcoord(markT+1);
           u2(1) = uvw(1,1);
           u2(2) = uvw(1,2);
           v2(1) = uvw(2,1);
           v2(2) = uvw(2,2);
           w2(1) = uvw(3,1);
           w2(2) = uvw(3,2);
           uslope = (u2(2)-u2(1))/(tpts(2)-tpts(1));
           vslope = (v2(2)-v2(1))/(tpts(2)-tpts(1));
           wslope = (w2(2)-w2(1))/(tpts(2)-tpts(1));  
           u = uslope*(tstar-tpts(1))+u2(1);
           v = vslope*(tstar-tpts(1))+v2(1);
           w = wslope*(tstar-tpts(1))+w2(1);
           vatm(1) = u;
           vatm(2) = v;
           vatm(3) = w;
        end if
     else
        !//Answer is just first entry of uvw
        vatm(1) = uvw(1,1);
        vatm(2) = uvw(2,1);
        vatm(3) = uvw(3,1);
     end if
     
     !Rotate by PSIOFFSET
     vtemp = vatm(1)
     vatm(1) = vtemp*cos(T%ATM%PSIOFFSET) + vatm(2)*sin(T%ATM%PSIOFFSET);
     vatm(2) = -vtemp*sin(T%ATM%PSIOFFSET) + vatm(2)*cos(T%ATM%PSIOFFSET);

     !//Multiply by scale
     call RandGaussian(noise)
     vatm(1) = T%ATM%IWINDSCALE*(1+T%ATM%RANDOMIZE*noise)*vatm(1)
     call RandGaussian(noise)
     vatm(2) = T%ATM%IWINDSCALE*(1+T%ATM%RANDOMIZE*noise)*vatm(2)
     call RandGaussian(noise)
     vatm(3) = T%ATM%IWINDSCALE*(1+T%ATM%RANDOMIZE*noise)*vatm(3)

     if ((T%ATM%bounds .eq. 1) .and. (T%ATM%boundflag .eq. 1)) then
        !write(*,*) 'You went out of bounds at T = ',tstar,' XYZ = ',xstar,ystar,zstar
        T%ATM%boundflag = 0;
     end if
  else
     vatm(1) = 0
     vatm(2) = 0
     vatm(3) = 0
  endif

  ! if (vatm(3) .gt. 4) then
  !    write(*,*) xi,xstar,T%ATM%xshift
  !    write(*,*) yi,ystar,T%ATM%yshift
  !    write(*,*) zi,zstar,T%ATM%zshift
  !    write(*,*) vatm(1),vatm(2),vatm(3)
  !    write(*,*) 'Ypts2 = ',ypts2
  !    write(*,*) u2
  !    write(*,*) v2
  !    write(*,*) w2
  !    PAUSE;STOP
  ! end if

  T%ATM%WRFX = vatm(1)*M2FT !Convert back to ft/s   //REVISIT - use math module, no hardcoded conversions
  T%ATM%WRFY = vatm(2)*M2FT
  T%ATM%WRFZ = vatm(3)*M2FT 

  T%ATM%markX = markX
  T%ATM%markY = markY
  T%ATM%markZ = markZ
  T%ATM%markT = markT

END SUBROUTINE WRFMODEL

SUBROUTINE TURBULENCE(T)
  use TOSIMDATATYPES
  !USE Math_Module, ONLY: FT2M, M2FT - Archaic from AREA-I days
  implicit none
  integer stepX,stepY,stepZ,stepT,extrapX,extrapY,extrapZ,extrapT,cord2(2)
  integer markX,markY,markZ,markT,gust,body
  integer tinterp,x1,x2,y1,y2,z1,z2,tt,ii,coord1(4),coord2(4),cord1(2)
  real*8 uvw(3,2),xpts2(2),ypts2(2),zpts2(2),zpts1,xpts1,ypts1,rx;
  real*8 u8(8),v8(8),w8(8),u4(4),v4(4),w4(4),uslope,vslope,wslope;
  real*8 u2(2),v2(2),w2(2),u,v,w,tpts(2),Lu,Lv,Lw,sigw,sigu,sigv,tstar;
  real*8 ugo,vgo,wgo,vatm(3),xstar,ystar,zstar,xtemp,vtemp,counter,noise
  character*1 letter
  character*10 number
  type(TOSIMSTRUCTURE) T

  !   /*   %%This function will take in x,y,z(ft),t(sec) and location and  */
  !   /*   %return u,v,w(ft/s). This uses a fast quad-linear interpolation */
  !   /*   %so many globals must be defined. location is a string that */
  !   /*   %contains the location of the data to be interpolated. */

  xtemp = T%ATM%XI
  T%ATM%XI = xtemp*cos(T%ATM%PSIOFFSET) + T%ATM%YI*sin(T%ATM%PSIOFFSET);
  T%ATM%YI = -xtemp*sin(T%ATM%PSIOFFSET) + T%ATM%YI*cos(T%ATM%PSIOFFSET);

  T%ATM%XI = T%ATM%XI*FT2M !Convert from feet to meters
  T%ATM%YI = T%ATM%YI*FT2M 
  T%ATM%ZI = T%ATM%ZI*FT2M 

  xstar = T%ATM%XI + T%ATM%xshiftT
  ystar = T%ATM%YI + T%ATM%yshiftT
  zstar = T%ATM%ZI + T%ATM%zshiftT

  if (T%ATM%TURBLEVEL .gt. 0) then
     stepX = 1;stepY = 1;stepZ = 1;stepT = 1;
     extrapX = 0;extrapY = 0;extrapZ = 0;extrapT = 0;
     if (zstar .lt. 0) then
        zstar = -zstar
     endif
     tinterp = 2;

     uvw(1,1)=0;uvw(2,1)=0;uvw(3,1)=0;
     uvw(1,2)=0;uvw(2,2)=0;uvw(3,2)=0;

     markX = T%ATM%markXT
     markY = T%ATM%markYT

     !%%Check X
     if (markX .eq. T%ATM%dimT) then
        markX = markX - 1;
     end if
     if ((xstar .ge. T%ATM%xcoordT(markX)) .and. (xstar .le. T%ATM%xcoordT(markX+1))) then
        !%%You're in between the markers so keep going
     else
        !%Find markX
        if (xstar .gt. T%ATM%xcoordT(T%ATM%dimT)) then
           !%use endpt
           markX = T%ATM%dimT
           stepX = -1
           extrapX = 1
           counter = 0
           do while ((T%ATM%xshiftT + T%ATM%XI .gt. T%ATM%xcoordT(T%ATM%dimT)) .and. (counter .lt. 20))
              T%ATM%xshiftT = -1.9 * T%ATM%xcoordT(T%ATM%dimT) + T%ATM%xshiftT
              counter = counter + 1
           end do
        else if (xstar .lt. T%ATM%xcoordT(1)) then 
           !%use starpt
           markX = 1;
           stepX = 1;
           extrapX = 1;    
           counter = 0
           do while ((T%ATM%xshiftT + T%ATM%XI .lt. T%ATM%xcoordT(1)) .and. (counter .lt. 20))
              T%ATM%xshiftT = 1.9 * T%ATM%xcoordT(T%ATM%dimT) + T%ATM%xshiftT
              counter = counter + 1
           end do
        else
           call FINDGT(T%ATM%xcoordT,T%ATM%dimT,xstar,markX)
           if (markX .eq. T%ATM%dimT) then
              markX = markX - 1;
           else if (markX .le. 0) then
              markX = 1;
           end if
        end if
     end if
     !%%Check Y
     if (markY .eq. T%ATM%dimT) then
        markY = markY - 1;
     end if
     if ((ystar .ge. T%ATM%ycoordT(markY)) .and. (ystar .le. T%ATM%ycoordT(markY+1))) then
        !%%You're in between the markers so keep going
     else
        !%Find markY
        if (ystar .gt. T%ATM%ycoordT(T%ATM%dimT)) then
           !%use endpt
           markY = T%ATM%dimT;
           stepY = -1;
           extrapY = 1;
           counter = 0
           do while ((T%ATM%yshiftT + T%ATM%YI .gt. T%ATM%ycoordT(T%ATM%dimT)) .and. (counter .lt. 20))
              T%ATM%yshiftT = -1.9 * T%ATM%ycoordT(T%ATM%dimT) + T%ATM%yshiftT
              counter = counter + 1
           end do
        else if (ystar .lt. T%ATM%ycoordT(1)) then
           markY = 1;
           stepY = 1;
           extrapY = 1;
           counter = 0
           do while ((T%ATM%yshiftT + T%ATM%YI .lt. T%ATM%ycoordT(1)) .and. (counter .lt. 20))
              T%ATM%yshiftT = 1.9 * T%ATM%ycoordT(T%ATM%dimT) + T%ATM%yshiftT
              counter = counter + 1;
           end do
        else
           call FINDGT(T%ATM%ycoordT,T%ATM%dimT,ystar,markY)
           if (markY .eq. T%ATM%dimT) then
              markY = markY - 1;
           else if (markY .le. 0) then
              markY = 1;
           end if
        end if
     end if

     !We start with 8 points
     !%%To start we have 4 discrete point (4 corners of a square)
     xpts2(1) = T%ATM%xcoordT(markX)
     xpts2(2) = T%ATM%xcoordT(markX+stepX);
     ypts2(1) = T%ATM%ycoordT(markY)
     ypts2(2) = T%ATM%ycoordT(markY+stepY);
     x1 = markX;x2 = markX+stepX;
     y1 = markY;y2 = (markY+stepY);
     !%%Use U0,V0,W0
     u4(1) = T%ATM%UTURB(y1,x1)
     u4(2) = T%ATM%UTURB(y1,x2)
     u4(3) = T%ATM%UTURB(y2,x2)
     u4(4) = T%ATM%UTURB(y2,x1)
     v4(1) = T%ATM%VTURB(y1,x1)
     v4(2) = T%ATM%VTURB(y1,x2)
     v4(3) = T%ATM%VTURB(y2,x2)
     v4(4) = T%ATM%VTURB(y2,x1)
     w4(1) = T%ATM%WTURB(y1,x1)
     w4(2) = T%ATM%WTURB(y1,x2)
     w4(3) = T%ATM%WTURB(y2,x2)
     w4(4) = T%ATM%WTURB(y2,x1)

     !%%%%%interpY%%%%%%%%%%%
     if (extrapY .eq. 1) then
        !%%You don't need to interpolate on y
        ypts1 = ypts2(1);
        u2(1) = u4(1);
        u2(2) = u4(2);
        v2(1) = v4(1);
        v2(2) = v4(2);
        w2(1) = w4(1);
        w2(2) = w4(2);
        T%ATM%boundsT = 1;
     else
        !%%Interpolate between Y points(interpolate pts 1-2 and 3-4)
        !%%Pts 1,4 : 2,3
        cord1 = (/1,2/);
        cord2 = (/4,3/);
        do ii = 1,2
           uslope = (u4(cord2(ii))-u4(cord1(ii)))/(ypts2(2)-ypts2(1));
           vslope = (v4(cord2(ii))-v4(cord1(ii)))/(ypts2(2)-ypts2(1));
           wslope = (w4(cord2(ii))-w4(cord1(ii)))/(ypts2(2)-ypts2(1));
           u2(ii) = uslope*(ystar-ypts2(1))+u4(cord1(ii));
           v2(ii) = vslope*(ystar-ypts2(1))+v4(cord1(ii));
           w2(ii) = wslope*(ystar-ypts2(1))+w4(cord1(ii));
           !REVISIT - can clean up with USE Math_Module, ONLY: linterp
           ! linterp(x1,y1,x2,y2,xin,yout)
           ! call linterp( ypts2(1),u4(cord1(ii)),ypts2(2),u4(cord2(ii)),ystar,u2(ii) )
           ! DK 8/18/2015
           ! Use functions to make your life easier without all this math everywhere :D
        end do
        ypts1 = ystar;
     end if

     !%%%%interpX%%%%%%%%%%%%
     if (extrapX .eq. 1) then
        !%%You don't need to interpolate on x
        xpts1 = xpts2(1);
        u = u2(1);
        v = v2(1);
        w = w2(1);
        T%ATM%boundsT = 1;
     else
        !%%Interpolate between X points
        uslope = (u2(2)-u2(1))/(xpts2(2)-xpts2(1));
        vslope = (v2(2)-v2(1))/(xpts2(2)-xpts2(1));
        wslope = (w2(2)-w2(1))/(xpts2(2)-xpts2(1));
        u = uslope*(xstar-xpts2(1))+u2(1);
        v = vslope*(xstar-xpts2(1))+v2(1);
        w = wslope*(xstar-xpts2(1))+w2(1);
        xpts1 = xstar;
     end if

     !%%%%Save wind values%%%%%

     vatm(1) = u
     vatm(2) = v
     vatm(3) = w

     !Rotate by PSIOFFSET
     vtemp = vatm(1)
     vatm(1) = vtemp*cos(T%ATM%PSIOFFSET) + vatm(2)*sin(T%ATM%PSIOFFSET);
     vatm(2) = -vtemp*sin(T%ATM%PSIOFFSET) + vatm(2)*cos(T%ATM%PSIOFFSET);

     !//Multiply by scale
     call RandGaussian(noise)
     vatm(1) = T%ATM%TURBLEVEL*((1+T%ATM%RANDOMIZE*noise))*vatm(1)
     call RandGaussian(noise)
     vatm(2) = T%ATM%TURBLEVEL*((1+T%ATM%RANDOMIZE*noise))*vatm(2)
     call RandGaussian(noise)
     vatm(3) = T%ATM%TURBLEVEL*((1+T%ATM%RANDOMIZE*noise))*vatm(3)

     if ((T%ATM%boundsT .eq. 1) .and. (T%ATM%boundflagT .eq. 1)) then
        !write(*,*) 'You went out of turbulence bounds at T = ',tstar,' XYZ = ',xstar,ystar,zstar
        T%ATM%boundflagT = 0;
     end if
  else
     vatm(1) = 0
     vatm(2) = 0
     vatm(3) = 0
  endif

  T%ATM%WINDGUST(1) = vatm(1)*M2FT  !Convert from m/s back to ft/s
  T%ATM%WINDGUST(2) = vatm(2)*M2FT
  T%ATM%WINDGUST(3) = vatm(3)*M2FT

  T%ATM%markXT = markX
  T%ATM%markYT = markY
  T%ATM%markZT = markZ

END SUBROUTINE TURBULENCE


