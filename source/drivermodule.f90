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
     integer :: CONTOFFON = 0                         ! Units: 'nd', Desc: 'Contact Flag (0=Off, 1=On)'
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
     integer :: THR_DYNOFFON = 0
     integer :: THR_ELASOFFON = 0
     integer :: CONTROLOFFON = 0
     integer :: WAYPOINT = 1                          ! Defaults to 1 but apparently can change in input file
     integer :: NUMWAYPOINTS = 0                         ! Defaults to 1 but apparently can change in input file
     integer :: COMMANDFLAG(MAXWP) = 1           ! 0 or 1 to advance waypoint command     real*8 :: COMMANDFLAG(MAXWP) =
     real*8 :: ALC = 0                                ! Driver aero parameter
     real*8 :: ALS = 0                                ! Driver aero parameter
     real*8 :: DXD = 0                                ! Driver aero parameter
     real*8 :: DYD = 0                                ! Driver aero parameter
     real*8 :: RNEW = 0                               ! Driver aero parameter
     real*8 :: C_T = 0                                ! Driver aero parameter
     real*8 :: C_TAU = 0                              ! Driver aero parameter
     real*8 :: LPHI12 = 0                             ! Driver aero parameter
     real*8 :: LPHI34 = 0                             ! Driver aero parameter
     real*8 :: LTHETA12 = 0                           ! Driver aero parameter
     real*8 :: LTHETA34 = 0                           ! Driver aero parameter
     real*8 :: OMEGAMAX = 0                           ! Driver aero parameter
     real*8 :: OMEGAVEC(4,1) = 0                      ! Driver aero parameter
     real*8 :: THRUSTVEC(4,1) = 0                     ! Driver aero parameter
     real*8 :: MUVEC(4,1) = 0                         ! Driver aero parameter
     real*8 :: KT = 0                                 ! Driver Aero Parameter
     real*8 :: OMEGA0 = 0                             ! Driver Aero Parameter
     real*8 :: IRR = 0                                ! Driver Aero Parameter
     real*8 :: MINWAYPOINT = 1.0D0
     real*8 :: MS_MIN = 0.0                           ! Maximum PWM signal (microseconds)
     real*8 :: MS_MAX = 0.0                           ! Minimum PWM signal (microseconds)
     real*8 :: DOWNWASH = 0.0                         ! Units: 'nd', Magnitude of Downwash
     real*8 :: DIAMETER = 0.0                         ! Units: 'nd', Diameter of Downwash (ft)
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
     real*8 :: MS_0 = 0.0                             ! Units: 'us', Desc: Microsecond impulse to keep driver hovering // REVISIT can take this out later
     real*8 :: MS_ROLL = 0.0                          ! Units: 'us', Desc: Microsecond impulse to roll  // REVISIT can take this out later
     real*8 :: MS_PITCH = 0.0                         ! Units: 'us', Desc: Microsecond impulse to pitch // REVISIT can take this out later
     real*8 :: MS_YAW = 0.0                           ! Units: 'us', Desc: Microsecond impulse to yaw   // REVISIT can take this out later
     real*8 :: PWM2F(4,1) = 0.0                       ! Units: 'lbf',Desc: Force converted as a function of input microsecond pulse
     real*8 :: DELTATIME = 0.0                        ! Simulation timestep
     real*8 :: XCOM(MAXWP) = 500                         ! Units: 'm', X Waypoint command
     real*8 :: YCOM(MAXWP) = 500                         ! Units: 'm', Y Waypoint command
     real*8 :: ZCOM(MAXWP) = -200                        ! Units: 'm', Altitude command
     character*256 U0name,V0name,W0name
     character*256 Udtname,Vdtname,Wdtname,AIRWAKEPATH
     character(128) :: INPUTFILE = ' '                ! Units: 'nd', Desc: 'Driver Input File'     chara
  end type DRIVERSTRUCTURE