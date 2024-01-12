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
