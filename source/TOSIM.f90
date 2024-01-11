module TOSIMDATATYPES
  IMPLICIT NONE

  ! SIMULATION STRUCTURE
  
  type SIMULATIONSTRUCTURE
     character(128) :: SIMINPUTFILE = ' '             ! Units: 'nd', Desc: 'Simulation Input File'
     character(128) :: RESTARTFILE = ' '              ! Units: 'nd', Desc: 'Simulation Input File'
     integer :: IOUTSKIP = 1                          ! Units: 'nd', Desc: 'Output Skip Parameter'
     integer :: IDEBUG = 0                            ! Units: 'nd', Desc: 'Debug Flag'
     integer :: IDX = 0                               ! Units: 'nd', Desc: 'Integration Index'
     integer :: IDXOUT = 0                            ! Units: 'nd', Desc: ''
     integer :: IECHOOFFON = 1                        ! Units: 'nd', Desc: 'Input Data Echo Flag'
     integer :: DQFLAG = 0                            ! Units: 'nd', Desc: 'Data Quality Flag (0=Data Not Loaded Successfully, 1=Data Loaded Successfully)'
     integer :: CREATERESTART = 1                     ! Units: 'nd', Desc: 'Create Restart Point'
     integer :: RESTART = 0                           ! Units: 'nd', Desc: 'Use a restart file'
     real*8 :: TIME = 0.0                             ! Units: 's', Desc: 'Time'
     real*8 :: RESTARTTIME = 0                        ! Units: 's', Desc: 'Time to create restart time
     real*8 :: DELTATIME = 0.0                        ! Units: 's', Desc: 'Delta Time'
     real*8 :: INITIALTIME = 0.0                      ! Units: 's', Desc: 'Initial Time'
     real*8 :: CPUTIMEUSER = 0.0                      ! Units: 's', Desc: 'Elapsed CPU Time by User Process'
     real*8 :: CPUTIMESYSTEM = 0.0                    ! Units: 's', Desc: 'Elapsed CPU Time by System Process'
     real*8 :: CPUTIMETOTAL = 0.0                     ! Units: 's', Desc: 'Elapsed CPU Time by User and System Processes'
     real*8 :: FINALTIME = 0.0                        ! Units: 's', Desc: 'Final Time'
  end type SIMULATIONSTRUCTURE

  ! TOSIM STRUCTURE

  type TOSIMSTRUCTURE
     character(128) :: FILEINPUTFILE = ' '            ! Units: 'nd', Desc: 'File of Files Input File'
     character(128) :: SIMINPUTFILE = ' '             ! Units: 'nd', Desc: 'Simulation Input File'
     type(SIMULATIONSTRUCTURE) :: SIM
  end type TOSIMSTRUCTURE

end module TOSIMDATATYPES

PROGRAM TOSIM
  use TOSIMDATATYPES
  implicit none
  integer openflag,readflag
  character(128) inputfilename
  character(12) inputfiletype
  type(TOSIMSTRUCTURE) T
  CHARACTER(len=128)::rec,default
  
  readflag = 0
  openflag = 0

  ! Get Input File of Files
  write(*,*) '========================================================='
  call getarg(1,T%FILEINPUTFILE)
  if (len(trim(T%FILEINPUTFILE)) .lt. 1) then
     default = 'Input_Files/Hovering/TOSIM.ifiles'
     write(*,'(a)') 'Using default input file: ',default
     T%FILEINPUTFILE = default
  endif

  ! Open File of Files
  open(unit=93,file=T%FILEINPUTFILE,status='old',iostat=openflag)
  if (openflag .ne. 0) then
     write(*,*) 'Error Opening TOSIM Input File: ',T%FILEINPUTFILE; PAUSE; STOP
  end if
  rewind(93)

  !Read through file of files getting all the input files we need
  do while (readflag .eq. 0)
     inputfiletype = ' '; inputfilename = ' '
     read(unit=93,fmt=*,iostat=readflag) inputfiletype,inputfilename
     write(*,*) 'Input file: ', inputfilename
     if ((inputfiletype.eq.'SIM') .or. (inputfiletype.eq.'sim') .or. (inputfiletype.eq.'Sim')) then
        write(*,*) 'Simulation File Found: ',inputfilename
        T%SIMINPUTFILE = inputfilename;  
     end if
  end do
  !And then close the file
  close(93)

  !Load Data
  write(*,*) '========================================================='
  call SIMULATION(T,1)

  !How many states are there to simulate?
  !T%SIM%NOSTATES = 
  
end PROGRAM TOSIM

!!!!!!!!!!!!!!!!!!!! SUBROUTINE SIMULATION !!!!!!!!!!!!!!!!!!!!!

SUBROUTINE SIMULATION(T,iflag)
  use TOSIMDATATYPES
  implicit none
  integer iflag,openflag,readflag
  type(TOSIMSTRUCTURE) T

 !Compute
 if (iflag .eq. 3) then

 end if

 !Echo
 if (iflag .eq. 2) then

 end if

 !Load
 if (iflag .eq. 1) then
    open(unit=90,file=T%SIMINPUTFILE,status='old',iostat=openflag)
    if (openflag .ne. 0) then
       write(*,*) 'Error Opening Simulation Input File: ',T%SIMINPUTFILE; PAUSE; STOP
    end if
    rewind(90)
    write(*,*) 'Loading Simulation Input File: ',T%SIMINPUTFILE
 end if
 
end SUBROUTINE SIMULATION
 
