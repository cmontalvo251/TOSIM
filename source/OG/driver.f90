!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!! DRIVER STRUCTURE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
module DRIVERDATATYPES
  IMPLICIT NONE
  include 'drivermodule.f90'
end module DRIVERDATATYPES

!!!!!!!!!!!!!!!!!!!!!!!!!!!! README !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!! BEFORE YOU CALL DRIVER(DRIVE,3) to run the routine you must pass a few variables
!! to this data ball. They are listed below.
!! DRIVE%TIME -- You must pass the simulation time variable to the local DRIVE variable
!In order to compute the aero model properly you need
!to pass x,y,z to the ATMOSPHERE model
!if (DRIVE%AEROOFFON .eq. 1) then
!    ATM%XI = DRIVE%STATE(1)
!     ATM%YI = DRIVE%STATE(2)
!     ATM%ZI = DRIVER%STATE(3)
!    Compute Atmopsheric density and winds - Same for DRIVER
!    call ATMOSPHERE()
!    !I'm assuming you have an atmospher model that takes XI,YI,ZI
!    !And returns VX,VY,VZ
!    DRIVE%VXWIND = ATM%VXWIND
!    DRIVE%VYWIND = ATM%VYWIND
!    DRIVE%VZWIND = ATM%VZWIND
!    DRIVE%DEN = ATM%DEN
!end if
!!!!!!!!!!!!!!!!!!!!!!!!!!! SUBROUTINE DRIVER !!!!!!!!!!!!!!!!!!!!!!!!!!!

SUBROUTINE DRIVER(DRIVE,iflag)
 use DRIVERDATATYPES
 implicit none
 integer i,iflag,ifind,openflag,readflag
 real*8 m,readreal,ctheta,stheta,sphi,cphi,spsi,cpsi,ttheta,dspeed,psiprev,omegar
 real*8 tend,slope,accel,noise,freq,vATM_I(3,1),vATM_A(3,1),uaero,vaero,waero,V_A,sumomega
 real*8 xcg,ycg,zcg,phi,theta,psi,ub,vb,wb,pb,qb,rb,rCF_B(3,1),forcevec(4,1),thrust
 real*8 Gammavec(3,1),bquad,C_Ftether_I(3,1),C_Ftether_B(3,1),S_rCF_B(3,3),C_Mtether_B(3,1)
 real*8 xcgdot,ycgdot,zcgdot,phidot,thetadot,psidot,ubdot,vbdot,wbdot,c1,c2,c3,pbdot,qbdot,rbdot
 real*8 rReel_I(3,1),rCG_I(3,1),v_CG_I(3,1),S_wt_B(3,3),v_Reel_I(3,1),deti
 real*8 TVEC(4),TDOTVEC(4),TDBLDOTVEC(4)
 real*8 sigmaF,omegaF,zetaF,C1F(4),C2F(4),C3F(4),idx,W2Tpwm(4,1),W0,j
 character*256 xgridname,ygridname,zgridname
 character*1 letter
 character*10 number
 type(DRIVERSTRUCTURE) DRIVE

!!!!!!!!!!!!!!!!!!!!!!!!!! COMPUTE FLAG iflag = 3 !!!!!!!!!!!!!!!!!!!!!!!!!
  
 if (iflag .eq. 3) then  
    
    !Extract everything from state vector
    xcg = DRIVE%STATE(1)
    ycg = DRIVE%STATE(2)
    zcg = DRIVE%STATE(3)
    phi = DRIVE%STATE(4) 
    theta = DRIVE%STATE(5)
    psi = DRIVE%STATE(6)
    ub = DRIVE%STATE(7)
    vb = DRIVE%STATE(8)
    wb = DRIVE%STATE(9)
    pb = DRIVE%STATE(10)
    qb = DRIVE%STATE(11) 
    rb = DRIVE%STATE(12)
    TVEC(1)    = DRIVE%STATE(13)
    TDOTVEC(1) = DRIVE%STATE(14)
    TVEC(2)    = DRIVE%STATE(15)
    TDOTVEC(2) = DRIVE%STATE(16)
    TVEC(3)    = DRIVE%STATE(17)
    TDOTVEC(3) = DRIVE%STATE(18)
    TVEC(4)    = DRIVE%STATE(19)
    TDOTVEC(4) = DRIVE%STATE(20)
    ! TVEC(1) = DRIVE%STATE(13)
    ! TVEC(2) = DRIVE%STATE(14)
    ! TVEC(3) = DRIVE%STATE(15)
    ! TVEC(4) = DRIVE%STATE(16)

    !Set some things for other models
    DRIVE%XCG = DRIVE%STATE(1) 
    DRIVE%YCG = DRIVE%STATE(2)
    DRIVE%ZCG = DRIVE%STATE(3)
    DRIVE%PSI = DRIVE%STATE(6) !psi
    DRIVE%SPEED = DRIVE%STATE(7) !ub

    !Distance from CG to tether attachment point
    rCF_B(1,1) = DRIVE%SLREEL
    rCF_B(2,1) = DRIVE%BLREEL 
    rCF_B(3,1) = DRIVE%WLREEL

    !- This is where we're going to hi-jack the model and compute the derivatives using Lisa Schibelius'
    !DRIVEDRIVER MODEL
    if (DRIVE%MODNO .eq. 3) then

       ! Unwrap State Vector - This is the same for the DRIVEDRIVER

       ! Aircraft to Inertial Transformation Matrix - Same for DRIVEDRIVER
       ctheta = cos(theta);
       stheta = sin(theta);
       ttheta = stheta / ctheta;
       cphi = cos(phi);
       sphi = sin(phi);
       spsi = sin(psi);
       cpsi = cos(psi);
       DRIVE%TIC(1,1) = ctheta * cpsi;
       DRIVE%TIC(2,1) = ctheta * spsi;
       DRIVE%TIC(3,1) = -stheta;
       DRIVE%TIC(1,2) = sphi * stheta * cpsi - cphi * spsi;
       DRIVE%TIC(2,2) = sphi * stheta * spsi + cphi * cpsi;
       DRIVE%TIC(3,2) = sphi * ctheta;
       DRIVE%TIC(1,3) = cphi * stheta * cpsi + sphi * spsi;
       DRIVE%TIC(2,3) = cphi * stheta * spsi - sphi * cpsi;
       DRIVE%TIC(3,3) = cphi * ctheta;

       ! Inertial to Aircraft Transformation Matrix - Same for DRIVEDRIVER
       
       DRIVE%TCI = transpose(DRIVE%TIC)
  
       ! Gravity Forces and Moments - Same for DRIVER
  
       DRIVE%FXGRAV = 0.0; DRIVE%FYGRAV = 0.0; DRIVE%FZGRAV = 0.0;
       DRIVE%MXGRAV = 0.0; DRIVE%MYGRAV = 0.0; DRIVE%MZGRAV = 0.0;
       if (DRIVE%GRAVOFFON .eq. 1) then
          DRIVE%FXGRAV = DRIVE%TIC(3,1)*DRIVE%WEIGHT
          DRIVE%FYGRAV = DRIVE%TIC(3,2)*DRIVE%WEIGHT
          DRIVE%FZGRAV = DRIVE%TIC(3,3)*DRIVE%WEIGHT
       end if

       ! Aerodynamic Forces and Moments - Still Same for DRIVER
       
       DRIVE%FXAERO = 0.0; DRIVE%FYAERO = 0.0; DRIVE%FZAERO = 0.0;
       DRIVE%MXAERO = 0.0; DRIVE%MYAERO = 0.0; DRIVE%MZAERO = 0.0;
       
       if (DRIVE%AEROOFFON .eq. 1) then
          vATM_I(1,1) = DRIVE%VXWIND
          vATM_I(2,1) = DRIVE%VYWIND
          vATM_I(3,1) = DRIVE%VZWIND
          vATM_A = matmul(DRIVE%TCI,vATM_I)

          !Add in atmospheric winds

          uaero = ub - vATM_A(1,1)
          vaero = vb - vATM_A(2,1)
          waero = wb - vATM_A(3,1)
          
          !Compute total velocity

          V_A = sqrt(uaero**2 + vaero**2 + waero**2)

          !Quadcopter Aerodynamic Model written by Lisa Schibelius - 12/2016

          !Recompute KT
          DRIVE%KT = DRIVE%C_T*((DRIVE%DEN*qPI*(DRIVE%RNEW**4)/4))

          !Compute Thrust

          ! call PWM2FORCE(T)
          sigmaF = 0.000437554764978899 !0.000437554764978899
          omegaF = 45.42   !18.65
          zetaF  = 0.942     !0.8533

          !write(*,*) 'MUvec = ',DRIVE%MUVEC

          !!! Second order filter
          do idx = 1,4
              C1F(idx) = -2*zetaF*TDOTVEC(idx)
              C2F(idx) = -(omegaF)*TVEC(idx)
              C3F(idx) = DRIVE%MUVEC(idx,1)*sigmaF*omegaF   ! replaced sigma with force
              TDBLDOTVEC(idx) = omegaF*(C1F(idx) + C2F(idx) + C3F(idx))
              DRIVE%THRUSTVEC(idx,1) = TVEC(idx)
          end do

          DRIVE%OMEGAVEC = sqrt(DRIVE%THRUSTVEC/DRIVE%KT)
          sumomega = sum(DRIVE%OMEGAVEC)

          !!! Make sure angular velocities of rotor does not go beyond the limit
          IF (sumomega .ge. DRIVE%OMEGAMAX*4) then
            do j = 1,4
              if (DRIVE%OMEGAVEC(j,1) .gt. DRIVE%OMEGAMAX) then
              DRIVE%OMEGAVEC(j,1) = DRIVE%OMEGAMAX
              end if
              if (DRIVE%OMEGAVEC(j,1) .lt. 0.00D0) then
                DRIVE%OMEGAVEC(j,1) = 0.00D0
              end if
            end do
            sumomega = sum(DRIVE%OMEGAVEC)
            DRIVE%THRUSTVEC = DRIVE%KT*DRIVE%OMEGAVEC**2
            do j = 1,4
              TVEC(idx) = DRIVE%THRUSTVEC(idx,1)
            end do
          ENDIF
          forcevec = DRIVE%THRUSTVEC
          thrust = sum(DRIVE%THRUSTVEC)

          !write(*,*) 'Rotor Thrust = ',forcevec,thrust

          !!! Adding constraint to run Monte Carlo
          !!! This constraint actually messed up my altitude controller
          ! if (thrust .gt. DRIVE%WEIGHT/cos(30.0*qPI/180)) then !You need to put in theta of the quad 
          !   thrust = DRIVE%WEIGHT/cos(30.0*qPI/180) !!Not just a static 30 degrees. That's why this didn't work.
          ! end if
          ! I was thinking this would be better.
          ! if (thrust .lt. DRIVE%WEIGHT/cos(theta)) then
          ! Increase thrust by the difference/4
          ! Make sense?

          !Aerodynamic Forces
          if (sumomega .gt. 1e-2) then
             DRIVE%FXAERO = -thrust*(((DRIVE%ALC/(sumomega*DRIVE%RNEW))+DRIVE%DXD)*uaero - ((DRIVE%ALS)/(sumomega*DRIVE%RNEW))*vaero)
             DRIVE%FYAERO = -thrust*(((DRIVE%ALS)/(sumomega*DRIVE%RNEW))*uaero + (((DRIVE%ALC)/(sumomega*DRIVE%RNEW))+DRIVE%DYD)*vaero)
             DRIVE%FZAERO = -thrust
          end if


          omegar = DRIVE%OMEGAVEC(1,1) - DRIVE%OMEGAVEC(2,1) + DRIVE%OMEGAVEC(3,1) - DRIVE%OMEGAVEC(4,1)
          Gammavec(1,1) = DRIVE%IRR * omegar * qb
          Gammavec(2,1) = -DRIVE%IRR * omegar * pb
          Gammavec(3,1) = 0
          !gotodynamics

          !!!!!!!!! Aerodynamics
          bquad = DRIVE%C_TAU*((DRIVE%DEN*qPI*(DRIVE%RNEW**5)/4))
	  
	  !!! According to dynamic equations, a positive roll will have rotors 1,4 > 2,3. This was previously 2,3>1,4
          ! Since T3 = T1*Ltheta_front/Ltheta_back we're just going to do this for simplicity
          DRIVE%MXAERO = Gammavec(1,1) + (DRIVE%LPHI12*(TVEC(1) - TVEC(2)) + DRIVE%LPHI34*(TVEC(4) - TVEC(3)))
          DRIVE%MYAERO = Gammavec(2,1) + DRIVE%LTHETA12*(TVEC(1) +TVEC(2) - TVEC(3) -TVEC(4))
          DRIVE%MZAERO = Gammavec(3,1) + bquad*(DRIVE%OMEGAVEC(1,1)**2 - DRIVE%OMEGAVEC(2,1)**2 + DRIVE%OMEGAVEC(3,1)**2 - DRIVE%OMEGAVEC(4,1)**2)
          ! DRIVE%MZAERO = Gammavec(3,1) + bquad*(TVEC(1)**2 - TVEC(2)**2 + TVEC(3)**2 - TVEC(4)**2)

          !At this point we should have F(XYZ)AERO and M(XYZ)AERO populated

          DRIVE%FXCONT = 0.0; DRIVE%FYCONT = 0.0; DRIVE%FZCONT = 0.0;          
          DRIVE%MXCONT = 0.0; DRIVE%MYCONT = 0.0; DRIVE%MZCONT = 0.0;

          !Don't forget to add Tether forces
          if ((DRIVE%THR_DYNOFFON .eq. 1) .and. (DRIVE%THR_ELASOFFON .eq. 1)) then
             if (isnan(DRIVE%FTETHERX) .or. isnan(DRIVE%FTETHERY)) then
                write(*,*) 'Sorry dude Nans in tether model detected - I suggest lowering your timestep'
                write(*,*) 'Current Time = ',DRIVE%TIME
                write(*,*) 'Tether Forces = ',DRIVE%FTETHERX,DRIVE%FTETHERY
                STOP
             else
                C_Ftether_I(1,1) = DRIVE%FTETHERX
                C_Ftether_I(2,1) = DRIVE%FTETHERY
                C_Ftether_I(3,1) = DRIVE%FTETHERZ
                C_Ftether_B = matmul(DRIVE%TCI,C_Ftether_I)
                DRIVE%FXCONT = C_Ftether_B(1,1)
                DRIVE%FYCONT = C_Ftether_B(2,1)
                DRIVE%FZCONT = C_Ftether_B(3,1)
                
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
        
                DRIVE%MXCONT = C_Mtether_B(1,1)
                DRIVE%MYCONT = C_Mtether_B(2,1)
                DRIVE%MZCONT = C_Mtether_B(3,1)
             end if
          end if

          ! Total Forces and Moments

          DRIVE%FXTOTAL = DRIVE%FXGRAV + DRIVE%FXAERO + DRIVE%FXCONT
          DRIVE%FYTOTAL = DRIVE%FYGRAV + DRIVE%FYAERO + DRIVE%FYCONT
          !write(*,*) 'Z Force = ',DRIVE%FZGRAV,DRIVE%FZAERO,DRIVE%FZCONT
          DRIVE%FZTOTAL = DRIVE%FZGRAV + DRIVE%FZAERO + DRIVE%FZCONT
          DRIVE%MXTOTAL = DRIVE%MXGRAV + DRIVE%MXAERO + DRIVE%MXCONT
          DRIVE%MYTOTAL = DRIVE%MYGRAV + DRIVE%MYAERO + DRIVE%MYCONT
          DRIVE%MZTOTAL = DRIVE%MZGRAV + DRIVE%MZAERO + DRIVE%MZCONT

          ! State Derivatives
  
          xcgdot = DRIVE%TIC(1,1)*ub + DRIVE%TIC(1,2)*vb + DRIVE%TIC(1,3)*wb
          ycgdot = DRIVE%TIC(2,1)*ub + DRIVE%TIC(2,2)*vb + DRIVE%TIC(2,3)*wb
          zcgdot = DRIVE%TIC(3,1)*ub + DRIVE%TIC(3,2)*vb + DRIVE%TIC(3,3)*wb  

          phidot = pb + sphi * ttheta * qb + cphi * ttheta * rb;
          thetadot = cphi * qb - sphi * rb;
          psidot = (sphi / ctheta) * qb + (cphi / ctheta) * rb;
          ubdot = DRIVE%FXTOTAL/DRIVE%MASS + rb*vb - qb*wb
          vbdot = DRIVE%FYTOTAL/DRIVE%MASS + pb*wb - rb*ub
          !write(*,*) 'Weight and Z =',DRIVE%FZTOTAL,DRIVE%MASS
          wbdot = DRIVE%FZTOTAL/DRIVE%MASS + qb*ub - pb*vb
          
          c1 = DRIVE%MXTOTAL - pb*(qb*DRIVE%IXZ-rb*DRIVE%IXY) - qb*(qb*DRIVE%IYZ-rb*DRIVE%IYY) - rb*(qb*DRIVE%IZZ-rb*DRIVE%IYZ)
          c2 = DRIVE%MYTOTAL - pb*(rb*DRIVE%IXX-pb*DRIVE%IXZ) - qb*(rb*DRIVE%IXY-pb*DRIVE%IYZ) - rb*(rb*DRIVE%IXZ-pb*DRIVE%IZZ)
          c3 = DRIVE%MZTOTAL - pb*(pb*DRIVE%IXY-qb*DRIVE%IXX) - qb*(pb*DRIVE%IYY-qb*DRIVE%IXY) - rb*(pb*DRIVE%IYZ-qb*DRIVE%IXZ)
          pbdot = DRIVE%IXXI*c1 + DRIVE%IXYI*c2 + DRIVE%IXZI*c3
          qbdot = DRIVE%IXYI*c1 + DRIVE%IYYI*c2 + DRIVE%IYZI*c3
          rbdot = DRIVE%IXZI*c1 + DRIVE%IYZI*c2 + DRIVE%IZZI*c3

          ! Wrap State Derivatives
  
          DRIVE%STATEDOT(1) = xcgdot
          DRIVE%STATEDOT(2) = ycgdot
          DRIVE%STATEDOT(2) = ycgdot
          DRIVE%STATEDOT(3) = zcgdot
          DRIVE%STATEDOT(4) = phidot
          DRIVE%STATEDOT(5) = thetadot
          DRIVE%STATEDOT(6) = psidot
          DRIVE%STATEDOT(7) = ubdot
          DRIVE%STATEDOT(8) = vbdot
          DRIVE%STATEDOT(9) = wbdot
          DRIVE%STATEDOT(10) = pbdot 
          DRIVE%STATEDOT(11) = qbdot 
          DRIVE%STATEDOT(12) = rbdot
          DRIVE%STATEDOT(13) = TDOTVEC(1)
          DRIVE%STATEDOT(14) = TDBLDOTVEC(1)
          DRIVE%STATEDOT(15) = TDOTVEC(2)
          DRIVE%STATEDOT(16) = TDBLDOTVEC(2)
          DRIVE%STATEDOT(17) = TDOTVEC(3)
          DRIVE%STATEDOT(18) = TDBLDOTVEC(3)
          DRIVE%STATEDOT(19) = TDOTVEC(4)
          DRIVE%STATEDOT(20) = TDBLDOTVEC(4)

          !write(*,*) 'Statedot = ',DRIVE%STATEDOT
       
          !!Save some stuff for Tether model
          DRIVE%XDOT = xcgdot
          DRIVE%YDOT = ycgdot
          DRIVE%ZDOT = zcgdot

          !Compute Reel Locations - r_reel = r_cg + TIB*r_body
          rCG_I(1,1) = xcg 
          rCG_I(2,1) = ycg
          rCG_I(3,1) = zcg ! + (1/3) !REVISIT
          rReel_I = rCG_I + matmul(DRIVE%TIC,rCF_B)
          DRIVE%XREEL = rReel_I(1,1)
          DRIVE%YREEL = rReel_I(2,1)
          DRIVE%ZREEL = rReel_I(3,1)
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
          v_Reel_I = v_CG_I + matmul(DRIVE%TIC,matmul(S_wt_B,rCF_B))
          DRIVE%XREELDOT = v_Reel_I(1,1)
          DRIVE%YREELDOT = v_Reel_I(2,1)
          DRIVE%ZREELDOT = v_Reel_I(3,1)
       end if !AEROOFFON

    end if !End MODNO
    ! Integration Model 
    if (DRIVE%MODNO .eq. 0) then
       !!!Compute Driver speed 
       if (DRIVE%RESTARTSPEED .eq. -999) then
          !DRIVE%SPEED = DRIVE%FINALSPEED
          accel = 0
       else
          !Ramp in speed - assume constant decel/acceleration of 2 ft/s
          dspeed = 3
          tend = abs(DRIVE%RESTARTSPEED-DRIVE%FINALSPEED)/dspeed
          if (DRIVE%TIME .gt. tend) then
             !DRIVE%SPEED = DRIVE%FINALSPEED
             accel = 0
          else
             !DRIVE%SPEED = DRIVE%RESTARTSPEED + sign(1.0,DRIVE%FINALSPEED-DRIVE%RESTARTSPEED)*dspeed*(DRIVE%TIME)
             accel = sign(1.0,DRIVE%FINALSPEED-DRIVE%RESTARTSPEED)*dspeed
          end if
       end if

       !!Compute Derivatives
       DRIVE%XDOT = DRIVE%SPEED !!This assumes you are flying straight - REVISIT REVISIT
       DRIVE%YDOT = vb
       DRIVE%ZDOT = wb
       DRIVE%STATEDOT(1) = DRIVE%XDOT
       DRIVE%STATEDOT(2) = DRIVE%YDOT
       DRIVE%STATEDOT(3) = DRIVE%ZDOT
       DRIVE%STATEDOT(4:6) = 0 !Phi theta psi
       call RandUniform(noise)
       DRIVE%STATEDOT(7) = accel + (1.0D0-2*noise)*DRIVE%XDDOTNOISE !udot
       call RandUniform(noise)
       freq = 0
       if (DRIVE%YDDOTPERIOD .ne. 0) then
          freq = (2*qPI)/(DRIVE%YDDOTPERIOD)*cos((2*qPI)/(DRIVE%YDDOTPERIOD)*DRIVE%TIME)
       end if
       DRIVE%STATEDOT(8) = 0 + (1.0D0-2*noise)*DRIVE%XDDOTNOISE + DRIVE%YDDOTSCALE*freq !vdot
       call RandUniform(noise)
       DRIVE%STATEDOT(9) = (1.0D0-2*noise)*DRIVE%XDDOTNOISE !wdot
       DRIVE%STATEDOT(10:12) = 0 !p,q,rdot

       !Compute Reel Locations
       DRIVE%XREEL = DRIVE%XCG + cos(DRIVE%PSI)*(DRIVE%SLREEL-DRIVE%SLCG) - sin(DRIVE%PSI)*(DRIVE%BLREEL-DRIVE%BLCG) 
       DRIVE%YREEL = DRIVE%YCG + sin(DRIVE%PSI)*(DRIVE%SLREEL-DRIVE%SLCG) + cos(DRIVE%PSI)*(DRIVE%BLREEL-DRIVE%BLCG) 
       DRIVE%ZREEL = DRIVE%ZCG + DRIVE%WLREEL - DRIVE%WLCG 
       DRIVE%XREELDOT = DRIVE%XDOT
       DRIVE%YREELDOT = DRIVE%YDOT
       DRIVE%ZREELDOT = DRIVE%ZDOT
       
    end if

 ! Constant Model

    if (DRIVE%MODNO .eq. 1) then

     DRIVE%SPEED = DRIVE%FINALSPEED
        
     ! if (DRIVE%TIME .gt. 1000) then
     !    DRIVE%XCG = DRIVE%XCGINITIAL + DRIVE%SPEED*1000 + (DRIVE%SPEED+5)*(DRIVE%TIME-1000)
     !    speed = DRIVE%SPEED+5
     ! end if
     DRIVE%XDOT = DRIVE%SPEED*cos(DRIVE%PSI)
     DRIVE%YDOT = DRIVE%SPEED*sin(DRIVE%PSI) !!!DO NOT FORGET TO FIX THIS TOO

     !!Compute CG location -- This assumes that speed is constant

     DRIVE%XCG = DRIVE%XCGINITIAL + DRIVE%SPEED*cos(DRIVE%PSI)*DRIVE%TIME
     DRIVE%YCG = DRIVE%YCGINITIAL + DRIVE%SPEED*sin(DRIVE%PSI)*DRIVE%TIME
     DRIVE%ZCG = DRIVE%ZCGINITIAL

     !Compute CG speed

     DRIVE%ZDOT = 0.0
     DRIVE%XREEL = DRIVE%XCG + cos(DRIVE%PSI)*(DRIVE%SLREEL-DRIVE%SLCG) - sin(DRIVE%PSI)*(DRIVE%BLREEL-DRIVE%BLCG) 
     DRIVE%YREEL = DRIVE%YCG + sin(DRIVE%PSI)*(DRIVE%SLREEL-DRIVE%SLCG) + cos(DRIVE%PSI)*(DRIVE%BLREEL-DRIVE%BLCG) 
     DRIVE%ZREEL = DRIVE%ZCG + DRIVE%WLREEL - DRIVE%WLCG 
     DRIVE%XREELDOT = DRIVE%XDOT
     DRIVE%YREELDOT = DRIVE%YDOT
     DRIVE%ZREELDOT = DRIVE%ZDOT

     !Place everything in state vector

     DRIVE%STATE(1) = DRIVE%XCG
     DRIVE%STATE(2) = DRIVE%YCG
     DRIVE%STATE(3) = DRIVE%ZCG
     DRIVE%STATE(4) = 0 !phi 
     DRIVE%STATE(5) = 0 !theta
     DRIVE%STATE(6) = DRIVE%PSI !psi
     DRIVE%STATE(7) = DRIVE%SPEED
     DRIVE%STATE(8:12) = 0

   ! Helpful variables
     
     DRIVE%THETA = 0
     DRIVE%PHI = 0
     ctheta = cos(DRIVE%THETA)
     stheta = sin(DRIVE%THETA)
     cphi = cos(DRIVE%PHI)
     sphi = sin(DRIVE%PHI)
     cpsi = cos(DRIVE%PSI)
     spsi = sin(DRIVE%PSI)

     ! Reel Position and Velocity
     
     DRIVE%TIS(1,1) = ctheta*cpsi
     DRIVE%TIS(2,1) = ctheta*spsi
     DRIVE%TIS(3,1) = -stheta
     DRIVE%TIS(1,2) = sphi*stheta*cpsi - cphi*spsi
     DRIVE%TIS(2,2) = sphi*stheta*spsi + cphi*cpsi
     DRIVE%TIS(3,2) = sphi*ctheta
     DRIVE%TIS(1,3) = cphi*stheta*cpsi + sphi*spsi
     DRIVE%TIS(2,3) = cphi*stheta*spsi - sphi*cpsi
     DRIVE%TIS(3,3) = cphi*ctheta
  end if
  
  ! Table Look-Up Model 

  if (DRIVE%MODNO .eq. 2) then
    
   ifind = 0
  
   ! Position Pointer  

   if (DRIVE%TIME .le. DRIVE%TIMETAB(DRIVE%IP)) then 
    ifind = -1 
    do while ((ifind.ne.0) .and. (DRIVE%IP.gt.1))
     DRIVE%IP = DRIVE%IP - 1 
     if (DRIVE%TIMETAB(DRIVE%IP)   .le. DRIVE%TIME) then 
     if (DRIVE%TIMETAB(DRIVE%IP+1) .gt. DRIVE%TIME) then 
      ifind = 0
     end if
     end if 
    end do 
   end if
   if (DRIVE%TIME .gt. DRIVE%TIMETAB(DRIVE%IP+1)) then 
    ifind = 1
    do while ((ifind.ne.0) .and. (DRIVE%IP.lt.DRIVE%TABSIZE-1))
     DRIVE%IP = DRIVE%IP + 1 
     if (DRIVE%TIMETAB(DRIVE%IP)   .le. DRIVE%TIME) then 
     if (DRIVE%TIMETAB(DRIVE%IP+1) .gt. DRIVE%TIME) then 
      ifind = 0 
     end if 
     end if 
    end do 
   end if
   if (ifind .eq. 0) then
    m = (DRIVE%TIME-DRIVE%TIMETAB(DRIVE%IP))/(DRIVE%TIMETAB(DRIVE%IP+1)-DRIVE%TIMETAB(DRIVE%IP))
   else if (ifind .eq. -1) then
    m = 0.0
   else if (ifind .eq. 1) then
    m = 1.0
   end if
  
   ! Interpolate
    
   DRIVE%XCG   = DRIVE%XCGINITIAL + DRIVE%XCGTAB(DRIVE%IP)   + m*(DRIVE%XCGTAB(DRIVE%IP+1)-DRIVE%XCGTAB(DRIVE%IP))
   DRIVE%YCG   = DRIVE%YCGINITIAL + DRIVE%YCGTAB(DRIVE%IP)   + m*(DRIVE%YCGTAB(DRIVE%IP+1)-DRIVE%YCGTAB(DRIVE%IP))
   DRIVE%ZCG   = DRIVE%ZCGINITIAL + DRIVE%ZCGTAB(DRIVE%IP)   + m*(DRIVE%ZCGTAB(DRIVE%IP+1)-DRIVE%ZCGTAB(DRIVE%IP))
   DRIVE%PHI   = DRIVE%PHITAB(DRIVE%IP)   + m*(DRIVE%PHITAB(DRIVE%IP+1)-DRIVE%PHITAB(DRIVE%IP))
   DRIVE%THETA = DRIVE%THETATAB(DRIVE%IP) + m*(DRIVE%THETATAB(DRIVE%IP+1)-DRIVE%THETATAB(DRIVE%IP))
   DRIVE%PSI   = DRIVE%PSITAB(DRIVE%IP)   + m*(DRIVE%PSITAB(DRIVE%IP+1)-DRIVE%PSITAB(DRIVE%IP))
   DRIVE%UB    = DRIVE%UBTAB(DRIVE%IP)    + m*(DRIVE%UBTAB(DRIVE%IP+1)-DRIVE%UBTAB(DRIVE%IP))
   DRIVE%VB    = DRIVE%VBTAB(DRIVE%IP)    + m*(DRIVE%VBTAB(DRIVE%IP+1)-DRIVE%VBTAB(DRIVE%IP))
   DRIVE%WB    = DRIVE%WBTAB(DRIVE%IP)    + m*(DRIVE%WBTAB(DRIVE%IP+1)-DRIVE%WBTAB(DRIVE%IP))
   DRIVE%PB    = DRIVE%PBTAB(DRIVE%IP)    + m*(DRIVE%PBTAB(DRIVE%IP+1)-DRIVE%PBTAB(DRIVE%IP))
   DRIVE%QB    = DRIVE%QBTAB(DRIVE%IP)    + m*(DRIVE%QBTAB(DRIVE%IP+1)-DRIVE%QBTAB(DRIVE%IP))
   DRIVE%RB    = DRIVE%RBTAB(DRIVE%IP)    + m*(DRIVE%RBTAB(DRIVE%IP+1)-DRIVE%RBTAB(DRIVE%IP))

   !Place state in statevector

   DRIVE%STATE(1) = DRIVE%XCG
   DRIVE%STATE(2) = DRIVE%YCG
   DRIVE%STATE(3) = DRIVE%ZCG
   DRIVE%STATE(4) = DRIVE%PHI
   DRIVE%STATE(5) = DRIVE%THETA
   DRIVE%STATE(6) = DRIVE%PSI
   DRIVE%STATE(7) = DRIVE%UB
   DRIVE%STATE(8) = DRIVE%VB
   DRIVE%STATE(9) = DRIVE%WB
   DRIVE%STATE(10) = DRIVE%PB
   DRIVE%STATE(11) = DRIVE%QB
   DRIVE%STATE(12) = DRIVE%RB

   ! Helpful variables

   ctheta = cos(DRIVE%THETA)
   stheta = sin(DRIVE%THETA)
   cphi = cos(DRIVE%PHI)
   sphi = sin(DRIVE%PHI)
   cpsi = cos(DRIVE%PSI)
   spsi = sin(DRIVE%PSI)

   ! Reel Position and Velocity
    
   DRIVE%TIS(1,1) = ctheta*cpsi
   DRIVE%TIS(2,1) = ctheta*spsi
   DRIVE%TIS(3,1) = -stheta
   DRIVE%TIS(1,2) = sphi*stheta*cpsi - cphi*spsi
   DRIVE%TIS(2,2) = sphi*stheta*spsi + cphi*cpsi
   DRIVE%TIS(3,2) = sphi*ctheta
   DRIVE%TIS(1,3) = cphi*stheta*cpsi + sphi*spsi
   DRIVE%TIS(2,3) = cphi*stheta*spsi - sphi*cpsi
   DRIVE%TIS(3,3) = cphi*ctheta

   !!Compute Driver speed in inertial coordinates

   DRIVE%XDOT = DRIVE%TIS(1,1)*DRIVE%UB + DRIVE%TIS(1,2)*DRIVE%VB + DRIVE%TIS(1,3)*DRIVE%WB  
   DRIVE%YDOT = DRIVE%TIS(2,1)*DRIVE%UB + DRIVE%TIS(2,2)*DRIVE%VB + DRIVE%TIS(2,3)*DRIVE%WB  
   DRIVE%ZDOT = DRIVE%TIS(3,1)*DRIVE%UB + DRIVE%TIS(3,2)*DRIVE%VB + DRIVE%TIS(3,3)*DRIVE%WB  

   !!!Compute Reel velocity in Driver frame

   DRIVE%UREEL = DRIVE%UB + DRIVE%QB*(DRIVE%WLREEL-DRIVE%WLCG) - DRIVE%RB*(DRIVE%BLREEL-DRIVE%BLCG) 
   DRIVE%VREEL = DRIVE%VB + DRIVE%RB*(DRIVE%SLREEL-DRIVE%SLCG) - DRIVE%PB*(DRIVE%WLREEL-DRIVE%WLCG)
   DRIVE%WREEL = DRIVE%WB + DRIVE%PB*(DRIVE%BLREEL-DRIVE%BLCG) - DRIVE%QB*(DRIVE%SLREEL-DRIVE%SLCG)

   DRIVE%XREEL = DRIVE%XCG + DRIVE%TIS(1,1)*(DRIVE%SLREEL-DRIVE%SLCG) + DRIVE%TIS(1,2)*(DRIVE%BLREEL-DRIVE%BLCG) + DRIVE%TIS(1,3)*(DRIVE%WLREEL-DRIVE%WLCG)  
   DRIVE%YREEL = DRIVE%YCG + DRIVE%TIS(2,1)*(DRIVE%SLREEL-DRIVE%SLCG) + DRIVE%TIS(2,2)*(DRIVE%BLREEL-DRIVE%BLCG) + DRIVE%TIS(2,3)*(DRIVE%WLREEL-DRIVE%WLCG)  
   DRIVE%ZREEL = DRIVE%ZCG + DRIVE%TIS(3,1)*(DRIVE%SLREEL-DRIVE%SLCG) + DRIVE%TIS(3,2)*(DRIVE%BLREEL-DRIVE%BLCG) + DRIVE%TIS(3,3)*(DRIVE%WLREEL-DRIVE%WLCG)  

   DRIVE%XREELDOT = DRIVE%TIS(1,1)*DRIVE%UREEL + DRIVE%TIS(1,2)*DRIVE%VREEL + DRIVE%TIS(1,3)*DRIVE%WREEL  
   DRIVE%YREELDOT = DRIVE%TIS(2,1)*DRIVE%UREEL + DRIVE%TIS(2,2)*DRIVE%VREEL + DRIVE%TIS(2,3)*DRIVE%WREEL  
   DRIVE%ZREELDOT = DRIVE%TIS(3,1)*DRIVE%UREEL + DRIVE%TIS(3,2)*DRIVE%VREEL + DRIVE%TIS(3,3)*DRIVE%WREEL  
    
  end if

  ! This is pretty sloppy. Nemo wants this to run only once per rk4 function call
  ! should probably just add these as states but this will do for now
  ! What we will do is just divide everything out by 4 so when this runs 4 times we should
  ! be good.
  call COMPUTEINTEGRAL(DRIVE) ! Calls integral routine
    
  RETURN
  
end if
  
!!!!!!!!!!!!!!!!!!!!!! ECHO DATA iflag = 2 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 
 if (iflag .eq. 2) then
 
  write(25,*) ' '
  write(25,*) 'Driver'  
  write(25,*) ' '
  write(25,*) 'Driver Input File: '
  write(25,*) trim(DRIVE%INPUTFILE)
  write(25,*) ' '
  write(25,*) 'Module off or on: ',DRIVE%OFFON
  write(25,*) 'Model (0=Integration,1=Constant, 2=Table,3=6DOF Model): ',DRIVE%MODNO
  if (DRIVE%MODNO .eq. 3) then
     !Output driver stuff
     write(25,*) 'Gravity Flag (0=Off, 1=On): ',DRIVE%GRAVOFFON
     write(25,*) 'Aerodynamics Flag (0=Off, 1=On): ',DRIVE%AEROOFFON
     write(25,*) 'Contact Flag (0=Off, 1=On): ',DRIVE%CONTOFFON
     write(25,*) 'Mass (kg): ',DRIVE%MASS
     write(25,*) 'Weight (N): ',DRIVE%WEIGHT
     write(25,*) 'Stationline of Mass Center (m): ',DRIVE%SLCG
     write(25,*) 'Buttline of Mass Center (m): ',DRIVE%BLCG
     write(25,*) 'Waterline of Mass Center (m): ',DRIVE%WLCG
     write(25,*) 'Ixx (kg m^2): ',DRIVE%IXX
     write(25,*) 'Iyy (kg m^2): ',DRIVE%IYY
     write(25,*) 'Izz (kg m^2): ',DRIVE%IZZ
     write(25,*) 'Ixy (kg m^2): ',DRIVE%IXY
     write(25,*) 'Ixz (kg m^2): ',DRIVE%IXZ
     write(25,*) 'Iyz (kg m^2): ',DRIVE%IYZ
     write(25,*) 'Ixx Inverse (1/(kg m^2)): ',DRIVE%IXXI
     write(25,*) 'Iyy Inverse (1/(kg m^2)): ',DRIVE%IYYI
     write(25,*) 'Izz Inverse (1/(kg m^2)): ',DRIVE%IZZI
     write(25,*) 'Ixy Inverse (1/(kg m^2)): ',DRIVE%IXYI
     write(25,*) 'Ixz Inverse (1/(kg m^2)): ',DRIVE%IXZI
     write(25,*) 'Iyz Inverse (1/(kg m^2)): ',DRIVE%IYZI
     write(25,*) 'Turn Radius of AC(m): ',  DRIVE%TURNRADIUS
     write(25,*) 'Aero Parameter: ', DRIVE%ALC
     write(25,*) 'Aero Parameter: ', DRIVE%ALS
     write(25,*) 'Aero Parameter: ', DRIVE%DXD
     write(25,*) 'Aero Parameter: ', DRIVE%DYD
     write(25,*) 'Aero Parameter: ', DRIVE%RNEW
     write(25,*) 'Aero Parameter: ', DRIVE%C_T
     write(25,*) 'Aero Parameter: ', DRIVE%C_TAU
     write(25,*) 'Aero Parameter: ', DRIVE%LPHI12
     write(25,*) 'Aero Parameter: ', DRIVE%LPHI34
     write(25,*) 'Aero Parameter: ', DRIVE%LTHETA12
     write(25,*) 'Aero Parameter: ', DRIVE%LTHETA34
     write(25,*) 'Aero Parameter: ', DRIVE%OMEGAMAX
  else
     write(25,*) ' '
     write(25,*) 'Stationline of Tether Reel Point on Driver: ', DRIVE%SLREEL
     write(25,*) 'Buttline of Tether Reel Point on Driver: ', DRIVE%BLREEL
     write(25,*) 'Waterline of Tether Reel Point on Driver: ', DRIVE%WLREEL
     write(25,*) 'Stationline of Airwake grid start on Driver: ', DRIVE%SLAIRWAKE
     write(25,*) 'Buttline of Airwake grid start on Driver: ', DRIVE%BLAIRWAKE
     write(25,*) 'Waterline of Airwake grid start on Driver: ', DRIVE%WLAIRWAKE
     if ((DRIVE%MODNO .eq. 1) .or. (DRIVE%MODNO .eq. 0 )) then
        write(25,*) 'Driver Speed from .DRIVER File(ft/s): ',DRIVE%FINALSPEED
        write(25,*) 'Restart Speed from RESTART File (ft/s): ',DRIVE%RESTARTSPEED
        write(25,*) 'Driver Azimuthal Direction (deg): ',57.3*DRIVE%PSI
        write(25,*) 'Driver Initial X (ft): ',DRIVE%XCGINITIAL
        write(25,*) 'Driver Initial Y (ft): ',DRIVE%YCGINITIAL
        write(25,*) 'Driver Initial Z (ft): ',DRIVE%ZCGINITIAL
        write(25,*) 'Driver Noise X (ft/s^2): ',DRIVE%XDDOTNOISE
        write(25,*) 'Driver Noise Y (ft/s^2): ',DRIVE%YDDOTSCALE
        write(25,*) 'Driver Noise Z (ft/s^2): ',DRIVE%YDDOTPERIOD
        write(25,*) ' '
     end if
     if (DRIVE%MODNO .eq. 2) then
        write(25,*) 'Time (s),      Xcg (ft),      Ycg (ft),      Zcg (ft)'
        write(25,*) '----------------------------------------------------'
        do i=1,DRIVE%TABSIZE  
           write(25,fmt='(4e18.8)') DRIVE%TIMETAB(i),DRIVE%XCGTAB(i),DRIVE%YCGTAB(i),DRIVE%ZCGTAB(i)
        end do
        write(25,*) ' '
        write(25,*) 'Time (s),    Phi (deg),  Theta (deg),    Psi (deg)'
        write(25,*) '----------------------------------------------------'
        do i=1,DRIVE%TABSIZE  
           write(25,fmt='(4e18.8)') DRIVE%TIMETAB(i),57.3*DRIVE%PHITAB(i),57.3*DRIVE%THETATAB(i),57.3*DRIVE%PSITAB(i)
        end do
        write(25,*) ' '
        write(25,*) 'Time (s),     Ub (ft/s),     Vb (ft/s),     Wb (ft/s)'
        write(25,*) '----------------------------------------------------'
        do i=1,DRIVE%TABSIZE   
           write(25,fmt='(4e18.8)') DRIVE%TIMETAB(i),DRIVE%UBTAB(i),DRIVE%VBTAB(i),DRIVE%WBTAB(i)
        end do
        write(25,*) ' '
        write(25,*) 'Time (s),     Pb (r/s),      Qb (r/s),    Rb (r/s)'
        write(25,*) '----------------------------------------------------'
        do i=1,DRIVE%TABSIZE  
           write(25,fmt='(4e18.8)') DRIVE%TIMETAB(i),DRIVE%PBTAB(i),DRIVE%QBTAB(i),DRIVE%RBTAB(i)
        end do
        write(25,*) ' '
     end if
  end if
  write(25,*) 'Data Quality Flag (nd, 0=Data Not Loaded Successfully, 1=Data Loaded Successfully): ',DRIVE%DQFLAG
  
  RETURN
  
 end if
  
!!!!!!!!!!!!!!!!!!!!!!!!!!!!! LOAD DATA iflag = 1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 
 if (iflag .eq. 1) then
   
  open(unit=94,file=DRIVE%INPUTFILE,status='old',iostat=openflag)
  if (openflag .ne. 0) then
     write(*,*) DRIVE%INPUTFILE
   write(*,*) 'Error Opening Driver Input File => ',DRIVE%INPUTFILE,' <= ';PAUSE; STOP
  end if
  rewind(94)
  read(unit=94,fmt=*,iostat=readflag) readreal; DRIVE%OFFON = readreal
  read(unit=94,fmt=*,iostat=readflag) readreal; DRIVE%MODNO = readreal

  if (DRIVE%MODNO .eq. 3) then
     read(unit=94,fmt=*,iostat=readflag) DRIVE%GRAVOFFON
     read(unit=94,fmt=*,iostat=readflag) DRIVE%AEROOFFON
     read(unit=94,fmt=*,iostat=readflag) DRIVE%CONTOFFON
     read(unit=94,fmt=*,iostat=readflag) DRIVE%WEIGHT
     read(unit=94,fmt=*,iostat=readflag) DRIVE%GRAVITY
     read(unit=94,fmt=*,iostat=readflag) DRIVE%SLCG
     read(unit=94,fmt=*,iostat=readflag) DRIVE%BLCG
     read(unit=94,fmt=*,iostat=readflag) DRIVE%WLCG
     read(unit=94,fmt=*,iostat=readflag) DRIVE%SLREEL
     read(unit=94,fmt=*,iostat=readflag) DRIVE%BLREEL
     read(unit=94,fmt=*,iostat=readflag) DRIVE%WLREEL
     read(unit=94,fmt=*,iostat=readflag) DRIVE%IXX
     read(unit=94,fmt=*,iostat=readflag) DRIVE%IYY
     read(unit=94,fmt=*,iostat=readflag) DRIVE%IZZ
     read(unit=94,fmt=*,iostat=readflag) DRIVE%IXY
     read(unit=94,fmt=*,iostat=readflag) DRIVE%IXZ
     read(unit=94,fmt=*,iostat=readflag) DRIVE%IYZ
     read(unit=94,fmt=*,iostat=readflag) DRIVE%TURNRADIUS
     read(unit=94,fmt=*,iostat=readflag) DRIVE%ALC
     read(unit=94,fmt=*,iostat=readflag) DRIVE%ALS
     read(unit=94,fmt=*,iostat=readflag) DRIVE%DXD
     read(unit=94,fmt=*,iostat=readflag) DRIVE%DYD
     read(unit=94,fmt=*,iostat=readflag) DRIVE%RNEW
     read(unit=94,fmt=*,iostat=readflag) DRIVE%C_T
     read(unit=94,fmt=*,iostat=readflag) DRIVE%C_TAU
     read(unit=94,fmt=*,iostat=readflag) DRIVE%LPHI12
     read(unit=94,fmt=*,iostat=readflag) DRIVE%LPHI34
     read(unit=94,fmt=*,iostat=readflag) DRIVE%LTHETA12
     read(unit=94,fmt=*,iostat=readflag) DRIVE%LTHETA34
     read(unit=94,fmt=*,iostat=readflag) DRIVE%OMEGAMAX
     read(unit=94,fmt=*,iostat=readflag) DRIVE%IRR
     read(unit=94,fmt=*,iostat=readflag) DRIVE%MS_MIN
     read(unit=94,fmt=*,iostat=readflag) DRIVE%MS_MAX
     read(unit=94,fmt=*,iostat=readflag) readreal; DRIVE%CONTROLOFFON = int(readreal)
     read(unit=94,fmt=*,iostat=readflag) DRIVE%KPXDRIVE
     read(unit=94,fmt=*,iostat=readflag) DRIVE%KIXDRIVE
     read(unit=94,fmt=*,iostat=readflag) DRIVE%KDXDRIVE
     read(unit=94,fmt=*,iostat=readflag) DRIVE%KPYDRIVE
     read(unit=94,fmt=*,iostat=readflag) DRIVE%KIYDRIVE
     read(unit=94,fmt=*,iostat=readflag) DRIVE%KDYDRIVE
     read(unit=94,fmt=*,iostat=readflag) DRIVE%KPZDRIVE
     read(unit=94,fmt=*,iostat=readflag) DRIVE%KIZDRIVE
     read(unit=94,fmt=*,iostat=readflag) DRIVE%KDZDRIVE
     read(unit=94,fmt=*,iostat=readflag) DRIVE%KPPHI
     read(unit=94,fmt=*,iostat=readflag) DRIVE%KIPHI
     read(unit=94,fmt=*,iostat=readflag) DRIVE%KDPHI
     read(unit=94,fmt=*,iostat=readflag) DRIVE%KPTHETA
     read(unit=94,fmt=*,iostat=readflag) DRIVE%KITHETA
     read(unit=94,fmt=*,iostat=readflag) DRIVE%KDTHETA
     read(unit=94,fmt=*,iostat=readflag) DRIVE%KPPSI
     read(unit=94,fmt=*,iostat=readflag) DRIVE%KIPSI
     read(unit=94,fmt=*,iostat=readflag) DRIVE%KDPSI
     read(unit=94,fmt=*,iostat=readflag) DRIVE%XINTEGRAL 
     read(unit=94,fmt=*,iostat=readflag) DRIVE%YINTEGRAL 
     read(unit=94,fmt=*,iostat=readflag) DRIVE%ZINTEGRAL
     read(unit=94,fmt=*,iostat=readflag) DRIVE%PHIINTEGRAL 
     read(unit=94,fmt=*,iostat=readflag) DRIVE%THETAINTEGRAL
     read(unit=94,fmt=*,iostat=readflag) DRIVE%PSIINTEGRAL
     read(unit=94,fmt=*,iostat=readflag) DRIVE%UCOMMAND !!Changed to UCOMMAND for forward flight 
     read(unit=94,fmt=*,iostat=readflag) DRIVE%YCOMMAND
     read(unit=94,fmt=*,iostat=readflag) DRIVE%ZCOMMAND
     read(unit=94,fmt=*,iostat=readflag) DRIVE%UINTEGRAL
     !read(unit=94,fmt=*,iostat=readflag) DRIVE%WAYPOINT

     DRIVE%WAYPOINT = 1

     !!!DO SOME CALCULATIONS ON driver
     DRIVE%MASS = DRIVE%WEIGHT/DRIVE%GRAVITY 
     deti = + DRIVE%IXX*(DRIVE%IYY*DRIVE%IZZ-DRIVE%IYZ*DRIVE%IYZ) - DRIVE%IXY*(DRIVE%IXY*DRIVE%IZZ-DRIVE%IYZ*DRIVE%IXZ) + DRIVE%IXZ*(DRIVE%IXY*DRIVE%IYZ-DRIVE%IYY*DRIVE%IXZ)
     DRIVE%IXXI = (DRIVE%IYY*DRIVE%IZZ-DRIVE%IYZ*DRIVE%IYZ)/deti
     DRIVE%IXYI = (DRIVE%IYZ*DRIVE%IXZ-DRIVE%IXY*DRIVE%IZZ)/deti
     DRIVE%IXZI = (DRIVE%IXY*DRIVE%IYZ-DRIVE%IYY*DRIVE%IXZ)/deti
     DRIVE%IYYI = (DRIVE%IXX*DRIVE%IZZ-DRIVE%IXZ*DRIVE%IXZ)/deti
     DRIVE%IYZI = (DRIVE%IXY*DRIVE%IXZ-DRIVE%IXX*DRIVE%IYZ)/deti
     DRIVE%IZZI = (DRIVE%IXX*DRIVE%IYY-DRIVE%IXY*DRIVE%IXY)/deti
  else     
     read(unit=94,fmt=*,iostat=readflag) DRIVE%SLREEL
     read(unit=94,fmt=*,iostat=readflag) DRIVE%BLREEL
     read(unit=94,fmt=*,iostat=readflag) DRIVE%WLREEL
     read(unit=94,fmt=*,iostat=readflag) DRIVE%AIRWAKE
     !Read location of airwake data
     read(unit=94,fmt=*,iostat=readflag) DRIVE%AIRWAKEPATH
     read(unit=94,fmt=*,iostat=readflag) DRIVE%SLAIRWAKE
     read(unit=94,fmt=*,iostat=readflag) DRIVE%BLAIRWAKE
     read(unit=94,fmt=*,iostat=readflag) DRIVE%WLAIRWAKE

     if (DRIVE%AIRWAKE .eq. 1) then

        !Set TCOORD
        do i = 1,NTIMES
           DRIVE%TCOORD(i) = 0 + (i-1)*1.0D0
        end do

        !%%%%%%%Import Initial UVW matrices%%%%%%%%% */

        write(number, '(i1)' )  1
        letter = trim('U')
        DRIVE%U0name = trim(DRIVE%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        letter = trim('V')
        DRIVE%V0name = trim(DRIVE%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        letter = trim('W')
        DRIVE%W0name = trim(DRIVE%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        write(number, '(i1)' )  2
        letter = trim('U')
        DRIVE%Udtname = trim(DRIVE%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        letter = trim('V')
        DRIVE%Vdtname = trim(DRIVE%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        letter = trim('W')
        DRIVE%Wdtname = trim(DRIVE%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'

        call IMPORTWAKE(DRIVE%UDRIVER,DRIVE%U0name);
        call IMPORTWAKE(DRIVE%UDRIVERDT,DRIVE%Udtname);
        call IMPORTWAKE(DRIVE%VDRIVER,DRIVE%V0name);
        call IMPORTWAKE(DRIVE%VDRIVERDT,DRIVE%Vdtname);
        call IMPORTWAKE(DRIVE%WDRIVER,DRIVE%W0name);
        call IMPORTWAKE(DRIVE%WDRIVERDT,DRIVE%Wdtname);

        !!Import X,Y,Z Grid data
        xgridname = trim(DRIVE%AIRWAKEPATH)//'X_Grid.txt'
        ygridname = trim(DRIVE%AIRWAKEPATH)//'Y_Grid.txt'
        zgridname = trim(DRIVE%AIRWAKEPATH)//'Z_Grid.txt'

        call IMPORTWAKE(DRIVE%XDRIVER,xgridname);
        call IMPORTWAKE(DRIVE%YDRIVER,ygridname);
        call IMPORTWAKE(DRIVE%ZDRIVER,zgridname);

        do i = 1,IMAX
           DRIVE%XCOORD(i) = DRIVE%XDRIVER(i,1,1);
        end do

        write(*,*) 'Wake Data Load Complete'
     end if

  end if
  

  if ((DRIVE%MODNO .eq. 1) .or. (DRIVE%MODNO .eq. 0)) then
     read(unit=94,fmt=*,iostat=readflag) DRIVE%XCGINITIAL
     read(unit=94,fmt=*,iostat=readflag) DRIVE%YCGINITIAL
     read(unit=94,fmt=*,iostat=readflag) DRIVE%ZCGINITIAL
     read(unit=94,fmt=*,iostat=readflag) DRIVE%FINALSPEED
     read(unit=94,fmt=*,iostat=readflag) DRIVE%PSI
     read(unit=94,fmt=*,iostat=readflag) DRIVE%DOWNWASHONOFF
     read(unit=94,fmt=*,iostat=readflag) DRIVE%DOWNWASH
     read(unit=94,fmt=*,iostat=readflag) DRIVE%DIAMETER
     read(unit=94,fmt=*,iostat=readflag) DRIVE%XDDOTNOISE
     read(unit=94,fmt=*,iostat=readflag) DRIVE%YDDOTSCALE
     read(unit=94,fmt=*,iostat=readflag) DRIVE%YDDOTPERIOD
  end if
  if (DRIVE%MODNO .eq. 2) then
   read(unit=94,fmt=*,iostat=readflag) readreal; DRIVE%TABSIZE = int(readreal)
   do i=1,DRIVE%TABSIZE  
    read(unit=94,fmt=*,iostat=readflag) DRIVE%TIMETAB(i),DRIVE%XCGTAB(i),DRIVE%YCGTAB(i),DRIVE%ZCGTAB(i),DRIVE%PHITAB(i),DRIVE%THETATAB(i),DRIVE%PSITAB(i), & 
                                        DRIVE%UBTAB(i),DRIVE%VBTAB(i),DRIVE%WBTAB(i),DRIVE%PBTAB(i),DRIVE%QBTAB(i),DRIVE%RBTAB(i)
   end do
  end if

  close(94) 
  write(*,*) 'DRIVER Load Complete'

  DRIVE%DQFLAG = 1

      
  RETURN
   
 end if
   
 RETURN
END SUBROUTINE DRIVER

!!Import routine to load data files placed in text files
SUBROUTINE IMPORTWAKE(mat,filename)
  use DRIVERDATATYPES
  implicit none
  integer uvw,time
  integer ii,jj,kk,nii,njj,ierr;
  real*8 mat(55,77,61),tempmat(77)
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

SUBROUTINE AIRWAKE(DRIVE,XI,YI,ZI)
  use DRIVERDATATYPES
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
  type(DRIVERSTRUCTURE) DRIVE

  !   /*   %%This function will take in x,y,z(ft),t(sec) and location and  */
  !   /*   %return u,v,w(m/s). This uses a fast quad-linear interpolation */
  ! using matrices XDRIVER,YDRIVER,ZDRIVER to interpolate against
  !   /*   %so many globals must be defined. location is a string that */
  !   /*   %contains the location of the data to be interpolated. */
  
  DRIVE%VWAKE(1) = 0
  DRIVE%VWAKE(2) = 0
  DRIVE%VWAKE(3) = 0

  rI_I(1,1) = XI
  rI_I(2,1) = YI
  rI_I(3,1) = ZI
  
  rS_I(1,1) = DRIVE%XCG
  rS_I(2,1) = DRIVE%YCG
  rS_I(3,1) = DRIVE%ZCG

  rAIRWAKE_S(1,1) = DRIVE%SLAIRWAKE
  rAIRWAKE_S(2,1) = DRIVE%BLAIRWAKE
  rAIRWAKE_S(3,1) = DRIVE%WLAIRWAKE

  rAwP_S = matmul(transpose(DRIVE%TIS),rI_I-rS_I)-rAIRWAKE_S

  xstar = -rAwP_S(1,1)
  ystar = rAwP_S(2,1)
  zstar = -rAwP_S(3,1)

  !!Loop tstar
  tstar = DRIVE%TIME
  tshift = 0
  if (tstar .gt. DRIVE%TCOORD(NTIMES)) then
     tshift = -floor(abs(tstar-DRIVE%TCOORD(1))/DRIVE%TCOORD(NTIMES))
  end if
  tstar = tstar + tshift*DRIVE%TCOORD(NTIMES)
  ! write(*,*) 'Tstar = ',tstar, 'tshift = ',tshift

  stepX = 1;stepY = 1;stepZ = 1;stepT = 1;
  extrapX = 0;extrapY = 0;extrapZ = 0;extrapT = 0;
  if (zstar .lt. 0) then
     zstar = -zstar
  endif
  tinterp = 2;

  uvw(1,1)=0;uvw(2,1)=0;uvw(3,1)=0;
  uvw(1,2)=0;uvw(2,2)=0;uvw(3,2)=0;

  markX = DRIVE%markX
  markY = DRIVE%markY
  markZ = DRIVE%markZ
  markT = DRIVE%markT

  !%%Check X
  if (markX .eq. IMAX) then
     markX = markX - 1;
  end if
  if ((xstar .ge. DRIVE%XCOORD(markX)) .and. (xstar .le. DRIVE%XCOORD(markX+1))) then
     !%%You're in between the markers so keep going
  else
     if (xstar .gt. DRIVE%XCOORD(IMAX)) then
        markX = IMAX;
        stepX = -1;
        extrapX = 1;
        ! write(*,*) 'Out of bounds on x'
     elseif (xstar .lt. DRIVE%XCOORD(1)) then
        markX = 1;
        stepX = 1;
        extrapX = 1;
     else
        call FINDGE(DRIVE%XCOORD,IMAX,xstar,markX)
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
     DRIVE%YCOORD(j) = DRIVE%YDRIVER(markX,j,1); 
     ! write(*,*) DRIVE%YCOORD(j)
  end do
  ! write(*,*) '------------'
  do k = 1,KMAX
     DRIVE%ZCOORD(k) = DRIVE%ZDRIVER(markX,1,k);    
     ! write(*,*) DRIVE%ZCOORD(k)
  end do

  !%%Check Y
  if (markY .eq. JMAX) then
     markY = markY - 1;
  end if
  if ((ystar .ge. DRIVE%YCOORD(markY)) .and. (ystar .le. DRIVE%YCOORD(markY+1))) then
     !%%You're in between the markers so keep going
  else
     if (ystar .gt. DRIVE%YCOORD(JMAX)) then
        markY = JMAX;
        stepY = -1;
        extrapY = 1;
     elseif (ystar .lt. DRIVE%YCOORD(1)) then
        markY = 1;
        stepY = 1;
        extrapY = 1;
     else
        call FINDGE(DRIVE%YCOORD,JMAX,ystar,markY)
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

  if ((zstar .ge. DRIVE%ZCOORD(markZ)) .and. (zstar .le. DRIVE%ZCOORD(markZ+1))) then
     !%%You're in between the markers so keep going
  else
     !%Find markZ
     if (zstar .gt. DRIVE%ZCOORD(KMAX)) then
        !%use endpt
        markZ = KMAX;
        stepZ = -1;
        extrapZ = 1;
        DRIVE%VWAKE(1) = 0
        DRIVE%VWAKE(2) = 0
        DRIVE%VWAKE(3) = 0
        RETURN
     else if (zstar .lt. DRIVE%ZCOORD(1)) then
        markZ = 1;
        stepZ = 1;
        extrapZ = 1;
     else
        call FINDGE(DRIVE%ZCOORD,KMAX,zstar,markZ)
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
  !write(*,*) tstar,markT,DRIVE%TCOORD(markT)
  if ((tstar .ge. DRIVE%TCOORD(markT)) .and. (tstar .le. DRIVE%TCOORD(markT+1))) then
     !%%You're in between the markers so keep going
  else
     !%Find markT
     if (tstar .gt. DRIVE%TCOORD(NTIMES)) then
        !%use endpt
        markT = NTIMES;
        extrapT = 1;
     else if (tstar .lt. DRIVE%TCOORD(1)) then
        !%use start pt
        markT = 1;
        extrapT = 1;
     else
        call FINDGE(DRIVE%TCOORD,NTIMES,tstar,markT)
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
     DRIVE%U0name = trim(DRIVE%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
     letter = trim('V')
     DRIVE%V0name = trim(DRIVE%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
     letter = trim('W')
     DRIVE%W0name = trim(DRIVE%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
     !%%only import at markT
     call IMPORTWAKE(DRIVE%UDRIVER,DRIVE%U0name);
     call IMPORTWAKE(DRIVE%VDRIVER,DRIVE%V0name);
     call IMPORTWAKE(DRIVE%WDRIVER,DRIVE%W0name);
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
        DRIVE%Udtname = trim(DRIVE%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        letter = trim('V')
        DRIVE%Vdtname = trim(DRIVE%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        letter = trim('W')
        DRIVE%Wdtname = trim(DRIVE%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        !%%only import at markT
        call IMPORTWAKE(DRIVE%UDRIVERDT,DRIVE%Udtname);
        call IMPORTWAKE(DRIVE%VDRIVERDT,DRIVE%Vdtname);
        call IMPORTWAKE(DRIVE%WDRIVERDT,DRIVE%Wdtname);
     end if !(extrapT eq. 1 ) 
  end if

  ! write(*,*) 'markT = ',markT
  ! write(*,*) 'tinterp = ',tinterp

  !%%Interpolation Scheme
  do tt = 1,tinterp 
     !%Interpolate Spatially

     !%%To start we have 8 discrete point (8 corners of a cube)
     xpts2(1) = DRIVE%XCOORD(markX)
     xpts2(2) = DRIVE%XCOORD(markX+stepX);
     ypts2(1) = DRIVE%YCOORD(markY)
     ypts2(2) = DRIVE%YCOORD(markY+stepY);
     zpts2(1) = DRIVE%ZCOORD(markZ)
     zpts2(2) = DRIVE%ZCOORD(markZ+stepZ);
     x1 = markX;x2 = markX+stepX;
     y1 = markY;y2 = (markY+stepY);
     z1 = markZ;z2 = markZ+stepZ;
     ! write(*,*) 'x1,x2,y1,y2,z1,z2 = ',x1,x2,y1,y2,z1,z2
     if (tt .eq. 1) then
        !%%Use UDRIVER,VDRIVER,WDRIVER
        u8(1) = DRIVE%UDRIVER(x1,y1,z1);
        u8(2) = DRIVE%UDRIVER(x2,y1,z1);
        u8(3) = DRIVE%UDRIVER(x2,y2,z1);
        u8(4) = DRIVE%UDRIVER(x1,y2,z1);
        u8(5) = DRIVE%UDRIVER(x1,y1,z2);
        u8(6) = DRIVE%UDRIVER(x2,y1,z2);
        u8(7) = DRIVE%UDRIVER(x2,y2,z2);
        u8(8) = DRIVE%UDRIVER(x1,y2,z2);
        v8(1) = DRIVE%VDRIVER(x1,y1,z1);
        v8(2) = DRIVE%VDRIVER(x2,y1,z1);
        v8(3) = DRIVE%VDRIVER(x2,y2,z1);
        v8(4) = DRIVE%VDRIVER(x1,y2,z1);
        v8(5) = DRIVE%VDRIVER(x1,y1,z2);
        v8(6) = DRIVE%VDRIVER(x2,y1,z2);
        v8(7) = DRIVE%VDRIVER(x2,y2,z2);
        v8(8) = DRIVE%VDRIVER(x1,y2,z2);
        w8(1) = DRIVE%WDRIVER(x1,y1,z1);
        w8(2) = DRIVE%WDRIVER(x2,y1,z1);
        w8(3) = DRIVE%WDRIVER(x2,y2,z1);
        w8(4) = DRIVE%WDRIVER(x1,y2,z1);
        w8(5) = DRIVE%WDRIVER(x1,y1,z2);
        w8(6) = DRIVE%WDRIVER(x2,y1,z2);
        w8(7) = DRIVE%WDRIVER(x2,y2,z2);
        w8(8) = DRIVE%WDRIVER(x1,y2,z2);
        ! do i = 1,8
        !    write(*,*) 'u8 = ',u8(i)
        ! end do
     else
        !%%Use Udt,Vdt,Wdt
        u8(1) = DRIVE%UDRIVERDT(x1,y1,z1);
        u8(2) = DRIVE%UDRIVERDT(x2,y1,z1);
        u8(3) = DRIVE%UDRIVERDT(x2,y2,z1);
        u8(4) = DRIVE%UDRIVERDT(x1,y2,z1);
        u8(5) = DRIVE%UDRIVERDT(x1,y1,z2);
        u8(6) = DRIVE%UDRIVERDT(x2,y1,z2);
        u8(7) = DRIVE%UDRIVERDT(x2,y2,z2);
        u8(8) = DRIVE%UDRIVERDT(x1,y2,z2);
        v8(1) = DRIVE%VDRIVERDT(x1,y1,z1);
        v8(2) = DRIVE%VDRIVERDT(x2,y1,z1);
        v8(3) = DRIVE%VDRIVERDT(x2,y2,z1);
        v8(4) = DRIVE%VDRIVERDT(x1,y2,z1);
        v8(5) = DRIVE%VDRIVERDT(x1,y1,z2);
        v8(6) = DRIVE%VDRIVERDT(x2,y1,z2);
        v8(7) = DRIVE%VDRIVERDT(x2,y2,z2);
        v8(8) = DRIVE%VDRIVERDT(x1,y2,z2);
        w8(1) = DRIVE%WDRIVERDT(x1,y1,z1);
        w8(2) = DRIVE%WDRIVERDT(x2,y1,z1);
        w8(3) = DRIVE%WDRIVERDT(x2,y2,z1);
        w8(4) = DRIVE%WDRIVERDT(x1,y2,z1);
        w8(5) = DRIVE%WDRIVERDT(x1,y1,z2);
        w8(6) = DRIVE%WDRIVERDT(x2,y1,z2);
        w8(7) = DRIVE%WDRIVERDT(x2,y2,z2);
        w8(8) = DRIVE%WDRIVERDT(x1,y2,z2);
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
     tpts(1) = DRIVE%TCOORD(markT)
     tpts(2) = DRIVE%TCOORD(markT+1);
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

  DRIVE%VWAKE(1) = -(vatm(1) - DRIVE%UDRIVER(markX,markY,KMAX))
  DRIVE%VWAKE(2) = vatm(2)
  DRIVE%VWAKE(3) = -vatm(3)

  DRIVE%markX = markX
  DRIVE%markY = markY
  DRIVE%markZ = markZ
  DRIVE%markT = markT

END SUBROUTINE AIRWAKE

SUBROUTINE DOWNWASH(DRIVE,STATE)
  use DRIVERDATATYPES
  implicit none
  real*8 xcg,ycg,zcg,downwash_angle,zdownwash,delx,dely,delz,n,STATE(3)
  type(DRIVERSTRUCTURE) DRIVE

  DRIVE%VWAKE = 0

  !Extract state of Towed Body
  xcg = STATE(1)
  ycg = STATE(2)
  zcg = STATE(3)

  !Compute z location of downwash
  downwash_angle = atan2(DRIVE%DOWNWASH,DRIVE%SPEED)
  zdownwash = delx*tan(downwash_angle)

  !Check and see if the towed body is in Driver airwake
  delx = abs(DRIVE%XCG - xcg)
  dely = abs(DRIVE%YCG - ycg)
  delz = abs(abs(DRIVE%ZCG - zcg) - zdownwash)

  if ((dely .lt. DRIVE%DIAMETER) .and. (delz .lt. DRIVE%DIAMETER)) then
     DRIVE%VWAKE(1) = 0.0
     DRIVE%VWAKE(2) = 0.0
     call RandUniform(n)
     DRIVE%VWAKE(3) = DRIVE%DOWNWASH*(1+(1-2*n)*0.1D0)
  end if

END SUBROUTINE DOWNWASH

SUBROUTINE DRIVER_CONTROL(DRIVE)
  use DRIVERDATATYPES
  implicit none
  type(DRIVERSTRUCTURE) DRIVE
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

  if (DRIVE%CONTROLOFFON .eq. 1) then
     !Quadcopter control
     DRIVE%OMEGAVEC = 0
     DRIVE%THRUSTVEC = 0
     DRIVE%MUVEC = 0

     !Get OMEGA0 and KT for DRIVE
     DRIVE%KT = DRIVE%C_T*((4.0D0*DRIVE%DEN*(DRIVE%RNEW**4))/(qPI**2))
     DRIVE%OMEGA0 = sqrt(DRIVE%WEIGHT/(4.0D0*DRIVE%KT))

     xdot = DRIVE%STATEDOT(1)
     ydot = DRIVE%STATEDOT(2)
     zdot = DRIVE%STATEDOT(3)
     phi = DRIVE%STATE(4)
     theta = DRIVE%STATE(5)
     psi = DRIVE%STATE(6)
     p = DRIVE%STATE(10)
     q = DRIVE%STATE(11)
     r = DRIVE%STATE(12)

     ! Don't forget to change 'nwp' in declaration block for number of waypoints

     if (nwp .eq. 1) then
        !! Just trying to stay in hover for now
        xwaypoint = 0.0D0
        ywaypoint = 0.0D0
        zwaypoint = -20.0D0

     elseif(nwp .eq. 2) then
        ! Up & Down Test, 2
        xwaypoint(1,1) = 0.0D0
        xwaypoint(2,1) = 0.0D0

        ywaypoint(1,1) = 0.0D0
        ywaypoint(2,1) = 0.0D0

        zwaypoint(1,1) = -85.0D0
        zwaypoint(2,1) = -10.0D0
     elseif (nwp .eq. 4) then
        ! Square Pattern, 4

        xwaypoint(1,1) = 40.0D0
        xwaypoint(2,1) = 40.0D0
        xwaypoint(3,1) = 0.0D0
        xwaypoint(4,1) = 0.0D0

        ywaypoint(1,1) = 0.0D0
        ywaypoint(2,1) = 40.0D0
        ywaypoint(3,1) = 40.0D0
        ywaypoint(4,1) = 0.0D0

        ! Keeping constant altitude
        do ctr = 1,nwp
           zwaypoint(ctr,1) = -20.0D0
        end do

     elseif (nwp .eq. 6) then
        ! Square Pattern, 4

        xwaypoint(1,1) = 100.0D0
        xwaypoint(2,1) = 0.0D0
        xwaypoint(3,1) = 0.0D0
        xwaypoint(4,1) = 0.0D0
        xwaypoint(5,1) = 0.0D0
        xwaypoint(6,1) = 0.0D0

        ywaypoint(1,1) = 0.0D0
        ywaypoint(2,1) = 0.0D0
        ywaypoint(3,1) = 100.0D0
        ywaypoint(4,1) = 0.0D0
        ywaypoint(5,1) = 0.0D0
        ywaypoint(6,1) = 0.0D0

        ! Keeping constant altitude
        do ctr = 1,nwp-2
           zwaypoint(ctr,1) = -10.0D0
        end do
        zwaypoint(5,1) = -70.0D0
        zwaypoint(6,1) = -10.0D0

     elseif (nwp .eq. 8) then
        !      Octagon Pattern, 8
        xwaypoint(1,1) = 0
        xwaypoint(2,1) = 50.0D0
        xwaypoint(3,1) = 150.0D0
        xwaypoint(4,1) = 200.0D0
        xwaypoint(5,1) = 200.0D0
        xwaypoint(6,1) = 150.0D0
        xwaypoint(7,1) = 50.0D0
        xwaypoint(8,1) = 0.0D0

        ywaypoint(1,1) = 100.0D0
        ywaypoint(2,1) = 200.0D0
        ywaypoint(3,1) = 200.0D0
        ywaypoint(4,1) = 100.0D0
        ywaypoint(5,1) = 0.0D0
        ywaypoint(6,1) = -100.0D0
        ywaypoint(7,1) = -100.0D0
        ywaypoint(8,1) = 0.0D0

        do ctr = 1,nwp
           zwaypoint(ctr,1) = -10.0D0
        end do
     end if

     ! gotocontrols
     ! DRIVE%XCOMMAND =  xwaypoint(DRIVE%WAYPOINT,1)
     ! DRIVE%YCOMMAND =  ywaypoint(DRIVE%WAYPOINT,1)
     ! DRIVE%ZCOMMAND =  zwaypoint(DRIVE%WAYPOINT,1)

     !DRIVE%ZCOMMAND = -30.0 -- these are now set in the input file 
     !DRIVE%UCOMMAND = 28.93
     !DRIVE%YCOMMAND = 0.0

     !delx = (DRIVE%XCOMMAND - DRIVE%STATE(1))*-1.00D0
     dely = (DRIVE%YCOMMAND - DRIVE%STATE(2))
     !write(*,*) 'Zstuff = ',DRIVE%ZCOMMAND,DRIVE%STATE(3)
     delz = (DRIVE%ZCOMMAND - DRIVE%STATE(3))*-1.00D0

     ! Dwaypoint = sqrt((delx)**2 + (dely)**2 + (delz)**2)

     ! Change this to 4(square) or 8(octagon) depending on the shape you want to simulate
     ! if (Dwaypoint .lt. 1.0D0) then
     !   DRIVE%WAYPOINT= DRIVE%WAYPOINT+1
     !   if (DRIVE%WAYPOINT .gt. nwp) then
     !     DRIVE%WAYPOINT=1
     !   end if
     ! end if

     !!! Attitude Controller
     DRIVE%PHICOMMAND = DRIVE%KPYDRIVE*dely + DRIVE%KIYDRIVE*DRIVE%YINTEGRAL - DRIVE%KDYDRIVE*ydot

     !DRIVE%PHICOMMAND = 20.0*qPI/180.0
     
     ! REVISIT hardcoded xdotcommand = 40 ft/s
     ! For now the KP and KD gains for 'x' is controlling the 'u' velocity at which the quad is travelling
     ! Ok shit so the 'u' velocity couldn't really be controlled so -24.5 deg is what 
     ! the quad needs to be to achieve this velocity
     ! DRIVE%THETACOMMAND = -24.5*qPI/180!DRIVE%KPXDRIVE*(DRIVE%STATE(7) - 40.0) + DRIVE%KIXDRIVE*DRIVE%XINTEGRAL + DRIVE%KDXDRIVE*(DRIVE%STATEDOT(7))
     ! DRIVE%THETACOMMAND = DRIVE%KPXDRIVE*delx + DRIVE%KIXDRIVE*DRIVE%XINTEGRAL - DRIVE%KDXDRIVE*xdot

     DRIVE%THETACOMMAND = DRIVE%KPXDRIVE*(DRIVE%STATE(7) - DRIVE%UCOMMAND) + DRIVE%KIXDRIVE*DRIVE%UINTEGRAL

     !write(*,*) 'integral = ',DRIVE%YINTEGRAL,DRIVE%XINTEGRAL

     !DRIVE%THETACOMMAND = 0.0
     
     DRIVE%PSICOMMAND = 0.0

     !write(*,*) 'Xcontrol = ',delx,DRIVE%XINTEGRAL,xdot
     !write(*,*) 'ptpcom = ',DRIVE%PHICOMMAND,DRIVE%THETACOMMAND,DRIVE%PSICOMMAND

     if (abs(DRIVE%THETACOMMAND) .gt. 30*qPI/180) then
        DRIVE%THETACOMMAND = sign(30*qPI/180,DRIVE%THETACOMMAND)
     end if
     if (abs(DRIVE%PHICOMMAND) .gt. 30*qPI/180) then
        DRIVE%PHICOMMAND = sign(30*qPI/180,DRIVE%PHICOMMAND)
     end if
     if (abs(DRIVE%PSICOMMAND) .gt. 30*qPI/180) then
        DRIVE%PSICOMMAND = sign(30*qPI/180,DRIVE%PSICOMMAND)
     end if

     ! Hovering microseconds and altitude control
     munominal = 1614.855 + DRIVE%KPZDRIVE*(delz) + DRIVE%KIZDRIVE*DRIVE%ZINTEGRAL +DRIVE%KDZDRIVE*zdot  ! Nominal microsecond pulse for hover

     DRIVE%MS_ROLL = DRIVE%KPPHI*(DRIVE%PHICOMMAND-phi) + DRIVE%KIPHI*DRIVE%PHIINTEGRAL - DRIVE%KDPHI*p
     DRIVE%MS_PITCH = DRIVE%KPTHETA*(DRIVE%THETACOMMAND - theta) + DRIVE%KITHETA*DRIVE%THETAINTEGRAL- DRIVE%KDTHETA*q
     DRIVE%MS_YAW = DRIVE%KPPSI*(DRIVE%PSICOMMAND-psi) + DRIVE%KIPSI*DRIVE%PSIINTEGRAL - DRIVE%KDPSI*r

     !write(*,*) 'Att = ',phi,theta,psi,p,q,r

     DRIVE%MUVEC(1,1) = munominal + DRIVE%MS_ROLL + DRIVE%MS_PITCH + DRIVE%MS_YAW
     DRIVE%MUVEC(2,1) = munominal - DRIVE%MS_ROLL + DRIVE%MS_PITCH - DRIVE%MS_YAW
     DRIVE%MUVEC(3,1) = munominal - DRIVE%MS_ROLL - DRIVE%MS_PITCH + DRIVE%MS_YAW
     DRIVE%MUVEC(4,1) = munominal + DRIVE%MS_ROLL - DRIVE%MS_PITCH - DRIVE%MS_YAW

     !write(*,*) 'muvec = ',DRIVE%MUVEC

     ! Now we saturate the microseconds so that it doesn't go over 1900 or under 1100
     do j = 1,4
        if (DRIVE%MUVEC(j,1) .gt. 1900.00D0) then
           DRIVE%OMEGAVEC(j,1) = 1900.00D0
        end if
        if (DRIVE%MUVEC(j,1) .lt. 1100.00D0) then
           DRIVE%MUVEC(j,1) = 1100.00D0
        end if
     end do

  end if
end SUBROUTINE DRIVER_CONTROL

SUBROUTINE COMPUTEINTEGRAL(DRIVE)
 use DRIVERDATATYPES
 implicit none
 type(DRIVERSTRUCTURE) DRIVE
 ! Using trapezoidal rule(ish) to compute integral error term
 ! REVISIT Replaced xintegral with u velocity term!!!!
 ! The 1/4 is used because this is called once every rk4 call
 ! and since we're using trap(ish) we need to divide by 4. Untested CJM - 2/13/2018
 DRIVE%XINTEGRAL     = DRIVE%XINTEGRAL     + -(1.0/4.0)*((DRIVE%XCOMMAND     - DRIVE%STATE(1))/2)*DRIVE%DELTATIME
 DRIVE%YINTEGRAL     = DRIVE%YINTEGRAL     + (1.0/4.0)*((DRIVE%YCOMMAND     - DRIVE%STATE(2))/2)*DRIVE%DELTATIME
 DRIVE%ZINTEGRAL     = DRIVE%ZINTEGRAL     + -(1.0/4.0)*((DRIVE%ZCOMMAND     - DRIVE%STATE(3))/2)*DRIVE%DELTATIME
 DRIVE%PHIINTEGRAL   = DRIVE%PHIINTEGRAL   +    (1.0/4.0)*((DRIVE%PHICOMMAND   - DRIVE%STATE(4))/2)*DRIVE%DELTATIME
 DRIVE%THETAINTEGRAL = DRIVE%THETAINTEGRAL +    (1.0/4.0)*((DRIVE%THETACOMMAND - DRIVE%STATE(5))/2)*DRIVE%DELTATIME
 DRIVE%PSIINTEGRAL   = DRIVE%PSIINTEGRAL   +    (1.0/4.0)*((DRIVE%PHICOMMAND   - DRIVE%STATE(6))/2)*DRIVE%DELTATIME
 DRIVE%UINTEGRAL     = DRIVE%UINTEGRAL     + -(1.0/4.0)*((DRIVE%UCOMMAND     - DRIVE%STATE(7))/2)*DRIVE%DELTATIME
END SUBROUTINE COMPUTEINTEGRAL !COMPUTEINTEGRAL

SUBROUTINE PRINTINTEGRAL(DRIVE) !!When do I run this?
 use DRIVERDATATYPES
 implicit none
 type(DRIVERSTRUCTURE) DRIVE
 ! Using trapezoidal rule(ish) to compute integral error term
 ! REVISIT Replaced xintegral with u velocity term!!!!
 ! The 1/4 is used because this is called once every rk4 call
 ! and since we're using trap(ish) we need to divide by 4. Untested CJM - 2/13/2018
 write(*,*) DRIVE%XINTEGRAL,DRIVE%YINTEGRAL,DRIVE%ZINTEGRAL,DRIVE%PHIINTEGRAL,DRIVE%THETAINTEGRAL,DRIVE%PSIINTEGRAL,DRIVE%UINTEGRAL
 !95713.325840134436       0.77060269505657364       -38.613821568062889        5.6208864356321653E-005  -3.6252368913735496E-003   1.5246634597683634E-002  -25.123939964081060 
END SUBROUTINE PRINTINTEGRAL !PRINTINTEGRAL

SUBROUTINE PWM2FORCE(DRIVE)
  use DRIVERDATATYPES
  implicit none
  type(DRIVERSTRUCTURE) DRIVE
  real*8 :: pp = 4

   ! IF (abs(DRIVE%MS_ROLL) .gt. DRIVE%MS_MAX) then
   !    DRIVE%MS_ROLL = sign(DRIVE%MS_MAX,DRIVE%MS_ROLL)
   ! elseif (abs(DRIVE%MS_ROLL) .lt. DRIVE%MS_MIN) then
   !    DRIVE%MS_ROLL = sign(DRIVE%MS_MIN,DRIVE%MS_ROLL)
   ! end if

   ! IF (abs(DRIVE%MS_PITCH) .gt. DRIVE%MS_MAX) then
   !    DRIVE%MS_PITCH = sign(DRIVE%MS_MAX,DRIVE%MS_PITCH)
   ! elseif (abs(DRIVE%MS_PITCH) .lt. DRIVE%MS_MIN) then
   !    DRIVE%MS_PITCH = sign(DRIVE%MS_MIN,DRIVE%MS_PITCH)
   ! end if

   ! IF (abs(DRIVE%MS_YAW) .gt. DRIVE%MS_MAX) then
   !    DRIVE%MS_YAW = sign(DRIVE%MS_MAX,DRIVE%MS_YAW)
   ! elseif (abs(DRIVE%MS_YAW) .lt. DRIVE%MS_MIN) then
   !    DRIVE%MS_YAW = sign(DRIVE%MS_MIN,DRIVE%MS_YAW)
   ! end if
   
  ! DRIVE%MS_0 = 2.35252990909077E-06*DRIVE%MS_0**2 - (0.00487992485215825)*DRIVE%MS_0 + 1.74554459880932
    do pp = 1,4
      DRIVE%PWM2F(pp,1) = 2.35252990909077E-06*DRIVE%MUVEC(1,1)**2 - (0.00487992485215825)*DRIVE%MUVEC(1,1) + 1.74554459880932
    end do
END SUBROUTINE
