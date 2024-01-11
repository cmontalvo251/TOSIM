!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!! COPTER STRUCTURE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
module COPTERDATATYPES
  IMPLICIT NONE
  include 'coptermodule'
end module COPTERDATATYPES

!!!!!!!!!!!!!!!!!!!!!!!!!!!! README !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!! BEFORE YOU CALL COPTER(QUAD,3) to run the routine you must pass a few variables
!! to this data ball. They are listed below.
!! QUAD%TIME -- You must pass the simulation time variable to the local quad variable
!In order to compute the aero model properly you need
!to pass x,y,z to the ATMOSPHERE model
!if (QUAD%AEROOFFON .eq. 1) then
!    ATM%XI = QUAD%STATE(1)
!     ATM%YI = QUAD%STATE(2)
!     ATM%ZI = QUADR%STATE(3)
!    Compute Atmopsheric density and winds - Same for Quad
!    call ATMOSPHERE()
!    !I'm assuming you have an atmospher model that takes XI,YI,ZI
!    !And returns VX,VY,VZ
!    QUAD%VXWIND = ATM%VXWIND
!    QUAD%VYWIND = ATM%VYWIND
!    QUAD%VZWIND = ATM%VZWIND
!    QUAD%DEN = ATM%DEN
!end if
!!!!!!!!!!!!!!!!!!!!!!!!!!! SUBROUTINE COPTER !!!!!!!!!!!!!!!!!!!!!!!!!!!

SUBROUTINE COPTER(QUAD,iflag)
 use COPTERDATATYPES
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
 type(COPTERSTRUCTURE) QUAD

!!!!!!!!!!!!!!!!!!!!!!!!!! COMPUTE FLAG iflag = 3 !!!!!!!!!!!!!!!!!!!!!!!!!
  
 if (iflag .eq. 3) then  
    
    !Extract everything from state vector
    xcg = QUAD%STATE(1)
    ycg = QUAD%STATE(2)
    zcg = QUAD%STATE(3)
    phi = QUAD%STATE(4) 
    theta = QUAD%STATE(5)
    psi = QUAD%STATE(6)
    ub = QUAD%STATE(7)
    vb = QUAD%STATE(8)
    wb = QUAD%STATE(9)
    pb = QUAD%STATE(10)
    qb = QUAD%STATE(11) 
    rb = QUAD%STATE(12)
    TVEC(1)    = QUAD%STATE(13)
    TDOTVEC(1) = QUAD%STATE(14)
    TVEC(2)    = QUAD%STATE(15)
    TDOTVEC(2) = QUAD%STATE(16)
    TVEC(3)    = QUAD%STATE(17)
    TDOTVEC(3) = QUAD%STATE(18)
    TVEC(4)    = QUAD%STATE(19)
    TDOTVEC(4) = QUAD%STATE(20)
    ! TVEC(1) = QUAD%STATE(13)
    ! TVEC(2) = QUAD%STATE(14)
    ! TVEC(3) = QUAD%STATE(15)
    ! TVEC(4) = QUAD%STATE(16)

    !Set some things for other models
    QUAD%XCG = QUAD%STATE(1) 
    QUAD%YCG = QUAD%STATE(2)
    QUAD%ZCG = QUAD%STATE(3)
    QUAD%PSI = QUAD%STATE(6) !psi
    QUAD%SPEED = QUAD%STATE(7) !ub

    !Distance from CG to tether attachment point
    rCF_B(1,1) = QUAD%SLREEL
    rCF_B(2,1) = QUAD%BLREEL 
    rCF_B(3,1) = QUAD%WLREEL

    !- This is where we're going to hi-jack the model and compute the derivatives using Lisa Schibelius'
    !QUADCOPTER MODEL
    if (QUAD%MODNO .eq. 3) then

       ! Unwrap State Vector - This is the same for the QUADCOPTER

       ! Aircraft to Inertial Transformation Matrix - Same for QUADCOPTER
       ctheta = cos(theta);
       stheta = sin(theta);
       ttheta = stheta / ctheta;
       cphi = cos(phi);
       sphi = sin(phi);
       spsi = sin(psi);
       cpsi = cos(psi);
       QUAD%TIC(1,1) = ctheta * cpsi;
       QUAD%TIC(2,1) = ctheta * spsi;
       QUAD%TIC(3,1) = -stheta;
       QUAD%TIC(1,2) = sphi * stheta * cpsi - cphi * spsi;
       QUAD%TIC(2,2) = sphi * stheta * spsi + cphi * cpsi;
       QUAD%TIC(3,2) = sphi * ctheta;
       QUAD%TIC(1,3) = cphi * stheta * cpsi + sphi * spsi;
       QUAD%TIC(2,3) = cphi * stheta * spsi - sphi * cpsi;
       QUAD%TIC(3,3) = cphi * ctheta;

       ! Inertial to Aircraft Transformation Matrix - Same for QUADCOPTER
       
       QUAD%TCI = transpose(QUAD%TIC)
  
       ! Gravity Forces and Moments - Same for Quadcopter
  
       QUAD%FXGRAV = 0.0; QUAD%FYGRAV = 0.0; QUAD%FZGRAV = 0.0;
       QUAD%MXGRAV = 0.0; QUAD%MYGRAV = 0.0; QUAD%MZGRAV = 0.0;
       if (QUAD%GRAVOFFON .eq. 1) then
          QUAD%FXGRAV = QUAD%TIC(3,1)*QUAD%WEIGHT
          QUAD%FYGRAV = QUAD%TIC(3,2)*QUAD%WEIGHT
          QUAD%FZGRAV = QUAD%TIC(3,3)*QUAD%WEIGHT
       end if

       ! Aerodynamic Forces and Moments - Still Same for Quadcopter
       
       QUAD%FXAERO = 0.0; QUAD%FYAERO = 0.0; QUAD%FZAERO = 0.0;
       QUAD%MXAERO = 0.0; QUAD%MYAERO = 0.0; QUAD%MZAERO = 0.0;
       
       if (QUAD%AEROOFFON .eq. 1) then
          vATM_I(1,1) = QUAD%VXWIND
          vATM_I(2,1) = QUAD%VYWIND
          vATM_I(3,1) = QUAD%VZWIND
          vATM_A = matmul(QUAD%TCI,vATM_I)

          !Add in atmospheric winds

          uaero = ub - vATM_A(1,1)
          vaero = vb - vATM_A(2,1)
          waero = wb - vATM_A(3,1)
          
          !Compute total velocity

          V_A = sqrt(uaero**2 + vaero**2 + waero**2)

          !Quadcopter Aerodynamic Model written by Lisa Schibelius - 12/2016

          !Recompute KT
          QUAD%KT = QUAD%C_T*((QUAD%DEN*qPI*(QUAD%RNEW**4)/4))

          !Compute Thrust

          ! call PWM2FORCE(T)
          sigmaF = 0.000437554764978899 !0.000437554764978899
          omegaF = 45.42   !18.65
          zetaF  = 0.942     !0.8533

          !write(*,*) 'MUvec = ',QUAD%MUVEC

          !!! Second order filter
          do idx = 1,4
              C1F(idx) = -2*zetaF*TDOTVEC(idx)
              C2F(idx) = -(omegaF)*TVEC(idx)
              C3F(idx) = QUAD%MUVEC(idx,1)*sigmaF*omegaF   ! replaced sigma with force
              TDBLDOTVEC(idx) = omegaF*(C1F(idx) + C2F(idx) + C3F(idx))
              QUAD%THRUSTVEC(idx,1) = TVEC(idx)
          end do

          QUAD%OMEGAVEC = sqrt(QUAD%THRUSTVEC/QUAD%KT)
          sumomega = sum(QUAD%OMEGAVEC)

          !!! Make sure angular velocities of rotor does not go beyond the limit
          IF (sumomega .ge. QUAD%OMEGAMAX*4) then
            do j = 1,4
              if (QUAD%OMEGAVEC(j,1) .gt. QUAD%OMEGAMAX) then
              QUAD%OMEGAVEC(j,1) = QUAD%OMEGAMAX
              end if
              if (QUAD%OMEGAVEC(j,1) .lt. 0.00D0) then
                QUAD%OMEGAVEC(j,1) = 0.00D0
              end if
            end do
            sumomega = sum(QUAD%OMEGAVEC)
            QUAD%THRUSTVEC = QUAD%KT*QUAD%OMEGAVEC**2
            do j = 1,4
              TVEC(idx) = QUAD%THRUSTVEC(idx,1)
            end do
          ENDIF
          forcevec = QUAD%THRUSTVEC
          thrust = sum(QUAD%THRUSTVEC)

          !write(*,*) 'Rotor Thrust = ',forcevec,thrust

          !!! Adding constraint to run Monte Carlo
          !!! This constraint actually messed up my altitude controller
          ! if (thrust .gt. QUAD%WEIGHT/cos(30.0*qPI/180)) then !You need to put in theta of the quad 
          !   thrust = QUAD%WEIGHT/cos(30.0*qPI/180) !!Not just a static 30 degrees. That's why this didn't work.
          ! end if
          ! I was thinking this would be better.
          ! if (thrust .lt. QUAD%WEIGHT/cos(theta)) then
          ! Increase thrust by the difference/4
          ! Make sense?

          !Aerodynamic Forces
          if (sumomega .gt. 1e-2) then
             QUAD%FXAERO = -thrust*(((QUAD%ALC/(sumomega*QUAD%RNEW))+QUAD%DXD)*uaero - ((QUAD%ALS)/(sumomega*QUAD%RNEW))*vaero)
             QUAD%FYAERO = -thrust*(((QUAD%ALS)/(sumomega*QUAD%RNEW))*uaero + (((QUAD%ALC)/(sumomega*QUAD%RNEW))+QUAD%DYD)*vaero)
             QUAD%FZAERO = -thrust
          end if


          omegar = QUAD%OMEGAVEC(1,1) - QUAD%OMEGAVEC(2,1) + QUAD%OMEGAVEC(3,1) - QUAD%OMEGAVEC(4,1)
          Gammavec(1,1) = QUAD%IRR * omegar * qb
          Gammavec(2,1) = -QUAD%IRR * omegar * pb
          Gammavec(3,1) = 0
          !gotodynamics

          !!!!!!!!! Aerodynamics
          bquad = QUAD%C_TAU*((QUAD%DEN*qPI*(QUAD%RNEW**5)/4))
	  
	  !!! According to dynamic equations, a positive roll will have rotors 1,4 > 2,3. This was previously 2,3>1,4
          ! Since T3 = T1*Ltheta_front/Ltheta_back we're just going to do this for simplicity
          QUAD%MXAERO = Gammavec(1,1) + (QUAD%LPHI12*(TVEC(1) - TVEC(2)) + QUAD%LPHI34*(TVEC(4) - TVEC(3)))
          QUAD%MYAERO = Gammavec(2,1) + QUAD%LTHETA12*(TVEC(1) +TVEC(2) - TVEC(3) -TVEC(4))
          QUAD%MZAERO = Gammavec(3,1) + bquad*(QUAD%OMEGAVEC(1,1)**2 - QUAD%OMEGAVEC(2,1)**2 + QUAD%OMEGAVEC(3,1)**2 - QUAD%OMEGAVEC(4,1)**2)
          ! QUAD%MZAERO = Gammavec(3,1) + bquad*(TVEC(1)**2 - TVEC(2)**2 + TVEC(3)**2 - TVEC(4)**2)

          !At this point we should have F(XYZ)AERO and M(XYZ)AERO populated

          QUAD%FXCONT = 0.0; QUAD%FYCONT = 0.0; QUAD%FZCONT = 0.0;          
          QUAD%MXCONT = 0.0; QUAD%MYCONT = 0.0; QUAD%MZCONT = 0.0;

          !Don't forget to add Tether forces
          if ((QUAD%THR_DYNOFFON .eq. 1) .and. (QUAD%THR_ELASOFFON .eq. 1)) then
             if (isnan(QUAD%FTETHERX) .or. isnan(QUAD%FTETHERY)) then
                write(*,*) 'Sorry dude Nans in tether model detected - I suggest lowering your timestep'
                write(*,*) 'Current Time = ',QUAD%TIME
                write(*,*) 'Tether Forces = ',QUAD%FTETHERX,QUAD%FTETHERY
                STOP
             else
                C_Ftether_I(1,1) = QUAD%FTETHERX
                C_Ftether_I(2,1) = QUAD%FTETHERY
                C_Ftether_I(3,1) = QUAD%FTETHERZ
                C_Ftether_B = matmul(QUAD%TCI,C_Ftether_I)
                QUAD%FXCONT = C_Ftether_B(1,1)
                QUAD%FYCONT = C_Ftether_B(2,1)
                QUAD%FZCONT = C_Ftether_B(3,1)
                
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
        
                QUAD%MXCONT = C_Mtether_B(1,1)
                QUAD%MYCONT = C_Mtether_B(2,1)
                QUAD%MZCONT = C_Mtether_B(3,1)
             end if
          end if

          ! Total Forces and Moments

          QUAD%FXTOTAL = QUAD%FXGRAV + QUAD%FXAERO + QUAD%FXCONT
          QUAD%FYTOTAL = QUAD%FYGRAV + QUAD%FYAERO + QUAD%FYCONT
          !write(*,*) 'Z Force = ',QUAD%FZGRAV,QUAD%FZAERO,QUAD%FZCONT
          QUAD%FZTOTAL = QUAD%FZGRAV + QUAD%FZAERO + QUAD%FZCONT
          QUAD%MXTOTAL = QUAD%MXGRAV + QUAD%MXAERO + QUAD%MXCONT
          QUAD%MYTOTAL = QUAD%MYGRAV + QUAD%MYAERO + QUAD%MYCONT
          QUAD%MZTOTAL = QUAD%MZGRAV + QUAD%MZAERO + QUAD%MZCONT

          ! State Derivatives
  
          xcgdot = QUAD%TIC(1,1)*ub + QUAD%TIC(1,2)*vb + QUAD%TIC(1,3)*wb
          ycgdot = QUAD%TIC(2,1)*ub + QUAD%TIC(2,2)*vb + QUAD%TIC(2,3)*wb
          zcgdot = QUAD%TIC(3,1)*ub + QUAD%TIC(3,2)*vb + QUAD%TIC(3,3)*wb  

          phidot = pb + sphi * ttheta * qb + cphi * ttheta * rb;
          thetadot = cphi * qb - sphi * rb;
          psidot = (sphi / ctheta) * qb + (cphi / ctheta) * rb;
          ubdot = QUAD%FXTOTAL/QUAD%MASS + rb*vb - qb*wb
          vbdot = QUAD%FYTOTAL/QUAD%MASS + pb*wb - rb*ub
          !write(*,*) 'Weight and Z =',QUAD%FZTOTAL,QUAD%MASS
          wbdot = QUAD%FZTOTAL/QUAD%MASS + qb*ub - pb*vb
          
          c1 = QUAD%MXTOTAL - pb*(qb*QUAD%IXZ-rb*QUAD%IXY) - qb*(qb*QUAD%IYZ-rb*QUAD%IYY) - rb*(qb*QUAD%IZZ-rb*QUAD%IYZ)
          c2 = QUAD%MYTOTAL - pb*(rb*QUAD%IXX-pb*QUAD%IXZ) - qb*(rb*QUAD%IXY-pb*QUAD%IYZ) - rb*(rb*QUAD%IXZ-pb*QUAD%IZZ)
          c3 = QUAD%MZTOTAL - pb*(pb*QUAD%IXY-qb*QUAD%IXX) - qb*(pb*QUAD%IYY-qb*QUAD%IXY) - rb*(pb*QUAD%IYZ-qb*QUAD%IXZ)
          pbdot = QUAD%IXXI*c1 + QUAD%IXYI*c2 + QUAD%IXZI*c3
          qbdot = QUAD%IXYI*c1 + QUAD%IYYI*c2 + QUAD%IYZI*c3
          rbdot = QUAD%IXZI*c1 + QUAD%IYZI*c2 + QUAD%IZZI*c3

          ! Wrap State Derivatives
  
          QUAD%STATEDOT(1) = xcgdot
          QUAD%STATEDOT(2) = ycgdot
          QUAD%STATEDOT(2) = ycgdot
          QUAD%STATEDOT(3) = zcgdot
          QUAD%STATEDOT(4) = phidot
          QUAD%STATEDOT(5) = thetadot
          QUAD%STATEDOT(6) = psidot
          QUAD%STATEDOT(7) = ubdot
          QUAD%STATEDOT(8) = vbdot
          QUAD%STATEDOT(9) = wbdot
          QUAD%STATEDOT(10) = pbdot 
          QUAD%STATEDOT(11) = qbdot 
          QUAD%STATEDOT(12) = rbdot
          QUAD%STATEDOT(13) = TDOTVEC(1)
          QUAD%STATEDOT(14) = TDBLDOTVEC(1)
          QUAD%STATEDOT(15) = TDOTVEC(2)
          QUAD%STATEDOT(16) = TDBLDOTVEC(2)
          QUAD%STATEDOT(17) = TDOTVEC(3)
          QUAD%STATEDOT(18) = TDBLDOTVEC(3)
          QUAD%STATEDOT(19) = TDOTVEC(4)
          QUAD%STATEDOT(20) = TDBLDOTVEC(4)

          !write(*,*) 'Statedot = ',QUAD%STATEDOT
       
          !!Save some stuff for Tether model
          QUAD%XDOT = xcgdot
          QUAD%YDOT = ycgdot
          QUAD%ZDOT = zcgdot

          !Compute Reel Locations - r_reel = r_cg + TIB*r_body
          rCG_I(1,1) = xcg 
          rCG_I(2,1) = ycg
          rCG_I(3,1) = zcg ! + (1/3) !REVISIT
          rReel_I = rCG_I + matmul(QUAD%TIC,rCF_B)
          QUAD%XREEL = rReel_I(1,1)
          QUAD%YREEL = rReel_I(2,1)
          QUAD%ZREEL = rReel_I(3,1)
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
          v_Reel_I = v_CG_I + matmul(QUAD%TIC,matmul(S_wt_B,rCF_B))
          QUAD%XREELDOT = v_Reel_I(1,1)
          QUAD%YREELDOT = v_Reel_I(2,1)
          QUAD%ZREELDOT = v_Reel_I(3,1)
       end if !AEROOFFON

    end if !End MODNO
    ! Integration Model 
    if (QUAD%MODNO .eq. 0) then
       !!!Compute Coptercopter speed 
       if (QUAD%RESTARTSPEED .eq. -999) then
          !QUAD%SPEED = QUAD%FINALSPEED
          accel = 0
       else
          !Ramp in speed - assume constant decel/acceleration of 2 ft/s
          dspeed = 3
          tend = abs(QUAD%RESTARTSPEED-QUAD%FINALSPEED)/dspeed
          if (QUAD%TIME .gt. tend) then
             !QUAD%SPEED = QUAD%FINALSPEED
             accel = 0
          else
             !QUAD%SPEED = QUAD%RESTARTSPEED + sign(1.0,QUAD%FINALSPEED-QUAD%RESTARTSPEED)*dspeed*(QUAD%TIME)
             accel = sign(1.0,QUAD%FINALSPEED-QUAD%RESTARTSPEED)*dspeed
          end if
       end if

       !!Compute Derivatives
       QUAD%XDOT = QUAD%SPEED !!This assumes you are flying straight - REVISIT REVISIT
       QUAD%YDOT = vb
       QUAD%ZDOT = wb
       QUAD%STATEDOT(1) = QUAD%XDOT
       QUAD%STATEDOT(2) = QUAD%YDOT
       QUAD%STATEDOT(3) = QUAD%ZDOT
       QUAD%STATEDOT(4:6) = 0 !Phi theta psi
       call RandUniform(noise)
       QUAD%STATEDOT(7) = accel + (1.0D0-2*noise)*QUAD%XDDOTNOISE !udot
       call RandUniform(noise)
       freq = 0
       if (QUAD%YDDOTPERIOD .ne. 0) then
          freq = (2*qPI)/(QUAD%YDDOTPERIOD)*cos((2*qPI)/(QUAD%YDDOTPERIOD)*QUAD%TIME)
       end if
       QUAD%STATEDOT(8) = 0 + (1.0D0-2*noise)*QUAD%XDDOTNOISE + QUAD%YDDOTSCALE*freq !vdot
       call RandUniform(noise)
       QUAD%STATEDOT(9) = (1.0D0-2*noise)*QUAD%XDDOTNOISE !wdot
       QUAD%STATEDOT(10:12) = 0 !p,q,rdot

       !Compute Reel Locations
       QUAD%XREEL = QUAD%XCG + cos(QUAD%PSI)*(QUAD%SLREEL-QUAD%SLCG) - sin(QUAD%PSI)*(QUAD%BLREEL-QUAD%BLCG) 
       QUAD%YREEL = QUAD%YCG + sin(QUAD%PSI)*(QUAD%SLREEL-QUAD%SLCG) + cos(QUAD%PSI)*(QUAD%BLREEL-QUAD%BLCG) 
       QUAD%ZREEL = QUAD%ZCG + QUAD%WLREEL - QUAD%WLCG 
       QUAD%XREELDOT = QUAD%XDOT
       QUAD%YREELDOT = QUAD%YDOT
       QUAD%ZREELDOT = QUAD%ZDOT
       
    end if

 ! Constant Model

    if (QUAD%MODNO .eq. 1) then

     QUAD%SPEED = QUAD%FINALSPEED
        
     ! if (QUAD%TIME .gt. 1000) then
     !    QUAD%XCG = QUAD%XCGINITIAL + QUAD%SPEED*1000 + (QUAD%SPEED+5)*(QUAD%TIME-1000)
     !    speed = QUAD%SPEED+5
     ! end if
     QUAD%XDOT = QUAD%SPEED*cos(QUAD%PSI)
     QUAD%YDOT = QUAD%SPEED*sin(QUAD%PSI) !!!DO NOT FORGET TO FIX THIS TOO

     !!Compute CG location -- This assumes that speed is constant

     QUAD%XCG = QUAD%XCGINITIAL + QUAD%SPEED*cos(QUAD%PSI)*QUAD%TIME
     QUAD%YCG = QUAD%YCGINITIAL + QUAD%SPEED*sin(QUAD%PSI)*QUAD%TIME
     QUAD%ZCG = QUAD%ZCGINITIAL

     !Compute CG speed

     QUAD%ZDOT = 0.0
     QUAD%XREEL = QUAD%XCG + cos(QUAD%PSI)*(QUAD%SLREEL-QUAD%SLCG) - sin(QUAD%PSI)*(QUAD%BLREEL-QUAD%BLCG) 
     QUAD%YREEL = QUAD%YCG + sin(QUAD%PSI)*(QUAD%SLREEL-QUAD%SLCG) + cos(QUAD%PSI)*(QUAD%BLREEL-QUAD%BLCG) 
     QUAD%ZREEL = QUAD%ZCG + QUAD%WLREEL - QUAD%WLCG 
     QUAD%XREELDOT = QUAD%XDOT
     QUAD%YREELDOT = QUAD%YDOT
     QUAD%ZREELDOT = QUAD%ZDOT

     !Place everything in state vector

     QUAD%STATE(1) = QUAD%XCG
     QUAD%STATE(2) = QUAD%YCG
     QUAD%STATE(3) = QUAD%ZCG
     QUAD%STATE(4) = 0 !phi 
     QUAD%STATE(5) = 0 !theta
     QUAD%STATE(6) = QUAD%PSI !psi
     QUAD%STATE(7) = QUAD%SPEED
     QUAD%STATE(8:12) = 0

   ! Helpful variables
     
     QUAD%THETA = 0
     QUAD%PHI = 0
     ctheta = cos(QUAD%THETA)
     stheta = sin(QUAD%THETA)
     cphi = cos(QUAD%PHI)
     sphi = sin(QUAD%PHI)
     cpsi = cos(QUAD%PSI)
     spsi = sin(QUAD%PSI)

     ! Reel Position and Velocity
     
     QUAD%TIS(1,1) = ctheta*cpsi
     QUAD%TIS(2,1) = ctheta*spsi
     QUAD%TIS(3,1) = -stheta
     QUAD%TIS(1,2) = sphi*stheta*cpsi - cphi*spsi
     QUAD%TIS(2,2) = sphi*stheta*spsi + cphi*cpsi
     QUAD%TIS(3,2) = sphi*ctheta
     QUAD%TIS(1,3) = cphi*stheta*cpsi + sphi*spsi
     QUAD%TIS(2,3) = cphi*stheta*spsi - sphi*cpsi
     QUAD%TIS(3,3) = cphi*ctheta
  end if
  
  ! Table Look-Up Model 

  if (QUAD%MODNO .eq. 2) then
    
   ifind = 0
  
   ! Position Pointer  

   if (QUAD%TIME .le. QUAD%TIMETAB(QUAD%IP)) then 
    ifind = -1 
    do while ((ifind.ne.0) .and. (QUAD%IP.gt.1))
     QUAD%IP = QUAD%IP - 1 
     if (QUAD%TIMETAB(QUAD%IP)   .le. QUAD%TIME) then 
     if (QUAD%TIMETAB(QUAD%IP+1) .gt. QUAD%TIME) then 
      ifind = 0
     end if
     end if 
    end do 
   end if
   if (QUAD%TIME .gt. QUAD%TIMETAB(QUAD%IP+1)) then 
    ifind = 1
    do while ((ifind.ne.0) .and. (QUAD%IP.lt.QUAD%TABSIZE-1))
     QUAD%IP = QUAD%IP + 1 
     if (QUAD%TIMETAB(QUAD%IP)   .le. QUAD%TIME) then 
     if (QUAD%TIMETAB(QUAD%IP+1) .gt. QUAD%TIME) then 
      ifind = 0 
     end if 
     end if 
    end do 
   end if
   if (ifind .eq. 0) then
    m = (QUAD%TIME-QUAD%TIMETAB(QUAD%IP))/(QUAD%TIMETAB(QUAD%IP+1)-QUAD%TIMETAB(QUAD%IP))
   else if (ifind .eq. -1) then
    m = 0.0
   else if (ifind .eq. 1) then
    m = 1.0
   end if
  
   ! Interpolate
    
   QUAD%XCG   = QUAD%XCGINITIAL + QUAD%XCGTAB(QUAD%IP)   + m*(QUAD%XCGTAB(QUAD%IP+1)-QUAD%XCGTAB(QUAD%IP))
   QUAD%YCG   = QUAD%YCGINITIAL + QUAD%YCGTAB(QUAD%IP)   + m*(QUAD%YCGTAB(QUAD%IP+1)-QUAD%YCGTAB(QUAD%IP))
   QUAD%ZCG   = QUAD%ZCGINITIAL + QUAD%ZCGTAB(QUAD%IP)   + m*(QUAD%ZCGTAB(QUAD%IP+1)-QUAD%ZCGTAB(QUAD%IP))
   QUAD%PHI   = QUAD%PHITAB(QUAD%IP)   + m*(QUAD%PHITAB(QUAD%IP+1)-QUAD%PHITAB(QUAD%IP))
   QUAD%THETA = QUAD%THETATAB(QUAD%IP) + m*(QUAD%THETATAB(QUAD%IP+1)-QUAD%THETATAB(QUAD%IP))
   QUAD%PSI   = QUAD%PSITAB(QUAD%IP)   + m*(QUAD%PSITAB(QUAD%IP+1)-QUAD%PSITAB(QUAD%IP))
   QUAD%UB    = QUAD%UBTAB(QUAD%IP)    + m*(QUAD%UBTAB(QUAD%IP+1)-QUAD%UBTAB(QUAD%IP))
   QUAD%VB    = QUAD%VBTAB(QUAD%IP)    + m*(QUAD%VBTAB(QUAD%IP+1)-QUAD%VBTAB(QUAD%IP))
   QUAD%WB    = QUAD%WBTAB(QUAD%IP)    + m*(QUAD%WBTAB(QUAD%IP+1)-QUAD%WBTAB(QUAD%IP))
   QUAD%PB    = QUAD%PBTAB(QUAD%IP)    + m*(QUAD%PBTAB(QUAD%IP+1)-QUAD%PBTAB(QUAD%IP))
   QUAD%QB    = QUAD%QBTAB(QUAD%IP)    + m*(QUAD%QBTAB(QUAD%IP+1)-QUAD%QBTAB(QUAD%IP))
   QUAD%RB    = QUAD%RBTAB(QUAD%IP)    + m*(QUAD%RBTAB(QUAD%IP+1)-QUAD%RBTAB(QUAD%IP))

   !Place state in statevector

   QUAD%STATE(1) = QUAD%XCG
   QUAD%STATE(2) = QUAD%YCG
   QUAD%STATE(3) = QUAD%ZCG
   QUAD%STATE(4) = QUAD%PHI
   QUAD%STATE(5) = QUAD%THETA
   QUAD%STATE(6) = QUAD%PSI
   QUAD%STATE(7) = QUAD%UB
   QUAD%STATE(8) = QUAD%VB
   QUAD%STATE(9) = QUAD%WB
   QUAD%STATE(10) = QUAD%PB
   QUAD%STATE(11) = QUAD%QB
   QUAD%STATE(12) = QUAD%RB

   ! Helpful variables

   ctheta = cos(QUAD%THETA)
   stheta = sin(QUAD%THETA)
   cphi = cos(QUAD%PHI)
   sphi = sin(QUAD%PHI)
   cpsi = cos(QUAD%PSI)
   spsi = sin(QUAD%PSI)

   ! Reel Position and Velocity
    
   QUAD%TIS(1,1) = ctheta*cpsi
   QUAD%TIS(2,1) = ctheta*spsi
   QUAD%TIS(3,1) = -stheta
   QUAD%TIS(1,2) = sphi*stheta*cpsi - cphi*spsi
   QUAD%TIS(2,2) = sphi*stheta*spsi + cphi*cpsi
   QUAD%TIS(3,2) = sphi*ctheta
   QUAD%TIS(1,3) = cphi*stheta*cpsi + sphi*spsi
   QUAD%TIS(2,3) = cphi*stheta*spsi - sphi*cpsi
   QUAD%TIS(3,3) = cphi*ctheta

   !!Compute Copter speed in inertial coordinates

   QUAD%XDOT = QUAD%TIS(1,1)*QUAD%UB + QUAD%TIS(1,2)*QUAD%VB + QUAD%TIS(1,3)*QUAD%WB  
   QUAD%YDOT = QUAD%TIS(2,1)*QUAD%UB + QUAD%TIS(2,2)*QUAD%VB + QUAD%TIS(2,3)*QUAD%WB  
   QUAD%ZDOT = QUAD%TIS(3,1)*QUAD%UB + QUAD%TIS(3,2)*QUAD%VB + QUAD%TIS(3,3)*QUAD%WB  

   !!!Compute Reel velocity in Copter frame

   QUAD%UREEL = QUAD%UB + QUAD%QB*(QUAD%WLREEL-QUAD%WLCG) - QUAD%RB*(QUAD%BLREEL-QUAD%BLCG) 
   QUAD%VREEL = QUAD%VB + QUAD%RB*(QUAD%SLREEL-QUAD%SLCG) - QUAD%PB*(QUAD%WLREEL-QUAD%WLCG)
   QUAD%WREEL = QUAD%WB + QUAD%PB*(QUAD%BLREEL-QUAD%BLCG) - QUAD%QB*(QUAD%SLREEL-QUAD%SLCG)

   QUAD%XREEL = QUAD%XCG + QUAD%TIS(1,1)*(QUAD%SLREEL-QUAD%SLCG) + QUAD%TIS(1,2)*(QUAD%BLREEL-QUAD%BLCG) + QUAD%TIS(1,3)*(QUAD%WLREEL-QUAD%WLCG)  
   QUAD%YREEL = QUAD%YCG + QUAD%TIS(2,1)*(QUAD%SLREEL-QUAD%SLCG) + QUAD%TIS(2,2)*(QUAD%BLREEL-QUAD%BLCG) + QUAD%TIS(2,3)*(QUAD%WLREEL-QUAD%WLCG)  
   QUAD%ZREEL = QUAD%ZCG + QUAD%TIS(3,1)*(QUAD%SLREEL-QUAD%SLCG) + QUAD%TIS(3,2)*(QUAD%BLREEL-QUAD%BLCG) + QUAD%TIS(3,3)*(QUAD%WLREEL-QUAD%WLCG)  

   QUAD%XREELDOT = QUAD%TIS(1,1)*QUAD%UREEL + QUAD%TIS(1,2)*QUAD%VREEL + QUAD%TIS(1,3)*QUAD%WREEL  
   QUAD%YREELDOT = QUAD%TIS(2,1)*QUAD%UREEL + QUAD%TIS(2,2)*QUAD%VREEL + QUAD%TIS(2,3)*QUAD%WREEL  
   QUAD%ZREELDOT = QUAD%TIS(3,1)*QUAD%UREEL + QUAD%TIS(3,2)*QUAD%VREEL + QUAD%TIS(3,3)*QUAD%WREEL  
    
  end if

  ! This is pretty sloppy. Nemo wants this to run only once per rk4 function call
  ! should probably just add these as states but this will do for now
  ! What we will do is just divide everything out by 4 so when this runs 4 times we should
  ! be good.
  call COMPUTEINTEGRAL(QUAD) ! Calls integral routine
    
  RETURN
  
end if
  
!!!!!!!!!!!!!!!!!!!!!! ECHO DATA iflag = 2 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 
 if (iflag .eq. 2) then
 
  write(25,*) ' '
  write(25,*) 'QQQQ  U  U   A   DDD   '
  write(25,*) 'Q  Q  U  U  A A  D  D  '
  write(25,*) 'Q  Q  U  U AAAAA D  D  '
  write(25,*) 'Q  Q  U  U A   A D  D  '
  write(25,*) 'QQQQQ UUUU A   A DDD   '
  write(25,*) ' '
  write(25,*) 'Copter Input File: '
  write(25,*) trim(QUAD%INPUTFILE)
  write(25,*) ' '
  write(25,*) 'Module off or on: ',QUAD%OFFON
  write(25,*) 'Model (0=Integration,1=Constant, 2=Table,3=Quad 6DOF Model): ',QUAD%MODNO
  if (QUAD%MODNO .eq. 3) then
     !Output quadcopter stuff
     write(25,*) 'Gravity Flag (0=Off, 1=On): ',QUAD%GRAVOFFON
     write(25,*) 'Aerodynamics Flag (0=Off, 1=On): ',QUAD%AEROOFFON
     write(25,*) 'Contact Flag (0=Off, 1=On): ',QUAD%CONTOFFON
     write(25,*) 'Mass (kg): ',QUAD%MASS
     write(25,*) 'Weight (N): ',QUAD%WEIGHT
     write(25,*) 'Stationline of Mass Center (m): ',QUAD%SLCG
     write(25,*) 'Buttline of Mass Center (m): ',QUAD%BLCG
     write(25,*) 'Waterline of Mass Center (m): ',QUAD%WLCG
     write(25,*) 'Ixx (kg m^2): ',QUAD%IXX
     write(25,*) 'Iyy (kg m^2): ',QUAD%IYY
     write(25,*) 'Izz (kg m^2): ',QUAD%IZZ
     write(25,*) 'Ixy (kg m^2): ',QUAD%IXY
     write(25,*) 'Ixz (kg m^2): ',QUAD%IXZ
     write(25,*) 'Iyz (kg m^2): ',QUAD%IYZ
     write(25,*) 'Ixx Inverse (1/(kg m^2)): ',QUAD%IXXI
     write(25,*) 'Iyy Inverse (1/(kg m^2)): ',QUAD%IYYI
     write(25,*) 'Izz Inverse (1/(kg m^2)): ',QUAD%IZZI
     write(25,*) 'Ixy Inverse (1/(kg m^2)): ',QUAD%IXYI
     write(25,*) 'Ixz Inverse (1/(kg m^2)): ',QUAD%IXZI
     write(25,*) 'Iyz Inverse (1/(kg m^2)): ',QUAD%IYZI
     write(25,*) 'Turn Radius of AC(m): ',  QUAD%TURNRADIUS
     write(25,*) 'Quadcopter Aero Parameter: ', QUAD%ALC
     write(25,*) 'Quadcopter Aero Parameter: ', QUAD%ALS
     write(25,*) 'Quadcopter Aero Parameter: ', QUAD%DXD
     write(25,*) 'Quadcopter Aero Parameter: ', QUAD%DYD
     write(25,*) 'Quadcopter Aero Parameter: ', QUAD%RNEW
     write(25,*) 'Quadcopter Aero Parameter: ', QUAD%C_T
     write(25,*) 'Quadcopter Aero Parameter: ', QUAD%C_TAU
     write(25,*) 'Quadcopter Aero Parameter: ', QUAD%LPHI12
     write(25,*) 'Quadcopter Aero Parameter: ', QUAD%LPHI34
     write(25,*) 'Quadcopter Aero Parameter: ', QUAD%LTHETA12
     write(25,*) 'Quadcopter Aero Parameter: ', QUAD%LTHETA34
     write(25,*) 'Quadcopter Aero Parameter: ', QUAD%OMEGAMAX
  else
     write(25,*) ' '
     write(25,*) 'Stationline of Tether Reel Point on Copter: ', QUAD%SLREEL
     write(25,*) 'Buttline of Tether Reel Point on Copter: ', QUAD%BLREEL
     write(25,*) 'Waterline of Tether Reel Point on Copter: ', QUAD%WLREEL
     write(25,*) 'Stationline of Airwake grid start on Copter: ', QUAD%SLAIRWAKE
     write(25,*) 'Buttline of Airwake grid start on Copter: ', QUAD%BLAIRWAKE
     write(25,*) 'Waterline of Airwake grid start on Copter: ', QUAD%WLAIRWAKE
     if ((QUAD%MODNO .eq. 1) .or. (QUAD%MODNO .eq. 0 )) then
        write(25,*) 'Copter Speed from .COPTER File(ft/s): ',QUAD%FINALSPEED
        write(25,*) 'Restart Speed from RESTART File (ft/s): ',QUAD%RESTARTSPEED
        write(25,*) 'Copter Azimuthal Direction (deg): ',57.3*QUAD%PSI
        write(25,*) 'Copter Initial X (ft): ',QUAD%XCGINITIAL
        write(25,*) 'Copter Initial Y (ft): ',QUAD%YCGINITIAL
        write(25,*) 'Copter Initial Z (ft): ',QUAD%ZCGINITIAL
        write(25,*) 'Copter Noise X (ft/s^2): ',QUAD%XDDOTNOISE
        write(25,*) 'Copter Noise Y (ft/s^2): ',QUAD%YDDOTSCALE
        write(25,*) 'Copter Noise Z (ft/s^2): ',QUAD%YDDOTPERIOD
        write(25,*) ' '
     end if
     if (QUAD%MODNO .eq. 2) then
        write(25,*) 'Time (s),      Xcg (ft),      Ycg (ft),      Zcg (ft)'
        write(25,*) '----------------------------------------------------'
        do i=1,QUAD%TABSIZE  
           write(25,fmt='(4e18.8)') QUAD%TIMETAB(i),QUAD%XCGTAB(i),QUAD%YCGTAB(i),QUAD%ZCGTAB(i)
        end do
        write(25,*) ' '
        write(25,*) 'Time (s),    Phi (deg),  Theta (deg),    Psi (deg)'
        write(25,*) '----------------------------------------------------'
        do i=1,QUAD%TABSIZE  
           write(25,fmt='(4e18.8)') QUAD%TIMETAB(i),57.3*QUAD%PHITAB(i),57.3*QUAD%THETATAB(i),57.3*QUAD%PSITAB(i)
        end do
        write(25,*) ' '
        write(25,*) 'Time (s),     Ub (ft/s),     Vb (ft/s),     Wb (ft/s)'
        write(25,*) '----------------------------------------------------'
        do i=1,QUAD%TABSIZE   
           write(25,fmt='(4e18.8)') QUAD%TIMETAB(i),QUAD%UBTAB(i),QUAD%VBTAB(i),QUAD%WBTAB(i)
        end do
        write(25,*) ' '
        write(25,*) 'Time (s),     Pb (r/s),      Qb (r/s),    Rb (r/s)'
        write(25,*) '----------------------------------------------------'
        do i=1,QUAD%TABSIZE  
           write(25,fmt='(4e18.8)') QUAD%TIMETAB(i),QUAD%PBTAB(i),QUAD%QBTAB(i),QUAD%RBTAB(i)
        end do
        write(25,*) ' '
     end if
  end if
  write(25,*) 'Data Quality Flag (nd, 0=Data Not Loaded Successfully, 1=Data Loaded Successfully): ',QUAD%DQFLAG
  
  RETURN
  
 end if
  
!!!!!!!!!!!!!!!!!!!!!!!!!!!!! LOAD DATA iflag = 1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 
 if (iflag .eq. 1) then
   
  open(unit=94,file=QUAD%INPUTFILE,status='old',iostat=openflag)
  if (openflag .ne. 0) then
     write(*,*) QUAD%INPUTFILE
   write(*,*) 'Error Opening Copter Input File => ',QUAD%INPUTFILE,' <= ';PAUSE; STOP
  end if
  rewind(94)
  read(unit=94,fmt=*,iostat=readflag) readreal; QUAD%OFFON = readreal
  read(unit=94,fmt=*,iostat=readflag) readreal; QUAD%MODNO = readreal

  if (QUAD%MODNO .eq. 3) then
     read(unit=94,fmt=*,iostat=readflag) QUAD%GRAVOFFON
     read(unit=94,fmt=*,iostat=readflag) QUAD%AEROOFFON
     read(unit=94,fmt=*,iostat=readflag) QUAD%CONTOFFON
     read(unit=94,fmt=*,iostat=readflag) QUAD%WEIGHT
     read(unit=94,fmt=*,iostat=readflag) QUAD%GRAVITY
     read(unit=94,fmt=*,iostat=readflag) QUAD%SLCG
     read(unit=94,fmt=*,iostat=readflag) QUAD%BLCG
     read(unit=94,fmt=*,iostat=readflag) QUAD%WLCG
     read(unit=94,fmt=*,iostat=readflag) QUAD%SLREEL
     read(unit=94,fmt=*,iostat=readflag) QUAD%BLREEL
     read(unit=94,fmt=*,iostat=readflag) QUAD%WLREEL
     read(unit=94,fmt=*,iostat=readflag) QUAD%IXX
     read(unit=94,fmt=*,iostat=readflag) QUAD%IYY
     read(unit=94,fmt=*,iostat=readflag) QUAD%IZZ
     read(unit=94,fmt=*,iostat=readflag) QUAD%IXY
     read(unit=94,fmt=*,iostat=readflag) QUAD%IXZ
     read(unit=94,fmt=*,iostat=readflag) QUAD%IYZ
     read(unit=94,fmt=*,iostat=readflag) QUAD%TURNRADIUS
     read(unit=94,fmt=*,iostat=readflag) QUAD%ALC
     read(unit=94,fmt=*,iostat=readflag) QUAD%ALS
     read(unit=94,fmt=*,iostat=readflag) QUAD%DXD
     read(unit=94,fmt=*,iostat=readflag) QUAD%DYD
     read(unit=94,fmt=*,iostat=readflag) QUAD%RNEW
     read(unit=94,fmt=*,iostat=readflag) QUAD%C_T
     read(unit=94,fmt=*,iostat=readflag) QUAD%C_TAU
     read(unit=94,fmt=*,iostat=readflag) QUAD%LPHI12
     read(unit=94,fmt=*,iostat=readflag) QUAD%LPHI34
     read(unit=94,fmt=*,iostat=readflag) QUAD%LTHETA12
     read(unit=94,fmt=*,iostat=readflag) QUAD%LTHETA34
     read(unit=94,fmt=*,iostat=readflag) QUAD%OMEGAMAX
     read(unit=94,fmt=*,iostat=readflag) QUAD%IRR
     read(unit=94,fmt=*,iostat=readflag) QUAD%MS_MIN
     read(unit=94,fmt=*,iostat=readflag) QUAD%MS_MAX
     read(unit=94,fmt=*,iostat=readflag) readreal; QUAD%CONTROLOFFON = int(readreal)
     read(unit=94,fmt=*,iostat=readflag) QUAD%KPXQUAD
     read(unit=94,fmt=*,iostat=readflag) QUAD%KIXQUAD
     read(unit=94,fmt=*,iostat=readflag) QUAD%KDXQUAD
     read(unit=94,fmt=*,iostat=readflag) QUAD%KPYQUAD
     read(unit=94,fmt=*,iostat=readflag) QUAD%KIYQUAD
     read(unit=94,fmt=*,iostat=readflag) QUAD%KDYQUAD
     read(unit=94,fmt=*,iostat=readflag) QUAD%KPZQUAD
     read(unit=94,fmt=*,iostat=readflag) QUAD%KIZQUAD
     read(unit=94,fmt=*,iostat=readflag) QUAD%KDZQUAD
     read(unit=94,fmt=*,iostat=readflag) QUAD%KPPHI
     read(unit=94,fmt=*,iostat=readflag) QUAD%KIPHI
     read(unit=94,fmt=*,iostat=readflag) QUAD%KDPHI
     read(unit=94,fmt=*,iostat=readflag) QUAD%KPTHETA
     read(unit=94,fmt=*,iostat=readflag) QUAD%KITHETA
     read(unit=94,fmt=*,iostat=readflag) QUAD%KDTHETA
     read(unit=94,fmt=*,iostat=readflag) QUAD%KPPSI
     read(unit=94,fmt=*,iostat=readflag) QUAD%KIPSI
     read(unit=94,fmt=*,iostat=readflag) QUAD%KDPSI
     read(unit=94,fmt=*,iostat=readflag) QUAD%XINTEGRAL 
     read(unit=94,fmt=*,iostat=readflag) QUAD%YINTEGRAL 
     read(unit=94,fmt=*,iostat=readflag) QUAD%ZINTEGRAL
     read(unit=94,fmt=*,iostat=readflag) QUAD%PHIINTEGRAL 
     read(unit=94,fmt=*,iostat=readflag) QUAD%THETAINTEGRAL
     read(unit=94,fmt=*,iostat=readflag) QUAD%PSIINTEGRAL
     read(unit=94,fmt=*,iostat=readflag) QUAD%UCOMMAND !!Changed to UCOMMAND for forward flight 
     read(unit=94,fmt=*,iostat=readflag) QUAD%YCOMMAND
     read(unit=94,fmt=*,iostat=readflag) QUAD%ZCOMMAND
     read(unit=94,fmt=*,iostat=readflag) QUAD%UINTEGRAL
     !read(unit=94,fmt=*,iostat=readflag) QUAD%WAYPOINT

     QUAD%WAYPOINT = 1

     !!!DO SOME CALCULATIONS ON Quadcopter
     QUAD%MASS = QUAD%WEIGHT/QUAD%GRAVITY 
     deti = + QUAD%IXX*(QUAD%IYY*QUAD%IZZ-QUAD%IYZ*QUAD%IYZ) - QUAD%IXY*(QUAD%IXY*QUAD%IZZ-QUAD%IYZ*QUAD%IXZ) + QUAD%IXZ*(QUAD%IXY*QUAD%IYZ-QUAD%IYY*QUAD%IXZ)
     QUAD%IXXI = (QUAD%IYY*QUAD%IZZ-QUAD%IYZ*QUAD%IYZ)/deti
     QUAD%IXYI = (QUAD%IYZ*QUAD%IXZ-QUAD%IXY*QUAD%IZZ)/deti
     QUAD%IXZI = (QUAD%IXY*QUAD%IYZ-QUAD%IYY*QUAD%IXZ)/deti
     QUAD%IYYI = (QUAD%IXX*QUAD%IZZ-QUAD%IXZ*QUAD%IXZ)/deti
     QUAD%IYZI = (QUAD%IXY*QUAD%IXZ-QUAD%IXX*QUAD%IYZ)/deti
     QUAD%IZZI = (QUAD%IXX*QUAD%IYY-QUAD%IXY*QUAD%IXY)/deti
  else     
     read(unit=94,fmt=*,iostat=readflag) QUAD%SLREEL
     read(unit=94,fmt=*,iostat=readflag) QUAD%BLREEL
     read(unit=94,fmt=*,iostat=readflag) QUAD%WLREEL
     read(unit=94,fmt=*,iostat=readflag) QUAD%AIRWAKE
     !Read location of airwake data
     read(unit=94,fmt=*,iostat=readflag) QUAD%AIRWAKEPATH
     read(unit=94,fmt=*,iostat=readflag) QUAD%SLAIRWAKE
     read(unit=94,fmt=*,iostat=readflag) QUAD%BLAIRWAKE
     read(unit=94,fmt=*,iostat=readflag) QUAD%WLAIRWAKE

     if (QUAD%AIRWAKE .eq. 1) then

        !Set TCOORD
        do i = 1,NTIMES
           QUAD%TCOORD(i) = 0 + (i-1)*1.0D0
        end do

        !%%%%%%%Import Initial UVW matrices%%%%%%%%% */

        write(number, '(i1)' )  1
        letter = trim('U')
        QUAD%U0name = trim(QUAD%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        letter = trim('V')
        QUAD%V0name = trim(QUAD%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        letter = trim('W')
        QUAD%W0name = trim(QUAD%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        write(number, '(i1)' )  2
        letter = trim('U')
        QUAD%Udtname = trim(QUAD%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        letter = trim('V')
        QUAD%Vdtname = trim(QUAD%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        letter = trim('W')
        QUAD%Wdtname = trim(QUAD%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'

        call IMPORTWAKE(QUAD%UCOPTER,QUAD%U0name);
        call IMPORTWAKE(QUAD%UCOPTERDT,QUAD%Udtname);
        call IMPORTWAKE(QUAD%VCOPTER,QUAD%V0name);
        call IMPORTWAKE(QUAD%VCOPTERDT,QUAD%Vdtname);
        call IMPORTWAKE(QUAD%WCOPTER,QUAD%W0name);
        call IMPORTWAKE(QUAD%WCOPTERDT,QUAD%Wdtname);

        !!Import X,Y,Z Grid data
        xgridname = trim(QUAD%AIRWAKEPATH)//'X_Grid.txt'
        ygridname = trim(QUAD%AIRWAKEPATH)//'Y_Grid.txt'
        zgridname = trim(QUAD%AIRWAKEPATH)//'Z_Grid.txt'

        call IMPORTWAKE(QUAD%XCOPTER,xgridname);
        call IMPORTWAKE(QUAD%YCOPTER,ygridname);
        call IMPORTWAKE(QUAD%ZCOPTER,zgridname);

        do i = 1,IMAX
           QUAD%XCOORD(i) = QUAD%XCOPTER(i,1,1);
        end do

        write(*,*) 'Wake Data Load Complete'
     end if

  end if
  

  if ((QUAD%MODNO .eq. 1) .or. (QUAD%MODNO .eq. 0)) then
     read(unit=94,fmt=*,iostat=readflag) QUAD%XCGINITIAL
     read(unit=94,fmt=*,iostat=readflag) QUAD%YCGINITIAL
     read(unit=94,fmt=*,iostat=readflag) QUAD%ZCGINITIAL
     read(unit=94,fmt=*,iostat=readflag) QUAD%FINALSPEED
     read(unit=94,fmt=*,iostat=readflag) QUAD%PSI
     read(unit=94,fmt=*,iostat=readflag) QUAD%DOWNWASHONOFF
     read(unit=94,fmt=*,iostat=readflag) QUAD%DOWNWASH
     read(unit=94,fmt=*,iostat=readflag) QUAD%DIAMETER
     read(unit=94,fmt=*,iostat=readflag) QUAD%XDDOTNOISE
     read(unit=94,fmt=*,iostat=readflag) QUAD%YDDOTSCALE
     read(unit=94,fmt=*,iostat=readflag) QUAD%YDDOTPERIOD
  end if
  if (QUAD%MODNO .eq. 2) then
   read(unit=94,fmt=*,iostat=readflag) readreal; QUAD%TABSIZE = int(readreal)
   do i=1,QUAD%TABSIZE  
    read(unit=94,fmt=*,iostat=readflag) QUAD%TIMETAB(i),QUAD%XCGTAB(i),QUAD%YCGTAB(i),QUAD%ZCGTAB(i),QUAD%PHITAB(i),QUAD%THETATAB(i),QUAD%PSITAB(i), & 
                                        QUAD%UBTAB(i),QUAD%VBTAB(i),QUAD%WBTAB(i),QUAD%PBTAB(i),QUAD%QBTAB(i),QUAD%RBTAB(i)
   end do
  end if

  close(94) 
  write(*,*) 'COPTER Load Complete'

  QUAD%DQFLAG = 1

      
  RETURN
   
 end if
   
 RETURN
END SUBROUTINE COPTER

!!Import routine to load data files placed in text files
SUBROUTINE IMPORTWAKE(mat,filename)
  use COPTERDATATYPES
  implicit none
  integer uvw,time
  integer ii,jj,kk,nii,njj,ierr;
  real*8 mat(55,77,61),tempmat(77)
  character*256 filename

  write(*,*) 'Importing: ',filename
  
  open(unit=78,file=filename,status='old',iostat=ierr)
  if (ierr .ne. 0) then 
     write(*,*) 'Copter Airwake File defined incorrectly'
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

SUBROUTINE AIRWAKE(QUAD,XI,YI,ZI)
  use COPTERDATATYPES
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
  type(COPTERSTRUCTURE) QUAD

  !   /*   %%This function will take in x,y,z(ft),t(sec) and location and  */
  !   /*   %return u,v,w(m/s). This uses a fast quad-linear interpolation */
  ! using matrices XCOPTER,YCOPTER,ZCOPTER to interpolate against
  !   /*   %so many globals must be defined. location is a string that */
  !   /*   %contains the location of the data to be interpolated. */
  
  QUAD%VWAKE(1) = 0
  QUAD%VWAKE(2) = 0
  QUAD%VWAKE(3) = 0

  rI_I(1,1) = XI
  rI_I(2,1) = YI
  rI_I(3,1) = ZI
  
  rS_I(1,1) = QUAD%XCG
  rS_I(2,1) = QUAD%YCG
  rS_I(3,1) = QUAD%ZCG

  rAIRWAKE_S(1,1) = QUAD%SLAIRWAKE
  rAIRWAKE_S(2,1) = QUAD%BLAIRWAKE
  rAIRWAKE_S(3,1) = QUAD%WLAIRWAKE

  rAwP_S = matmul(transpose(QUAD%TIS),rI_I-rS_I)-rAIRWAKE_S

  xstar = -rAwP_S(1,1)
  ystar = rAwP_S(2,1)
  zstar = -rAwP_S(3,1)

  !!Loop tstar
  tstar = QUAD%TIME
  tshift = 0
  if (tstar .gt. QUAD%TCOORD(NTIMES)) then
     tshift = -floor(abs(tstar-QUAD%TCOORD(1))/QUAD%TCOORD(NTIMES))
  end if
  tstar = tstar + tshift*QUAD%TCOORD(NTIMES)
  ! write(*,*) 'Tstar = ',tstar, 'tshift = ',tshift

  stepX = 1;stepY = 1;stepZ = 1;stepT = 1;
  extrapX = 0;extrapY = 0;extrapZ = 0;extrapT = 0;
  if (zstar .lt. 0) then
     zstar = -zstar
  endif
  tinterp = 2;

  uvw(1,1)=0;uvw(2,1)=0;uvw(3,1)=0;
  uvw(1,2)=0;uvw(2,2)=0;uvw(3,2)=0;

  markX = QUAD%markX
  markY = QUAD%markY
  markZ = QUAD%markZ
  markT = QUAD%markT

  !%%Check X
  if (markX .eq. IMAX) then
     markX = markX - 1;
  end if
  if ((xstar .ge. QUAD%XCOORD(markX)) .and. (xstar .le. QUAD%XCOORD(markX+1))) then
     !%%You're in between the markers so keep going
  else
     if (xstar .gt. QUAD%XCOORD(IMAX)) then
        markX = IMAX;
        stepX = -1;
        extrapX = 1;
        ! write(*,*) 'Out of bounds on x'
     elseif (xstar .lt. QUAD%XCOORD(1)) then
        markX = 1;
        stepX = 1;
        extrapX = 1;
     else
        call FINDGE(QUAD%XCOORD,IMAX,xstar,markX)
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
     QUAD%YCOORD(j) = QUAD%YCOPTER(markX,j,1); 
     ! write(*,*) QUAD%YCOORD(j)
  end do
  ! write(*,*) '------------'
  do k = 1,KMAX
     QUAD%ZCOORD(k) = QUAD%ZCOPTER(markX,1,k);    
     ! write(*,*) QUAD%ZCOORD(k)
  end do

  !%%Check Y
  if (markY .eq. JMAX) then
     markY = markY - 1;
  end if
  if ((ystar .ge. QUAD%YCOORD(markY)) .and. (ystar .le. QUAD%YCOORD(markY+1))) then
     !%%You're in between the markers so keep going
  else
     if (ystar .gt. QUAD%YCOORD(JMAX)) then
        markY = JMAX;
        stepY = -1;
        extrapY = 1;
     elseif (ystar .lt. QUAD%YCOORD(1)) then
        markY = 1;
        stepY = 1;
        extrapY = 1;
     else
        call FINDGE(QUAD%YCOORD,JMAX,ystar,markY)
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

  if ((zstar .ge. QUAD%ZCOORD(markZ)) .and. (zstar .le. QUAD%ZCOORD(markZ+1))) then
     !%%You're in between the markers so keep going
  else
     !%Find markZ
     if (zstar .gt. QUAD%ZCOORD(KMAX)) then
        !%use endpt
        markZ = KMAX;
        stepZ = -1;
        extrapZ = 1;
        QUAD%VWAKE(1) = 0
        QUAD%VWAKE(2) = 0
        QUAD%VWAKE(3) = 0
        RETURN
     else if (zstar .lt. QUAD%ZCOORD(1)) then
        markZ = 1;
        stepZ = 1;
        extrapZ = 1;
     else
        call FINDGE(QUAD%ZCOORD,KMAX,zstar,markZ)
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
  !write(*,*) tstar,markT,QUAD%TCOORD(markT)
  if ((tstar .ge. QUAD%TCOORD(markT)) .and. (tstar .le. QUAD%TCOORD(markT+1))) then
     !%%You're in between the markers so keep going
  else
     !%Find markT
     if (tstar .gt. QUAD%TCOORD(NTIMES)) then
        !%use endpt
        markT = NTIMES;
        extrapT = 1;
     else if (tstar .lt. QUAD%TCOORD(1)) then
        !%use start pt
        markT = 1;
        extrapT = 1;
     else
        call FINDGE(QUAD%TCOORD,NTIMES,tstar,markT)
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
     QUAD%U0name = trim(QUAD%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
     letter = trim('V')
     QUAD%V0name = trim(QUAD%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
     letter = trim('W')
     QUAD%W0name = trim(QUAD%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
     !%%only import at markT
     call IMPORTWAKE(QUAD%UCOPTER,QUAD%U0name);
     call IMPORTWAKE(QUAD%VCOPTER,QUAD%V0name);
     call IMPORTWAKE(QUAD%WCOPTER,QUAD%W0name);
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
        QUAD%Udtname = trim(QUAD%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        letter = trim('V')
        QUAD%Vdtname = trim(QUAD%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        letter = trim('W')
        QUAD%Wdtname = trim(QUAD%AIRWAKEPATH)//trim(letter)//'_'//trim(number)//'.txt'
        !%%only import at markT
        call IMPORTWAKE(QUAD%UCOPTERDT,QUAD%Udtname);
        call IMPORTWAKE(QUAD%VCOPTERDT,QUAD%Vdtname);
        call IMPORTWAKE(QUAD%WCOPTERDT,QUAD%Wdtname);
     end if !(extrapT eq. 1 ) 
  end if

  ! write(*,*) 'markT = ',markT
  ! write(*,*) 'tinterp = ',tinterp

  !%%Interpolation Scheme
  do tt = 1,tinterp 
     !%Interpolate Spatially

     !%%To start we have 8 discrete point (8 corners of a cube)
     xpts2(1) = QUAD%XCOORD(markX)
     xpts2(2) = QUAD%XCOORD(markX+stepX);
     ypts2(1) = QUAD%YCOORD(markY)
     ypts2(2) = QUAD%YCOORD(markY+stepY);
     zpts2(1) = QUAD%ZCOORD(markZ)
     zpts2(2) = QUAD%ZCOORD(markZ+stepZ);
     x1 = markX;x2 = markX+stepX;
     y1 = markY;y2 = (markY+stepY);
     z1 = markZ;z2 = markZ+stepZ;
     ! write(*,*) 'x1,x2,y1,y2,z1,z2 = ',x1,x2,y1,y2,z1,z2
     if (tt .eq. 1) then
        !%%Use UCOPTER,VCOPTER,WCOPTER
        u8(1) = QUAD%UCOPTER(x1,y1,z1);
        u8(2) = QUAD%UCOPTER(x2,y1,z1);
        u8(3) = QUAD%UCOPTER(x2,y2,z1);
        u8(4) = QUAD%UCOPTER(x1,y2,z1);
        u8(5) = QUAD%UCOPTER(x1,y1,z2);
        u8(6) = QUAD%UCOPTER(x2,y1,z2);
        u8(7) = QUAD%UCOPTER(x2,y2,z2);
        u8(8) = QUAD%UCOPTER(x1,y2,z2);
        v8(1) = QUAD%VCOPTER(x1,y1,z1);
        v8(2) = QUAD%VCOPTER(x2,y1,z1);
        v8(3) = QUAD%VCOPTER(x2,y2,z1);
        v8(4) = QUAD%VCOPTER(x1,y2,z1);
        v8(5) = QUAD%VCOPTER(x1,y1,z2);
        v8(6) = QUAD%VCOPTER(x2,y1,z2);
        v8(7) = QUAD%VCOPTER(x2,y2,z2);
        v8(8) = QUAD%VCOPTER(x1,y2,z2);
        w8(1) = QUAD%WCOPTER(x1,y1,z1);
        w8(2) = QUAD%WCOPTER(x2,y1,z1);
        w8(3) = QUAD%WCOPTER(x2,y2,z1);
        w8(4) = QUAD%WCOPTER(x1,y2,z1);
        w8(5) = QUAD%WCOPTER(x1,y1,z2);
        w8(6) = QUAD%WCOPTER(x2,y1,z2);
        w8(7) = QUAD%WCOPTER(x2,y2,z2);
        w8(8) = QUAD%WCOPTER(x1,y2,z2);
        ! do i = 1,8
        !    write(*,*) 'u8 = ',u8(i)
        ! end do
     else
        !%%Use Udt,Vdt,Wdt
        u8(1) = QUAD%UCOPTERDT(x1,y1,z1);
        u8(2) = QUAD%UCOPTERDT(x2,y1,z1);
        u8(3) = QUAD%UCOPTERDT(x2,y2,z1);
        u8(4) = QUAD%UCOPTERDT(x1,y2,z1);
        u8(5) = QUAD%UCOPTERDT(x1,y1,z2);
        u8(6) = QUAD%UCOPTERDT(x2,y1,z2);
        u8(7) = QUAD%UCOPTERDT(x2,y2,z2);
        u8(8) = QUAD%UCOPTERDT(x1,y2,z2);
        v8(1) = QUAD%VCOPTERDT(x1,y1,z1);
        v8(2) = QUAD%VCOPTERDT(x2,y1,z1);
        v8(3) = QUAD%VCOPTERDT(x2,y2,z1);
        v8(4) = QUAD%VCOPTERDT(x1,y2,z1);
        v8(5) = QUAD%VCOPTERDT(x1,y1,z2);
        v8(6) = QUAD%VCOPTERDT(x2,y1,z2);
        v8(7) = QUAD%VCOPTERDT(x2,y2,z2);
        v8(8) = QUAD%VCOPTERDT(x1,y2,z2);
        w8(1) = QUAD%WCOPTERDT(x1,y1,z1);
        w8(2) = QUAD%WCOPTERDT(x2,y1,z1);
        w8(3) = QUAD%WCOPTERDT(x2,y2,z1);
        w8(4) = QUAD%WCOPTERDT(x1,y2,z1);
        w8(5) = QUAD%WCOPTERDT(x1,y1,z2);
        w8(6) = QUAD%WCOPTERDT(x2,y1,z2);
        w8(7) = QUAD%WCOPTERDT(x2,y2,z2);
        w8(8) = QUAD%WCOPTERDT(x1,y2,z2);
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
     tpts(1) = QUAD%TCOORD(markT)
     tpts(2) = QUAD%TCOORD(markT+1);
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

  QUAD%VWAKE(1) = -(vatm(1) - QUAD%UCOPTER(markX,markY,KMAX))
  QUAD%VWAKE(2) = vatm(2)
  QUAD%VWAKE(3) = -vatm(3)

  QUAD%markX = markX
  QUAD%markY = markY
  QUAD%markZ = markZ
  QUAD%markT = markT

END SUBROUTINE AIRWAKE

SUBROUTINE DOWNWASH(QUAD,STATE)
  use COPTERDATATYPES
  implicit none
  real*8 xcg,ycg,zcg,downwash_angle,zdownwash,delx,dely,delz,n,STATE(3)
  type(COPTERSTRUCTURE) QUAD

  QUAD%VWAKE = 0

  !Extract state of Towed Body
  xcg = STATE(1)
  ycg = STATE(2)
  zcg = STATE(3)

  !Compute z location of downwash
  downwash_angle = atan2(QUAD%DOWNWASH,QUAD%SPEED)
  zdownwash = delx*tan(downwash_angle)

  !Check and see if the towed body is in Copter airwake
  delx = abs(QUAD%XCG - xcg)
  dely = abs(QUAD%YCG - ycg)
  delz = abs(abs(QUAD%ZCG - zcg) - zdownwash)

  if ((dely .lt. QUAD%DIAMETER) .and. (delz .lt. QUAD%DIAMETER)) then
     QUAD%VWAKE(1) = 0.0
     QUAD%VWAKE(2) = 0.0
     call RandUniform(n)
     QUAD%VWAKE(3) = QUAD%DOWNWASH*(1+(1-2*n)*0.1D0)
  end if

END SUBROUTINE DOWNWASH

SUBROUTINE COPTER_CONTROL(QUAD)
  use COPTERDATATYPES
  implicit none
  type(COPTERSTRUCTURE) QUAD
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

  if (QUAD%CONTROLOFFON .eq. 1) then
     !Quadcopter control
     QUAD%OMEGAVEC = 0
     QUAD%THRUSTVEC = 0
     QUAD%MUVEC = 0

     !Get OMEGA0 and KT for QUAD
     QUAD%KT = QUAD%C_T*((4.0D0*QUAD%DEN*(QUAD%RNEW**4))/(qPI**2))
     QUAD%OMEGA0 = sqrt(QUAD%WEIGHT/(4.0D0*QUAD%KT))

     xdot = QUAD%STATEDOT(1)
     ydot = QUAD%STATEDOT(2)
     zdot = QUAD%STATEDOT(3)
     phi = QUAD%STATE(4)
     theta = QUAD%STATE(5)
     psi = QUAD%STATE(6)
     p = QUAD%STATE(10)
     q = QUAD%STATE(11)
     r = QUAD%STATE(12)

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
     ! QUAD%XCOMMAND =  xwaypoint(QUAD%WAYPOINT,1)
     ! QUAD%YCOMMAND =  ywaypoint(QUAD%WAYPOINT,1)
     ! QUAD%ZCOMMAND =  zwaypoint(QUAD%WAYPOINT,1)

     !QUAD%ZCOMMAND = -30.0 -- these are now set in the input file 
     !QUAD%UCOMMAND = 28.93
     !QUAD%YCOMMAND = 0.0

     !delx = (QUAD%XCOMMAND - QUAD%STATE(1))*-1.00D0
     dely = (QUAD%YCOMMAND - QUAD%STATE(2))
     !write(*,*) 'Zstuff = ',QUAD%ZCOMMAND,QUAD%STATE(3)
     delz = (QUAD%ZCOMMAND - QUAD%STATE(3))*-1.00D0

     ! Dwaypoint = sqrt((delx)**2 + (dely)**2 + (delz)**2)

     ! Change this to 4(square) or 8(octagon) depending on the shape you want to simulate
     ! if (Dwaypoint .lt. 1.0D0) then
     !   QUAD%WAYPOINT= QUAD%WAYPOINT+1
     !   if (QUAD%WAYPOINT .gt. nwp) then
     !     QUAD%WAYPOINT=1
     !   end if
     ! end if

     !!! Attitude Controller
     QUAD%PHICOMMAND = QUAD%KPYQUAD*dely + QUAD%KIYQUAD*QUAD%YINTEGRAL - QUAD%KDYQUAD*ydot

     !QUAD%PHICOMMAND = 20.0*qPI/180.0
     
     ! REVISIT hardcoded xdotcommand = 40 ft/s
     ! For now the KP and KD gains for 'x' is controlling the 'u' velocity at which the quad is travelling
     ! Ok shit so the 'u' velocity couldn't really be controlled so -24.5 deg is what 
     ! the quad needs to be to achieve this velocity
     ! QUAD%THETACOMMAND = -24.5*qPI/180!QUAD%KPXQUAD*(QUAD%STATE(7) - 40.0) + QUAD%KIXQUAD*QUAD%XINTEGRAL + QUAD%KDXQUAD*(QUAD%STATEDOT(7))
     ! QUAD%THETACOMMAND = QUAD%KPXQUAD*delx + QUAD%KIXQUAD*QUAD%XINTEGRAL - QUAD%KDXQUAD*xdot

     QUAD%THETACOMMAND = QUAD%KPXQUAD*(QUAD%STATE(7) - QUAD%UCOMMAND) + QUAD%KIXQUAD*QUAD%UINTEGRAL

     !write(*,*) 'integral = ',QUAD%YINTEGRAL,QUAD%XINTEGRAL

     !QUAD%THETACOMMAND = 0.0
     
     QUAD%PSICOMMAND = 0.0

     !write(*,*) 'Xcontrol = ',delx,QUAD%XINTEGRAL,xdot
     !write(*,*) 'ptpcom = ',QUAD%PHICOMMAND,QUAD%THETACOMMAND,QUAD%PSICOMMAND

     if (abs(QUAD%THETACOMMAND) .gt. 30*qPI/180) then
        QUAD%THETACOMMAND = sign(30*qPI/180,QUAD%THETACOMMAND)
     end if
     if (abs(QUAD%PHICOMMAND) .gt. 30*qPI/180) then
        QUAD%PHICOMMAND = sign(30*qPI/180,QUAD%PHICOMMAND)
     end if
     if (abs(QUAD%PSICOMMAND) .gt. 30*qPI/180) then
        QUAD%PSICOMMAND = sign(30*qPI/180,QUAD%PSICOMMAND)
     end if

     ! Hovering microseconds and altitude control
     munominal = 1614.855 + QUAD%KPZQUAD*(delz) + QUAD%KIZQUAD*QUAD%ZINTEGRAL +QUAD%KDZQUAD*zdot  ! Nominal microsecond pulse for hover

     QUAD%MS_ROLL = QUAD%KPPHI*(QUAD%PHICOMMAND-phi) + QUAD%KIPHI*QUAD%PHIINTEGRAL - QUAD%KDPHI*p
     QUAD%MS_PITCH = QUAD%KPTHETA*(QUAD%THETACOMMAND - theta) + QUAD%KITHETA*QUAD%THETAINTEGRAL- QUAD%KDTHETA*q
     QUAD%MS_YAW = QUAD%KPPSI*(QUAD%PSICOMMAND-psi) + QUAD%KIPSI*QUAD%PSIINTEGRAL - QUAD%KDPSI*r

     !write(*,*) 'Att = ',phi,theta,psi,p,q,r

     QUAD%MUVEC(1,1) = munominal + QUAD%MS_ROLL + QUAD%MS_PITCH + QUAD%MS_YAW
     QUAD%MUVEC(2,1) = munominal - QUAD%MS_ROLL + QUAD%MS_PITCH - QUAD%MS_YAW
     QUAD%MUVEC(3,1) = munominal - QUAD%MS_ROLL - QUAD%MS_PITCH + QUAD%MS_YAW
     QUAD%MUVEC(4,1) = munominal + QUAD%MS_ROLL - QUAD%MS_PITCH - QUAD%MS_YAW

     !write(*,*) 'muvec = ',QUAD%MUVEC

     ! Now we saturate the microseconds so that it doesn't go over 1900 or under 1100
     do j = 1,4
        if (QUAD%MUVEC(j,1) .gt. 1900.00D0) then
           QUAD%OMEGAVEC(j,1) = 1900.00D0
        end if
        if (QUAD%MUVEC(j,1) .lt. 1100.00D0) then
           QUAD%MUVEC(j,1) = 1100.00D0
        end if
     end do

  end if
end SUBROUTINE COPTER_CONTROL

SUBROUTINE COMPUTEINTEGRAL(QUAD)
 use COPTERDATATYPES
 implicit none
 type(COPTERSTRUCTURE) QUAD
 ! Using trapezoidal rule(ish) to compute integral error term
 ! REVISIT Replaced xintegral with u velocity term!!!!
 ! The 1/4 is used because this is called once every rk4 call
 ! and since we're using trap(ish) we need to divide by 4. Untested CJM - 2/13/2018
 QUAD%XINTEGRAL     = QUAD%XINTEGRAL     + -(1.0/4.0)*((QUAD%XCOMMAND     - QUAD%STATE(1))/2)*QUAD%DELTATIME
 QUAD%YINTEGRAL     = QUAD%YINTEGRAL     + (1.0/4.0)*((QUAD%YCOMMAND     - QUAD%STATE(2))/2)*QUAD%DELTATIME
 QUAD%ZINTEGRAL     = QUAD%ZINTEGRAL     + -(1.0/4.0)*((QUAD%ZCOMMAND     - QUAD%STATE(3))/2)*QUAD%DELTATIME
 QUAD%PHIINTEGRAL   = QUAD%PHIINTEGRAL   +    (1.0/4.0)*((QUAD%PHICOMMAND   - QUAD%STATE(4))/2)*QUAD%DELTATIME
 QUAD%THETAINTEGRAL = QUAD%THETAINTEGRAL +    (1.0/4.0)*((QUAD%THETACOMMAND - QUAD%STATE(5))/2)*QUAD%DELTATIME
 QUAD%PSIINTEGRAL   = QUAD%PSIINTEGRAL   +    (1.0/4.0)*((QUAD%PHICOMMAND   - QUAD%STATE(6))/2)*QUAD%DELTATIME
 QUAD%UINTEGRAL     = QUAD%UINTEGRAL     + -(1.0/4.0)*((QUAD%UCOMMAND     - QUAD%STATE(7))/2)*QUAD%DELTATIME
END SUBROUTINE COMPUTEINTEGRAL !COMPUTEINTEGRAL

SUBROUTINE PRINTINTEGRAL(QUAD) !!When do I run this?
 use COPTERDATATYPES
 implicit none
 type(COPTERSTRUCTURE) QUAD
 ! Using trapezoidal rule(ish) to compute integral error term
 ! REVISIT Replaced xintegral with u velocity term!!!!
 ! The 1/4 is used because this is called once every rk4 call
 ! and since we're using trap(ish) we need to divide by 4. Untested CJM - 2/13/2018
 write(*,*) QUAD%XINTEGRAL,QUAD%YINTEGRAL,QUAD%ZINTEGRAL,QUAD%PHIINTEGRAL,QUAD%THETAINTEGRAL,QUAD%PSIINTEGRAL,QUAD%UINTEGRAL
 !95713.325840134436       0.77060269505657364       -38.613821568062889        5.6208864356321653E-005  -3.6252368913735496E-003   1.5246634597683634E-002  -25.123939964081060 
END SUBROUTINE PRINTINTEGRAL !PRINTINTEGRAL

SUBROUTINE PWM2FORCE(QUAD)
  use COPTERDATATYPES
  implicit none
  type(COPTERSTRUCTURE) QUAD
  real*8 :: pp = 4

   ! IF (abs(QUAD%MS_ROLL) .gt. QUAD%MS_MAX) then
   !    QUAD%MS_ROLL = sign(QUAD%MS_MAX,QUAD%MS_ROLL)
   ! elseif (abs(QUAD%MS_ROLL) .lt. QUAD%MS_MIN) then
   !    QUAD%MS_ROLL = sign(QUAD%MS_MIN,QUAD%MS_ROLL)
   ! end if

   ! IF (abs(QUAD%MS_PITCH) .gt. QUAD%MS_MAX) then
   !    QUAD%MS_PITCH = sign(QUAD%MS_MAX,QUAD%MS_PITCH)
   ! elseif (abs(QUAD%MS_PITCH) .lt. QUAD%MS_MIN) then
   !    QUAD%MS_PITCH = sign(QUAD%MS_MIN,QUAD%MS_PITCH)
   ! end if

   ! IF (abs(QUAD%MS_YAW) .gt. QUAD%MS_MAX) then
   !    QUAD%MS_YAW = sign(QUAD%MS_MAX,QUAD%MS_YAW)
   ! elseif (abs(QUAD%MS_YAW) .lt. QUAD%MS_MIN) then
   !    QUAD%MS_YAW = sign(QUAD%MS_MIN,QUAD%MS_YAW)
   ! end if
   
  ! QUAD%MS_0 = 2.35252990909077E-06*QUAD%MS_0**2 - (0.00487992485215825)*QUAD%MS_0 + 1.74554459880932
    do pp = 1,4
      QUAD%PWM2F(pp,1) = 2.35252990909077E-06*QUAD%MUVEC(1,1)**2 - (0.00487992485215825)*QUAD%MUVEC(1,1) + 1.74554459880932
    end do
END SUBROUTINE
