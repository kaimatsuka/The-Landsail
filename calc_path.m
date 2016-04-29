function [ay,vy,theta,phi] = calc_path(vy0,dt,psi)

load_parameters;
%define matrices PHI and THETA for main program loops
%for smoother plot make N_phi and N_theta larger (this will make the program take longer to run)
N_phi = 100;								%number of divisions in PHI
phi_max = pi/2;								%maximum phi = 180 degrees (pi rad)
PHI = [0:phi_max/N_phi:phi_max];			%vector phi in radians from head-to-wind to 180 degrees (rad)
P = N_phi+1;                                %number of loops to be made for varying phi
N_theta = 120;								%number of divisions in THETA
theta_max = pi/3;							%maximum theta to test during loop (rad)
THETA = [0:theta_max/N_theta:theta_max];	%define vector of wing angles from zero to 90 deg (rad)
T = N_theta+1;								%number of loops to be made for varying theta

%BEGIN MAIN PROGRAM LOOP THROUGH ALL VALUES OF PHI
for p = 1:1:P		%loop for entire range of phi (all possible headings)

%     Steps_Remaining = P-p+1		%display number of steps remaining in loop to track progress
	phi = PHI(p);				%lookup value phi for this loop (rad)
    t = 1;						%initialize t counting variable

	while t <= T	%find VY_max for loop through for all values of theta
        
        %setup theta and vy for this value of t  
        theta=THETA(t);								%lookup current theta (rad)
        
        %do calculations necessary to find net thrust
		if V_TRUE*cos(phi) + vy0 > 0			%if apparent wind is from bow
			beta = atan((V_TRUE*sin(phi))/(vy0+V_TRUE*cos(phi)));						%calc beta for beta < pi/2 (rad)
        else								%if apparent wind is from stern
            beta = atan((V_TRUE*sin(phi-(pi/2))-vy0)/(V_TRUE*cos(phi-(pi/2))))+pi/2;		%calc beta for beta > pi/2 (rad)
        end %end if 
        
        alpha = beta-theta;                                             %calc alpha - wing angle of attack (rad)
        V_APPARENT = sqrt((vy0+V_TRUE*cos(phi))^2+(V_TRUE*sin(phi))^2);	%calc magnitude of apparent wind velocity (in/sec)
        V_FRONT = V_APPARENT*cos(beta);									%calc apparent wind velocity in direction of vehicle velocity (in/sec)
        SIGN_V_FRONT = cos(beta)/abs(cos(beta));						%direction of front component of apparent wind (+1 from bow, -1 from stern)
        
        %wing lift and drag with rough approximation to lift drop off after stall
        if alpha > stall %if angle of attack is greater than stall angle
            CL_WING = (CL_WING_MAX*exp(stall))*exp(-CL_DROP_FACTOR*alpha);  %calc reduced wing lift coefficient
            CD_WING = CD0_WING+((CL_WING^2)/(pi*AR_WING));				    %calc wing drag coefficient for this angle of attack
        else %if angle of attack is less than stall angle
            CL_WING = 2*pi*sin(alpha)/(1+2/AR_WING);					    %calc wing lift coefficient for this angle of attack
            CD_WING = CD0_WING+((CL_WING^2)/(pi*AR_WING));				    %calc wing drag coefficient for this angle of attack
        end	%end if loop
        
        AP_WING = A_WING*(1-((1-cos(psi))*alpha)/(pi/2));	%linear approximation to adjust effective wing area based on alpha and psi (in^2)
        
        %NOTE: when alpha is zero, AP_WING = A_WING, if alpha is 90 deg, AP_WING = A_WING*cos(psi)
        %this leads to the linear approximation: AP_WING = A_WING*(1-((1-cos(psi))*alpha)/(pi/2));
        F_LIFT_WING = N_WING*0.5*CL_WING*RHO_AIR*(V_APPARENT^2)*AP_WING;	%calc total wing lift force (lbs)
        F_DRAG_WING = N_WING*0.5*CD_WING*RHO_AIR*(V_APPARENT^2)*AP_WING;	%calc total wing drag force (lbs)
      
        %yacht/fuselage drag (not including crossbeams)
        F_DRAG_BODY = SIGN_V_FRONT*0.5*CD_BODY*RHO_AIR*(V_FRONT^2)*AP_BODY;	%calc fuselage drag force (lbs)
        
        %crossbeam lift (down force) and drag
    	if SIGN_V_FRONT == 1 %for apparent wind from bow
            F_LIFT_BEAM = N_BEAM*0.5*CL_BEAM*RHO_AIR*(V_FRONT^2)*A_BEAM;	%calc total crossbeam lift force (lbs)
            F_DRAG_BEAM = N_BEAM*0.5*CD_BEAM*RHO_AIR*(V_FRONT^2)*A_BEAM;	%calc total crossbeam drag force (lbs)
        else %for apparent wind from stern
            F_LIFT_BEAM = 0;                                				%crossbeam generates zero lift
            F_DRAG_BEAM = -2*N_BEAM*0.5*CD_BEAM*RHO_AIR*(V_FRONT^2)*A_BEAM;	%calc total crossbeam drag force (lbs)
        end	%end if loop
    
        %wheel drag (rolling viscous friction)
        RPS_WHEEL_FT = vy0/(pi*DIAM_WHEEL_FT);										%calc front wheel rev per sec (rps)
        RPS_WHEEL_RE = vy0/(pi*DIAM_WHEEL_RE);										%calc rear wheel rev per sec (rps)
        F_DRAG_WHEEL = B_WHEEL*(N_WHEEL_FT*RPS_WHEEL_FT + N_WHEEL_RE*RPS_WHEEL_RE);	%calc total wheel drag force (lbs)
        F_DRAG_WHEEL = max(F_DRAG_WHEEL,F_DRAG_WHEEL_MIN);          				%use minimum wheel rolling resistance for low speeds (lbs)

        %sum up all forward thrust forces
        F_SIDE = F_LIFT_WING*cos(beta)*cos(psi)+F_DRAG_WING*sin(beta);		%calc sideways sliding force (lbs)
        F_THRUST = F_LIFT_WING*sin(beta)*cos(psi)-F_DRAG_WING*cos(beta)-F_DRAG_BODY-F_DRAG_BEAM-F_DRAG_WHEEL;	%calc net thrust force (lbs)
        
        ay = F_THRUST/MASS;
        vy = (ay*dt)+vy0;
        
        %SLIP/FLIP required quantities (righting/flipping moment and side friction)
        F_DOWN = WEIGHT + F_LIFT_BEAM + F_LIFT_WING*sin(psi);   %total down force from crossbeam and wing (lb)
        M_RIGHT = WEIGHT*ARM_CG + F_LIFT_BEAM*ARM_BEAM ...
                  + F_LIFT_WING*sin(psi)*ARM_CE;				%available righting moment (in*lb)
        F_FLIP = F_SIDE*cos(zeta) + F_THRUST*sin(zeta);			%component of rig force perpendicular to supporing wheels (lbs)
        M_FLIP = F_FLIP*ARM_WING;								%flipping moment from rig (in*lb)
        F_FRICT = F_DOWN*FRCT;									%total available sideways friction force (lbs)
        
        A_CENTRI = vy0^2/TURN_RADIUS; %centrifugal acceleration(in/sec^2)
        A_CENTRI_FPS = A_CENTRI/12;
        M_FLIP_TURN = F_FLIP*ARM_WING+MASS*A_CENTRI_FPS*H_CG*cos(abs(zeta-delta));
        %END OF ALL FORCE AND MOMENT CALCULATIONS FOR THIS LOOP
        
        %perform slip criteria calculations
  		if F_SIDE > F_FRICT	%if side sliding force from rig exceeds available friction force
			SLIP(t) = 1;		%set SLIP to 1 (yes)
		else				%if friction is greater than sliding force
			SLIP(t) = 0;		%set SLIP to 0 (no)
		end	%end SLIP if loop
        
		%perform flip criteria calculations
		if M_FLIP > M_RIGHT	 %if flipping moment from rig exceeds available righting moment
			FLIP(t) = 1;            %set FLIP to 1 (yes)
        else			     %if righting moment is greater than flipping moment
			FLIP(t) = 0;			%set FLIP to 0 (no)
		end	%end FLIP if loop
		
        if M_FLIP_TURN > M_RIGHT
            FLIP_TURN(t) = 1;
        else
            FLIP_TURN(t) = 0;
        end
        
        %check for positive thrust force near zero, if so go to next theta value
        if vy > 0 & alpha > 0		%only continue for loops that lead to positive velocity/accel and angle of attack
            VY(t) = vy;			%store final value of vehicle speed from this loop (in/sec) 
            AY(t) = ay;         %store final value of vehical acceleration from this loop (in/sec^2)
            t = t+1;			%increment t to calculate vehicle speed for next theta
        else %if yacht velocity or angle of attack is not positive
     		VY(t) = 0;				%assign this velocity to zero (do not keep track of negative velocities)
            AY(t) = 0;             %store final value of vehicle acceleration from this loop (in/sec^2)
            t = t+1;				%increment t to calculate vehicle speed for next theta
        end
    end
    
    %truncate VY matrix to remove infinite element
    V_LIMIT(p) = max(VY); %store V_LIMIT vector as maximum velocity before SLIP and FLIP are considered (in/sec)
    %truncate AY matrix to remove infinite element
    A_LIMIT(p) = max(AY);
    
    %change all velocity entries to zero if SLIP or FLIP = 1 (yes)
    for z = 1:1:T %loop through entire matrix for each theta
 		if (SLIP(z) + FLIP(z) +FLIP_TURN(z) > 0)	%if FLIP or SLIP = 1 (yes)
  	  		VY(z) = 0;	%set this velocity element to zero
            AY(z) = 0;  %set this acceleration element to zero
     	end	%end if loop
    end	%end for loop
    
    %store theta that corresponds to this AMAX
    A_MAX(p) = max(AY);         %store maximum acceleration found for this angle phi to AMAX(p)
	q = 1;						%initialize counting var q = 1
    while AY(q) ~= A_MAX(p)     %while AY_trunk(q) not equal to AMAX
        q = q+1;				%increment q until VY_TRUNK(q) = VMAX(p)
	end							%end while loop
    THETA_MAX(p) = THETA(q);	%store theta that corresponds to AMAX(p) (rad)
    
    A_MAX_FORWARD(p) = A_MAX(p)*cos(phi);
end %end of "for 1:1:P" loop

ind = find(A_MAX_FORWARD==max(A_MAX_FORWARD));
theta = THETA_MAX(ind);
phi = PHI(ind);
ay = max(A_MAX);
vy = VY(ind);