%This script finds the maximum velocity vs heading of a landyacht based on:
%1. True wind speed
%2. Symmetric wing Lift/Drag correlations
%			a. Drag coefficient at zero angle of attack
%			b. Wing span, area & aspect ratio
%			c. Lift reduction after stall angle is reached
%3. Parasitic drag
%			a. Fuselage Drag
%			b. Crossbeam drag
%			c. Wheel rolling resistance
%4. Flipping criteria
%			a. Vehicle weight and geometry
%			b. Down force generation
%5. Side slipping
%			a. Available friction force
%			b. Down force generation
%6. Number of wings/sails
%7. Number of wheels (3 or 4)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MAIN VARIABLE DEFINITIONS
%all units are inches, pounds, seconds and radians unless otherwise stated
%phi = angle between true wind velocity and vehicle velocity
%theta = angle of wing(s) with respect to vehicle velocity (wing/sail trim)
%beta = angle between apparent wind and vehicle velocity
%alpha = angle of attack for wing(s) (alpha = beta - theta)
%V_TRUE = velocity of true wind
%vy = vehicle velocity relative to ground
%AR_WING = wing aspect ratio
%F_LIFT_WING = lift force on wing (perpendicular to apparent wind)
%F_DRAG_WING = drag force on wing (parallel to apparent wind)
%F_THRUST = net force "thrust" on vehicle in direction of vehicle velocity
%F_SIDE = sideways force on vehicle that causes slipping
%FLIP = a test value for flipping criteria, 0=no, 1=yes
%SLIP = a test value for slipping criteria, 0=no, 1=yes
%V_LIMIT = limiting yacht velocity when slipping and flipping criteria are ignored
%V_MAX = actual predicted maximum yacht velocity based on flip/slip criteria
%"CG" refers to center of gravity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;	%clear memory before running program
close all;

PLOT_ON = 0;    % TURN ON PLOTS IF EQUAL 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% YACHT DATA ENTRY
%SINGLE VALUE PARAMETERS THAT REMAIN CONSTANT FOR ENTIRE SIMULATION
%Change these values to match your vehicle's geometry and configuration

%air properties and other constants
RHO_AIR = 1.175*10^-7;					%density of air (lb*sec^2/in^4)
MU_AIR = 2.7326*10^-8;					%viscosity of air (lb*sec/in^2)
g = 32.2;								%gravity (ft/sec^2)

%true wind speed
% V_TRUE_MPH = 20;						%true wind velocity (mph)
V_TRUE_MPH = 11.1847;					%true wind velocity (mph)
V_TRUE = V_TRUE_MPH*(528/360)*12;   	%true wind velocity (in/sec)

%wing properties (assuming symmetric section)
N_WING   = 1;                   %number of wings/sails
CD0_WING = 0.2;                 %wing drag coefficient at zero angle of attack
S_WING   = 24;					%wing span (in)
tip_chord = 6;                  %(in)
root_chord = 12;                %(in)
A_WING   = S_WING*(tip_chord + root_chord)/2;		%wing area (in^2)
%HCE      = 13.43;					%height of wing center of effort from ground (in) 
Sail_Ground_Clearance = 2.7633; %in
HCE = Sail_Ground_Clearance + (S_WING^2 * (tip_chord/2 + (root_chord-tip_chord)/6)/A_WING);				%height of wing center of effort from ground for trapezoidal sail (in)
DIST_CE  = 5.8;                 %distance of center of effort from rear beam (in)
psi_deg  = 0;					%lean of mast from side stay slack (deg)
stall_deg = 18;					%wing stall angle- normally 16-20 deg (deg)
CL_DROP_FACTOR = 1.0;			%factor for exponetial lift dropoff after stall

psi = psi_deg*pi/180;           %lean of mast from side stay slack (rad)
stall = stall_deg*pi/180;		%stall angle (rad)
ARM_WING = HCE*cos(psi);		%moment arm for rig flipping moment (in)
C_WING = A_WING/S_WING;			%average wing chord length (in)
AR_WING = (S_WING^2)/A_WING;	%aspect ratio
CL_WING_MAX = 2*pi*sin(stall)/(1+2/AR_WING);	%maximum coefficient of lift before stall

%total yacht weight
% WEIGHT = 4.0;   %total vehicle weight including pilot (lb)
WEIGHT = 2.5;   %total vehicle weight including pilot (lb)
MASS = WEIGHT/g; % total vehicle mass (slug)

%fuselage/body properties
% AP_BODY = 10;       %total frontal fuselage projected area (in^2) DEFAULT
AP_BODY = 4.5;		%total frontal fuselage projected area (in^2)
CD_BODY = 0.5;		%vehicle fuselage drag coefficient 
% TRACK = 36;         %vehicle width/track (wheel center to center) (in) DEFAULT
TRACK = 24;			%vehicle width/track (wheel center to center) (in)
% WHEELBASE = 48;     %vehicle wheelbase (in)- only necessary for 3 wheel vehicle DEFAULT
WHEELBASE = 36;		%vehicle wheelbase (in)- only necessary for 3 wheel vehicle
% DIST_CG = 9;        %horizontal distance from CG to rear wheels (in)- only necessary for 3 wheels DEFAULT
DIST_CG = 5.8;		%horizontal distance from CG to rear wheels (in)- only necessary for 3 wheels
H_CG = 2;           %height of CG from ground

%wheel properties
N_WHEEL = 3;				%number of wheels
FRCT = 0.6745;				%coefficient of friction between wheels and road
% DIAM_WHEEL_FT = 3.0;        %front wheel diameter (in) DEFAULT
DIAM_WHEEL_FT = 2;          %front wheel diameter (in) 
% DIAM_WHEEL_RE = 4.0;        %rear wheel diameter (in) DEFAULT
DIAM_WHEEL_RE = 2.5;        %rear wheel diameter (in)
% B_WHEEL = 0.007;            %drag coefficient of rolling wheel, per wheel (lb/(rev/sec)) DEFAULT
B_WHEEL = 0.02;             %drag coefficient of rolling wheel, per wheel (lb/(rev/sec)) 
TURN_ANGLE_WHEEL_DEG = 45;  %angle between front wheel and body axis when turning
TURN_RADIUS = WHEELBASE/tan(TURN_ANGLE_WHEEL_DEG*pi/180); %turning radius
delta = atan2(DIST_CG,TURN_RADIUS); 
% ^^ angle between rear beam and the line connecting center of
%    turning radius and CG

Rolling_fric_coeff = 0.05112;
% F_DRAG_WHEEL_MIN = 0.5;		%total rolling resistance at zero velocity (lbs) DEFAULT
F_DRAG_WHEEL_MIN = Rolling_fric_coeff * WEIGHT;		%total rolling resistance at zero velocity (lbs)
%for small models on wheels with bearings B_WHEEL is on the order of 0.010 lb/(rev/sec)
%for full sized landyachts B_WHEEL is on the order of 0.10 lb/(rev/sec)

%crossbeam properties (lift-drag relations are for a symmetric naca section)
W_BEAM = 24;					%width of crossbeam available for generating down force (in)
T_BEAM = 0.4;					%maximum thickness of crossbeam wing section (in)
C_BEAM = 2.0;					%average crossbeam chord length (in)
A_BEAM = C_BEAM*W_BEAM;			%foil surface area per crossbeam (in^2)
AP_BEAM = T_BEAM*W_BEAM;		%projected frontal beam area (in^2)
AR_BEAM = (W_BEAM^2)/A_BEAM;	%aspect ratio of crossbeam

%relations for symmetric airfoil crossbeam with constant angle of attack
gamma_deg = 0;                                  %crossbeam angle of attack for generating down force (deg)
gamma = gamma_deg*pi/180;                       %crossbeam angle of attack for generating down force (rad)
CD0_BEAM = 0.10;								%crossbeam drag coefficient at zero angle of attack
CL_BEAM = 2*pi*sin(gamma)/(1+2/AR_BEAM);		%crossbeam coefficient of lift
CD_BEAM = CD0_BEAM+((CL_BEAM^2)/(pi*AR_BEAM));	%crossbeam coefficient of drag
%END OF YACHT DATA ENTRY

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% PRE-LOOP CALCULATIONS
%CG/BEAM moment arm for righting moment calculation
if N_WHEEL == 4	%for a 4 wheeled symmetric yacht (track front = track rear)

    N_WHEEL_FT = 2;         %two front wheels
    N_WHEEL_RE = 2;			%two rear wheels
    zeta = 0;				%angle between flipping force and F_SIDE is zero (rad)
    ARM_CG = TRACK/2;		%cg moment arm for righting moment (in)
    ARM_BEAM = TRACK/2;		%beam moment arm for righting moment (in)
    N_BEAM = 2;				%number of crossbeams for down force generation (probably has 2 crossbeams)

elseif N_WHEEL == 3	%for a 3 wheeled symmetric yacht (2 wheels in rear)

    N_WHEEL_FT = 1;							%one front wheel
    N_WHEEL_RE = 2;							%two rear wheels
    zeta = atan((TRACK/2)/WHEELBASE);		%angle made between front wheel and each rear wheel (rad)
    ARM_CG = (WHEELBASE-DIST_CG)*sin(zeta);	%cg moment arm for righting moment (in)
    ARM_CE = (WHEELBASE-DIST_CE)*sin(zeta); %CE moment arm for righting moment (in)
    ARM_BEAM = (TRACK/2)*cos(zeta);			%beam moment arm for righting moment (in)
    N_BEAM = 1;								%number of crossbeams for down force generation (rear crossbeam only)
    
    if PLOT_ON
        % plot top down view
        figure()
        h(1) = plot([0 WHEELBASE], [0 0],'b','LineWidth',5); % plot wheel base
        hold on, grid on,
        h(2) = plot([WHEELBASE WHEELBASE], [-TRACK/2 TRACK/2],'b','LineWidth',5);
        h(3) = plot([WHEELBASE-DIST_CG],0,'rx','MarkerSize',10,'LineWidth',3);
        xlabel('X (in)'), ylabel('Y (in)')
        title('Top Down View of Car and CG location')
        legend([h(1), h(3)],'Base','CG','location','best')
        axis('equal')
        clear h;

        % plot side view
        figure()
        h(1) = plot([0 WHEELBASE], [DIAM_WHEEL_FT DIAM_WHEEL_RE/2],'b','LineWidth',5); % plot wheel base (approximate length)
        hold on, grid on,
        h(2) = circle(0,DIAM_WHEEL_FT/2,DIAM_WHEEL_FT/2); % draw front wheel
        h(3) = circle(WHEELBASE,DIAM_WHEEL_RE/2,DIAM_WHEEL_RE/2); % draw rear wheel
        title('Side View of Car')
        axis('equal')
        clear h;
    end
end

%main program loop parameters/starting values
e = 0.050;		%error sensitivity for thrust equilibrium check (lbs)
dv = 0;			%initial value for velocity change (in/sec)
vy0 = 0;		%initial value for vehicle velocity (in/sec)
vy = vy0;		%assign initial value to vy (in/sec)
F_THRUST = 1;	%initial non-zero value for thrust force (lbs)
ay0 = F_THRUST/MASS;    %initial value for acceleration (in/sec^2)
ay = ay0;               %assign initial value to ay (in/sec^2)

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% MAIN PROGRAM LOOP
%BEGIN MAIN PROGRAM LOOP THROUGH ALL VALUES OF PHI
for p = 1:1:P		%loop for entire range of phi (all possible headings)

%     Steps_Remaining = P-p+1		%display number of steps remaining in loop to track progress
	phi = PHI(p);				%lookup value phi for this loop (rad)
    t = 1;						%initialize t counting variable

	while t <= T	%find VY_max for loop through for all values of theta
        
        %setup theta and vy for this value of t  
        theta=THETA(t);								%lookup current theta (rad)
        SIGN_F_THRUST = F_THRUST/abs(F_THRUST);		%find the sign of F_THRUST
        vy = vy + SIGN_F_THRUST*dv;					%increment/decrement guess for vehicle velocity (in/sec)

        %do calculations necessary to find net thrust
		if V_TRUE*cos(phi) + vy > 0			%if apparent wind is from bow
			beta = atan((V_TRUE*sin(phi))/(vy+V_TRUE*cos(phi)));						%calc beta for beta < pi/2 (rad)
        else								%if apparent wind is from stern
            beta = atan((V_TRUE*sin(phi-(pi/2))-vy)/(V_TRUE*cos(phi-(pi/2))))+pi/2;		%calc beta for beta > pi/2 (rad)
        end %end if 
        
        alpha = beta-theta;                                             %calc alpha - wing angle of attack (rad)
        V_APPARENT = sqrt((vy+V_TRUE*cos(phi))^2+(V_TRUE*sin(phi))^2);	%calc magnitude of apparent wind velocity (in/sec)
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
        RPS_WHEEL_FT = vy/(pi*DIAM_WHEEL_FT);										%calc front wheel rev per sec (rps)
        RPS_WHEEL_RE = vy/(pi*DIAM_WHEEL_RE);										%calc rear wheel rev per sec (rps)
        F_DRAG_WHEEL = B_WHEEL*(N_WHEEL_FT*RPS_WHEEL_FT + N_WHEEL_RE*RPS_WHEEL_RE);	%calc total wheel drag force (lbs)
        F_DRAG_WHEEL = max(F_DRAG_WHEEL,F_DRAG_WHEEL_MIN);          				%use minimum wheel rolling resistance for low speeds (lbs)

        %sum up all forward thrust forces
        F_SIDE = F_LIFT_WING*cos(beta)*cos(psi)+F_DRAG_WING*sin(beta);		%calc sideways sliding force (lbs)
        F_THRUST = F_LIFT_WING*sin(beta)*cos(psi)-F_DRAG_WING*cos(beta)-F_DRAG_BODY-F_DRAG_BEAM-F_DRAG_WHEEL;	%calc net thrust force (lbs)
        
        % calculate acceleration in the forward thrust direction
        ay = F_THRUST/MASS;
        
        %SLIP/FLIP required quantities (righting/flipping moment and side friction)
        F_DOWN = WEIGHT + F_LIFT_BEAM + F_LIFT_WING*sin(psi);   %total down force from crossbeam and wing (lb)
        M_RIGHT = WEIGHT*ARM_CG + F_LIFT_BEAM*ARM_BEAM ...
                  + F_LIFT_WING*sin(psi)*ARM_CE;				%available righting moment (in*lb)
        F_FLIP = F_SIDE*cos(zeta) + F_THRUST*sin(zeta);			%component of rig force perpendicular to supporing wheels (lbs)
        M_FLIP = F_FLIP*ARM_WING;								%flipping moment from rig (in*lb)
        F_FRICT = F_DOWN*FRCT;									%total available sideways friction force (lbs)
        
        A_CENTRI = vy^2/TURN_RADIUS; %centrifugal acceleration(in/sec^2)
        A_CENTRI_FPS = A_CENTRI/12;
        M_FLIP_TURN = F_FLIP*ARM_WING+MASS*A_CENTRI_FPS*H_CG*cos(abs(zeta-delta));
        %END OF ALL FORCE AND MOMENT CALCULATIONS FOR THIS LOOP

        %scale velocity step change for next loop
        X = 2;						%scale factor for calculating dv
        dv = X*abs(F_THRUST);		%scale velocity change by F_THRUST (if X is too large loop becomes unstable)
		
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
		if vy > 0 & alpha > 0 & ay > 0		%only continue for loops that lead to positive velocity/accel and angle of attack
			if abs(F_THRUST) < e	%if thrust is close to zero (lbs)
                VY(t) = vy;			%store final value of vehicle speed from this loop (in/sec) 
                AY(t) = ay;         %store final value of vehical acceleration from this loop (in/sec^2)
        		t = t+1;			%increment t to calculate vehicle speed for next theta
     		end 	  				%end thrust check if loop
        else %if yacht velocity or angle of attack is not positive
     		VY(t) = 0;				%assign this velocity to zero (do not keep track of negative velocities)
            AY(t) = 0;             %store final value of vehicle acceleration from this loop (in/sec^2)
            t = t+1;				%increment t to calculate vehicle speed for next theta
            vy = vy0;				%reset initial velocity guess
            ay = ay0;
        end	%end else loop
        
    end	%end of while  t <= T loop
	
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
    
    %store theta that corresponds to this VMAX
    V_MAX(p) = max(VY);			%store maximum velocity found for this angle phi to VMAX(p)
    A_MAX(p) = max(AY);         %store maximum acceleration found for this angle phi to AMAX(p)
	q = 1;						%initialize counting var q = 1
%    while VY(q) ~= V_MAX(p)		%while VY_TRUNK(q) not equal VMAX
    while AY(q) ~= A_MAX(p)     %while AY_trunk(q) not equal to AMAX
        q = q+1;				%increment q until VY_TRUNK(q) = VMAX(p)
	end							%end while loop
    THETA_MAX(p) = THETA(q);	%store theta that corresponds to AMAX(p) (rad)
    
end %end of "for 1:1:P" loop
%END MAIN PROGRAM LOOP

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ORGANIZE DATA 
%ORGANIZE DATA INTO MAXIMUM SPEED INFORMATION
TOP_ACCEL_IN_SEC = max(A_MAX);                  %store top accel in in/sec^2
TOP_SPEED_IN_SEC = max(V_MAX);					%store top speed in in/sec (in/sec)
TOP_ACCEL_FT_SEC = TOP_ACCEL_IN_SEC/12;         %convert top accel to ft/sec^2
TOP_SPEED_FT_SEC = TOP_SPEED_IN_SEC/12;			%convert top speed to ft/sec (ft/sec)
TOP_SPEED_MPH = TOP_SPEED_FT_SEC*(360/528);		%convert top speed to mph (mph)
V_LIMIT_MPH = (V_LIMIT/12)*(360/528);			%convert limit speed to mph (mph)
V_MAX_FT_SEC = V_MAX/12;						%convert maximum speed matrix to mph (ft/sec)
V_MAX_MPH = (V_MAX_FT_SEC)*(360/528);			%convert maximum speed matrix to mph (mph)

for j= 1:1:P-1							%loop through all of matrix A_MAX
%   if V_MAX(j) == TOP_SPEED_IN_SEC		%look for entry of maximum velocity
    if A_MAX(j) == TOP_ACCEL_IN_SEC     %look for entry of maximum acceleration
      J = j;							%if this is that entry, save entry number
   end									%end if loop
end										%end for loop

THETA_TOP_ACCEL_RAD = THETA_MAX(J);
PHI_TOP_ACCEL_RAD   = PHI(J);
THETA_TOP_ACCEL_DEG = THETA_TOP_ACCEL_RAD*180/pi;
PHI_TOP_ACCEL_DEG   = PHI_TOP_ACCEL_RAD*180/pi;
V_TOP_ACCEL_IN_SEC  = V_MAX(J);
V_TOP_ACCEL_FT_SEC  = V_TOP_ACCEL_IN_SEC/12;

%THETA_TOP_SPEED_RAD = THETA_MAX(J);					%store theta that corresponds to TOP_SPEED_MPH (rad)
%PHI_TOP_SPEED_RAD   = PHI(J);						%store phi that corresponds to TOP_SPEED_MPH (rad)
%THETA_TOP_SPEED_DEG = THETA_TOP_SPEED_RAD*180/pi;	%convert to degrees (deg)
%PHI_TOP_SPEED_DEG   = PHI_TOP_SPEED_RAD*180/pi;     %convert to degrees (deg)
%plot velocity results in polar coordinates
% RHO_MAX = 3*ceil(max(V_LIMIT_MPH)/3);				%get maximum radius in whole numbers divisible by 3
% 
% MP = 1;												%subplot dimension
% NP = 1;												%subplot dimension
% 
% figure()
% subplot(MP,NP,1),polar(0,RHO_MAX);					%create polar plot with whole number scale
% subplot(MP,NP,1),hold;								%hold plot so data can be plotted
% subplot(MP,NP,1),polar(PHI,V_LIMIT_MPH,'ro--');		%polar plot limit velocity vs phi in red
% subplot(MP,NP,1),polar(-PHI,V_LIMIT_MPH,'ro--');	%polar plot limit velocity vs -phi (mirror image)
% subplot(MP,NP,1),polar(PHI,V_MAX_MPH,'bx-'); 		%polar plot maximum velocity vs phi in blue
% subplot(MP,NP,1),polar(-PHI,V_MAX_MPH,'bx-');		%polar plot maximum velocity vs -phi (mirror image)
% %subplot(MP,NP,1),title('Polar Vel. Plot (mph)');	%title plot
% subplot(MP,NP,1),hold;								%release plot to return to default condition
% % legend('Blue','Red')

%display desired values
% TOP_SPEED_MPH = round(TOP_SPEED_MPH)				%round and print maximum velocity (mph)
% PHI_TOP_SPEED_DEG = round(PHI_TOP_SPEED_DEG)		%round and print phi for TOP SPEED (deg)
% THETA_TOP_SPEED_DEG = round(THETA_TOP_SPEED_DEG)	%round and print theta for TOP SPEED (deg)
TOP_ACCEL_FT_SEC
V_TOP_ACCEL_FT_SEC
PHI_TOP_ACCEL_DEG
THETA_TOP_ACCEL_DEG

%calculate best angle to move forward (assume no slipping or tipping)
% d = 14; %(ft) course length
% T_COURSE = d*12./(V_LIMIT.*cos(PHI));
% 
% figure()
% plot(PHI(1:18)*180/pi,T_COURSE(1:18)), hold on, grid on
% title('Course Completion Time vs Heading','fontweight','bold')
% xlabel('Heading Angle (deg)','fontweight','bold'), ylabel('time(sec)','fontweight','bold')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%END MAIN PROGRAM