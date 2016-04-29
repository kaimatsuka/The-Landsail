% load_parameters
% YACHT DATA ENTRY
%SINGLE VALUE PARAMETERS THAT REMAIN CONSTANT FOR ENTIRE SIMULATION
%Change these values to match your vehicle's geometry and configuration

%air properties and other constants
RHO_AIR = 1.175*10^-7;					%density of air (lb*sec^2/in^4)
MU_AIR = 2.7326*10^-8;					%viscosity of air (lb*sec/in^2)
g = 32.2;								%gravity (ft/sec^2)

%true wind speed
V_TRUE_MPH = 11.1847;					%true wind velocity (mph)
V_TRUE = V_TRUE_MPH*(528/360)*12;   	%true wind velocity (in/sec)

%wing properties (assuming symmetric section)
N_WING   = 1;                   %number of wings/sails
CD0_WING = 0.2;                 %wing drag coefficient at zero angle of attack
S_WING   = 24;					%wing span (in)
tip_chord = 1;                  %(in)
root_chord = 12;                %(in)
A_WING   = S_WING*(tip_chord + root_chord)/2;		%wing area (in^2)
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
WEIGHT = 2.5;   %total vehicle weight including pilot (lb)
MASS = WEIGHT/g; % total vehicle mass (slug)

%fuselage/body properties
AP_BODY = 4.5;		%total frontal fuselage projected area (in^2)
CD_BODY = 0.5;		%vehicle fuselage drag coefficient 
TRACK = 16;			%vehicle width/track (wheel center to center) (in)
WHEELBASE = 24.1;	%vehicle wheelbase (in)- only necessary for 3 wheel vehicle
DIST_CG = 5.8;		%horizontal distance from CG to rear wheels (in)- only necessary for 3 wheels
H_CG = 2;           %height of CG from ground

%wheel properties
N_WHEEL = 3;				%number of wheels
FRCT = 0.6745;				%coefficient of friction between wheels and road
DIAM_WHEEL_FT = 2;          %front wheel diameter (in) 
DIAM_WHEEL_RE = 2.5;        %rear wheel diameter (in)
B_WHEEL = 0.02;             %drag coefficient of rolling wheel, per wheel (lb/(rev/sec)) 
TURN_ANGLE_WHEEL_DEG = 45;  %angle between front wheel and body axis when turning
TURN_RADIUS = WHEELBASE/tan(TURN_ANGLE_WHEEL_DEG*pi/180); %turning radius
delta = atan2(DIST_CG,TURN_RADIUS); 
% ^^ angle between rear beam and the line connecting center of
%    turning radius and CG

Rolling_fric_coeff = 0.05112;
F_DRAG_WHEEL_MIN = Rolling_fric_coeff * WEIGHT;		%total rolling resistance at zero velocity (lbs)
%for small models on wheels with bearings B_WHEEL is on the order of 0.010 lb/(rev/sec)
%for full sized landyachts B_WHEEL is on the order of 0.10 lb/(rev/sec)

%crossbeam properties (lift-drag relations are for a symmetric naca section)
W_BEAM = 16;					%width of crossbeam available for generating down force (in)
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

N_BEAM = 1;								%number of crossbeams for down force generation (rear crossbeam only)
N_WHEEL_FT = 1;							%one front wheel
N_WHEEL_RE = 2;							%two rear wheels
zeta_ = atan((TRACK/2)/WHEELBASE);		%angle made between front wheel and each rear wheel (rad)
ARM_CG = (WHEELBASE-DIST_CG)*sin(zeta_);	%cg moment arm for righting moment (in)
ARM_CE = (WHEELBASE-DIST_CE)*sin(zeta_); %CE moment arm for righting moment (in)
ARM_BEAM = (TRACK/2)*cos(zeta_);			%beam moment arm for righting moment (in)

DIST = 14;
%END OF YACHT DATA ENTRY