% load_parameters
% YACHT DATA ENTRY
%SINGLE VALUE PARAMETERS THAT REMAIN CONSTANT FOR ENTIRE SIMULATION
%Change these values to match your vehicle's geometry and configuration

%air properties and other constants
car.RHO_AIR = 1.175*10^-7;					%density of air (lb*sec^2/in^4)
MU_AIR = 2.7326*10^-8;					%viscosity of air (lb*sec/in^2)
g = 32.2;								%gravity (ft/sec^2)

%true wind speed
V_TRUE_MPH = 11.1847;					%true wind velocity (mph)
car.V_TRUE = V_TRUE_MPH*(528/360)*12;   	%true wind velocity (in/sec)

%wing properties (assuming symmetric section)
car.N_WING   = 1;                   %number of wings/sails
car.CD0_WING = 0.2;                 %wing drag coefficient at zero angle of attack
baseCar.S_WING   = 24;					%wing span (in)
baseCar.tip_chord = 1;                  %(in)
baseCar.root_chord = 12;                %(in)
baseCar.Sail_Ground_Clearance = 2.7633; %in
DIST_CE  = 5.8;                 %distance of center of effort from rear beam (in)
baseCar.psi_deg  = 0;					%lean of mast from side stay slack (deg)
stall_deg = 18;					%wing stall angle- normally 16-20 deg (deg)
car.CL_DROP_FACTOR = 1.0;			%factor for exponetial lift dropoff after stall

car.stall = stall_deg*pi/180;		%stall angle (rad)

%total yacht weight
car.WEIGHT = 2.5;   %total vehicle weight including pilot (lb)
car.MASS = car.WEIGHT/g; % total vehicle mass (slug)

%fuselage/body properties
baseCar.AP_BODY = 4.5;		%total frontal fuselage projected area (in^2)
car.CD_BODY = 0.5;		%vehicle fuselage drag coefficient 
baseCar.TRACK = 16;			%vehicle width/track (wheel center to center) (in)
baseCar.WHEELBASE = 24.1;	%vehicle wheelbase (in)- only necessary for 3 wheel vehicle
DIST_CG = 5.8;		%horizontal distance from CG to rear wheels (in)- only necessary for 3 wheels
car.H_CG = 2;           %height of CG from ground

%wheel properties
N_WHEEL = 3;				%number of wheels
car.FRCT = 0.6745;				%coefficient of friction between wheels and road
car.DIAM_WHEEL_FT = 2;          %front wheel diameter (in) 
car.DIAM_WHEEL_RE = 2.5;        %rear wheel diameter (in)
car.B_WHEEL = 0.02;             %drag coefficient of rolling wheel, per wheel (lb/(rev/sec)) 
TURN_ANGLE_WHEEL_DEG = 45;  %angle between front wheel and body axis when turning

Rolling_fric_coeff = 0.05112;
car.F_DRAG_WHEEL_MIN = Rolling_fric_coeff * car.WEIGHT;		%total rolling resistance at zero velocity (lbs)
%for small models on wheels with bearings B_WHEEL is on the order of 0.010 lb/(rev/sec)
%for full sized landyachts B_WHEEL is on the order of 0.10 lb/(rev/sec)

%crossbeam properties (lift-drag relations are for a symmetric naca section)
baseCar.W_BEAM = 16;					%width of crossbeam available for generating down force (in)
baseCar.T_BEAM = 0.4;					%maximum thickness of crossbeam wing section (in)
baseCar.C_BEAM = 2.0;					%average crossbeam chord length (in)

%relations for symmetric airfoil crossbeam with constant angle of attack
gamma_deg = 0;                                  %crossbeam angle of attack for generating down force (deg)
gamma = gamma_deg*pi/180;                       %crossbeam angle of attack for generating down force (rad)
CD0_BEAM = 0.10;								%crossbeam drag coefficient at zero angle of attack

car.N_BEAM = 1;								%number of crossbeams for down force generation (rear crossbeam only)
car.N_WHEEL_FT = 1;							%one front wheel
car.N_WHEEL_RE = 2;							%two rear wheels

DIST = 14;
%END OF YACHT DATA ENTRY