% calc_random_car.m
%
% Description:
%   This script will randomly vary the parameters of the car ship.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

VARY_WING = 0;  % 1 is to vary parameters
VARY_FUSE = 0;  % 0 is to not vary parameters

load_variation_parameters;

% WING --------------------------------------------------------------------

if VARY_WING
   
    car.S_WING = check_allowable(baseCar.S_WING,(rand-0.5)*SD.S_WING,0,36); % wing span (in)
    car.root_chord = check_allowable(baseCar.root_chord,(rand-0.5)*SD.root_chord,0,36); % root chord (in)
    car.tip_chord = check_allowable(baseCar.tip_chord,(rand-0.5)*SD.tip_chord,0,car.root_chord);    % tip chord (in)
    car.psi_deg = check_allowable(baseCar.psi_deg,(rand-0.5)*SD.psi_deg,0,45);  % mast deflection (deg)
else
    car.S_WING = baseCar.S_WING;
    car.root_chord = baseCar.root_chord;
    car.tip_chord = baseCar.tip_chord;
    car.psi_deg = baseCar.psi_deg;
end

% FUSELAGE ----------------------------------------------------------------

if VARY_FUSE
   
    car.AP_BODY = check_allowable(baseCar.AP_BODY,(rand-0.5)*SD.AP_BODY,1,10);  % projected frontal area
    car.TRACK   = check_allowable(baseCar.TRACK,(rand-0.5)*SD.TRACK,1,30);  % track, vehicle width (in)
    car.WHEELBASE = check_allowable(baseCar.WHEELBASE,(rand-0.5)*SD.WHEELBASE,10,40);
    car.W_BEAM = car.TRACK;
    car.T_BEAM = check_allowable(baseCar.T_BEAM,(rand-0.5)*SD.T_BEAM,0.1,5);
    car.C_BEAM = check_allowable(baseCar.C_BEAM,(rand-0.5)*SD.C_BEAM,1,10);
else
    car.AP_BODY = baseCar.AP_BODY;
    car.TRACK = baseCar.TRACK;
    car.WHEELBASE = baseCar.WHEELBASE;
    car.W_BEAM = baseCar.W_BEAM;
    car.T_BEAM = baseCar.T_BEAM;
    car.C_BEAM = baseCar.C_BEAM;
end

car.A_WING   = car.S_WING*(car.tip_chord + car.root_chord)/2;		%wing area (in^2)
car.Sail_Ground_Clearance = baseCar.Sail_Ground_Clearance;
car.HCE = car.Sail_Ground_Clearance + (car.S_WING^2 * (car.tip_chord/2 + (car.root_chord-car.tip_chord)/6)/car.A_WING);				%height of wing center of effort from ground for trapezoidal sail (in)
car.psi = car.psi_deg*pi/180;           %lean of mast from side stay slack (rad)
car.ARM_WING = car.HCE*cos(car.psi);		%moment arm for rig flipping moment (in)
car.C_WING = car.A_WING/car.S_WING;			%average wing chord length (in)
car.AR_WING = (car.S_WING^2)/car.A_WING;	%aspect ratio
car.CL_WING_MAX = 2*pi*sin(car.stall)/(1+2/car.AR_WING);	%maximum coefficient of lift before stall

car.TURN_RADIUS = car.WHEELBASE/tan(TURN_ANGLE_WHEEL_DEG*pi/180); %turning radius
car.delta = atan2(DIST_CG,car.TURN_RADIUS); 
% ^^ angle between rear beam and the line connecting center of
%    turning radius and CG
car.A_BEAM = car.C_BEAM*car.W_BEAM;			%foil surface area per crossbeam (in^2)
car.AP_BEAM = car.T_BEAM*car.W_BEAM;		%projected frontal beam area (in^2)
car.AR_BEAM = (car.W_BEAM^2)/car.A_BEAM;	%aspect ratio of crossbeam
car.CL_BEAM = 2*pi*sin(gamma)/(1+2/car.AR_BEAM);		%crossbeam coefficient of lift
car.CD_BEAM = CD0_BEAM+((car.CL_BEAM^2)/(pi*car.AR_BEAM));	%crossbeam coefficient of drag

car.zeta_ = atan((car.TRACK/2)/car.WHEELBASE);		%angle made between front wheel and each rear wheel (rad)
car.ARM_CG = (car.WHEELBASE-DIST_CG)*sin(car.zeta_);	%cg moment arm for righting moment (in)
car.ARM_CE = (car.WHEELBASE-DIST_CE)*sin(car.zeta_); %CE moment arm for righting moment (in)
car.ARM_BEAM = (car.TRACK/2)*cos(car.zeta_);			%beam moment arm for righting moment (in)



