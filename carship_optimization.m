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

MONTE_CARLO = 0; 

N_ITERATIONS = 1;

if MONTE_CARLO
    VARY_WING = 0;   % 1 is to vary parameters
    VARY_FUSE = 0;   % 0 is to not vary parameters
    VARY_WEIGHT = 0; 
else
    VARY_WING = 0;
    VARY_FUSE = 0;
    VARY_WEIGHT = 0;
    N_ITERATIONS = 1;
end

WORSE_CASE = 0;  % will run worse case scenario of complete stop at each turn
NUM_SUCCESS = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load_base_parameters;
load_variation_parameters;

if MONTE_CARLO
    calc_random_car;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% INITIAL VALUES
x_threshold = 4;        %threshold to wall before car needs to begin turn (in)
TOTAL_TIME_BASE = 60;   % seconds
dt = 0.25;              % interval of time to update accel, velo, pos (s)
total_time = 10*60;     % total loop time (s)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display('Begin Monte Carlo Optimization Process');
for jj = 1:N_ITERATIONS

    if mod(jj,10) == 0
        fprintf('.');
    end
    if mod(jj,50) == 0
       fprintf(' %d iterations\n',jj)                                       
    end
    
    % randomly vary car via Monte Carlo Iteration Process
    calc_random_car;
    
    dv = 0;                 %initial value for velocity change (in/sec)
    vy0 = 0;                %initial value for vehicle velocity (in/sec)
    v0 = vy0;               %assign initial value to vy (in/sec)
    F_THRUST = 1;           %initial non-zero value for thrust force (lbs)
    ay0 = F_THRUST/car.MASS;    %initial value for acceleration (in/sec^2)
    a = ay0;                %assign initial value to ay (in/sec^2)
    y0 = 24;                %initial race track position (in)
    y = y0;                 %assign initial position to y (in)
    x0 = -24;               %initial x-race track position (in)
    x = x0;                 %assign initial position to x (in)
    RIGHT = 1;  % GOES RIGHT FIRST; if == 1, right; if == 0, left (negative heading and theta angles)
    j = 1;

    for t = 1:total_time/dt

        if RIGHT == 1
            [a(t),v(t),theta(t),phi(t)] = calc_path(v0,dt,car);
        else
            [a(t),v(t),theta(t),phi(t)] = calc_path(v0,dt,car);
            theta(t) = -theta(t);
            phi(t)   = -phi(t);
        end

        d(t)  = (0.5*a(t)*dt^2)+(v(t)*dt);  % change in distance (in)
        x(t)  = (d(t)*sin(phi(t)))+x0;      % x-coordinate (in)
        y(t)  = (d(t)*cos(phi(t)))+y0;      % y-coordinate (in)
        vx(t) = (v(t)*sin(phi(t)));         % x-velocity (in/s)
        vy(t) = (v(t)*cos(phi(t)));         % y-velocity (in/s)
        ax(t) = abs((a(t)*sin(phi(t))));         % x-acceleration (in/s^2)
        ay(t) = abs((a(t)*cos(phi(t))));         % y-acceleration (in/s^2)

        if(y(t)/12 >= DIST)
            TOTAL_TIME_SEC = (t-1)*dt;
            break;
        end

        y0 = y(t);
        x0 = x(t);
        v0 = v(t);

        if ((24-x0 < x_threshold) && j >= 5)
            RIGHT = 0;
            j = 1;
            if WORSE_CASE
               v0 = 0; 
            end
        elseif ((24+x0 < x_threshold) && j >= 5)
            RIGHT = 1;
            j = 1;
            if WORSE_CASE
               v0 = 0; 
            end
        else
            j = j+1;
        end
    end 
    
    if TOTAL_TIME_SEC < TOTAL_TIME_BASE
       NUM_SUCCESS = NUM_SUCCESS + 1;
%       TOTAL_TIME_BASE = TOTAL_TIME_SEC;  -- this command is for genetic
%       monte carlo algorithm
       CAR_SUCCESS(NUM_SUCCESS).car_prop = car;
       CAR_SUCCESS(NUM_SUCCESS).ttl_time = TOTAL_TIME_SEC;
       CAR_SUCCESS(NUM_SUCCESS).time_vec = (0:dt:TOTAL_TIME_SEC)';
       CAR_SUCCESS(NUM_SUCCESS).x     = x(:);            % x-pos
       CAR_SUCCESS(NUM_SUCCESS).y     = y(:);            % y-pos
       CAR_SUCCESS(NUM_SUCCESS).v     = v(:);            % speed
       CAR_SUCCESS(NUM_SUCCESS).vx    = vx(:);           % x-velo
       CAR_SUCCESS(NUM_SUCCESS).vy    = vy(:);           % y-velo
       CAR_SUCCESS(NUM_SUCCESS).a     = a(:);            % acceleration
       CAR_SUCCESS(NUM_SUCCESS).ax    = ax(:);           % x-accel
       CAR_SUCCESS(NUM_SUCCESS).ay    = ay(:);           % y-accel
       CAR_SUCCESS(NUM_SUCCESS).theta = theta(:);        % mast deflection
       CAR_SUCCESS(NUM_SUCCESS).phi   = phi(:);          % heading angle
    end
    
    clear a v theta phi d x y vx vy ax ay;
end
display(['End Monte Carlo Optimization Process with ' num2str(NUM_SUCCESS) ' successes.']);
%%END MONTE CARLO%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% POST MONTE CARLO ANALYSIS
min_idx = find([CAR_SUCCESS(:).ttl_time]==min([CAR_SUCCESS(:).ttl_time]));
FASTEST_CAR = CAR_SUCCESS(min_idx);
display(['The fastest time for best case scenario is ' num2str(FASTEST_CAR.ttl_time) ' seconds.']);

% PLOT VEHICLE
plot_car(FASTEST_CAR.car_prop,DIST_CG);

% PLOT TRAJECTORY
plot_traj(FASTEST_CAR.time_vec, FASTEST_CAR.v, FASTEST_CAR.a,...
          FASTEST_CAR.x,       FASTEST_CAR.y,...
          FASTEST_CAR.vx,      FASTEST_CAR.vy,...
          FASTEST_CAR.ax,      FASTEST_CAR.ay,...
          FASTEST_CAR.theta,   FASTEST_CAR.phi);
          