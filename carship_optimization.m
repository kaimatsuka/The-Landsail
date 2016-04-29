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

load_parameters;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

% INITIAL VALUES
e = 0.050;              %error sensitivity for thrust equilibrium check (lbs)
x_threshold = 3;        %threshold to wall before car needs to begin turn (in)
dv = 0;                 %initial value for velocity change (in/sec)
vy0 = 0;                %initial value for vehicle velocity (in/sec)
v0 = vy0;               %assign initial value to vy (in/sec)
F_THRUST = 1;           %initial non-zero value for thrust force (lbs)
ay0 = F_THRUST/MASS;    %initial value for acceleration (in/sec^2)
a = ay0;                %assign initial value to ay (in/sec^2)
y0 = 24;                %initial race track position (in)
y = y0;                 %assign initial position to y (in)
x0 = -24;               %initial x-race track position (in)
x = x0;                 %assign initial position to x (in)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt = 0.25;
total_time = 5*60;

fig = figure();
subplot(2,3,[1,4])
    hold on; box on; grid on;
    plot([-2 2],[y0,y0]/12,'g','linewidth',4)
    plot([-2 2],[DIST,DIST],'r','linewidth',4)
    axis([-2 2 0 DIST+1]);
    xlabel('x (ft)','fontsize',10);   ylabel('y (ft)','fontsize',10);
    title('Race Track Position','fontsize',12);
subplot(2,3,2)
    hold on; box on; grid on;
    axis([0 25 -3.5 3.5]);
    xlabel('time (s)','fontsize',10); ylabel('v (ft/s)','fontsize',10);
    title('Velocity vs. Time','fontsize',12);
subplot(2,3,3)
    hold on; box on; grid on;
    axis([0 25 -0.5 0.5]);
    xlabel('time (s)','fontsize',10); ylabel('a (ft/s^2)','fontsize',10);
    title('Acceleration vs. Time','fontsize',12);
subplot(2,3,5)
    hold on; box on; grid on;
    axis([0 25 -90 90]);
    xlabel('time (s)','fontsize',10); ylabel('\theta (deg)','fontsize',10);
    title('Mast Deflection vs. Time','fontsize',12);
subplot(2,3,6)
    hold on; box on; grid on;
    axis([0 25 -90 90]);
    xlabel('time (s)','fontsize',10); ylabel('\phi (deg)','fontsize',10);
    title('Heading Angle vs. Time','fontsize',12);
    
RIGHT = 1;  % GOES RIGHT FIRST; if == 1, right; if == 0, left (negative heading and theta angles)
j = 1;
for t = 1:total_time/dt
    
    if RIGHT == 1
        [a(t),v(t),theta(t),phi(t)] = calc_path(v0,dt,psi);
    else
        [a(t),v(t),theta(t),phi(t)] = calc_path(v0,dt,psi);
        theta(t) = -theta(t);
        phi(t)   = -phi(t);
    end
    
    d(t)  = (0.5*a(t)*dt^2)+(v(t)*dt);
    x(t)  = (d(t)*sin(phi(t)))+x0;
    y(t)  = (d(t)*cos(phi(t)))+y0;
    vx(t) = (v(t)*sin(phi(t)));
    vy(t) = (v(t)*cos(phi(t)));
    ax(t) = (a(t)*sin(phi(t)));
    ay(t) = (a(t)*cos(phi(t)));
    subplot(2,3,[1,4])
        pos_plot(t) = plot(x(t)/12,y(t)/12,'x','markersize',12);
        if(t>1)
            delete(pos_plot(t-1));
            plot(x(1:t-1)/12,y(1:t-1)/12,'-k','linewidth',2);
        end
    subplot(2,3,2)
        velo_plot(t) = plot((t-1)*dt,v(t)/12,'x','markersize',12);
        velox_plot(t) = plot((t-1)*dt,vx(t)/12,'rx','markersize',8);
        veloy_plot(t) = plot((t-1)*dt,vy(t)/12,'gx','markersize',8);
        if(t>1)
            delete(velo_plot(t-1));
            delete(velox_plot(t-1));
            delete(veloy_plot(t-1));
            plot((1:t-1)*dt,v(1:t-1)/12,'-k','linewidth',2);
            plot((1:t-1)*dt,vx(1:t-1)/12,'-r');
            plot((1:t-1)*dt,vy(1:t-1)/12,'-g');
        end
    subplot(2,3,3)
        accel_plot(t) = plot((t-1)*dt,a(t)/12,'x','markersize',12);
        accelx_plot(t) = plot((t-1)*dt,ax(t)/12,'rx','markersize',8);
        accely_plot(t) = plot((t-1)*dt,ay(t)/12,'gx','markersize',8);
        if(t>1)
            delete(accel_plot(t-1));
            delete(accelx_plot(t-1));
            delete(accely_plot(t-1));
            plot((1:t-1)*dt,a(1:t-1)/12,'-k','linewidth',2);
            plot((1:t-1)*dt,ax(1:t-1)/12,'-r');
            plot((1:t-1)*dt,ay(1:t-1)/12,'-g');
        end       
    subplot(2,3,5)
        theta_plot(t) = plot((t-1)*dt,theta(t)*180/pi,'x','markersize',12);
        if(t>1)
            delete(theta_plot(t-1));
            plot((1:t-1)*dt,theta(1:t-1)*180/pi,'-k','linewidth',2);
        end
    subplot(2,3,6)
        phi_plot(t) = plot((t-1)*dt,phi(t)*180/pi,'x','markersize',12);
        if(t>1)
            delete(phi_plot(t-1));
            plot((1:t-1)*dt,phi(1:t-1)*180/pi,'-k','linewidth',2);
        end
        
    if(y(t)/12 >= DIST)
        TOTAL_TIME_SEC = (t-1)*dt
        break;
    end
    
    y0 = y(t);
    x0 = x(t);
    v0 = v(t);
    drawnow;
    
    if ((24-x0 < x_threshold) && j >= 5)
        RIGHT = 0;
        j = 1;
    elseif ((24+x0 < x_threshold) && j >= 5)
        RIGHT = 1;
        j = 1;
    else
        j = j+1;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%END MAIN PROGRAM