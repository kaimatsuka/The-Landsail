%plot_traj.m
%   
% Description:
%   This function plots the trajectory of the car ship, as well as relative
%   components including velocity, acceleration, heading angle, mast
%   deflection angle
% 
% Input:
%   time    : time vector (s)
%   v       : speed (in/s)
%   a       : acceleration magnitude (in/s2)
%   x       : x-position (in)
%   y       : y-position (in)
%   vx      : x-velocity (in/s)
%   vy      : y-velocity (in/s)
%   ax      : x-acceleration (in/s^2)
%   ay      : y-acceleration (in/s^2)
%   theta   : mast deflection (rad)
%   phi     : heading angle (rad)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ ] = plot_traj( time,v,a,x,y,vx,vy,ax,ay,theta,phi )

fig = figure();
subplot(2,3,[1,4])
    hold on; box on; grid on;
    plot([-2 2],[24,24]/12,'g','linewidth',4)
    plot([-2 2],[14,14],'r','linewidth',4)
    axis([-2 2 0 14+1]);
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

for t = 1:length(time)

    subplot(2,3,[1,4])
        pos_plot(t) = plot(x(t)/12,y(t)/12,'x','markersize',12);
        if(t>1)
            delete(pos_plot(t-1));
            plot(x(1:t-1)/12,y(1:t-1)/12,'-k','linewidth',2);
        end
    subplot(2,3,2)
        velo_plot(t) = plot(time(t),v(t)/12,'x','markersize',12);
        velox_plot(t) = plot(time(t),vx(t)/12,'rx','markersize',8);
        veloy_plot(t) = plot(time(t),vy(t)/12,'gx','markersize',8);
        if(t>1)
            delete(velo_plot(t-1));
            delete(velox_plot(t-1));
            delete(veloy_plot(t-1));
            plot(time(1:t-1),v(1:t-1)/12,'-k','linewidth',2);
            plot(time(1:t-1),vx(1:t-1)/12,'-r');
            plot(time(1:t-1),vy(1:t-1)/12,'-g');
        end
    subplot(2,3,3)
        accel_plot(t) = plot(time(t),a(t)/12,'x','markersize',12);
        accelx_plot(t) = plot(time(t),ax(t)/12,'rx','markersize',8);
        accely_plot(t) = plot(time(t),ay(t)/12,'gx','markersize',8);
        if(t>1)
            delete(accel_plot(t-1));
            delete(accelx_plot(t-1));
            delete(accely_plot(t-1));
            plot(time(1:t-1),a(1:t-1)/12,'-k','linewidth',2);
            plot(time(1:t-1),ax(1:t-1)/12,'-r');
            plot(time(1:t-1),ay(1:t-1)/12,'-g');
        end       
    subplot(2,3,5)
        theta_plot(t) = plot(time(t),theta(t)*180/pi,'x','markersize',12);
        if(t>1)
            delete(theta_plot(t-1));
            plot(time(1:t-1),theta(1:t-1)*180/pi,'-k','linewidth',2);
        end
    subplot(2,3,6)
        phi_plot(t) = plot(time(t),phi(t)*180/pi,'x','markersize',12);
        if(t>1)
            delete(phi_plot(t-1));
            plot(time(1:t-1),phi(1:t-1)*180/pi,'-k','linewidth',2);
        end
        
    drawnow;
end        

end

