% plot_car.m
%
% Description:
%   This function plots the car's side and top view
%
% Inputs:
%   car : structure containing all car properties
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ ] = plot_car( car , DIST_CG)

    % plot top down view
    figure()
    h(1) = plot([0 car.WHEELBASE], [0 0],'b','LineWidth',5); % plot wheel base
    hold on, grid on,
    h(2) = plot([car.WHEELBASE car.WHEELBASE], [-car.TRACK/2 car.TRACK/2],'b','LineWidth',5);
    h(3) = plot([car.WHEELBASE-DIST_CG],0,'rx','MarkerSize',10,'LineWidth',3);
    xlabel('X (in)'), ylabel('Y (in)')
    title('Top Down View of Car and CG location')
    legend([h(1), h(3)],'Base','CG','location','best')
    axis('equal')
    clear h;

    % plot side view
    figure()
    h(1) = plot([0 car.WHEELBASE], [car.DIAM_WHEEL_FT car.DIAM_WHEEL_RE/2],'b','LineWidth',5); % plot wheel base (approximate length)
    hold on, grid on,
    h(2) = circle(0,car.DIAM_WHEEL_FT/2,car.DIAM_WHEEL_FT/2); % draw front wheel
    h(3) = circle(car.WHEELBASE,car.DIAM_WHEEL_RE/2,car.DIAM_WHEEL_RE/2); % draw rear wheel
    xlabel('X (in)'), ylabel('Z (in)')
    title('Side View of Car')
    axis('equal')
    clear h;

    % plot sail
    figure()
    h(1) = plot([0 car.root_chord], [0 0], 'b','linewidth',5);  % plot root chord
    hold on, grid on,
    h(2) = plot([car.root_chord-car.tip_chord car.root_chord],[car.S_WING car.S_WING],'b','linewidth',5);
    h(3) = plot([car.root_chord car.root_chord],[0 car.S_WING],'b','linewidth',5);
    h(4) = plot([0 car.root_chord-car.tip_chord],[0 car.S_WING],'b','linewidth',5);
    h(5) = plot([0 car.root_chord],[car.HCE-car.Sail_Ground_Clearance car.HCE-car.Sail_Ground_Clearance],'g','linewidth',2);
    xlabel('X (in)'), ylabel('Z (in)')
    legend([h(1) h(5)],'Sail Shape','Center of Effort','location','northwest');
    title('Sail Geometry')
    axis([-1 car.root_chord+1 -1 car.S_WING+1]);
    clear h;
end

