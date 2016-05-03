% load_variation_parameters.m
%
% Description:
%   This file assigns variation parameters (standard deviation) associated
%   with each adjustable UAV parameter.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Wing --------------------------------------------------------------------

    SD.S_WING = 5;      % WING SPAN (IN)
    SD.tip_chord = 0;   % WING TIP-CHORD (IN) (KEEP 0 IF KEEP TRIANGULAR CONFIG)
    SD.root_chord = 5;  % WING ROOT-CHORD (IN)
    SD.psi_deg = 10;    % MAST LEANING ANGLE (DEG)
    
% Fuselage ----------------------------------------------------------------

    SD.AP_BODY = 3;         % PROJECTED FRONTAL FUSELAGE AREA (IN2)
    SD.TRACK   = 5;         % VEHICLE WIDTH (IN)
    SD.WHEELBASE = 10;      % VEHICLE LENGTH (IN)
    SD.W_BEAM = SD.TRACK;   % WIDTH OF CROSS BEAM AVAILABLE
    SD.T_BEAM = 0;          % THICKNESS OF CROSS BEAM (IN)
    SD.C_BEAM = 0;          % AVERAGE CROSS BEAM CHORD LENGTH (IN)
    
    