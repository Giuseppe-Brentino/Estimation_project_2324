function ctrl = parameters_controller ()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% QUADROTOR PARAMETERS - Controller                                       %
% Authors:  Mattia Giurato (mattia.giurato@polimi.it)                     %
%           Paolo Gattazzo (paolo.gattazzo@polimi.it)                     %
% Date: 23/04/2018                                                        %
% Adapted to ANT-X 2DoF by Salvatore Meraglia (salvatore.meraglia@polimi.it) %
% Date: 22/12/2022                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Normalized mixer

ctrl.Kt = 0.25;
ctrl.Kq = 0.25;
ctrl.b = 2;

Kt_b = ctrl.Kt*ctrl.b;
ctrl.mixer = [ -1/(4*ctrl.Kt), -2^(1/2)/(4*Kt_b),  2^(1/2)/(4*Kt_b),  1/(4*ctrl.Kq);
    -1/(4*ctrl.Kt),  2^(1/2)/(4*Kt_b), -2^(1/2)/(4*Kt_b),  1/(4*ctrl.Kq);
    -1/(4*ctrl.Kt),  2^(1/2)/(4*Kt_b),  2^(1/2)/(4*Kt_b), -1/(4*ctrl.Kq);
    -1/(4*ctrl.Kt), -2^(1/2)/(4*Kt_b), -2^(1/2)/(4*Kt_b), -1/(4*ctrl.Kq)];

%% Angular rate controller

%q
ctrl.KF_Q = 0.0;
ctrl.KP_Q = 0.09;
ctrl.KI_Q = 0.21;
ctrl.KD_Q = 0.0016;
ctrl.M_MAX = 1;
ctrl.M_MIN = -1;

ctrl.N_filter_rate = 100;
ctrl.sample_time = 1/250;

%% Attitude controller

%theta
ctrl.KP_PITCH = 12;
ctrl.Q_MAX = 10; % 10 [rad/s] ~ 600 deg/s
ctrl.Q_MIN = -10;

%% Velocity controller

ctrl.KF_X_DOT = 0.0;
ctrl.KP_X_DOT = 0.5;
ctrl.KI_X_DOT = 0.04;
ctrl.KD_X_DOT = 0;

ctrl.F_X_MIN = -1;
ctrl.F_X_MAX = 1;

ctrl.N_filter_vel = 10;

%% Position controller

ctrl.KP_X = 2;

% velocity setpoint saturation
ctrl.VEL_MIN = -10;
ctrl.VEL_MAX = 10;

%% Baseline thrust (normalized)

ctrl.BASELINE_T = 0.4;

end