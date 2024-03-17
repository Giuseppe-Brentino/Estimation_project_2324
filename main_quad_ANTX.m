%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ANT-X SIMULATOR - MAIN                                                  %
% Authors:  Mattia Giurato (mattia.giurato@polimi.it)                     %
%           Paolo Gattazzo (paolo.gattazzo@polimi.it)                     %
% Date: 13/12/2017                                                        %
% Adapted to ANT-X 2DoF by:  Salvatore Meraglia (salvatore.meraglia@polimi.it)%
% Date: 22/12/2022                                                        %
%
% Further modified to include structure three-state identified longitudinal model
% 06/01/23 ML
% 
% Further modified to pick outputs with measurement error
% 03/01/24 ML
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clearvars;
close all;
clc;

addpath('datasets','common','common/simulator-toolbox','common/simulator-toolbox/attitude_library','common/simulator-toolbox/trajectory_library');

%% Model parameters

% Initial model (state: longitudinal velocity, pitch rate, pitch angle; input: normalised pitching moment; outputs: state and longitudinal acceleration)

Xu=-0.1068;

Xq=0.1192;

Mu=-5.9755;

Mq=-2.6478;

Xd=-10.1647;

Md=450.71;

A=[Xu, Xq, -9.81; Mu, Mq, 0; 0, 1, 0];

B=[Xd; Md; 0];

C=[1, 0, 0; 0, 1, 0; 0, 0, 1; Xu, Xq, 0]; 

D=[0; 0; 0; Xd];

real_parameters = [ Xu; Xq; Mu; Mq; Xd; Md];

% Noise

%noise.Enabler = 0;
noise.Enabler = 1;

noise.pos_stand_dev = noise.Enabler * 0.0011;                            	%[m]

noise.vel_stand_dev = noise.Enabler * 0.01;                               %[m/s]

noise.attitude_stand_dev = noise.Enabler * deg2rad(0.33);                 %[rad]
noise.ang_rate_stand_dev = noise.Enabler * deg2rad(1);                   %[rad/s]

% seed used in the generation of the white noise
seed.x = 1;
seed.vx = 2;
seed.theta = 3;
seed.q= 4;
% Delays

delay.position_filter = 1;
delay.attitude_filter = 1;
delay.mixer = 1;

%% Load controller parameters

ctrl = parameters_controller();                   

%% M injection example (sweeep: first column time vector, secondo column time history of pitching moment) 

load ExcitationM;

ctrl.SetPoint=[0,0];

%% Values selected

t=ExcitationM(:,1);

simulation_time=t(end)-t(1);

%% Simulation

simulation = sim('Simulator_Single_Axis');

time = 0:ctrl.sample_time:simulation_time;

data = struct;
data.ax = simulation.ax.Data;
data.q = simulation.q.Data;
data.Mtot = simulation.Mtot.Data;

%% Task 1


input = data.Mtot;              % Normalized control moment
output = [data.q data.ax];      % Measured acceleration and pitch rate
sim_data = iddata(output, input, ctrl.sample_time); 
data_fd = fft(sim_data); % output of the simulation in the frequency domain

%%% FARSI COMMENTARE LA FUNZIONE DA CHATGPT
odefun = 'drone_model';

% Initial guess for the identification
guess = zeros(6,1);
parameters = {'Xu', guess(1); 'Xq', guess(2); 'Mu',  guess(3); 'Mq',  guess(4);...
    'Xd',  guess(5); 'Md',  guess(6)};
sys_init = idgrey(odefun, parameters, 'c');

% Model Identification
identification = struct;
estimated_model = greyest(data_fd, sys_init);
identification.parameters = estimated_model.Report.Parameters.ParVector;
identification.fit = estimated_model.Report.Fit.FitPercent;
identification.covariance = getcov(estimated_model);

estimation_error = (identification.parameters-real_parameters) ./ real_parameters * 100;

%%%%%%%%%%%%%%%%%%%%% TEMPORANEO - DA AGGIUSTARE %%%%%%%%%%%%%%%%%%%%%%%%%%
% % validation 
% 
% seed.x = 5;
% seed.vx = 6;
% seed.theta = 7;
% seed.q = 8;
% simulation = sim('Simulator_Single_Axis');
% 
% time = 0:ctrl.sample_time:simulation_time;
% 
% data = struct;
% data.ax = simulation.ax.Data;
% data.q = simulation.q.Data;
% data.Mtot = simulation.Mtot.Data;
% 
% input = data.Mtot;              % Normalized control moment
% output = [data.q data.ax];      % Measured acceleration and pitch rate
% 
% real_sim = iddata(output, input, ctrl.sample_time); 

% A = estimated_model.A;
% B = estimated_model.B;
% C = [1 0 0 ; estimated_model.C(1,:) ; 0 0 1 ; estimated_model.C(2,:)];
% D = [0; 0 ; estimated_model.D];
% simulation = sim('Simulator_Single_Axis');
% time = 0:ctrl.sample_time:simulation_time;
% 
% data = struct;
% data.ax = simulation.ax.Data;
% data.q = simulation.q.Data;
% data.Mtot = simulation.Mtot.Data;
% 
% input = data.Mtot;              % Normalized control moment
% output = [data.q data.ax];      % Measured acceleration and pitch rate
% 
% est_sim = iddata(output, input, ctrl.sample_time);
% 
% figure
% compare(real_sim,est_sim)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% plots

% % % % figure 
% % % % plot(sensors_time,data.Mtot,"b")
% % % % grid on
% % % % hold on
% % % % plot(ExcitationM(:,1),ExcitationM(:,2),"r")
% % % % legend('Total Moment','Excitation Moment','Location','best')
% % % % title('Total Moment with excitation')
% % % % grid on
% % % % axis tight
% % % % 
% % % % figure 
% % % % plot(sensors_time,data.ax)
% % % % title('Longitudinal Acceleration')
% % % % grid on
% % % % axis tight
% % % % 
% % % % figure 
% % % % plot(sensors_time,data.q)
% % % % title('Pitch rate')
% % % % grid on
% % % % axis tight


error_plot = figure;
bar(estimation_error);
title('Estimation error','Interpreter','latex')
set(gca,'XTickLabel',parameters(:,1));
ylim([-0.12 0.15])
grid on

%% TASK 2

clc
 estimated_matrix.A = estimated_model.A;
 estimated_matrix.B = estimated_model.B;
 estimated_matrix.C = [1 0 0 ; estimated_model.C(1,:) ; 0 0 1 ; estimated_model.C(2,:)];
 estimated_matrix.D = [0; 0 ; estimated_model.D];
 
 eta0 = [0.1 10 80];
 lb = [0.01; 0.01; 20];
 ub = [125; 125; 90];
 A_constr = [1 -1 0];
 b_constr = 0;
 
 % optimization

options = optimoptions('fmincon','Display', 'iter');
 [x,resnorm,residual,exitflag,output] = fmincon(@obj_function,eta0,A_constr,b_constr,[],[],lb,ub,[],...
     options,estimated_matrix,ctrl,delay,seed,noise,odefun);

 % lb,ub,A_constr,b_constr,[],


%% END OF CODE