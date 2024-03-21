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

rng default;

addpath('datasets','common','common/simulator-toolbox','common/simulator-toolbox/attitude_library','common/simulator-toolbox/trajectory_library');
addpath('functions');
%% Model parameters

% Initial model (state: longitudinal velocity, pitch rate, pitch angle; input: normalised pitching moment; outputs: state and longitudinal acceleration)

Xu=-0.1068;

Xq=0.1192;

Mu=-5.9755;

Mq=-2.6478;

Xd=-10.1647;

Md=450.71;

g = 9.81;

A=[Xu, Xq, -g; Mu, Mq, 0; 0, 1, 0];

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
set_param('Simulator_Single_Axis',"FastRestart","off");
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

%Nell amontecarlo definire un criterio di uscita dal ciclo (togliere
%possibilità di scelta del numero di modelli costruiti-> esempio usare
%criterio di tellerenza

N_scenarios = 2;  % number of scenarios
N_ic = 2;   % number of initial guesses for each optimization problem
N_sim = N_scenarios*N_ic;

%generate uncertain parameters
stoch_params_temp = mvnrnd(identification.parameters,identification.covariance,N_scenarios);
stoch_params = repelem(stoch_params_temp,N_ic,1);

% initialize model matrices
stoch_A = zeros(3,3,N_sim);
stoch_B = zeros(3,1,N_sim);
stoch_C = zeros(4,3,N_sim);
stoch_D = zeros(4,1,N_sim);

% initialize results matrices
full_eta =zeros(3,N_sim);
full_cost = zeros(N_sim,2);
full_cost(:,1) = repelem(1:N_scenarios,N_ic)';

% constraints
lb = [0.01; 0.01; 20];
ub = [10; 10; 90];
A_constr = [1 -1 0];
b_constr = 0;


% initial input sequence guess
%%%%%%%%%% DA DISCUTERE SE HA SENSO USARE SEMPRE LE STESS I.C. %%%%%%%%%%%%
eta0_temp = lb + (ub-lb) .* rand(3,N_ic);
eta0_mat = repmat(eta0_temp,1,N_scenarios);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% solver options
opts = optimoptions(@fmincon,'Algorithm','sqp','Display','none');
tic

set_param('Simulator_Single_Axis',"FastRestart","on");
st = struct;

parfor i = 1:N_sim

    estimated_matrix = st;
    % build state-space matrices
    stoch_A(:,:,i) = [stoch_params(i,1) stoch_params(i,2) -g;
        stoch_params(i,3) stoch_params(i,4)  0;
        0                 1         0];

    stoch_B(:,:,i) = [stoch_params(i,5); stoch_params(i,6);  0;];

    stoch_C(:,:,i) = [       1                  0          0;
        0                  1          0;
        0                  0          1;
        stoch_params(i,1) stoch_params(i,2) 0];

    stoch_D(:,:,i) = [0; 0 ; 0; stoch_params(i,5)];

    estimated_matrix.A = stoch_A(:,:,i);
    estimated_matrix.B = stoch_B(:,:,i);
    estimated_matrix.C = stoch_C(:,:,i);
    estimated_matrix.D = stoch_D(:,:,i);

    % display iteration
    disp("Simulation " + i + " out of " + N_sim );
    % optimize input sequence
    eta0 = eta0_mat(:,i);
    [full_eta(:,i),full_cost(i,2),exitflag,~] = fmincon(@obj_function,eta0,A_constr,b_constr,[],[],lb,ub,[],...
        opts,estimated_matrix,ctrl,delay,seed,noise,odefun);

    % save input-output data
end

%% Cost function analysis

% Compute optimal cost and input sequence per scenario
cost = zeros(N_scenarios,1);
eta = zeros(3,N_scenarios);

counter = 0;
for i = 1:N_ic:length(A(:,2))

    counter = counter+1;

    cost(counter) = min( full_cost(i:i+N_ic-1,2) );

    index = find( full_cost(i:i+N_ic-1,2) == cost(counter) ) + (i-1);

    eta(:,counter) = full_eta( :, index );

end

%% Aggregate results

% store scenarios
stoch.A = stoch_A;
stoch.B = stoch_B;
stoch.C = stoch_C;
stoch.D = stoch_D;
stoch.params = stoch_params;


save RESULTS eta J stoch

input= cell(1,N_sim);
output =cell(1,N_sim);
time_sisw=cell(1,N_sim);
for i=1:N_sim

    estimated_matrixstoch.A = stoch.A(:,:,i);
    estimated_matrixstoch.B = stoch.B(:,:,i);
    estimated_matrixstoch.C = stoch.C(:,:,i);
    estimated_matrixstoch.D = stoch.D(:,:,i);
    [input_sisw, sisw_output] = aggregate_results(eta(:,i),estimated_matrixstoch,ctrl,delay,seed,noise,odefun);

    input{i}=input_sisw;
    output{i}=sisw_output;
    time_sisw{i}=linspace(0,eta(3,i),length(input_sisw));
    %  figure
    % plot(time_sisw{i},input{i})
    % title(sprintf('Input %dth',i))
    % axis tight
    % figure
    % plot(time_sisw{i},output{i})
    %  title(sprintf('Output %dth',i))
    % axis tight
    Dev_standard_ax=std(sisw_output(:,1));
    Dev_standard_q=std(sisw_output(:,2));
end

%Histogram Plot
figure
histogram(J)
figure
histogram(stoch.params(:,1))
title('Xu')
figure
histogram(stoch.params(:,2))
title('Xq')
figure
histogram(stoch.params(:,3))
title('Mu')
figure
histogram(stoch.params(:,4))
title('Mq')
figure
histogram(stoch.params(:,5))
title('Xd')
figure
histogram(stoch.params(:,6))
title('Md')









%% END OF CODE