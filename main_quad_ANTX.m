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

model.A=A;
model.B=B;
model.C=C;
model.D=D;
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

%% Task 1
load_system('Simulator_Single_Axis');
set_param('Simulator_Single_Axis',"FastRestart","off");
time = 0:ctrl.sample_time:simulation_time;

%%% TODO FARSI COMMENTARE LA FUNZIONE DA CHATGPT
odefun= 'drone_model';
[identification, estimation_error, measures] = Model_identification(ExcitationM,model,ctrl,delay,seed,noise,odefun,simulation_time,real_parameters);

%% plots

figure
plot(time,measures(:,1),"b")
grid on
hold on
plot(ExcitationM(:,1),ExcitationM(:,2),"r")
legend('Total Moment','Excitation Moment','Location','best')
title('Total Moment with excitation')
grid on
axis tight

figure
plot(time,measures(:,3))
title('Longitudinal Acceleration')
grid on
axis tight

figure
plot(time,measures(:,2))
title('Pitch rate')
grid on
axis tight


error_plot = figure;
bar(estimation_error);
title('Estimation error','Interpreter','latex')
set(gca,'XTickLabel',{'Xu','Xq','Mu','Mq','Xd','Md'});
ylim([-0.2 0.17])
grid on


% TODO: COMPARE
 est_sim = ss(identification.matrix{1}, identification.matrix{2}, identification.matrix{3}, identification.matrix{4});
% real_sys = iddata( measures(:,2:3),measures(:,1), ctrl.sample_time);
% % y = lsim(est_sim, measures(:,1),time);
% 
% figure
% compare(real_sim, identification.estimated_model)

% Z-P PLOT
est_sys = ss(identification.matrix{1}, identification.matrix{2}, identification.matrix{3}, identification.matrix{4});
real_sys = ss(A,B,C,D);

figure
pz_opt = pzoptions;
pz_opt.Title.String = '';
pz_opt.XLabel.FontSize = 0.1;
pz_opt.YLabel.FontSize = 0.1;

pzplot(est_sys, real_sys)
grid on

axes('position',[0.61 0.61 0.25 0.15])
box on % put box around new pair of axes
hold on
grid on
pzplot(est_sys, real_sys,pz_opt)
ylim([-0.01 0.01])
xlim([3.08 3.09])
set(gca,'color','w');

axes('position',[0.25 0.38 .2 .3])
box on % put box around new pair of axes
hold on
grid on
pzplot(est_sys, real_sys,pz_opt)
ylim([-3.3 3.3])
xlim([-2.93 -2.91])
xlabel('')
ylabel('')
set(gca,'color','w');


%% TASK 2

N_scenarios = 50;  % number of scenarios
N_ic = 5;   % number of initial guesses for each optimization problem
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
ub = [10; 50; 90];
A_constr = [1 -1 0];
b_constr = 0;


% initial input sequence guess
%%%%%%%%%% DA DISCUTERE SE HA SENSO USARE SEMPRE LE STESS I.C. %%%%%%%%%%%%
eta0_mat = lb + (ub-lb) .* rand(3,N_sim);
% eta0_mat = repmat(eta0_temp,1,N_scenarios);
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


end
toc


%% Cost function analysis

% Initialize variables
cost = zeros(N_scenarios,1);
eta_matrix = zeros(3,N_scenarios);

mean_cost=zeros(N_scenarios,1);
std_cost=zeros(N_scenarios,1);

scenario.A = zeros(3,3,N_scenarios);
scenario.B = zeros(3,1,N_scenarios);
scenario.C = zeros(4,3,N_scenarios);
scenario.D = zeros(4,1,N_scenarios);

% post-process of the optimization
counter = 0;
for i = 1:N_ic:length(full_cost(:,2))

    % compute optimal cost and input for each scenario

    counter = counter+1;

    cost(counter) = min( full_cost(i:i+N_ic-1,2) );

    index = find( full_cost(i:i+N_ic-1,2) == cost(counter) ) + (i-1);

    eta_matrix(:,counter) = full_eta( :, index );

     %Compute statistical parameters
     mean_cost(counter)=mean(cost(1:counter));
     std_cost(counter)=std(cost(1:counter));
  
    % save s-s matrices for each scenario
    scenario.A(:,:,counter) = stoch_A(:,:,index);
    scenario.B(:,:,counter) = stoch_B(:,:,index);
    scenario.C(:,:,counter) = stoch_C(:,:,index);
    scenario.D(:,:,counter) = stoch_D(:,:,index);
end



%% Compare effectivness of eta with each scenario

%%%% TO DO: AGGIUNGERE EXTRA INPUT OPZIONALE A OBJ_FUNCTION PER INDICE PER
%%%% NON CALCOLARSI SEMPRE ESTIMATED MODEL
cost_matrix = zeros(N_scenarios);
theta_opt_matrix = zeros(6,N_scenarios);


identified_model = cell(N_scenarios,1);

parfor i = 1:N_scenarios       % iterate on eta
    sim_matrix = st;
    for j = 1:N_scenarios   % iterate on scenario
        sim_matrix.A =  scenario.A(:,:,j);
        sim_matrix.B =  scenario.B(:,:,j);
        sim_matrix.C =  scenario.C(:,:,j);
        sim_matrix.D =  scenario.D(:,:,j);
        if i == j
            [cost_matrix(i,j),identified_model{i}] = obj_function(eta_matrix(:,i),sim_matrix,ctrl,delay,seed,noise,odefun);
            theta_opt_matrix(:,i) = identified_model{i}.parameters;
            
        else
            [cost_matrix(i,j),~] = obj_function(eta_matrix(:,i),sim_matrix,ctrl,delay,seed,noise,odefun);
        end
    end
end

% compute eta that guarantees best average performance
eta_avg_vect = mean(cost_matrix,2);
[~,index] = min(eta_avg_vect);
eta_avg = eta_matrix(:,index);

% compute eta that guarantees best worst-case performance
eta_wc_vect = max(cost_matrix,[],2);
[~,index] = min(eta_wc_vect);
eta_wc = eta_matrix(:,index);


%% Analize optimal results

eta=[eta_wc eta_avg];
identification_opt=cell(size(eta,2),1);
for i=1:size(eta,2)
    [~, identification_opt{i}] = obj_function(eta(:,i),model,ctrl,delay,seed,noise,odefun);
end

%% plot task 2

%TODO: PLOT FIT!


%PLOT DVS
nbins=15;

figure
plot(mean_cost)
grid on
xlabel('Number of iterations') 
ylabel('Mean Cost')

figure
plot(std_cost)
grid on
xlabel('Number of iterations') 
ylabel('Std Cost')

figure
histogram(eta_matrix(1,:),nbins)
grid on
xlabel('f_1')

figure
histogram(eta_matrix(2,:),nbins)
grid on
xlabel('f_2')

figure
histogram(eta_matrix(3,:),nbins)
grid on
xlabel('T')

%Plot Input ETA_AVG

f1_avg = eta_avg(1);
f2_avg = eta_avg(2);
T_avg = eta_avg(3);

wait_time = 5;
simulation_time = 2*wait_time + T_avg;
% input
wait_vector = zeros(round(wait_time/ctrl.sample_time),1);
excitation_time = (0:ctrl.sample_time:T_avg)';
freq_fun = 2*pi*excitation_time.*(f1_avg + (f2_avg-f1_avg)/T_avg.*excitation_time);
excitation_value = [wait_vector; 0.1*sin(freq_fun); wait_vector] ;
excitation_time_AVG = (0:ctrl.sample_time:simulation_time)';
ExcitationM_AVG=[excitation_time_AVG, excitation_value];         % optimized input sequence
figure
plot(ExcitationM_AVG(:,1),ExcitationM_AVG(:,2))

%Plot Input ETA_WC

f1_wc = eta_wc(1);
f2_wc = eta_wc(2);
T_wc = eta_wc(3);

wait_time = 5;
simulation_time = 2*wait_time + T_wc;
% input
wait_vector = zeros(round(wait_time/ctrl.sample_time),1);
excitation_time = (0:ctrl.sample_time:T_wc)';
freq_fun = 2*pi*excitation_time.*(f1_wc + (f2_wc-f1_wc)/T_wc.*excitation_time);
excitation_value = [wait_vector; 0.1*sin(freq_fun); wait_vector] ;
excitation_time_WC = (0:ctrl.sample_time:simulation_time)';
ExcitationM_WC=[excitation_time_WC, excitation_value];         % optimized input sequence
figure
plot(ExcitationM_WC(:,1),ExcitationM_WC(:,2))


%surface plot scenario-eta-cost
figure
b=bar3(cost_matrix);
colorbar
for k = 1:length(b)
    zdata = b(k).ZData;     
    b(k).CData = zdata;     
end
ylabel('Input sequence index')
xlabel('Scenario index')
zlabel('Cost')



%save('ALL_DATA_50-5')






%% END OF CODE