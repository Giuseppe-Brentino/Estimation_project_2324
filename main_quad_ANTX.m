%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clearvars;
close all;
clc;

rng default;

addpath('datasets','common','common/simulator-toolbox','common/simulator-toolbox/attitude_library','common/simulator-toolbox/trajectory_library');
addpath('functions');
addpath('FILE_SAVE');
%FOR TASK 1: load(Task_1.mat)
%FOR TASK 2: load(MergeData.mat)

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

odefun= 'drone_model';
[identification, estimation_error, measures, simulation] = Model_identification...
    (ExcitationM,model,ctrl,delay,seed,noise,odefun,simulation_time,real_parameters);

normalized_cov = abs(100*diag(identification.covariance)./identification.parameters);

%cost function
J1 = sum(normalized_cov);
%% plots

% input_seq_plot = figure;
% plot(time,measures(:,1),"b")
% grid on
% hold on
% plot(ExcitationM(:,1),ExcitationM(:,2),"r")
% legend('Total Moment','Excitation Moment','Location','best')
% title('Total Moment with excitation')
% grid on
% xlabel('Time [s]')
% ylabel('Normalized moment [-]')
% axis tight
% exportStandardizedFigure(gcf,'input_plot',0.67, 'addMarkers', false, ...
%        'WHratio', 1.8)

long_acc = figure;
hold on
plot(time,measures(:,3))
plot(time,simulation.ax,'--')
title('Longitudinal Acceleration')
grid on
axis tight
xlabel('Time [s]')
ylabel('Acceleration [$m/s^2$]')
legend('Real model','Estimated model')
title(sprintf('Longitudinal Acceleration. FIT: %.2f %%',identification.fit(2)));
%exportStandardizedFigure(gcf,'long_acc',0.67, 'addMarkers', false)

pitch_rate_plot = figure;
hold on
plot(time,measures(:,2))
plot(time,simulation.q,'--')
title(sprintf('Pitch rate. FIT: %.2f %%',identification.fit(1)));
grid on
axis tight
xlabel('Time [s]')
ylabel('Pitch rate [rad/s]')
legend('Real model','Estimated model')
%exportStandardizedFigure(gcf,'pitchrate',0.67, 'addMarkers', false)

error_plot = figure;
bar(estimation_error);
title('Estimation error','Interpreter','latex')
set(gca,'XTickLabel',{'Xu','Xq','Mu','Mq','Xd','Md'});
ylabel('error [%]')
ylim([-0.12 0.15])
grid on
%exportStandardizedFigure(gcf,'error_plot',0.67, 'addMarkers', false, ...
        'WHratio', 1.3)

% COMPARE
 wmin = -3;  wmax =3;  Npoints = 1000;   
 w = logspace(wmin,wmax,Npoints);
 [mag,~] = bode(identification.estimated_model,w);  

    mag_q = squeeze(20*log10(mag(1,1,:))); 
    mag_acc = squeeze(20*log10(mag(2,1,:)));

figure;    
subplot(2,1,1)
loglog(w,mag_acc)
hold on
xline(1e-2*2*pi)
xline(50*2*pi)
xlabel('Frequency [rad/s]')
ylabel('Acceleration [dB]')
xlim([w(1) w(end)])
ylim([15 30])

subplot(2,1,2)
semilogx(w,mag_q,'DisplayName','Magnitude')
grid on
hold on
xline(1e-2*2*pi,'DisplayName','Lower bound')
xline(50*2*pi,'DisplayName','Upper bound')
xlim([w(1) w(end)])
xlabel('Frequency [rad/s]')
ylabel('Pitch rate [dB]')
legend;

%exportStandardizedFigure(gcf,'bode',0.67, 'addMarkers', false,'WHRatio',0.7)

% Z-P PLOT
est_sys = ss(identification.matrix{1}, identification.matrix{2}, identification.matrix{3}, identification.matrix{4});
real_sys = ss(A,B,C,D);

pz_plot = figure;
pz_opt = pzoptions;
pz_opt.Title.String = '';
pz_opt.XLabel.FontSize = 0.1;
pz_opt.YLabel.FontSize = 0.1;

pzplot(est_sys, real_sys)
legend('Estimated System', 'Real System')
a = findobj(gca,'type','line');
for i = 1:length(a)
    set(a(i),'markersize',10); %change marker size
    set(a(i), 'linewidth',2);  %change linewidth
end
grid on

axes('position',[0.61 0.7 0.25 0.15])
box on % put box around new pair of axes
hold on
grid on
pzplot(est_sys, real_sys,pz_opt)
ylim([-0.01 0.01])
xlim([3.08 3.09])
a = findobj(gca,'type','line');
for i = 1:length(a)
    set(a(i),'markersize',10); %change marker size
    set(a(i), 'linewidth',2);  %change linewidth
end
set(gca,'color','w');

axes('position',[0.61 0.3 0.25 0.25])
box on % put box around new pair of axes
hold on
grid on
pzplot(est_sys, real_sys,pz_opt)
ylim([-3.6 3.6])
xlim([-2.93 -2.91])
xlabel('')
ylabel('')
a = findobj(gca,'type','line');
for i = 1:length(a)
    set(a(i),'markersize',10); %change marker size
    set(a(i), 'linewidth',2);  %change linewidth
end
set(gca,'color','w');
%exportStandardizedFigure(gcf,'pz_plot',0.67, 'addMarkers', false, ...
     'WHratio', 1.3)

%% TASK 2

N_scenarios = 70;  % number of scenarios
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
full_eta = zeros(3,N_sim);
full_cost = zeros(N_sim,2);
full_cost(:,1) = repelem(1:N_scenarios,N_ic)'; %mette in ordine sparso gli elementi 

% constraints
lb = [0.01; 0.01; 20]; 
ub = [10; 50; 90];
A_constr = [1 -1 0];
b_constr = 0;


% initial input sequence guess
eta0_mat = lb + (ub-lb) .* rand(3,N_sim); 

% solver options
% opts = optimoptions(@ga,'Display','none','PlotFcn','gaplotbestf');
tic
opts = optimoptions(@fmincon,'Algorithm', 'active-set','MaxIter',3000,'Display','none');

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

    % optimization
    [full_eta(:,i),full_cost(i,2)] = fmincon(@obj_function,eta0,A_constr,b_constr,[],[],lb,ub,[],...
        opts,estimated_matrix,ctrl,delay,seed,noise,odefun);


end

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
tic
for i = 1:N_ic:length(full_cost(:,2))

    % compute optimal cost and input for each scenario

    counter = counter+1;

    cost(counter) = min( full_cost(i:i+N_ic-1,2) ); %fra i 5 input di uno scenario prendo quello che mi da min(J)

    
    index= find( full_cost(i:i+N_ic-1,2) == cost(counter),1,'first') + (i-1);

    eta_matrix(:,counter) = full_eta( :, index ); %eta_matrix=(input [f1 f2 T], Scenario)
    %Compute statistical parameters
     mean_cost(counter)=mean(cost(1:counter));
     std_cost(counter)=std(cost(1:counter));
  
    % save s-s matrices for each scenario
    scenario.A(:,:,counter) = stoch_A(:,:,index);
    scenario.B(:,:,counter) = stoch_B(:,:,index);
    scenario.C(:,:,counter) = stoch_C(:,:,index);
    scenario.D(:,:,counter) = stoch_D(:,:,index);
   
end

toc


%% Compare effectivness of eta with each scenario

cost_matrix = zeros(N_scenarios);
theta_opt_matrix = zeros(6,N_scenarios);


identified_model = cell(N_scenarios,1);
tic
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
toc
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
J_opt=zeros(2,1);
error_opt = zeros(6,2);
for i=1:size(eta,2)
    [~, identification_opt{i}] = obj_function(eta(:,i),model,ctrl,delay,seed,noise,odefun);

    % estimation error
   error_opt(:,i) = (identification_opt{i}.parameters-real_parameters) ./ real_parameters * 100;

%cost function
normalized_cov = abs(100*diag(identification_opt{i}.covariance)./identification_opt{i}.parameters);
J_opt(i) = sum(normalized_cov);

end

%% Compare the inputs

J_old_input = zeros(N_scenarios,1);

for i=1:N_scenarios

    sim_matrix.A =  scenario.A(:,:,j);
    sim_matrix.B =  scenario.B(:,:,j);
    sim_matrix.C =  scenario.C(:,:,j);
    sim_matrix.D =  scenario.D(:,:,j);

    [identified_model_old_input] = Model_identification(ExcitationM,sim_matrix,...
        ctrl,delay,seed,noise,odefun,simulation_time,real_parameters);

    normalized_cov = abs(100*diag(identified_model_old_input.covariance)./...
        identified_model_old_input.parameters);
    %cost function
    J_old_input(i) = sum(normalized_cov);

end

 J_old_input_mean=mean(J_scenario);
 J_old_input_min=min(J_scenario);
 J_old_input_max=max(J_scenario);
 J_old_input_opt_mean=mean(cost_matrix(index,:));
 J_old_input_opt_max=max(cost_matrix(index,:));
 J_old_input_opt_min=min(cost_matrix(index,:));


%% plot task 2

% mean
filtered_mean = movmean(mean_cost,10);
filtered_mean(1:10) =movmean(mean_cost(1:10),3);
figure();
hold on
grid on
plot( mean_cost);
plot( filtered_mean );
legend('Mean cost','Trend');
ylabel('Mean');
xlabel('Iterations')
title('Mean cost evolution')
%exportStandardizedFigure(gcf,'mean_cost_plot',0.67, 'addMarkers', false)

% std
filtered_std = movmean(std_cost,10);
filtered_std(1:10) =movmean(std_cost(1:10),3);
figure();
hold on
grid on
plot( std_cost);
plot( filtered_std );
legend('Cost standard deviation','Trend');
ylabel('Standard Deviation');
xlabel('Iterations')
title('Standard deviation evolution')
%exportStandardizedFigure(gcf,'std_cost_plot',0.67, 'addMarkers', false)


% Comparison of estimation error
error_plot = figure;
tiledlayout(6,1);

hold on
grid minor
bar(nexttile,abs([estimation_error,error_opt(:,2)]));
title('Estimation error','Interpreter','latex')
set(gca,'XTickLabel',{'Xu','Xq','Mu','Mq','Xd','Md'});
ylim([0 0.15])
legend('Initial Input','Input $\eta_{opt}$')
%exportStandardizedFigure(gcf,'error_opt_hist_plot',0.67, 'addMarkers', false)

%Cost distribution
figure
histogram(cost,nbins)
xlabel('Cost')
ylabel('Number of occurrencies')
%exportStandardizedFigure(gcf,'Cost_dist_plot',0.67, 'addMarkers', false)

% Input data distribution
figure
histogram(eta_matrix(1,:),nbins)
grid on
xlabel('$f_1$ [Hz]')
ylabel('Number of occurencies')
%exportStandardizedFigure(gcf,'f1_dist_plot',0.67, 'addMarkers', false)

figure
histogram(eta_matrix(2,:),nbins)
grid on
xlabel('$f_2$ [Hz]')
ylabel('Number of occurencies')
%exportStandardizedFigure(gcf,'f2_dist_plot',0.67, 'addMarkers', false)

figure
histogram(eta_matrix(3,:),nbins)
grid on
xlabel('T [s]')
ylabel('Number of occurencies')
%exportStandardizedFigure(gcf,'T_dist_plot',0.67, 'addMarkers', false)

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

excitation_opt = figure();
set(gcf, 'Position', get(0, 'Screensize'));
hold on 
grid on
plot(ExcitationM_WC(:,1),ExcitationM_WC(:,2))
xlabel('Time [s]')
ylabel('Normalized moment [-]')
title('Optimal excitation moment')
%exportStandardizedFigure(gcf,'excitation_opt_plot',1, 'addMarkers', false, 'WHratio',3)

%bar plot scenario-eta-cost
figure
b=bar3(cost_matrix);
c = colorbar;
c.Label.String = 'Cost';
for k = 1:length(b)
    zdata = b(k).ZData;     
    b(k).CData = zdata;     
end
ylabel('Input sequence index')
xlabel('Scenario index')
zlabel('Cost')
set(gca,'CLim',[0 0.6]);
view(62,22)
ylim([0 100])
%exportStandardizedFigure(gcf,'J3D',1, 'addMarkers', false, 'WHratio',1)

figure
b=bar3(cost_matrix);
for k = 1:length(b)
    zdata = b(k).ZData;     
    b(k).CData = zdata;     
end
ylabel('Input sequence index')
xlabel('Scenario index')
zlabel('Cost')
ylim([0 100])
view(-90,90)
set(gca,'CLim',[0 0.6]);
%exportStandardizedFigure(gcf,'J3D_top_view',1, 'addMarkers', false, 'WHratio',1)

%% FIT PLOT
addpath('FILE_SAVE');
load("out_simulink.mat") %TRUE MODEL
load("simulation_estimate_opt.mat") %ESTIMATED MODEL
% ExcitationM=ExcitationM_WC;
% t=ExcitationM(:,1);
% 
% simulation_time=t(end)-t(1); 
 %out= sim('Simulator_Single_Axis','SrcWorkspace', 'current'); %it use A, B, C, D of True Model 
 % A=identification_opt{1, 1}.matrix{1};
 % B=identification_opt{1, 1}.matrix{2};
 % C = [1 0 0 ; identification_opt{1, 1}.matrix{3}(1,:) ; 0 0 1 ; identification_opt{1, 1}.matrix{3}(2,:)];
 % D = [0; 0 ; identification_opt{1, 1}.matrix{4}];
 % simulation_estimate_opt = sim('Simulator_Single_Axis','SrcWorkspace', 'current'); 

figure
plot(simulation_estimate_opt.ax)
hold on
plot(out.ax,'--')
legend('Estimated model','Real model')
title(sprintf('Longitudinal Acceleration. FIT: %.2f %%',identification_opt{1, 1}.fit(2)))
grid on
xlabel('Time [s]')
ylabel('Acceleration [$m/s^2$]')
%exportStandardizedFigure(gcf,'long_acc',0.67, 'addMarkers', false);

figure
plot(simulation_estimate_opt.q)
hold on
plot(out.q,'--')
legend('Estimated model','Real model')
title(sprintf('Pitch rate. FIT: %.2f %%',identification_opt{1, 1}.fit(1)))
grid on
xlabel('Time [s]')
ylabel('Pitch rate [rad/s]')
%exportStandardizedFigure(gcf,'pitch_rate',0.67, 'addMarkers', false);
%% Variance Plot

figure
subplot(2,3,1)
bar([(identification.covariance(1,1)),(identification_opt{2, 1}.covariance(1,1))],'FaceAlpha',.85,'FaceColor',[0.56,0.61,0.80],'EdgeColor',[0.30,0.33,0.44]);
title('Variance','Interpreter','latex')
set(gca,'XTickLabel',{'Xu_{Task1}','Xu_{opt}'});
set(gca,'XTickLabel',{'Xu_{Task1}','Xu_{opt}'},'fontname','Palatino Linotype');

grid on

subplot(2,3,2)
bar([(identification.covariance(2,2)),(identification_opt{2, 1}.covariance(2,2))],'FaceAlpha',.85,'FaceColor',[0.56,0.61,0.80],'EdgeColor',[0.30,0.33,0.44]);
title('Variance','Interpreter','latex')
set(gca,'XTickLabel',{'Xq_{Task1}','Xq_{opt}'});
set(gca,'XTickLabel',{'Xq_{Task1}','Xq_{opt}'},'fontname','Palatino Linotype');

grid on

subplot(2,3,3)
bar([(identification.covariance(3,3)),(identification_opt{2, 1}.covariance(3,3))],'FaceAlpha',.85,'FaceColor',[0.56,0.61,0.80],'EdgeColor',[0.30,0.33,0.44]);
title('Variance','Interpreter','latex')
set(gca,'XTickLabel',{'Mu_{Task1}','Mu_{opt}'});
set(gca,'XTickLabel',{'Mu_{Task1}','Mu_{opt}'},'fontname','Palatino Linotype');

grid on

subplot(2,3,4)
bar([(identification.covariance(4,4)),(identification_opt{2, 1}.covariance(4,4))],'FaceAlpha',.85,'FaceColor',[0.56,0.61,0.80],'EdgeColor',[0.30,0.33,0.44]);
title('Variance','Interpreter','latex')
set(gca,'XTickLabel',{'Mq_{Task1}','Mq_{opt}'});
set(gca,'XTickLabel',{'Mq_{Task1}','Mq_{opt}'},'fontname','Palatino Linotype');

grid on

subplot(2,3,5)
bar([(identification.covariance(5,5)),(identification_opt{2, 1}.covariance(5,5))],'FaceAlpha',.85,'FaceColor',[0.56,0.61,0.80],'EdgeColor',[0.30,0.33,0.44]);
title('Variance','Interpreter','latex')
set(gca,'XTickLabel',{'Xd_{Task1}','Xd_{opt}'});
set(gca,'XTickLabel',{'Xd_{Task1}','Xd_{opt}'},'fontname','Palatino Linotype');

grid on

subplot(2,3,6)
bar([(identification.covariance(6,6)),(identification_opt{2, 1}.covariance(6,6))],'FaceAlpha',.85,'FaceColor',[0.56,0.61,0.80],'EdgeColor',[0.30,0.33,0.44]);
title('Variance','Interpreter','latex')
set(gca,'XTickLabel',{'Md_{Task1}','Md_{opt}'});
set(gca,'XTickLabel',{'Md_{Task1}','Md_{opt}'},'fontname','Palatino Linotype');

grid on

  
  %% Scatter plot

cost_scatter=cost';
figure  
hold on
scatter(2*pi*eta_matrix(1,:),2*pi*eta_matrix(2,:),[],cost_scatter,'filled','HandleVisibility','off')
c=colorbar;
c.Label.String='Cost';
set(gca,'CLim',[0 0.01],'xscale','log','yscale','log');
plot(2*pi*eta_matrix(1,index),2*pi*eta_matrix(2,index),'*','DisplayName','Optimal Solution');
grid on
xlabel('$f_1$ [rad/s]')
ylabel('$f_2$ [rad/s]')
legend
%exportStandardizedFigure(gcf,'scatterPlot',0.67, 'addMarkers', false,'WHratio',1,'changecolors',false)


%% END OF CODE