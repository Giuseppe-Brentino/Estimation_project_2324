close all;
clearvars;
clc;
rng default;

addpath('datasets','common','common/simulator-toolbox','common/simulator-toolbox/attitude_library','common/simulator-toolbox/trajectory_library');
addpath('functions');
addpath('DroneAnimation');

load_system('Simulator_Single_Axis');
set_param('Simulator_Single_Axis',"FastRestart","off")
%%  load data
load("ALL_DATA_30-5_3000iter_OK.mat");
Data30.scenario = scenario;
Data30.eta_matrix=eta_matrix;
Data30.cost=cost;
Data30.cost_matrix=cost_matrix;
Data30.theta_opt_matrix= theta_opt_matrix;
Data30.N_scenarios=N_scenarios;

load("ALL_DATA_70-5_3000iter.mat");
Data70.scenario = scenario;
Data70.eta_matrix=eta_matrix;
Data70.cost=cost;
Data70.cost_matrix=cost_matrix;
Data70.theta_opt_matrix= theta_opt_matrix;
Data70.N_scenarios=N_scenarios;

close all;

%% compute cost for missing parts
% % % cost_matrix = zeros(100);
% % % theta_opt_matrix = zeros(6,100);
% % % identified_model = cell(100,1);

cost_matrix_left=zeros(Data70.N_scenarios,Data30.N_scenarios );
cost_matrix_down=zeros(Data30.N_scenarios,Data70.N_scenarios );

st=struct;

set_param('Simulator_Single_Axis',"FastRestart","on");

% left-side matrix
tic
parfor i = 1:70       % iterate on eta
    sim_matrix = st;
    for k = 1:30   % iterate on scenario
        sim=(i-1)*30+k;
        disp("Sim "+ sim +" of "+70*30)
        sim_matrix.A =  Data30.scenario.A(:,:,k);
        sim_matrix.B =  Data30.scenario.B(:,:,k);
        sim_matrix.C =  Data30.scenario.C(:,:,k);
        sim_matrix.D =  Data30.scenario.D(:,:,k);

            [cost_matrix_left(i,k),~] = obj_function(Data70.eta_matrix(:,i),sim_matrix,ctrl,delay,seed,noise,odefun);
   
    end
end
toc
tic
parfor i = 1:30       % iterate on eta
    sim_matrix = st;
    for k = 1:70   % iterate on scenario
        sim=(i-1)*70+k;
        disp("Sim "+ sim +" of "+70*30)

        sim_matrix.A =  Data70.scenario.A(:,:,k);
        sim_matrix.B =  Data70.scenario.B(:,:,k);
        sim_matrix.C =  Data70.scenario.C(:,:,k);
        sim_matrix.D =  Data70.scenario.D(:,:,k);
       
            [cost_matrix_down(i,k),~] = obj_function(Data30.eta_matrix(:,i),sim_matrix,ctrl,delay,seed,noise,odefun);
   
    end
end
toc
cost_matrix=[Data70.cost_matrix cost_matrix_left; cost_matrix_down Data30.cost_matrix];
eta_matrix = [Data70.eta_matrix Data30.eta_matrix];

%%  compute optimal eta
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

%save('mergedData')


%%
%Grafici nel tempo OPTIMAL INPUT 

ExcitationM=ExcitationM_WC;
t=ExcitationM(:,1);

simulation_time=t(end)-t(1); 

 simulation= sim('Simulator_Single_Axis','SrcWorkspace', 'current'); %usa A, B, C, D del sistema originale 
 A=identification_opt{1, 1}.matrix{1};
 B=identification_opt{1, 1}.matrix{2};
 C = [1 0 0 ; identification_opt{1, 1}.matrix{3}(1,:) ; 0 0 1 ; identification_opt{1, 1}.matrix{3}(2,:)];
 D = [0; 0 ; identification_opt{1, 1}.matrix{4}];
 simulation_estimate_opt = sim('Simulator_Single_Axis','SrcWorkspace', 'current'); 

figure
plot(t,simulation_estimate_opt.ax.Data)
hold on
plot(t,simulation.ax.Data,'--')
legend('Estimated model','Real model')
title(sprintf('Longitudinal Acceleration. FIT: %.2f %%',identification_opt{1, 1}.fit(2)))
grid on
xlabel('Time [s]')
ylabel('Acceleration [$m/s^2$]')
%exportStandardizedFigure(gcf,'long_acc',0.67, 'addMarkers', false);

figure
plot(t,simulation_estimate_opt.q.Data)
hold on
plot(t,simulation.q.Data,'--')
legend('Estimated model','Real model')
title(sprintf('Pitch rate. FIT: %.2f %%',identification_opt{1, 1}.fit(1)))
grid on
xlabel('Time [s]')
ylabel('Pitch rate [rad/s]')
%exportStandardizedFigure(gcf,'pitch_rate',0.67, 'addMarkers', false);
%% Identification problem with input Task1
clear all
close all
load MergeData.mat
clc
addpath('datasets','common','common/simulator-toolbox','common/simulator-toolbox/attitude_library','common/simulator-toolbox/trajectory_library');
addpath('functions');
load ExcitationM; %input prof

ctrl.SetPoint=[0,0];
t=ExcitationM(:,1);
simulation_time_input_prof=t(end)-t(1);
N_scenario_tot=Data30.N_scenarios+Data70.N_scenarios;

identification_scenario_input_prof=cell(N_scenario_tot,1);
J_scenario=zeros(N_scenario_tot,1);

tic
for i=1:30
        
        sim_matrix30.A =  Data30.scenario.A(:,:,i);
        sim_matrix30.B =  Data30.scenario.B(:,:,i);
        sim_matrix30.C =  Data30.scenario.C(:,:,i);
        sim_matrix30.D =  Data30.scenario.D(:,:,i);

        [identification_scenario_input_prof{i,1}] = Model_identification(ExcitationM,sim_matrix30,ctrl,delay,seed,noise,odefun,simulation_time_input_prof,real_parameters);
        normalized_cov = abs(100*diag(identification_scenario_input_prof{i,1}.covariance)./identification.parameters);

        %cost function
        J_scenario(i) = sum(normalized_cov);





        % display iteration
        disp("Simulation " + i + " out of 30");
end
toc

tic
for i=31:100

        j=i-30;
        sim_matrix70.A =  Data70.scenario.A(:,:,j);
        sim_matrix70.B =  Data70.scenario.B(:,:,j);
        sim_matrix70.C =  Data70.scenario.C(:,:,j);
        sim_matrix70.D =  Data70.scenario.D(:,:,j);


        [identification_scenario_input_prof{i,1}] = Model_identification(ExcitationM,sim_matrix70,ctrl,delay,seed,noise,odefun,simulation_time_input_prof,real_parameters);
        normalized_cov = abs(100*diag(identification_scenario_input_prof{i,1}.covariance)./identification.parameters);

        %cost function
        J_scenario(i) = sum(normalized_cov);

        % display iteration
        disp("Simulation " + j + " out of 70");
end
toc

