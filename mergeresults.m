close all;
clearvars;
clc;
rng default;

addpath('datasets','common','common/simulator-toolbox','common/simulator-toolbox/attitude_library','common/simulator-toolbox/trajectory_library');
addpath('functions');

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

parfor i = 1:30       % iterate on eta
    sim_matrix = st;
    for k = 1:70   % iterate on scenario
        sim=(i-1)*30+k;
        disp("Sim "+ sim +" of "+70*30)

        sim_matrix.A =  Data30.scenario.A(:,:,k);
        sim_matrix.B =  Data30.scenario.B(:,:,k);
        sim_matrix.C =  Data30.scenario.C(:,:,k);
        sim_matrix.D =  Data30.scenario.D(:,:,k);
       
            [cost_matrix_down(i,k),~] = obj_function(Data30.eta_matrix(:,i),sim_matrix,ctrl,delay,seed,noise,odefun);
   
    end
end

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

save('mergedData')
