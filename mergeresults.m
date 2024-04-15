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
cost=[Data70.cost; Data30.cost];
counter=0;
for i=1:100
    counter=counter+1;
 %Compute statistical parameters
     mean_cost(counter)=mean(cost(1:counter));
     std_cost(counter)=std(cost(1:counter));
end
%%
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
bar(nexttile,abs([estimation_error,error_opt(:,2),error_opt(:,1)]));

title('Estimation error','Interpreter','latex')
set(gca,'XTickLabel',{'Xu','Xq','Mu','Mq','Xd','Md'});
ylim([0 0.2])
legend('Initial Input', 'Input \eta_{avg}', 'Input \eta_{wc}')
%exportStandardizedFigure(gcf,'error_opt_hist_plot',0.67, 'addMarkers', false)

nbins=15;

pd = fitdist(cost,'Exponential');
figure
plot(pd)
xlabel('Cost')
ylabel('Number of occurrencies')
%exportStandardizedFigure(gcf,'Cost_dist_plot',0.67, 'addMarkers', false,...
   % 'changeColors',false)

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
%exportStandardizedFigure(gcf,'excitation_opt_plot',1, 'addMarkers', false, ...
   %%% 'WHratio',3)

%surface plot scenario-eta-cost
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
view(62,22)

figure
b=bar3(cost_matrix);
for k = 1:length(b)
    zdata = b(k).ZData;     
    b(k).CData = zdata;     
end
ylabel('Input sequence index')
xlabel('Scenario index')
zlabel('Cost')
ylim([0 73])
view(-90,90)


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
legend('Sim','True')
title("Longitudinal Acceleration FIT " +identification_opt{1, 1}.fit(2))
grid on


figure
plot(t,simulation_estimate_opt.q.Data)
hold on
plot(t,simulation.q.Data,'--')
legend('Sim','True')
title("Pitch rate FIT" +identification_opt{1, 1}.fit(1))
grid on

figure
plot(time,simulation.theta.*(180/pi))
title("Theta Real [deg]")
grid on

figure
plot(t,simulation_estimate_opt.theta.*(180/pi))
hold on
%plot(t,,'--')
legend('Sim','True')
title("Theta")
grid on



%%
writematrix(identification_opt{2, 1}.covariance,'Matrice_opt.xls')

%% Varianza PLOT
figure
subplot(3,2,1)
bar([(identification.covariance(1,1)),(identification_opt{2, 1}.covariance(1,1))]);
title('Variance','Interpreter','latex')
set(gca,'XTickLabel',{'Xu_{Task1}','Xu_{opt}'});

grid on

subplot(3,2,2)
bar([(identification.covariance(2,2)),(identification_opt{2, 1}.covariance(2,2))]);
title('Variance','Interpreter','latex')
set(gca,'XTickLabel',{'Xq_{Task1}','Xq_{opt}'});

grid on

subplot(3,2,3)
bar([(identification.covariance(3,3)),(identification_opt{2, 1}.covariance(3,3))]);
title('Variance','Interpreter','latex')
set(gca,'XTickLabel',{'Mu_{Task1}','Mu_{opt}'});

grid on

subplot(3,2,4)
bar([(identification.covariance(4,4)),(identification_opt{2, 1}.covariance(4,4))]);
title('Variance','Interpreter','latex')
set(gca,'XTickLabel',{'Mq_{Task1}','Mq_{opt}'});

grid on

subplot(3,2,5)
bar([(identification.covariance(5,5)),(identification_opt{2, 1}.covariance(5,5))]);
title('Variance','Interpreter','latex')
set(gca,'XTickLabel',{'Xd_{Task1}','Xd_{opt}'});

grid on

subplot(3,2,6)
bar([(identification.covariance(6,6)),(identification_opt{2, 1}.covariance(6,6))]);
title('Variance','Interpreter','latex')
set(gca,'XTickLabel',{'Md_{Task1}','Md_{opt}'});

grid on