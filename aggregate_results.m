function [input, sisw_output] = aggregate_results(eta,estimated_model,ctrl,delay,seed,noise,odefun)
% model parameters
A = estimated_model.A;
B = estimated_model.B;
C = estimated_model.C;
D = estimated_model.D;
f1 = eta(1);
f2 = eta(2);
T = eta(3);
wait_time = 5;
simulation_time = 2*wait_time + T;
% input
wait_vector = zeros(round(wait_time/ctrl.sample_time),1);
excitation_time = (0:ctrl.sample_time:T)';
freq_fun = 2*pi*excitation_time.*(f1 + (f2-f1)/T.*excitation_time);
excitation_value = [wait_vector; 0.1*sin(freq_fun); wait_vector] ;
excitation_time = (0:ctrl.sample_time:simulation_time)';
ExcitationM =[ excitation_time, excitation_value];
% run simulation
simulation = sim('Simulator_Single_Axis','SrcWorkspace', 'current');
% run identification
data = struct;
data.ax = simulation.ax.Data;
data.q = simulation.q.Data;
data.Mtot = simulation.Mtot.Data;
input = data.Mtot;              % Normalized control moment
sisw_output = [data.q data.ax];      % Measured acceleration and pitch rate
end