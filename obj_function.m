function [J] = obj_function(estimated_model,odefun,sample_time)

A = estimated_model.A; 
B = estimated_model.B;
C = estimated_model.C;
D = estimated_model.D;

simulation = sim('Simulator_Single_Axis');

data = struct;
data.ax = simulation.ax.Data;
data.q = simulation.q.Data;
data.Mtot = simulation.Mtot.Data;

input = data.Mtot;              % Normalized control moment
output = [data.q data.ax];      % Measured acceleration and pitch rate
sim_data = iddata(output, input, sample_time); 
data_fd = fft(sim_data); % output of the simulation in the frequency domain



guess = zeros(6,1);
parameters = {'Xu', guess(1); 'Xq', guess(2); 'Mu',  guess(3); 'Mq',  guess(4);...
    'Xd',  guess(5); 'Md',  guess(6)};
sys_init = idgrey(odefun, parameters, 'c');

% Model Identification
estimated_model = greyest(data_fd, sys_init);
identification.parameters = estimated_model.Report.Parameters.ParVector;
identification.fit = estimated_model.Report.Fit.FitPercent;
identification.covariance = getcov(estimated_model);

%cost function
J = trace(identification.covariance);



end