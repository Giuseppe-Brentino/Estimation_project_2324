function [identification varargout] = Model_identification(ExcitationM,model,ctrl,delay,seed,noise,odefun,simulation_time,varargin)

A=model.A;
B=model.B;
C=model.C;
D=model.D;


simulation = sim('Simulator_Single_Axis','SrcWorkspace', 'current');
data = struct;
data.ax = simulation.ax.Data;
data.q = simulation.q.Data;
input = simulation.Mtot.Data;

output = [data.q data.ax]; % Measured acceleration and pitch rate

sim_data = iddata(output, input, ctrl.sample_time);
data_fd = fft(sim_data); % output of the simulation in the frequency domain

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
identification.matrix={estimated_model.A; estimated_model.B; estimated_model.C; estimated_model.D};
identification.estimated_model=estimated_model;


if nargout>=2
    real_parameters = varargin{1};
    varargout{1} = (identification.parameters-real_parameters) ./ real_parameters * 100; %Estimation Error
end
if nargout==3
     varargout{2} = [input output]; 
end

end