function [identification varargout] = Model_identification(ExcitationM,model,ctrl,delay,seed,noise,odefun,simulation_time,varargin)

A=model.A;
B=model.B;
C=model.C;
D=model.D;


simulation = sim('Simulator_Single_Axis','SrcWorkspace', 'current');

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
if nargout>=3
    varargout{2} = [input output];
end
if nargout==4
    A=identification.matrix{1};
    B=identification.matrix{2};
    C = [1 0 0 ; identification.matrix{3}(1,:) ; 0 0 1 ; identification.matrix{3}(2,:)];
    D = [0; 0 ; identification.matrix{4}];

    simulation = sim('Simulator_Single_Axis','SrcWorkspace', 'current');
    sim_est.ax = simulation.ax.Data;
    sim_est.q = simulation.q.Data;
    varargout{3} = sim_est;
end
end





