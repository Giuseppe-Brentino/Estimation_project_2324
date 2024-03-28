function [J, varargout] = obj_function(eta0,model,ctrl,delay,seed,noise,odefun)

% model parameters

f1 = eta0(1);
f2 = eta0(2);
T = eta0(3);

wait_time = 5;
simulation_time = 2*wait_time + T;
% input
wait_vector = zeros(round(wait_time/ctrl.sample_time),1);
excitation_time = (0:ctrl.sample_time:T)';
freq_fun = 2*pi*excitation_time.*(f1 + (f2-f1)/T.*excitation_time);
excitation_value = [wait_vector; 0.1*sin(freq_fun); wait_vector] ;
excitation_time = (0:ctrl.sample_time:simulation_time)';
ExcitationM=[excitation_time, excitation_value];         % optimized input sequence

[identification] = Model_identification(ExcitationM,model,ctrl,delay,seed,noise,odefun,simulation_time);

normalized_cov = abs(100*diag(identification.covariance)./identification.parameters);

%cost function
J = sum(normalized_cov);


if nargout==2
    varargout{1}=identification;
end

end