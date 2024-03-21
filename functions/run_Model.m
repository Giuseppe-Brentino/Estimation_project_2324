function [time, data] = run_Model(sample_time,simulation_time)

simulation = sim('Simulator_Single_Axis', 'SrcWorkspace', 'current');

time = 0:sample_time:simulation_time;

data = struct;
data.ax = simulation.ax.Data;
data.q = simulation.q.Data;
data.Mtot = simulation.Mtot.Data;

end