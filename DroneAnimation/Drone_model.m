close all
clc
 
% Drone Animation for Estimation and Learning in Aerospace Project A.Y.
% 23-24
% Input: simulation_estimate_opt.theta , ctrl.sample_time , simulation_time
x=out_simulink.x.Data;
t_x=out_simulink.x.Time;
theta=out_simulink.theta;
t_theta=linspace(0, simulation_time, length(theta))';
 %% 1. define the motion coordinates 
 % roll , pitch and yaw input in degree 
    t    = 0:ctrl.sample_time:simulation_time;   % simulation time for 10 second
    z     = 0*t;        % z in meter 
    y     = 0*t; 
    x     = x;
    yaw   = 0*t;  
    roll  = 0*t; 
    pitch = theta*180/pi;
 %% 6. animate by using the function makehgtform
 % Function for ANimation of QuadCopter
  drone_Animation(x,y,z,roll,pitch,yaw) 
  %%
fig2 = figure;
for i = 1:length(theta)
     plot(t_theta(1:i),rad2deg(theta(1:i)),'LineWidth',1.5);
     title('Evoultion of \theta')
      ylim([-25 25])
     drawnow
     pause(1e-15)
end
  %%
fig3 = figure;
for i = 1:length(x)
     plot(t_x(1:i),x(1:i),'LineWidth',1.5);
      title('Evoultion of x')
     ylim([-0.05 0.05])
     drawnow
     pause(1e-15)
end

 %% step5: Save the movie
myWriter = VideoWriter('drone_animation', 'Motion JPEG AVI');
myWriter = VideoWriter('drone_animation1', 'MPEG-4');
myWriter.Quality = 100;
myWritter.FrameRate = 120;

% Open the VideoWriter object, write the movie, and class the file
open(myWriter);
writeVideo(myWriter, movieVector);
close(myWriter); 