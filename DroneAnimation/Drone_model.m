close all
clc
 
% Drone Animation for Estimation and Learning in Aerospace Project A.Y.
% 23-24
% Input: simulation_estimate_opt.theta , ctrl.sample_time , simulation_time

 %% 1. define the motion coordinates 
 % roll , pitch and yaw input in degree 
    t    = 0:ctrl.sample_time:simulation_time;   % simulation time for 10 second
    z     = 0*t;        % z in meter 
    y     = 0*t; 
    x     = 0*t;
    yaw   = 0*t;  
    roll  = 0*t; 
    pitch = simulation_estimate_opt.theta'*180/pi;
 %% 6. animate by using the function makehgtform
 % Function for ANimation of QuadCopter
  drone_Animation(x,y,z,roll,pitch,yaw)
 
 
 %% step5: Save the movie
myWriter = VideoWriter('drone_animation', 'Motion JPEG AVI');
myWriter = VideoWriter('drone_animation1', 'MPEG-4');
myWriter.Quality = 100;
myWritter.FrameRate = 120;

% Open the VideoWriter object, write the movie, and class the file
open(myWriter);
writeVideo(myWriter, movieVector);
close(myWriter); 