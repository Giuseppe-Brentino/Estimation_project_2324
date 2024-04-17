close all
clc

addpath('datasets','common','common/simulator-toolbox','common/simulator-toolbox/attitude_library','common/simulator-toolbox/trajectory_library');
addpath('functions');
addpath('DroneAnimation');

load_system('Simulator_Single_Axis');
set_param('Simulator_Single_Axis',"FastRestart","off")

% Drone Animation for Estimation and Learning in Aerospace Project A.Y.
% 23-24
% Input: simulation_estimate_opt.theta , ctrl.sample_time , simulation_time

ExcitationM=ExcitationM_WC;
t=ExcitationM(:,1);

simulation_time=t(end)-t(1); 

out = sim('Simulator_Single_Axis','SrcWorkspace', 'current'); %usa A, B, C, D del sistema originale 

x = out.x_vero.Data(1:5:end);
t_x = out.x_vero.Time(1:5:end);
theta = out.theta_vero.Data(1:5:end);
t_theta = out.theta_vero.Time(1:5:end);
 %% 1. define the motion coordinates 
 % roll , pitch and yaw input in degree 
    t    = t_x;   % simulation time for 10 second

    z     = 0*t;        % z in meter 
    y     = 0*t; 
    yaw   = 0*t;  
    roll  = 0*t; 
    pitch = theta*180/pi;
 %% 6. animate by using the function makehgtform
 % Function for Animation of QuadCopter

tic
movieVector = drone_Animation(x,y,z,roll,pitch,yaw);
toc
 
 %% step5: Save animation video
myWriter = VideoWriter('drone_animation1', 'Motion JPEG AVI');
myWriter = VideoWriter('drone_animation1', 'MPEG-4');
myWriter.Quality = 100;
myWriter.FrameRate = 50;

% Open the VideoWriter object, write the movie, and class the file
open(myWriter);
writeVideo(myWriter, movieVector);
close(myWriter); 

%% step6: Evolution of theta

fig2 = figure;
fig2.Color = [1 1 1];
for i = 1:length(theta)
     plot(t_theta(1:i),rad2deg(theta(1:i)),'LineWidth',1.5);
     title('Evolution of $\theta$','FontSize',14,'Interpreter','latex')
     xlabel('t [s]','FontSize',14,'Interpreter','latex')
     ylabel('$\theta$ [deg]','Interpreter','latex','FontSize',14)

     movieVectorTheta(i) =  getframe(fig2);
     drawnow
     pause(0.02)
end

myWriterTheta = VideoWriter('EvolutionTheta', 'MPEG-4');
myWriterTheta.Quality = 100;
myWriterTheta.FrameRate = 50;

% Open the VideoWriter object, write the movie, and class the file
open(myWriterTheta);
writeVideo(myWriterTheta, movieVectorTheta);
close(myWriterTheta); 
%% step7: Evolution of x

fig3 = figure;
fig3.Color = [1 1 1];
for i = 1:length(x)
     plot(t_x(1:i),x(1:i),'LineWidth',1.5);
     title('Evolution of x','FontSize',14,'Interpreter','latex')
     xlabel('t [s]', 'FontSize',14,'Interpreter','latex')
     ylabel('x [m]','FontSize',14,'Interpreter','latex')

     movieVectorX(i) =  getframe(fig3);
     drawnow
     pause(0.02)

end
 
myWriterX = VideoWriter('EvolutionX', 'MPEG-4');
myWriterX.Quality = 100;
myWriterX.FrameRate = 50;

% Open the VideoWriter object, write the movie, and class the file
open(myWriterX);
writeVideo(myWriterX, movieVectorX);
close(myWriterX); 