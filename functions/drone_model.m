function [A, B, C, D] = drone_model(Xu, Xq, Mu, Mq, Xd, Md, ~)


g=9.81;
A=[Xu Xq -g; Mu Mq 0; 0 1 0];
B=[Xd Md 0]';
C=[0 1 0; Xu Xq 0];
D=[0 Xd]';

end