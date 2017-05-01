%% Problem 1
clc
clear
close all

global m l Iz torque torqueTime g t_OS tau_OS_C1 tau_OS_C7

load('torque_OS.mat')
load('Parameters.mat')
n=7; % number of links

l=l/100;

g=9.81;

% Reference 

%q_d=q_0;%[0;0;0;0;0;0;0]; 
%dq_d=[0;0;0;0;0;0;0];

tf=5;
%q_0=[pi/4,pi/4,pi/4,pi/4,pi/4,pi/4,pi/4]; % initial joint angles
dq_0=[0,0,0,0,0,0,0];   % initial joint velocities
q0(2)=1.03;
x0=[q0.',dq_0];

torque=[];
torqueTime = [];
q_g=[];
qTime_g=[];

load('Parameters.mat');

options = odeset('RelTol',1e-3,'AbsTol',1e-3*ones(1,14));
[T,X] = ode15s(@(t,x)planarArmODE(t,x),[0 tf],x0,options);

figure('Name','theta1');
plot(T, X(:,1),'r-');
% hold on
% plot(T, q_d(1)*ones(length(T)),'b-');
title('Theta 1 under ID Control');
xlabel('Time (s)')
ylabel('Angle (rad)')

figure('Name','theta2');
plot(T, X(:,2),'r-');
% hold on
% plot(T, q_d(2)*ones(length(T)), 'b-');
title('Theta 2 under ID Control');
xlabel('Time (s)')
ylabel('Angle (rad)')

figure('Name','theta3');
plot(T, X(:,3),'r-');
% hold on
% plot(T, q_d(3)*ones(length(T)), 'b-');
title('Theta 3 under ID Control');
xlabel('Time (s)')
ylabel('Angle (rad)')

figure('Name','theta4');
plot(T, X(:,4),'r-');
% hold on
% plot(T, q_d(4)*ones(length(T)), 'b-');
title('Theta 4 under ID Control');
xlabel('Time (s)')
ylabel('Angle (rad)')

figure('Name','theta5');
plot(T, X(:,5),'r-');
% hold on
% plot(T, q_d(5)*ones(length(T)), 'b-');
title('Theta 5 under ID Control');
xlabel('Time (s)')
ylabel('Angle (rad)')

figure('Name','theta6');
plot(T, X(:,6),'r-');
% hold on
% plot(T, q_d(6)*ones(length(T)), 'b-');
title('Theta 6 under ID Control');
xlabel('Time (s)')
ylabel('Angle (rad)')

figure('Name','theta7');
plot(T, X(:,7),'r-');
% hold on
% plot(T, q_d(7)*ones(length(T)), 'b-');
title('Theta 7 under ID Control');
xlabel('Time (s)')
ylabel('Angle (rad)')

%%%%%%%%
uIdx = resampleTime(T,torqueTime);
figure
plot(T,torque(1,uIdx))
hold on
plot(T,torque(2,uIdx))
plot(T,torque(3,uIdx))
plot(T,torque(4,uIdx))
plot(T,torque(5,uIdx))
plot(T,torque(6,uIdx))
plot(T,torque(7,uIdx))
xlabel('Time (s)')
ylabel('Torque (Nm)')
title('Input Torque')
legend('\tau_1','\tau_2')

function [ dx ] = planarArmODE(t,x)

global m l Iz torque torqueTime g t_OS tau_OS_C1 tau_OS_C7

% Feedback gain matrix Kp and Kd
Kp= 500*eye(7);
Kd= 50*eye(7);

q= x(1:7,1);
dq= x(8:14,1);

% Inertia matrix (7.83)
D = MassMatrix(l, Iz, m, q);

% Gravity vector (7.85-7.86)
grav = GravityTerm(l, g, m, q);

% Coriolis matrix (7.88)
C = CoriolisTerm(l, dq, m, q);

% Compute the desired state and their time derivatives from planned
% trajectory.
%ddq_d = [0;0;0;0;0;0;0];

invD=inv(D);
%invDC=invD*C;

% compute the linear controller
%e = q - q_d;
%de = dq - dq_d;
%u=-Kp*e-Kd*de+ddq_d; 

[~,index]=min(abs(t-t_OS));

tau = [linspace(tau_OS_C7(index),tau_OS_C1(index)/2,6) tau_OS_C1(index)].';%(1:7).';%D*u+C+grav;
torque =[torque, tau];
torqueTime =[torqueTime, t];

dx=zeros(14,1);
dx(1) = x(8);
dx(2) = x(9);
dx(3) = x(10);
dx(4) = x(11);
dx(5) = x(12);
dx(6) = x(13);
dx(7) = x(14);
C = CoriolisTerm(l, x(8:14), m, q);
dx(8:14) = -invD*C +invD*tau -invD*grav;
end

function uIdx = resampleTime(T,inputTime)
% Input:
%   T - ode45 output time values
%   inputTime - time's recorded within ode45
% This first loop filters out torque indexes that are repeated as
% ode45 restarts iterations.
uIdx = zeros(length(T),1);
for i = 1:length(T)
    try
        uIdx(i) = find(T(i)==inputTime,1,'last');
    catch ME
        if strcmp(ME.identifier,'MATLAB:subsassignnumelmismatch')
            if i>1
                uIdx(i) = uIdx(i-1);
            else
                uIdx(i) = 1;
            end
        else
            rethrow ME
        end
    end
end
end


