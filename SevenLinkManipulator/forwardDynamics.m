clc
clear
close all

global m l Iz torque torqueTime g tau_ID t_ID %t_OS tau_OS_C1 tau_OS_C7

load('torques_ID.mat')
%load('torque_OS.mat')
load('Parameters.mat')
n=7; % number of links

g=9.81;

tf=1.9;
dq_0=[0,0,0,0,0,0,0];% initial joint velocities
q0=[0,0,0,0,0,0,0];% initial joint angles
x0=[q0,dq_0];

torque=[];
torqueTime = [];
q_g=[];
qTime_g=[];

options = odeset('RelTol',1e-6,'AbsTol',1e-6);
[T,X] = ode113(@(t,x)planarArmODE(t,x),[0 tf],x0,options);

%% Plots

load('resultsFK.mat')
figure('Name','theta1');
plot(T, X(:,1),'r-');
hold on
plot(t_OS_FK, q_OS(:,1),'b-');
title('Theta 1');
xlabel('Time (s)')
ylabel('Angle (rad)')
figure('Name','theta2');
plot(T, X(:,2),'r-');
hold on
plot(t_OS_FK, q_OS(:,2),'b-');
title('Theta 2');
xlabel('Time (s)')
ylabel('Angle (rad)')
figure('Name','theta3');
plot(T, X(:,3),'r-');
hold on
plot(t_OS_FK, q_OS(:,3),'b-');
title('Theta 3');
xlabel('Time (s)')
ylabel('Angle (rad)')
figure('Name','theta4');
plot(T, X(:,4),'r-');
hold on
plot(t_OS_FK, q_OS(:,4),'b-');
title('Theta 4');
xlabel('Time (s)')
ylabel('Angle (rad)')
figure('Name','theta5');
plot(T, X(:,5),'r-');
hold on
plot(t_OS_FK, q_OS(:,5),'b-');
title('Theta 5');
xlabel('Time (s)')
ylabel('Angle (rad)')
figure('Name','theta6');
plot(T, X(:,6),'r-');
hold on
plot(t_OS_FK, q_OS(:,6),'b-');
title('Theta 6');
xlabel('Time (s)')
ylabel('Angle (rad)')
figure('Name','theta7');
plot(T, X(:,7),'r-');
hold on
plot(t_OS_FK, q_OS(:,7),'b-');
title('Theta 7');
xlabel('Time (s)')
ylabel('Angle (rad)')

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
plot(t_ID,tau_ID(1,:).')
hold on
plot(t_ID,tau_ID(2,:).')
plot(t_ID,tau_ID(3,:).')
plot(t_ID,tau_ID(4,:).')
plot(t_ID,tau_ID(5,:).')
plot(t_ID,tau_ID(6,:).')
plot(t_ID,tau_ID(7,:).')
xlabel('Time (s)')
ylabel('Torque (Nm)')
title('Input Torque')
legend('\tau_1','\tau_2')

%% Numerical integration

function [ dx ] = planarArmODE(t,x)

global m l Iz torque torqueTime g tau_ID t_ID %t_OS tau_OS_C1 tau_OS_C7

q= x(1:7,1);
%dq= x(8:14,1);

% Inertia matrix 
D = MassMatrix(l, Iz, m, q);

% Gravity vector 
grav = GravityTerm(l, g, m, q);

% Coriolis matrix 
%C = CoriolisTerm(l, dq, m, q);

invD=inv(D);

[~,index]=min(abs(t-t_ID));

a=(tau_ID(:,index+1)-tau_ID(:,index))/(t_ID(index+1)-t_ID(index));
b=tau_ID(:,index)-a*t_ID(index);
tau =a*t+b;
%linear interpolation y=ax+b

%[~,index]=min(abs(t-t_OS));
% tau=[linspace(tau_OS_C1(index),tau_OS_C7(index)/2,6) tau_OS_C7(index)].';

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
C = CoriolisTerm(l,x(8:14), m, q);
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