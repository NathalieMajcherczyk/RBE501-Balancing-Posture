%% Problem 1

clc
clear
close all

global q_d dq_d m1 m2 l1 r1 r2 I1 I2 torque torqueTime g
n=7; % number of links
g=9.81;

% Reference 
q_d=[0;0;0;0;0;0;0]; 
dq_d=[0;0;0;0;0;0;0];
tf=2;
q_0=[pi/4,pi/4,pi/4,pi/4,pi/4,pi/4,pi/4]; % initial joint angles
dq_0=[0,0,0,0,0,0,0];   % initial joint velocities
x0=[q_0,dq_0];

    

torque=[];

torqueTime = [];

m1=10; m2=5; l1=1; l2=1; r1=0.5; r2=0.5; I1=10/12; I2=5/12;



options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);

[T,X] = ode45(@(t,x)planarArmODE(t,x),[0 tf],x0,options);



figure('Name','theta1');

plot(T, X(:,1),'r-');

hold on

plot(T, q_d(1)*ones(length(T)),'b-');

title('Theta 1 under ID Control');

xlabel('Time (s)')

ylabel('Angle (rad)')



figure('Name','theta2');

plot(T, X(:,2),'r-');

hold on

plot(T, q_d(2)*ones(length(T)), 'b-');

title('Theta 2 under ID Control');

xlabel('Time (s)')

ylabel('Angle (rad)')

%%%%%%%%

uIdx = resampleTime(T,torqueTime);

figure

plot(T,torque(1,uIdx))

hold on

plot(T,torque(2,uIdx))

xlabel('Time (s)')

ylabel('Torque (Nm)')

title('Input Torque')

legend('\tau_1','\tau_2')



function [ dx ] = planarArmODE(t,x)



global q_d dq_d m1 m2 m3 m4 m5 m6 l1 r1 l2 r2 l3 r3 l4 r4 l5 r5 l6 r6 l7 r7 I1 I2 I3 I4 I5 I6 I7 torque torqueTime g



% Feedback gain matrix Kp and Kd

Kp= 500*eye(2);

Kd= 50*eye(2);



q= x(1:2,1);

dq= x(3:4,1);



% Inertia matrix (7.83)

% D=zeros(2,2);

% D(1,1)=m1*r1^2+m2*(l1^2+r1^2+2*l1*r2*cos(q(2)))+I1+I2;

% D(1,2)=m2*(r2^2+l1*r2*cos(q(2)))+I2;

% D(2,1)=D(1,2);

% D(2,2)=m2*r2^2+I2;

l = [l1; l2; l3; l4; l5; l6; l7];

m = [m1; m2; m3; m4; m5; m6; m7];

D = MassMatrix(l, dq_d, m, q_d);



% Gravity vector (7.85-7.86)

% grav=zeros(2,1);

% grav(1)=(m1*r1+m2*l1)*g*cos(q(1))+m2*r2*g*cos(q(1)+q(2));

% grav(2)=m2*r2*g*cos(q(1)+q(2));

grav = GravityTerm(l, m, g, dq_d)



% Coriolis matrix (7.88)

% h=-m2*l1*r2*sin(q(2));

% C=[h*dq(2) , h*dq(2)+h*dq(1);

%    -h*dq(1),        0        ];

C = CoriolisTerm(l, dq_d, m, q_d);



% Compute the desired state and their time derivatives from planned

% trajectory.

ddq_d =[0;0];



invD=inv(D);

invDC=invD*C;



% compute the linear controller

e = q - q_d;

de = dq - dq_d;

u=-Kp*e-Kd*de+ddq_d; 



tau = D*u+C*dq+grav;

torque =[torque, tau];

torqueTime =[torqueTime, t];



dx=zeros(4,1);

dx(1) = x(3);

dx(2) = x(4);

dx(3:4) = -invDC*x(3:4) +invD*tau -invD*grav;

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


