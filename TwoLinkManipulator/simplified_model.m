clear
clc
close all

%% 3D version (head and neck pitch and yaw)
syms theta1 d1 a1 alpha1;
T0_1 = dh2mat(theta1, d1, a1, alpha1);
T0_1 = subs(T0_1, {theta1, d1, a1, alpha1}, {0, 0, 0, -pi/2});

syms theta2 d2 a2 alpha2;
T1_2 = dh2mat(theta2, d2, a2, alpha2);
T1_2 = subs(T1_2, {theta2, d2, a2, alpha2}, {0, 12.5, 0, pi/2}); %alpha was 0

syms theta3 d3 a3 alpha3;
T2_3 = dh2mat(theta3, d3, a3, alpha3);
T2_3 = subs(T2_3, {theta3, d3, a3, alpha3}, {0, 0, 0, -pi/2}); %alpha was 0

syms theta4 d4 a4 alpha4;
T3_4 = dh2mat(theta4, d4, a4, alpha4);
% https://upload.wikimedia.org/wikipedia/commons/6/61/HeadAnthropometry.JPG
T3_4 = subs(T3_4, {theta4, d4, a4, alpha4}, {pi/3, 19.7-7.3, 0, -pi/2});

figure
hold on

axis([-15 5 -5 25 -5 5])

quiver3(T0_1(1,4), T0_1(2,4), T0_1(3,4), 1, 0, 0,5,'r','LineWidth',2);
quiver3(T0_1(1,4), T0_1(2,4), T0_1(3,4), 0, 1, 0,5,'g','LineWidth',2);
quiver3(T0_1(1,4), T0_1(2,4), T0_1(3,4), 0, 0, 1,5,'b','LineWidth',2);

pts1 = [[0 0 0]; [T0_1(1,4) T0_1(2,4) T0_1(3,4)]]; %create vectors to draw lines from
plot3(pts1(:,1), pts1(:,2), pts1(:,3),'-k'); % plot lines
plot3(T0_1(1,4), T0_1(2,4), T0_1(3,4), '.', 'MarkerSize', 50) %plot joint locations
quiver3(T0_1(1,4), T0_1(2,4), T0_1(3,4), T0_1(1,1), T0_1(2,1), T0_1(3,1),5,'r','LineWidth',2);
quiver3(T0_1(1,4), T0_1(2,4), T0_1(3,4), T0_1(1,2), T0_1(2,2), T0_1(3,2),5,'g','LineWidth',2);
quiver3(T0_1(1,4), T0_1(2,4), T0_1(3,4), T0_1(1,3), T0_1(2,3), T0_1(3,3),5,'b','LineWidth',2);

T0_2 = T0_1*T1_2; %get transform matrix between this joint and base
pts = [[T0_1(1,4) T0_1(2,4) T0_1(3,4)]; [T0_2(1,4) T0_2(2,4) T0_2(3,4)]];
plot3(pts(:,1), pts(:,2), pts(:,3),'-k')
plot3(T0_2(1,4), T0_2(2,4), T0_2(3,4), '.', 'MarkerSize', 50)
quiver3(T0_2(1,4), T0_2(2,4), T0_2(3,4), T0_2(1,1), T0_2(2,1), T0_2(3,1),5,'r','LineWidth',2);
quiver3(T0_2(1,4), T0_2(2,4), T0_2(3,4), T0_2(1,2), T0_2(2,2), T0_2(3,2),5,'g','LineWidth',2);
quiver3(T0_2(1,4), T0_2(2,4), T0_2(3,4), T0_2(1,3), T0_2(2,3), T0_2(3,3),5,'b','LineWidth',2);

T0_3 = T0_2*T2_3;
pts = [[T0_2(1,4) T0_2(2,4) T0_2(3,4)]; [T0_3(1,4) T0_3(2,4) T0_3(3,4)]];
plot3(pts(:,1), pts(:,2), pts(:,3),'-k')
plot3(T0_3(1,4), T0_3(2,4), T0_3(3,4), '.', 'MarkerSize', 50)
quiver3(T0_3(1,4), T0_3(2,4), T0_3(3,4), T0_3(1,1), T0_3(2,1), T0_3(3,1),5,'r','LineWidth',2);
quiver3(T0_3(1,4), T0_3(2,4), T0_3(3,4), T0_3(1,2), T0_3(2,2), T0_3(3,2),5,'g','LineWidth',2);
quiver3(T0_3(1,4), T0_3(2,4), T0_3(3,4), T0_3(1,3), T0_3(2,3), T0_3(3,3),5,'b','LineWidth',2);

T0_4 = T0_3*T3_4;
pts = [[T0_3(1,4) T0_3(2,4) T0_3(3,4)]; [T0_4(1,4) T0_4(2,4) T0_4(3,4)]];
plot3(pts(:,1), pts(:,2), pts(:,3),'-k')
plot3(T0_4(1,4), T0_4(2,4), T0_4(3,4), '.', 'MarkerSize', 50)
quiver3(T0_4(1,4), T0_4(2,4), T0_4(3,4), T0_4(1,1), T0_4(2,1), T0_4(3,1),5,'r','LineWidth',2);
quiver3(T0_4(1,4), T0_4(2,4), T0_4(3,4), T0_4(1,2), T0_4(2,2), T0_4(3,2),5,'g','LineWidth',2);
quiver3(T0_4(1,4), T0_4(2,4), T0_4(3,4), T0_4(1,3), T0_4(2,3), T0_4(3,3),5,'b','LineWidth',2);

view(3);

%% 2D version (head and neck pitch)

%% Forward Kinematics
syms theta1 d1 a1 alpha1;
T0_1 = dh2mat(pi/2+theta1, d1, a1, alpha1);
T0_1 = subs(T0_1, {theta1, d1, a1, alpha1}, {0, 0, 12.5, 0});

syms theta2 d2 a2 alpha2;
T1_2 = dh2mat(theta2, d2, a2, alpha2);
T1_2 = subs(T1_2, {theta2, d2, a2, alpha2}, {0, 0, 19.7-7.3, 0});

figure
hold on

pts1 = [[0 0 0]; [T0_1(1,4) T0_1(2,4) T0_1(3,4)]]; %create vectors to draw lines from
plot(pts1(:,1), pts1(:,2),'-b'); % plot lines
plot(T0_1(1,4), T0_1(2,4), '.', 'MarkerSize', 50) %plot joint locations

T0_2 = T0_1*T1_2; %get transform matrix between this joint and base
pts = [[T0_1(1,4) T0_1(2,4) T0_1(3,4)]; [T0_2(1,4) T0_2(2,4) T0_2(3,4)]];
plot(pts(:,1), pts(:,2),'-b')
plot(T0_2(1,4), T0_2(2,4), '.', 'MarkerSize', 50)

%% Inverse Kinematics

syms px py;
IKTheta2Pos = acos((px^2 + py^2 + 12.5^2 - 12.4^2)/(2*12.5*sqrt(px^2 + py^2)));
IKTheta2Neg = -1*IKTheta2Pos;

IKTheta1Pos = atan(py/px) + IKTheta2Pos;
IKTheta1Neg = atan(py/px) + IKTheta2Neg;

%% Dynamics

syms m1 theta1dot theta1doubledot theta1D;
K1 = (1/2) * m1 * (12.5)^2 * theta1dot;
P1 = m1 * (-9.8) * 12.5*sin(theta1D);
torque1 = m1 * (12.5)^2 * theta1doubledot + m1 * (-9.8) * 12.5*cos(theta1D);

syms m2 theta2dot theta2doubledot theta2D;
K2 = (1/2) * m2 * ((12.5)^2 * theta1dot^2 + 12.4^2*(theta1dot+theta2dot)^2 + 2*12.5*12.4*(theta1dot^2+theta1dot*theta2dot)*cos(theta2D));
P2 = m2 * (-9.8) * (12.5*sin(theta1D) + 12.4*sin(theta1D+theta2D));
torque2 = m2 * (12.4)^2 * theta2doubledot + m2 * (-9.8) * 12.4*cos(theta2D);

K = K1 + K2;
P = P1 + P2;
L = K - P;