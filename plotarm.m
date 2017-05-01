function [ T0_6 ] = plotarm( q1, q2, q3, q4, q5, q6 )
%plotarm Takes in joint angles to produce plot of ABB arm
%   This function takes in the joint angles of each of the ABB arm's
%   joints. From these as well as pregathered DH parameters, the function
%   returns the transformation matrix from the base to the end effector as
%   well as plotting each link on a 3D plot.

    figure
    hold on
    
    T0_1 = dh2mat(q1, 290, 0, -pi/2); %get transform matrx between this and previous joint
    pts1 = [[0 0 0]; [T0_1(1,4) T0_1(2,4) T0_1(3,4)]]; %create vectors to draw lines from
    plot3(pts1(:,1), pts1(:,2), pts1(:,3),'-b'); % plot lines
    plot3(T0_1(1,4), T0_1(2,4), T0_1(3,4), '.', 'MarkerSize', 50) %plot joint locations

    T1_2 = dh2mat(q2-(pi/2), 0, 270, 0);
    T0_2 = T0_1*T1_2; %get transform matrix between this joint and base
    pts = [[T0_1(1,4) T0_1(2,4) T0_1(3,4)]; [T0_2(1,4) T0_2(2,4) T0_2(3,4)]];
    plot3(pts(:,1), pts(:,2), pts(:,3),'-b')
    plot3(T0_2(1,4), T0_2(2,4), T0_2(3,4), '.', 'MarkerSize', 50)

    T2_3 = dh2mat(q3, 0, 70, -pi/2);
    T0_3 = T0_2*T2_3;
    pts = [[T0_2(1,4) T0_2(2,4) T0_2(3,4)]; [T0_3(1,4) T0_3(2,4) T0_3(3,4)]];
    plot3(pts(:,1), pts(:,2), pts(:,3),'-b')
    plot3(T0_3(1,4), T0_3(2,4), T0_3(3,4), '.', 'MarkerSize', 50)

    T3_4 = dh2mat(q4, 302, 0, -pi/2);
    T0_4 = T0_3*T3_4;
    pts = [[T0_3(1,4) T0_3(2,4) T0_3(3,4)]; [T0_4(1,4) T0_4(2,4) T0_4(3,4)]];
    plot3(pts(:,1), pts(:,2), pts(:,3),'-b')
    plot3(T0_4(1,4), T0_4(2,4), T0_4(3,4), '.', 'MarkerSize', 50)

    view(3);

end

