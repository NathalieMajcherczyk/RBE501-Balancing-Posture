function [ X,Y,Z ] = plotarm(q)
%PLOTARM plot ABB robot arm
%  This function will solve the forward kinematics and draw each link in
%  a 3D plot based upon an input vector of the joint angle configuration.
% 
n=7;

%l=[A B C D E F G];

load('Parameters.mat');

l=l/100;
q=q+q0.';

T=sym(zeros(4,4,n));
T0i=sym(zeros(4,4,n));
temp=sym(eye(4,4));
for i=1:n
    T(:,:,i)=dh2mat(q(i),sym(0),l(i),sym(0));
    T0i(:,:,i)=T(:,:,i)*temp;
    temp=T0i(:,:,i);
end

joint1=T0i(:,:,1);
joint2=joint1*T(:,:,2);
joint3=joint2*T(:,:,3);
joint4=joint3*T(:,:,4);
joint5=joint4*T(:,:,5);
joint6=joint5*T(:,:,6);
joint7=joint6*T(:,:,7);

X=[0 joint1(1,4) joint2(1,4) joint3(1,4) joint4(1,4) joint5(1,4) joint6(1,4) joint7(1,4)];
Y=[0 joint1(2,4) joint2(2,4) joint3(2,4) joint4(2,4) joint5(2,4) joint6(2,4) joint7(2,4)];
Z=[0 joint1(3,4) joint2(3,4) joint3(3,4) joint4(3,4) joint5(3,4) joint6(3,4) joint7(3,4)];

%figure
%plot3(X,Y,Z,'b-','linewidth',3)
%hold on;
%plot3(X,Y,Z,'r.','markersize',30)
% xlabel('X(mm)','fontsize',14)
% ylabel('Y(mm)','fontsize',14)
% zlabel('Z(mm)','fontsize',14)
% title('Arm Display')
% grid on
%T=joint6;

end

