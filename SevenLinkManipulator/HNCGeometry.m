%% Musculoskeletal Geometry
clear
clc, close all

X=[0;1;1.54;1.85;1.95;2.20;4.91];
Z=[0;1.63;3.29;4.97;6.73;9.52;18.18];

joints.X=[-0.5;0.5;1.5;1.58;2.12;1.78;2.62;7.2];
joints.Y=[-0.815;0.815;2.445;4.135;5.805;7.655;11.385;24.975];

X=X-joints.X(1);
Z=Z-joints.Y(1);

joints.X=joints.X-joints.X(1);
joints.Y=joints.Y-joints.Y(1);

plot(X,Z,'*')
hold on
plot(joints.X,joints.Y,'o-')

m=[0.030;0.025;0.020;0.022;0.025;0.055;4.553];
Iz=[0.118;0.085;0.058;0.060;0.074;0.222;165.463];

l=zeros(7,1);
q0=zeros(7,1);
for i=2:8
l(i-1)=sqrt((joints.X(i)-joints.X(i-1))^2+(joints.Y(i)-joints.Y(i-1))^2);
q0(i-1)=atan((joints.Y(i)-joints.Y(i-1))/(joints.X(i)-joints.X(i-1)));
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%hold on
%CoR_X=[1.3;1.77;2.1;2.2;2.33;2.34];
%CoR_Z=[0.41;1.46;3.33;4.95;6.73;11.72];
%plot(CoR_X,CoR_Z,'*')
% 
% r=sqrt(Iz./m)/2
% n=length(X);
% joints=zeros(n,2);
% [xout,yout]=circcirc(X(1),Z(1),r(1),X(2),Z(2),r(2))
% [yout,I]=min(yout)
% xout=xout(I)
% joints(2,1)=xout;
% joints(2,2)=yout;
% slope=(yout-Z(1))/(xout-X(1));
% intercept=Z(1)-slope*X(1);
% [xout,yout]=linecirc(slope,intercept,X(1),Z(1),r(1));
% [joints(1,1),I]=min(xout);
% joints(1,2)=yout(I);
% 
% %joints(1,:)=[-0.5,-0.5];
% %joints(2,:)=[0.5481,0.8264];
% 
% for i=3:n
%     slope=(joints(i-1,2)-Z(i-1))/(joints(i-1,1)-X(i-1));
%     intercept=Z(i-1)-slope*X(i-1);
%     dist=sqrt((joints(i-1,2)-Z(i-1))^2+(joints(i-1,1)-X(i-1))^2);
%     [xout,yout]=linecirc(slope,intercept,X(i-1),Z(i-1),dist);
%     [joints(i,2),I]=max(yout);
%     joints(i,1)=xout(I);
% end
% plot(X,Z,'*')
% hold on
%plot(joints(:,1),joints(:,2),'o-')
