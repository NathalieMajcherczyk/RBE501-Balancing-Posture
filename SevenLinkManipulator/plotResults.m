% load('resultsFK.mat')
% load('Parameters.mat')
% q=q_OS;
% %q0(2)=1.03;
% 
% %q=X(:,8:14);
% % 
% X=zeros(length(q),8);
% Y=zeros(length(q),8);
% Z=zeros(length(q),8);
% for i=1:length(X)
% [X(i,:),Y(i,:),Z(i,:)]=plotarm(q(i,:));
% end
% save('jointsFK.mat')
load('jointsFK.mat')
figure
F1(length(X)) = struct('cdata',[],'colormap',[]); % initializes frames
for i=1:length(X)
plot(X(i,:),Y(i,:),'b-')%,'linewidth',3)
hold on
plot(X(i,:),Y(i,:),'r.')%,'markersize',30)
hold on
axis([-0.3 0.3 -0.3 0.3])
F1(i) = getframe;
pause(0.05)
hold off
end

% save('frames_FK.mat','F1');
% v = VideoWriter('video_FK.avi');
% v.FrameRate=7;
% open(v);
% writeVideo(v,F1);
% close(v);