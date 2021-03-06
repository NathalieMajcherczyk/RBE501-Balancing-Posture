load('derivates.mat')
load('resultsFK.mat')
[b,a]=butter(1,0.1,'low');  %this makes a lowpass filter
y_filt = filtfilt(b,a,ddq_OS);
ddq_OS_filtered=y_filt;
plot(t_OS_FK,y_filt(:,1))
hold on
plot(t_OS_FK,ddq_OS(:,1))
[b,a]=butter(1,0.1,'low');  %this makes a lowpass filter
y_filt2=filtfilt(b,a,dq_OS);
dq_OS_filtered=y_filt2;
plot(t_OS_FK,y_filt2(:,1))
hold on
plot(t_OS_FK,dq_OS(:,1))
%save('OS_states.mat','t_OS_FK','q_OS','dq_OS','ddq_OS','ddq_OS_filtered','dq_OS_filtered');

figure
subplot(311)
plot(t_OS_FK,q_OS)
subplot(312)
plot(t_OS_FK,dq_OS)
subplot(313)
plot(t_OS_FK,ddq_OS)