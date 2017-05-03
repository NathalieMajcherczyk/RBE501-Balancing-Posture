load('OS_states.mat');
load('Parameters.mat');

t=t_OS_FK;
tau=zeros(7,length(t));

for i=1:length(t)
    tau(:,i)=InvDynNum(l,Iz,ddq_OS(i,:),dq_OS(i,:),m,q_OS(i,:));
end

for i=1:7
plot(t,tau(i,:));
hold on
end
xlabel('Time (s)','FontSize',14)
ylabel('Torques (Nm)','FontSize',14)
t_ID=t;
tau_ID=tau;
save('torques_ID.mat','t_ID','tau_ID');