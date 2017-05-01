load('resultsFK.mat');
load('Parameters.mat');

t=t_OS_FK;
tau=zeros(7,length(t));

for i=1:length(t)
    tau(:,i)=InvDynNum(l,m,q_OS(i,:));
end

for i=1:7
plot(t,tau(i,:));
hold on
end