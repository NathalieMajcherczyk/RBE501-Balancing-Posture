clear
clc, close all
format shortg

n=7; % number of joints

%% Frame Transformation Matrices

%d2hmat(theta,d,a,alpha)

syms t q1 q2 q3 q4 q5 q6 q7 q1(t) q2(t) q3(t) q4(t) q5(t) q6(t) q7(t) A B C D E F G q;

q1=q1(t);
q2=q2(t);
q3=q3(t);
q4=q4(t);
q5=q5(t);
q6=q6(t);
q7=q7(t);

q=[q1;q2;q3;q4;q5;q6;q7];

q_char=['q1';'q2';'q3';'q4';'q5';'q6';'q7'];

l=[A B C D E F G];

T=sym(zeros(4,4,n));
T0i=sym(zeros(4,4,n));
temp=sym(eye(4,4));

for i=1:n
    T(:,:,i)=dh2mat(q(i),sym(0),l(i),sym(0));
    T0i(:,:,i)=T(:,:,i)*temp;
    temp=T0i(:,:,i);
end

%% Jacobian

% Computation of A (3 DoF form of the Jacobian)

A_Jac=sym(zeros(3,n));

for i=1:n
temp1=subs(T0i(1:3,4,i),q(i),q_char(i));
A_Jac(:,i)=diff(temp1,q_char(i));
end

% Computation of B

B_Jac=sym(zeros(3,n));

for i=1:n
    B_Jac(:,i)=T0i(1:3,3,i);
end

% Full 6DoF Jacobian
J=[A_Jac;B_Jac]

% %% Instantaneous tip velocities
% syms dq
% 
% num_tip_velocity=J*dq
% torques=J.'*F

%% Inverse Dynamics

syms mA mB mC mD mE mF mG g %masses
syms IAx IAy IAz IBx IBy IBz ICx ICy ICz IDx IDy IDz IEx IEy IEz IFx IFy IFz IGx IGy IGz

m=[mA mB mC mD mE mF mG];

Inertia=[IAx IAy IAz;IBx IBy IBz;ICx ICy ICz;IDx IDy IDz;IEx IEy IEz;IFx IFy IFz;IGx IGy IGz];

dq=[diff(q1,t);diff(q2,t);diff(q3,t);diff(q4,t);diff(q5,t);diff(q6,t);diff(q7,t)];
ddq=[diff(q1,t,t);diff(q2,t,t);diff(q3,t,t);diff(q4,t,t);diff(q5,t,t);diff(q6,t,t);diff(q7,t,t)];

dq_char=['dq1';'dq2';'dq3';'dq4';'dq5';'dq6';'dq7'];
ddq_char=['ddq1';'ddq2';'ddq3';'ddq4';'ddq5';'ddq6';'ddq7'];

I=sym(zeros(3,3,n));
Jwi=sym(zeros(3,n,n));

for i=1:n
    I(1,1,i)=Inertia(i,1);
    I(2,2,i)=Inertia(i,2);
    I(3,3,i)=Inertia(i,3);
    Jwi(:,:,i)=[J(4:6,1:i) zeros(3,n-i)];
end
%q=sym([q1;q2;q3;q4;q5;q6;q7]);

% Kinetic energy

CG_joint_frame=sym(zeros(3,n));
CG=sym(zeros(3,n));
v_CG=sym(zeros(3,n));
v2_CG=sym(zeros(1,n));
Ki=sym(zeros(n,1));
temp=eye(4,4);

for i=1:n
    CG_joint_frame(:,i)=l(i)*[0.5*cos(q(i));0.5*sin(q(i));0];
    CG(:,i)=temp(1:3,1:3)*CG_joint_frame(:,i)+temp(1:3,4);
    temp=T0i(:,:,i);
    v_CG(:,i)=diff(CG(:,i),t);
    v2_CG(i)=simplify(v_CG(:,i).'*v_CG(:,i));
    Ki(i)=0.5*m(i)*v2_CG(i)+0.5*(dq.'*(Jwi(:,:,i).'*T0i(1:3,1:3,i)*I(:,:,i)*T0i(1:3,1:3,i).'*Jwi(:,:,i))*dq);
end

K=simplify(sum(Ki))

% Potential energy 

Pi=g*m(1:n).*CG(2,1:n);

P=simplify(sum(Pi))

% Lagrangian

L=K-P

% Derivatives of the Lagrangian

L_q=sym(zeros(n,1));
L_dq=sym(zeros(n,1));
dL_dq=sym(zeros(n,1));
dL_ddq=sym(zeros(n,1));
dL_ddq_dt=sym(zeros(n,1));


for i=1:n
L_q(i)=subs(L,q(i),q_char(i,:));    
L_dq(i)=subs(L,dq(i),dq_char(i,:));

dL_ddq(i)=diff(L_dq(i),dq_char(i,:));
dL_ddq(i)=subs(dL_ddq(i),dq_char(i,:),dq(i));
dL_ddq_dt(i)=diff(dL_ddq(i),t);

dL_dq(i)=diff(L_q(i),q_char(i,:));
dL_dq(i)=subs(dL_dq(i),q_char(i,:),q(i));
end

tau=sym(zeros(n,1));

% Equations of Motion
for i=1:n
tau(i)= dL_ddq_dt(i) - dL_dq(i);
end

tau

M=sym(zeros(n,n));
eta=sym(zeros(n,1));
V=sym(zeros(n,1));
grav=sym(zeros(n,1));

% Matrix-vector standard form
for i=1:n
        [cM,TM]=coeffs(tau(i),ddq);
        M(i,:)=cM(1:end-1); % Inertia term
        eta(i,:)=cM(end); % Coriolis and gravity terms combined
        [cV,TV]=coeffs(eta(i,:),dq);
        V(i,:)=cV(1:end-1)*TV(1:end-1).'; % Coriolis term
        grav(i,:)=cV(end).*TV(end); % gravity term
end

M=simplify(M)

V=simplify(V)

grav=simplify(grav)

% Verifications

%M.'-M % symmetric mass matrix
% 
for i=1:n
    M=subs(M,q(i),q_char(i,:));
    V=subs(V,dq(i),dq_char(i,:));
    V=subs(V,q(i),q_char(i,:));
    grav=subs(grav,q(i),q_char(i,:));
    tau=subs(tau,q(i),q_char(i,:));
    tau=subs(tau,dq(i),dq_char(i,:));
    tau=subs(tau,ddq(i),ddq_char(i,:));
end
%mass_matrix=matlabFunction(M,'File','MassMatrix'); 
%coriolis=matlabFunction(V,'File','CoriolisTerm');
%gravity=matlabFunction(grav,'File','GravityTerm');

tau=matlabFunction(tau,'File','InvDynNum');