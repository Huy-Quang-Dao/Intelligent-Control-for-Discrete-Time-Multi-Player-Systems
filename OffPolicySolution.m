% Code for paper: H Control for Discrete-Time Multi-Player
%                    Systems via Off-Policy Q-Learning.
% 
% Method : Off-Policy                   
% Programing Language : Matlab
% Purpose : Practice and Research

clear;clc;
%% System Parameters
A = [0.906488,0.0816012,-0.0005;...
    0.074349,0.90121,-0.000708383;...
    0,0,0.132655];
B1 = [-0.00150808,-0.0096,0.867345]';
B2 = [0.00951892,0.00038373,0]';
B3 = [-0.00563451,-0.08962,0.356478]';
E =  [0.0123956,0.068,-0.05673]';

Q = diag([5,5,5]);
R1=1;
R2=R1;
R3=R1;
gamma = 1 ;

n=size(A,2);
m1=size(B1,2);
m2=size(B2,2);
m3=size(B3,2);
p=size(E,2);
f=n^2+m1^2+m2^2+m3^2+p^2+m1*m2+m1*m3+m1*p+m2*m3+m2*p+m3*p+n*(m1+m2+m3+p)+20;

x0=[10;-10;10]; % Initial state

%% Optimal solution
K1_s = [0.5665,0.7300,0.1098];
K2_s = [0.4271,0.2752,0.0009];
K3_s = [-2.2643,-2.8476,0.0329];
K_s = [-2.2925,-2.6572,-0.0032];

%%
K1_0 = -[-1,-1,0];
K2_0 = -[-0.5,0,0];
K3_0 = -[1,2,-1];
K_0 = -[2,0,0];
i=1;
K1 = {}; K1{1} = K1_0;
K2 = {}; K2{1} = K2_0;
K3 = {}; K3{1} = K3_0;
K = {}; K{1} = K_0;
phi1 ={};phi2 ={};phi3 ={};phi4 ={};phi5 ={};phi6 ={};phi7 ={}; % Store data to serve LS solution
phi8 ={};phi9 ={};phi10 ={};phi11 ={};phi12 ={};phi13 ={};phi14 ={};phi15={}; % Store data to serve LS solution
phi= {}; psi={};% Store data to serve LS solution
%% Collect data to use Off-policy RL algorithm
for k=1:f+2
    if k==1
        x(:,k)=x0;
    else
        e1(:,k-1)=2*sin(k-1);
        e2(:,k-1)=2*sin(0.1*(k-1));
        e3(:,k-1)=2*sin(0.3*(k-1));
        % probing noise
        u1(:,k-1)=-K1{i}*x(:,k-1)+e1(:,k-1);
        u2(:,k-1)=-K2{i}*x(:,k-1)+e2(:,k-1);
        u3(:,k-1)=-K3{i}*x(:,k-1)+e3(:,k-1);
        w(:,k-1)=exp(-0.0001*(k-1))*sin(2*(k-1));
        x(:,k)=A*x(:,k-1)+B1*u1(:,k-1)+B2*u2(:,k-1)+B3*u3(:,k-1)+E*w(:,k-1);
    end
end
% Data is used to find solutions
%%
while(1)
    Phi=[];Psi=[];
    for k=1:f
        phi1{k}=kron(x(:,k)',x(:,k)')-kron(x(:,k+1)',x(:,k+1)');
        phi2{k}=2*(kron((u1(:,k)+K1{i}*x(:,k))',x(:,k)'));
        phi3{k}=2*(kron((u2(:,k)+K2{i}*x(:,k))',x(:,k)'));
        phi4{k}=2*(kron((u3(:,k)+K3{i}*x(:,k))',x(:,k)'));
        phi5{k}=kron((u1(:,k)+K1{i}*x(:,k))',(u1(:,k)-K1{i}*x(:,k))');
        phi6{k}=2*(kron(u2(:,k)',u1(:,k)')-kron((-K2{i}*x(:,k))',(-K1{i}*x(:,k))'));
        phi7{k}=2*(kron(u3(:,k)',u1(:,k)')-kron((-K3{i}*x(:,k))',(-K1{i}*x(:,k))'));
        phi8{k}=kron((u2(:,k)+K2{i}*x(:,k))',(u2(:,k)-K2{i}*x(:,k))');
        phi9{k}=2*(kron(u3(:,k)',u2(:,k)')-kron((-K3{i}*x(:,k))',(-K2{i}*x(:,k))'));
        phi10{k}=kron((u3(:,k)+K3{i}*x(:,k))',(u3(:,k)-K3{i}*x(:,k))');
        phi11{k}=2*(kron(w(:,k)',u1(:,k)')-kron((-K{i}*x(:,k))',(-K1{i}*x(:,k))'));
        phi12{k}=2*(kron(w(:,k)',u2(:,k)')-kron((-K{i}*x(:,k))',(-K2{i}*x(:,k))'));
        phi13{k}=2*(kron(w(:,k)',u3(:,k)')-kron((-K{i}*x(:,k))',(-K3{i}*x(:,k))'));
        phi14{k}=kron((w(:,k)+K{i}*x(:,k))',(w(:,k)-K{i}*x(:,k))');
        phi15{k}=2*(kron((w(:,k)+K{i}*x(:,k))',x(:,k)'));
        phi{k}=[phi1(k),phi2(k),phi3(k),phi4(k),phi5(k),phi6(k),phi7(k),phi8(k),phi9(k),phi10(k),phi11(k),phi12(k),phi13(k),phi14(k),phi15(k)];
        psi{k}=(x(:,k)')*(Q+(K1{i}')*R1*K1{i}+(K2{i}')*R2*K2{i}+(K3{i}')*R3*K3{i}-gamma^2*(K{i}')*K{i})*x(:,k);
    end
    for k=1:f
        Psi=[Psi;psi{k}];
    end
    Phic=[];
    for k=1:f
        Phic=[Phic;phi{k}];
    end
    Phi=cell2mat(Phic);
    vec_S = pinv(Phi'*Phi)*Phi'*Psi; 
    % Calculate the matrices needed for the equation Phi*vec_S = Psi
    vS1 = vec_S(1:n^2);
    vS2 = vec_S(n^2+1:n^2+n*m1);
    vS3 = vec_S(n^2+n*m1+1:n^2+n*m1+n*m2);
    vS4 = vec_S(n^2+n*m1+n*m2+1:n^2+n*m1+n*m2+n*m3);
    vS5 = vec_S(n^2+n*m1+n*m2+n*m3+1:n^2+n*m1+n*m2+n*m3+m1^2);
    vS6 = vec_S(n^2+n*m1+n*m2+n*m3+m1^2+1:n^2+n*m1+n*m2+n*m3+m1^2+m1*m2);
    vS7 = vec_S(n^2+n*m1+n*m2+n*m3+m1^2+m1*m2+1:n^2+n*m1+n*m2+n*m3+m1^2+m1*m2+m1*m3);
    vS8 = vec_S(n^2+n*m1+n*m2+n*m3+m1^2+m1*m2+m1*m3+1:n^2+n*m1+n*m2+n*m3+m1^2+m1*m2+m1*m3+m2^2);
    vS9 = vec_S(n^2+n*m1+n*m2+n*m3+m1^2+m1*m2+m1*m3+m2^2+1:n^2+n*m1+n*m2+n*m3+m1^2+m1*m2+m1*m3+m2^2+m2*m3);
    vS10 = vec_S(n^2+n*m1+n*m2+n*m3+m1^2+m1*m2+m1*m3+m2^2+m2*m3+1:n^2+n*m1+n*m2+n*m3+m1^2+m1*m2+m1*m3+m2^2+m2*m3+m3^2);
    vS11 = vec_S(n^2+n*m1+n*m2+n*m3+m1^2+m1*m2+m1*m3+m2^2+m2*m3+m3^2+1:n^2+n*m1+n*m2+n*m3+m1^2+m1*m2+m1*m3+m2^2+m2*m3+m3^2+m1*p);
    vS12 = vec_S(n^2+n*m1+n*m2+n*m3+m1^2+m1*m2+m1*m3+m2^2+m2*m3+m3^2+m1*p+1:n^2+n*m1+n*m2+n*m3+m1^2+m1*m2+m1*m3+m2^2+m2*m3+m3^2+m1*p+m2*p);
    vS13 = vec_S(n^2+n*m1+n*m2+n*m3+m1^2+m1*m2+m1*m3+m2^2+m2*m3+m3^2+m1*p+m2*p+1:n^2+n*m1+n*m2+n*m3+m1^2+m1*m2+m1*m3+m2^2+m2*m3+m3^2+m1*p+m2*p+m3*p);
    vS14 = vec_S(n^2+n*m1+n*m2+n*m3+m1^2+m1*m2+m1*m3+m2^2+m2*m3+m3^2+m1*p+m2*p+m3*p+1:n^2+n*m1+n*m2+n*m3+m1^2+m1*m2+m1*m3+m2^2+m2*m3+m3^2+m1*p+m2*p+m3*p+p^2);
    vS15 = vec_S(n^2+n*m1+n*m2+n*m3+m1^2+m1*m2+m1*m3+m2^2+m2*m3+m3^2+m1*p+m2*p+m3*p+p^2+1:end);
    S1 = reshape(vS1,n,n);
    S2 = reshape(vS2,n,m1);
    S3 = reshape(vS3,n,m2);
    S4 = reshape(vS4,n,m3);
    S5 = reshape(vS5,m1,m1);
    S6 = reshape(vS6,m1,m2);
    S7 = reshape(vS7,m1,m3);
    S8 = reshape(vS8,m2,m2);
    S9 = reshape(vS9,m2,m3);
    S10 = reshape(vS10,m3,m3);
    S11 = reshape(vS11,m1,p);
    S12 = reshape(vS12,m2,p);
    S13 = reshape(vS13,m3,p);
    S14 = reshape(vS14,p,p);
    S15 = reshape(vS15,n,p);

    K1{i+1} = pinv(S5+R1)*(S2' - (S6*K2{i}+S7*K3{i}+S11*K{i}));
    K2{i+1} = pinv(S8+R2)*(S3' - (S6'*K1{i}+S9*K3{i}+S12*K{i}));
    K3{i+1} = pinv(S10+R3)*(S4' - (S7'*K1{i}+S9'*K2{i}+S13*K{i}));
    K{i+1} = pinv(S14-gamma^2*eye(p))*(S15' - (S11'*K1{i}+S12'*K2{i}+S13'*K3{i}));
    % Find the control matrix (update)
    i = i+1;
    dK1(i-1)=norm(K1{i-1}-K1_s);
    dK2(i-1)=norm(K2{i-1}-K2_s);
    dK3(i-1)=norm(K3{i-1}-K3_s);
    dK(i-1)=norm(K{i-1}-K_s);
    if i>20
        break
    end
end

%% Plot 
figure(1)
subplot(4,1,1)
j=0:1:i-2;
plot(j,dK1,'-oc','LineWidth',3,'MarkerEdgeColor','m','MarkerFaceColor','c','MarkerSize',3)
xlim([0 inf])
grid on
ylabel('||K1^i-K1^*||')
xlabel('Iteration step')
legend('||K1^i-K1^*||')
subplot(4,1,2)
plot(j,dK2,'-oc','LineWidth',3,'MarkerEdgeColor','m','MarkerFaceColor','c','MarkerSize',3)
xlim([0 inf])
grid on
ylabel('||K2^i-K2^*||')
xlabel('Iteration step')
legend('||K2^i-K2^*||')
subplot(4,1,3)
plot(j,dK3,'-oc','LineWidth',3,'MarkerEdgeColor','m','MarkerFaceColor','c','MarkerSize',3)
xlim([0 inf])
grid on
ylabel('||K3^i-K3^*||')
xlabel('Iteration step')
legend('||K3^i-K3^*||')
subplot(4,1,4)
plot(j,dK,'-oc','LineWidth',3,'MarkerEdgeColor','m','MarkerFaceColor','c','MarkerSize',3)
xlim([0 inf])
grid on
ylabel('||K^i-K^*||')
xlabel('Iteration step')
legend('||K^i-K^*||')