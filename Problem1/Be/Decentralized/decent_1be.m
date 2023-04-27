clc
clear all
close all
s = tf('s');
%% 
G11=(-0.805/((0.3*s+1)*(1.6*s+1)));
G12=(0.055/((2.76*s+1)*(1.25*s+1)));
G21=(0.465/(1.3*s+1));
G22=(0.055/(3.3*s+1));
G = [G11 G12;G21 G22];

%% Defining the System in State Space
sys_decent=ss(G,'minimal')
A=sys_decent.A
B=sys_decent.B
C=sys_decent.C
D=sys_decent.D
%% Make Jordan fom of the system
eig(sys_decent)
[T,Aj] = jordan(A)

Bj = inv(T)*B

Cj = C*T

Q = T'*eye(6)*T
%% Make LQR with Jordan form
Q11 = [12.1111 3.0833;3.0833 1.3906]
Q22 = [1]
R = eye(1)

A11 = [-3.3333 0;0 -0.6250]
B11=[-0.3692;0.3692]
A22=[ -0.3030]
B22=[0.1250]


K11 = lqr(A11,B11,Q11,R)
K22 = lqr(A22,B22,Q22,R)

K_jordan = [-0.3715  0.1152 0 0 0 0;...
            0 0 0 0 0  K22]
K_dec = K_jordan*inv(T)
%%
tspan = [0 20];
x0 = [0.5;0.5;0.5;0.5;0.5;0.5];
[t,x] = ode45(@Decentralized_form2,tspan,x0);
% Solving with ODE45
x = x';
% For Dimensions
u = -K_dec*x;
% State Feedback
%% plot (states)
plot(t,x,'linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('States')
title(' Decentralized form for second system')
legend('x_1','x_2','x_3','x_4','x_5','x_6')
%% Plot (control signals)
figure
plot(t,u,'linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('Control Signals')
title(' Decentralized form for second system')
legend('u_1','u_2')

%% calculate the cost function
JJJ = [];
J(1,1)=0;
JJJ(1,:)=0;
tt = 0;
xT = x';
uT = u';
Q1=eye(6)
 for i = 1:68
     dt=t(i)-tt; 
     tt=t(i);
     S = (xT(i,:)*Q1*x(:,i))*dt;
     % S = (xT*Q*x)+(uT*R*u)
     J = S+J;
     JJJ(i+1,:)=J;
 end
 
display(J,'P_decent equals to')
