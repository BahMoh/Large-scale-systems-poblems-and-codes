clc
clear all
close all
s = tf('s');
%% 
G11 = (0.9*s+15.4)/(s^2+9.163*s+15.47);
G12 = -0.01/(s+6.931);
G21 = 0.025/(s+2.232);
G22 = (0.7549*s+13.9)/((s+2)*(s+6));
G = [G11 G12;G21 G22]

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

Q11 = [3.25 1.75;1.75 1.25]
Q22 = [4.0021 1.9670;1.9670 1.3115]
R = eye(1)
A11 = [-6.9310 0;0 -2.2320]
B11 = [-1.7025;1.7025]
A22 = [-6 0;0 -2]
B22 = [-2;2]

K11 = lqr(A11,B11,Q11,R)
K22 = lqr(A22,B22,Q22,R)

K_jordan = [K11 0 0 0 0;...
            0 0 0 0 K22]
K_dec = K_jordan*inv(T)
%%
tspan = [0 8];
x0 = [0.5;0.5;0.5;0.5;0.5;0.5];
[t,x] = ode45(@Decentralized_form1,tspan,x0);
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
title(' Decentralized form for first system')
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

 for i = 1:70
     dt=t(i)-tt; 
     tt=t(i);
     S = (xT(i,:)*Q*x(:,i))*dt;
     % S = (xT*Q*x)+(uT*R*u)
     J = S+J;
     JJJ(i+1,:)=J;
 end
 
display(J,'P_decent equals to')



