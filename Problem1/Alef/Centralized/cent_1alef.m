clc
clear all
close all

s = tf('s');

G11 = (0.9*s+15.4)/(s^2+9.163*s+15.47);
G12 = -0.01/(s+6.931);
G21 = 0.025/(s+2.232);
G22 = (0.7549*s+13.9)/((s+2)*(s+6));
G = [G11 G12;G21 G22];
Q = eye(6)
R = eye(2)

K_dc = dcgain(G)
[A,B,C,D] = ssdata(G)
K = lqr(A,B,Q,R)
% LQR Gain

tspan = [0 4];
x0 = [0.5;0.5;0.5;0.5;0.5;0.5];
[t,x] = ode45(@Centeralized_form1,tspan,x0);
% Solving with ODE45
x = x';
% For Dimensions
u = -K*x;
% State Feedback
Y = C*x;

plot(t,x,'linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('States')
title('Centralized form for first system')
legend('x_1','x_2','x_3','x_4','x_5','x_6')
% Plot (States)

figure

plot(t,u,'linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('Control Signals')
title('Centralized form for first system')
legend('u_1','u_2')
% Plot (Control Signal)

% Integral

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
 
display(J,'P_cen equals to')


