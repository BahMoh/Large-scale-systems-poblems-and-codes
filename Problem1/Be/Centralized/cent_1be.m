clc
clear all
close all

s = tf('s');

G11=(-0.805/((0.3*s+1)*(1.6*s+1)));
G12=(0.055/((2.76*s+1)*(1.25*s+1)));
G21=(0.465/(1.3*s+1));
G22=(0.055/(3.3*s+1));

G = [G11 G12;G21 G22];
Q = eye(6)
R = eye(2)

K_dc = dcgain(G)
[A,B,C,D] = ssdata(G)
K = lqr(A,B,Q,R)
% LQR Gain

tspan = [0 20];
x0 = [0.5;0.5;0.5;0.5;0.5;0.5];
[t,x] = ode45(@Centeralized_form2,tspan,x0);
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
title('Centralized form for second system')
legend('x_1','x_2','x_3','x_4','x_5','x_6')
% Plot (States)

figure

plot(t,u,'linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('Control Signals')
title('Centralized form for second system')
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
