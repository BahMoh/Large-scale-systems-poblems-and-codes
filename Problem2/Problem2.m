clc
clear all 
close all

A=[0 1 0;
  100 -10 -300;
  0 0 -5];
B=[0 0.5 0;
    -0.05 0.5 7;
   0 0 0];
C=eye(3);
D=zeros(3);
%% check the controllabity 
p = ctrb(A,B);
rank(p)
if(rank(p)==length(A))
    disp('system  is controllable');
else disp('system is not controllable')
end 
%% check the stabilizable
sys = ss(A,B,C,D)
[p z]=pzmap(sys)
A1=-6.1803*eye-A;
P1=[A1 B];
rank(P1);
if(rank(P1)==length(A))
    disp('system  is stabilizable');
else disp('system is not stabilizable')
end 
%% Design LQR
Q = eye(3);
R = eye(3);
K = lqr(A,B,Q,R)
closed_loop_eig=eig(A-B*K)
%% check the observibility
P2=obsv(A,C);
if(rank(P2)==length(A))
 disp('the system is observable');
else disp('the system is not observable')
end
%% state responce of system
time = [0 5];

x0 = [1;0.5;0.8];

[t,x] = ode45(@Exercise2,time,x0);
x = x';
u = -K*x;
%% plot (states)
plot(t,x,'linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('States')
title(' Decentralized form for Problem2')
legend('x_1','x_2','x_3')
%% Plot (control signals)
figure
plot(t,u,'linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('Control Signals')
title(' Decentralized form for problem2')
legend('u_1','u_2')
%% New system
A_new=A-3*eye(3)
E=eye(3);
S=zeros(3);
[P_tild,L,K_new] = care(A_new,B,Q,R,S,E)
%% plot the state responce of system
time = [0 5];

x0_new = [1;0.5;0.8];

[t,x_new] = ode45(@Exercise2_New,time,x0_new);
x_new = x_new';
u_new = -K_new*x_new;

plot(t,x_new,'linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('States')
title(' Decentralized form for Problem2 New')
legend('x_1','x_2','x_3')
%% Plot (control signals)
figure
plot(t,u_new,'linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('Control Signals')
title(' Decentralized form for problem2 New')
legend('u_1','u_2')




