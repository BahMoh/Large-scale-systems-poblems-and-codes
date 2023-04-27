function xp = Centeralized_form1(t,x)
% Centralized Form

s = tf('s');
G11 = (0.9*s+15.4)/(s^2+9.163*s+15.47);
G12 = -0.01/(s+6.931);
G21 = 0.025/(s+2.232);
G22 = (0.7549*s+13.9)/((s+2)*(s+6));
G = [G11 G12;G21 G22];

[A,B,C,D] = ssdata(G)


Q = eye(6);
R = eye(2);

K = lqr(A,B,Q,R);
u = -K*x;
xp = A*x+B*u;