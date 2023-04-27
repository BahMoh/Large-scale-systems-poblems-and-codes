function xp = Centeralized_form2(t,x)
% Centralized Form

s = tf('s');
G11=(-0.805/((0.3*s+1)*(1.6*s+1)));
G12=(0.055/((2.76*s+1)*(1.25*s+1)));
G21=(0.465/(1.3*s+1));
G22=(0.055/(3.3*s+1));

G = [G11 G12;G21 G22];

[A,B,C,D] = ssdata(G)


Q = eye(6);
R = eye(2);

K = lqr(A,B,Q,R);
u = -K*x;
xp = A*x+B*u;