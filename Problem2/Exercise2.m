function xp = Exercise2(t,x)
% Decentralized Form
A=[0 1 0;
    100 -10 -300;
    0 0 -5];
B=[0 0.5 0;
    -0.05 0.5 7
    ;0 0 0];
C=eye(3);
D=zeros(3);

K_dec =[-0.0824   -0.0066    0.1463;
   14.0549    0.8905  -23.5059;
   11.5409    0.9262  -20.4781];
u = -K_dec*x;

xp = A*x+B*u;

end