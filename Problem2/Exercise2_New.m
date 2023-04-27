function xp_new = Exercise2_New(t,x_new)
% Decentralized Form
A=[ -3     1     0;
   100   -13  -300;
     0     0    -8];
B=[0 0.5 0;
    -0.05 0.5 7
    ;0 0 0];
C=eye(3);
D=zeros(3);


K_dec =[-0.0473   -0.0043    0.0873;
    7.7158    0.5169  -13.3564;
    6.6282    0.6077  -12.2260];
u_new = -K_dec*x_new;
xp_new = A*x_new+B*u_new;
