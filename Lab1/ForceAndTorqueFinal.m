clear all
clc
%% Forces Vector Expression
% imput vector u=[uw u1 u2 u3]'
syms f1 f2 f3 f4 uw u1 u2 u3 u4 d c t th psi phi m g real
%input vector
u=[uw;u1;u2;u3];
%gravity acceleration vector
ag=[0;0;g];
%rotation matrix from B to I CHECKED ONCE
R=[cos(th)*cos(psi)   sin(phi)*sin(th)*cos(psi)-cos(phi)*sin(psi)   cos(phi)*sin(th)*cos(psi)+sin(phi)*sin(psi);
   cos(th)*sin(phi)   sin(phi)*sin(th)*sin(psi)+cos(phi)*cos(psi)   cos(phi)*sin(th)*sin(psi)-sin(phi)*cos(psi);
   -sin(th)           sin(phi)*cos(th)                              cos(phi)*cos(th)];

%finding the expression of fr as function of input CHECKED ONCE
M=[       -1        -1        -1        -1;
   -d*sin(t)  d*sin(t)  d*sin(t) -d*sin(t);
    d*sin(t)  d*sin(t) -d*sin(t) -d*sin(t);
           c        -c         c       -c];
M_inv=M^-1;

disp('t=45 degrees');

fr=M^-1*u;
%the function collect, gives as result the same vector  but collecting the
%parameter given as an input and their coefficient
fr=collect(fr,[uw,u1,u2,u3])
%analizing the force vector
Mv=[ 0  0  0  0;
     0  0  0  0;
    -1 -1 -1 -1];
x=m*R'*ag;
y=Mv*fr;

f=x+y

%% Torques Vector Expression
Mw=[-d*sin(t)  d*sin(t)  d*sin(t) -d*sin(t);
     d*sin(t)  d*sin(t) -d*sin(t) -d*sin(t);
           c        -c         c        -c];
n=Mw*fr;
%the function collect, gives as result the same vector  but collecting the
%parameter given as an input and their coefficient
n=collect(n,[uw,u1,u2,u3])
%latex codes for variables
latex(fr)
% latex(f)
% latex(n)