clear all
%%% Linearization
%% Symbols
syms x0 y0 z0 psi0 m g real
syms x y z phi theta psi u v w p q r real 
syms Jx Jy Jz Jxz real
syms fx fy fz l m n real
syms uw up uq ur real

%Last three eq
J=[Jx  0  0;
    0 Jy  0;
    0  0 Jz];
oM=[ 0  r -q;
    -r  0  p;
     q -p  0];
om=[p q r]';
lmn=[l m n]';
eq=J^-1*(oM*J*om+lmn)
 

%% Defining the states and input vectors
states = [x y z phi theta psi u v w p q r];
input = [uw up uq ur];

%% Defining the equlibrium point for the states and input
statesEq = [x0 y0 z0 0 0 psi0 0 0 0 0 0 0];
inputEq = [m*g 0 0 0];
%% Defining the functions for the 12 equation of motion for the quadracopter
f1 = u*cos(theta)*cos(psi) + v*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) + w*(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi));
f2 = u*cos(theta)*sin(psi) + v*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) + w*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi));
f3 = -u*sin(theta) + v*(sin(phi)*cos(theta)) + w*cos(phi)*cos(theta);
f4 = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
f5 = 0 + q*cos(phi) - r*sin(phi);
f6 = 0 + q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
f7 = r*v-q*w + fx*1/m;
f8 = p*w-r*u + fy*1/m;
f9 = q*u-p*v + fz*1/m;
f10 =(l + Jy*q*r - Jz*q*r)/Jx ;
f11 =(m - Jx*p*r + Jz*p*r)/Jy ;
f12 = (n + Jx*p*q - Jy*p*q)/Jz;

%% Inserting the expression for the forces f and m
f7 = subs(f7, fx, -g*m*sin(theta));
f8 = subs(f8, fy, g*m*cos(theta)*sin(phi));
f9 = subs(f9, fz, uw + g*m*cos(phi)*cos(theta));
f10 = subs(f10, l, up);
f11 = subs(f11, m, uq);
f12 = subs(f12, n, ur);

%% Defining g(x,u)
g = [f1 f2 f3 f4 f5 f6 f7 f8 f9 f10 f11 f12]';

%% Obtaining the jacobian of g(x,u) based on the states and input
statesJac = jacobian(g, states);
inputJac = jacobian(g, input);

%% Inserting the equlibrium point for the states and input

A = subs(statesJac, [states], [statesEq]);
B = subs(inputJac, [input], [inputEq]);


%% TODO
%Analyze controlability
%Insert values for the constants
%...