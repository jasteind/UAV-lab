clear all; clc; close all;
syms K z K1 K2
dt=0.005;
I=eye(1);
Ad = 1;
Bd = dt; 
H=1;

tf1_pitch_roll=((z*I-Ad+K*H*Ad)^-1)*z*K
tf2_pitch_roll=((z*I-Ad+K*H*Ad)^-1)*(Bd-K*H*Bd)

I=eye(2);
Ad = [1  0 ; 
      dt 1 ];
Bd = [dt; 0]; 
H=[0 1];
Kn=[K1;K2];

tf1_h=((z*I-Ad+Kn*H*Ad)^-1)*z*Kn
tf2_h=((z*I-Ad+Kn*H*Ad)^-1)*(Bd-Kn*H*Bd)