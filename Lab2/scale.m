clear all; clc
load sensors_a.mat
b=find(ans.signal2.time==21.8);
e=find(ans.signal2.time==23.8);
time=ans.signal2.time;
dt=time(2)-time(1);
yaw_rate=ans.signal2.Data(b:e,3);

I=trapz(dt,yaw_rate)
