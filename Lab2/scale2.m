clear all;clc
load sensors_b.mat
ans.signal1.Data=ans.signal1.Data/100;
ans.signal5.Data=ans.signal5.Data.*10^42;
save sensors_b.mat