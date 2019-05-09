% Mean and Covariance A&B
close all; clear
%use this just for A and B experiments
load sensors_b.mat

Ma=[ans.signal1.Data,ans.signal2.Data,ans.signal5.Data];

cov=cov(Ma);
mean=mean(Ma);
