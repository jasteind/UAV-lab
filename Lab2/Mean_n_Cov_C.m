%% Mean and Covariance c
close all; clear
%use this just for A and B experiments
load sensors_c.mat
l=find(ans.signal5.time==5);
time=ans.signal5.Time(l:end);
height=ans.signal5.Data(l:end);
acc=ans.signal1.Data(l:end,:);
gyro=ans.signal2.Data(l:end,:);

rounded_signal=round(height);
p=find(rounded_signal==0);
height(p)=1+height(p);


Ma=[acc,gyro,height];
Ma=Ma(1:end-1,:);

cov=cov(Ma);
mean=mean(Ma,'omitnan');

