%nijmeir
clear all; clc; close all

load sensors_d.mat

%% Obtain the inclinometerdata for pitch
%compute the pitch and roll inclinometer data from the 
%accelerometer measurements, and identify these estimates as raw pitch and roll measurements.

%Pitch from acc data

figure(3)
subplot(2,1,1)
plot(ans.signal2.Time,ans.signal2.Data(:,2))
xlabel('Time [s]')
ylabel('Angle/sec [deg/s]')
title('\omega_y (pitch rate)')

%Discrete integration for pitch
dt = ans.signal2.Time(2)-ans.signal2.Time(1);
T = (0:dt:ans.signal2.Time(end))';

pitch_I = zeros(1,length(ans.signal2.Time))';

pitch_I(1) = 0;
for t=2:ans.signal2.Time(end)/dt,
    I = trapz(T(1:t), ans.signal2.Data(1:t,2));
    pitch_I(t) = I;
end
subplot(2,1,2)
grid on;
plot(ans.signal2.Time, pitch_I)
xlabel('Time [s]')
ylabel('Angle [deg]')
hold off;

%% Roll Yaw data
%Pitch from acc data

figure(5)
subplot(2,1,1)
plot(ans.signal2.Time,ans.signal2.Data(:,3))
xlabel('Time [s]')
ylabel('Angle/sec [deg/rad')
title('\omega_z (yaw rate)')

%Discrete integration for pitch
dt = ans.signal2.Time(2)-ans.signal2.Time(1);
T = (0:dt:ans.signal2.Time(end))';

yaw_I = zeros(1,length(ans.signal2.Time))';

yaw_I(1) = 0;
for t=2:ans.signal2.Time(end)/dt,
    I = trapz(T(1:t), ans.signal2.Data(1:t,3));
    yaw_I(t) = I;
end
hold on;
grid on;
plot(ans.signal2.Time, yaw_I)
hold off;