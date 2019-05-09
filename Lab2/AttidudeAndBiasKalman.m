%% Task 5
%close all;
clear all;
%Obtaining measurements
load sensors_e3.mat

%Obtaining roll, pitch and yaw 
Om = ans.signal2.Data(:,1:3)';

%Obtaining measurements from the accelerometer
y_meas = ans.signal1.Data(:,1:3)';
y_meas(3,:) = y_meas(3,:)+9.81; 
y_meas = -y_meas;

%Kalman init
Q = zeros(6);
Q(1:3, 1:3) = 0.05*eye(3);
Q(4:6, 4:6) = 10^(-8);
R = 1;
P = eye(6);

%System init
h1_3 = [1 0 0;
        0 1 0;
        0 0 1;];
H = [h1_3 zeros(3)];
A_skew = zeros(6);
X = ones(6,1);
Y = zeros(3,1);
dt = 0.005;
P_save = [];
%For the ss
B = eye(6);
C = eye(1, 6);
D = 0;
startTime = 0;%1100;
for k = 1:length(Om)-1,
    
    %Refreshing the A matrix
    A_skew(1:3, 1:3) = skew(Om(:, k+startTime));
    A_skew(1:3, 4:6) = skew(Y(:, k));
    
    %Converting to discreete form
    A_d = expm(A_skew * dt);
    
    %Kalman update
    P_dot = A_d*P+P*A_d'+Q-P*H'*R^(-1)*H*P;
    P = P+dt*P_dot;
    
    K = P*H'*R^(-1);
    
    %Model prediction
    X = X-dt*A_d*X;
    %Obtaining a posteriori using K
    X = X + K*(y_meas(:, k+1)-H*X);
    Y(:, k+1) = X(1:3);
    bias(:, k+1) = X(4:6);
end

t = 0:dt:dt*(length(Om)-(+startTime+1));
figure(1)
ax1 = subplot(3,1,1);
plot(t, Y(1, :))
title('a_x')
ylabel('m/s^2')
grid on;
xlabel('Time [s]')

ax2 = subplot(3,1,2);
plot(t, Y(2, :))
title('a_y')
ylabel('m/s^2')
grid on;
xlabel('Time [s]')

ax3 = subplot(3,1,3);
plot(t, Y(3, :))
title('a_z')
ylabel('m/s^2')
xlabel('Time [s]')
grid on;

% figure(2)
% ax1 = subplot(3,1,1);
% plot(t, bias(1, :));
% title('Gyro bias in the x-axis')
% ylabel('deg')
% xlabel('Time [s]')
% 
% ax2 = subplot(3,1,2);
% plot(t, bias(2, :));
% title('Gyro bias in the y-axis')
% ylabel('deg')
% grid on;
% xlabel('Time [s]')
% 
% ax3 = subplot(3,1,3);
% plot(t, bias(3, :));
% title('Gyro bias in the z-axis')
% ylabel('deg')
% grid on;
% xlabel('Time [s]')