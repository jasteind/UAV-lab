close all;clear all;
%Obtaining measurements
load sensors_d.mat

%Obtaining roll, pitch and yaw 
Om = ans.signal2.Data(:,1:3)';

%Obtaining measurements from the accelerometer
y_meas = ans.signal1.Data(:,1:3)';
y_meas(3,:) = y_meas(3,:)+9.81; 
y_meas = -y_meas;

%Kalman init
Q = zeros(6);
Q(1:3, 1:3) = 0.0005*eye(3);
Q(4:6, 4:6) = 0.01*eye(3);
R = 0.05*eye(3);
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
P = eye(1);
P_save = [];
K_save = [];

for k = 1:length(Om)-1,
    
    %Refreshing the A matrix
    A_skew(1:3, 1:3) = skew(Om(:, k));
    A_skew(1:3, 4:6) = skew(Y(:, k));
    
    %Converting to discreete form
    A_d = expm(A_skew * dt);
    
    %Kalman update
    P_pred=A_d*P*A_d'+Q;

    %Fusing with measures
    K = P_pred*H'*(H*P_pred*H'+R)^-1;
    %P_save(:, :, k+1) = P;
    
    %Model prediction
    X = X-dt*A_d*X;
    %Obtaining a posteriori using K
    X = X + K*(y_meas(:, k+1)-H*X);
    K_save(:, :, k+1) = K;
    Y(:, k+1) = X(1:3);
    bias(:, k+1) = X(4:6);
    
    P=P_pred-K*H*P_pred;
end

t = 0:dt:dt*(length(Om)-1);

figure(3)
ax1 = subplot(3,1,1);
plot(t, Y(1, :), 'LineWidth', 2)
hold on
plot1 = plot(t, y_meas(1,:));
plot1.Color(4) = 0.35;
legend(ax1, {'Filtered acceleration', 'Measured acceleration'}, 'Location', 'southeast');
hold off
title('a_x')
ylabel('m/s^2')
grid on;
xlabel('Time [s]')

ax2 = subplot(3,1,2);
plot(t, Y(2, :), 'LineWidth', 2);
hold on
plot1 = plot(t, y_meas(2,:));
plot1.Color(4) = 0.35;
legend(ax2, {'Filtered acceleration', 'Measured acceleration'}, 'Location', 'southeast');
hold off
title('a_y')
ylabel('m/s^2')
grid on;
xlabel('Time [s]')

ax3 = subplot(3,1,3);
plot(t, Y(3, :), 'LineWidth', 1);
hold on
plot1 = plot(t, y_meas(3,:));
plot1.Color(4) = 0.80;
legend(ax3, {'Filtered acceleration', 'Measured acceleration'}, 'Location', 'southeast');
hold off
title('a_z')
ylabel('m/s^2')
xlabel('Time [s]')
grid on;

figure(4)
ax1 = subplot(3,1,1);
plot(t, bias(1, :));
title('Gyro bias_x')
ylabel('deg')
grid on;
xlabel('Time [s]')

ax2 = subplot(3,1,2);
plot(t, bias(2, :));
title('Gyro bias_y')
ylabel('deg')
grid on;
xlabel('Time [s]')

ax3 = subplot(3,1,3);
plot(t, bias(3, :));
title('Gyro bias_z')
ylabel('deg')
grid on;
xlabel('Time [s]')