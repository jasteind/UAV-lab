%% Kalman for roll
close all; clear all;
load sensors_e2.mat
load states_e2.mat
% Obtaining the sensor measurements
run pitch_roll.m
filterStart = a(end);

%Kalman init
dt = 0.005;
Q = eye(2)*0.000001;
R = eye(1);
H = eye(1,2);
P = eye(2);

%Initialize states
phi_filtered = 1;
bias_x = 1;

%Obtaining the 'input' measurements
om_x=ans.signal2.Data(filterStart+1:end,1);

%System
Ad = [1 -dt;
     0   1];
Bd = [dt; 0];  
      
states = [phi_filtered; bias_x];
input = [om_x'];
states_pred = [];
y = [phi_meas'];

%Roll rate, used as u(k)
for k=1:length(om_x)-1
        
    %Prediction using model
    states_pred=Ad*states(:,k)+Bd*input(:, k); 
    P_pred=Ad*P*Ad'+Q;

    %Fusing with measures
    K = P_pred*H'*(H*P_pred*H'+R)^-1;
    states(:,k+1)=states_pred+K*(y(:, k+1)-H*states_pred);
    P=P_pred-K*H*P_pred;
end

figure(1)
plot((filterStart+1:length(ans.signal2.Time))./200, phi_meas')
hold on;
grid on;
plot((filterStart+1:length(ans.signal2.Time))./200, states(1,:), 'Linewidth', 1.8)
plot((filterStart+1:length(state.time))./200, rad2deg(state.signals.values(filterStart+1:end,1)))
title('Roll Attitude Estimation')
legend('Measurement','Our Estimation','Simulink Model Estimation')
xlabel('seconds')
ylabel('deg')

figure(2)
plot((filterStart+1:length(ans.signal2.Time))./200, states(2,:), 'Linewidth', 1.8)
hold on;
grid on;
plot((filterStart+1:length(ans.signal2.Time))./200, mean(om_x,'omitnan')+states(2,:)*0)
title('Bias Estimation')
legend('Our Estimation','Input Mean')
xlabel('seconds')
ylabel('deg/s')

