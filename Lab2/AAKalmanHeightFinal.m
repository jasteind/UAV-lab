%% Kalman for Height
close all; clear all;
load sensors_e3.mat
load states_e3.mat
% Obtaining the sensor measurements
run pitch_roll.m
filterStart = a(end);
height_meas = ans.signal5.Data(filterStart+1:end); %% Height measurements

%Kalman init
dt = 0.005;
Q = eye(3)*0.00001;
R = eye(1);
H = [0 1 0];
P = eye(3);

%Initialize states
h_filtered = 0;
v_filtered = 0;
bias_h_rate= 1;

%Obtaining the 'input' measurements
z_acc=-(ans.signal1.Data(filterStart+1:end,3)+9.81);

%System
Ad = [1  0  -dt;
      dt 1   0; 
      0 0   1];
Bd = [dt; 0; 0];  
      
states = [v_filtered; h_filtered; bias_h_rate];
input = [z_acc'];
states_pred = [];
y = [height_meas'];

%z_acc, used as u(k)
for k=1:length(z_acc)-1
        
    %Prediction using model
    states_pred=Ad*states(:,k)+Bd*input(:, k); 
    P_pred=Ad*P*Ad'+Q;

    %Fusing with measures
    K = P_pred*H'*(H*P_pred*H'+R)^-1;
    states(:,k+1)=states_pred+K*(y(:, k+1)-H*states_pred);
    P=P_pred-K*H*P_pred;
end

figure(1)
plot((filterStart+1:length(ans.signal2.Time))./200, height_meas')
hold on;
grid on;
plot((filterStart+1:length(ans.signal2.Time))./200, states(2,:), 'Linewidth', 1.8)
plot((filterStart+1:length(state.time))./200, state.signals.values(filterStart+1:end,9))
title('Height Estimation')
legend('Measurement','Our Estimation','Simulink Model Estimation')
xlabel('seconds')
ylabel('m')

figure(2)
plot((filterStart+1:length(ans.signal2.Time))./200, states(3,:), 'Linewidth', 1.8)
hold on;
grid on;
plot((filterStart+1:length(ans.signal2.Time))./200, mean(z_acc,'omitnan')+states(2,:)*0)
title('Bias Estimation')
legend('Our Estimation','Input Mean')
xlabel('seconds')
ylabel('m/s^2')

