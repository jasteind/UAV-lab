%% Kalman for pitch
close all; clear all;
load sensors_c.mat
% Obtaining the sensor measurements
run pitch_roll.m
filterStart = a(end);
height_meas = ans.signal5.Data(filterStart+1:end); %% Height measurements

%Kalman init
dt = 0.005;
Q = eye(2)*0.001;
R = eye(1);
H = eye(1,2);
P = eye(2);

%Initialize states
phi_filtered = 1;
theta_filtered = 1;
height_filtered = 1;
height_rate_filtered = 1;
bias_x = 0;
bias_y = 0;
bias_h_rate = 0;

add_bias = 0; %Set to 1 to include bias term
if add_bias == 1
    bias_x = -0.0302; %mean of omega_x measurements from experiment C
    bias_y = -0.0194; %mean of the omega_y measurements from experiment C
    bias_h_rate = -9.844; %mean of the altidute measurements from experiment C
end

%Obtaining the 'input' measurements
om_x=ans.signal2.Data(filterStart+1:end,1);
om_y=ans.signal2.Data(filterStart+1:end,2);
z_acc=ans.signal1.Data(filterStart+1:end,3);

%System
Ad = [1 -dt;
     0   1];
Bd = [dt; 0];  
Cd = [1 0];
Dd = [0];
      
states = [theta_filtered; bias_y];
input = [om_y'];
states_pred = [];

y = [theta_meas'];

%Pitch rate, used as u(k)
K_save = [];
diff = [];
theta_pred(1)=0;
states_pred_saved = [1; 0];
for k=1:length(om_y)-1
        
    %Prediction using model
    states_pred=Ad*states(:,k)+Bd*input(:, k); %%Is dt correct
    states_pred_saved(:, k+1) = states_pred;
    P_pred=Ad*P*Ad'+Q;

    %Fusing with measures, optimal  K= 0.0437
    K = [0.5003; -0.0025]; %P_pred*H'*(H*P_pred*H'+R)^-1;
    states(:,k+1)=states_pred+K*(y(:, k+1)-H*states_pred);
    
end
z = tf('z', dt);
y2xf = K;
u2xf = (eye(2) - K*H)*(z*eye(2)- Ad)^(-1)*Bd;
trans_func = y2xf*u2xf(1);

output_tf = [];
for k=1:length(om_y)-1,   
    output_tf(k) = trans_func(1)*om_y(k);
end

figure(1)
plot(output_tf);

figure(5)   
plot(filterStart+1:length(ans.signal2.Time), y(1,:))
hold on;
grid on;

plot(filterStart+1:length(ans.signal2.Time), states(1,:), '--',  'Linewidth', 1)
hold off;

% title('\theta data vs \theta filtered')
% hold off;
% figure(6)
% plot(filterStart+1:length(ans.signal2.Time), states(1,:)-theta_meas')
% title('\theta filter vs \theta measured difference')

