%% Kalman for pitch
close all; clear all;
load sensors_b.mat
run pitch_roll.m
filterStart = a(end);
height_meas = ans.signal5.Data(filterStart+1:end); %% Height measurements

bias = 0;
add_bias = 1; %Set to 1 to include bias term
if add_bias == 1
    bias = -0.0194; %mean of the omega_y measurements from experiment C
end

theta_pred(1)=0;
dt = 0.005;
Q= [0.0001 0;
    0   0.001];
R= eye(1)*5;
H = eye(3,7);
P = eye(2);

%Initialize states
phi_filtered = 1;
theta_filtered = 1;
height_filtered = 1;
height_rate_filtered = 1;


%Obtaining the 'input' measurements
om_x=ans.signal2.Data(filterStart+1:end,1);
om_y=ans.signal2.Data(filterStart+1:end,2);
z_acc=ans.signa1.Data(filterStart+1:end,3);

%System
Ad = [1 0 0 0 -dt 0  0;
     0  1 0  0 -dt 0;
     0  0 1 dt 0 0 -0.5*dt^2;
     0  0 0 0  1 0 0;
     0  0 0 0  0 1 0;
     0  0 0 0  0 0 1];
Bd = [dt 0  0;
      0  dt 0;
      0  0  0.5*dt^2;
      0  0  dt;
      0  0  0;
      0  0  0;
      0  0  0];
  
states = [phi_filtered; theta_filtered; height_filtered; height_rate_filtered; bias_x; bias_y; bias_h_rate];

input = [om_x; om_y; z_acc];
states_pred = [];

y = [phi_meas; theta_meas; height_meas];


%Pitch rate, used as u(k)
K_save = [];
diff = [];
for k=1:length(om_y)-1
        
    %Prediction using model
    states_pred=Ad*states(:,k)+Bd*input(k); %%Is dt correct
    P_pred=Ad*P*Ad'+Q;

    %Fusing with measures, optimal  K= 0.0437
    K = P_pred*H'*(H*P_pred*H'+R)^-1;
    K_save(:,k) = K;
    K(2) = 0;
    states(:,k+1)=states_pred+K*(y(:, k+1)-H*states_pred);
    P=P_pred-K*H*P_pred;
end

figure(5)
plot(filterStart+1:length(ans.signal2.Time), theta_meas')
hold on;
grid on;
plot(filterStart+1:length(ans.signal2.Time), states(1,:), 'Linewidth', 1.8)

% title('\theta data vs \theta filtered')
% hold off;
% figure(6)
% plot(filterStart+1:length(ans.signal2.Time), states(1,:)-theta_meas')
% title('\theta filter vs \theta measured difference')

