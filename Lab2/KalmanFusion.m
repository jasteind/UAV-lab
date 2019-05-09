%% Kalman for pitch
close all; clear all;
load sensors_d.mat
run pitch_roll.m

bias = 0;
add_bias = 1; %Set to 1 to include bias term

dt=0.005;
Q= [0.5, 0;
    0, 0.5];
R= [  5, 0;
      0, 5];
H=[1,0;
   0,1];
P = eye(2,2); %% obs

phi_pred(1)=0;
phi_filtered = [];
phi_filtered(1) = 1;
theta_pred(1)=0;
theta_filtered = [];
theta_filtered(1) = 1;
%Pitch rate, used as u(k)
filterStart = a(end);
om_x = ans.signal2.Data(filterStart+1:end,1);
om_y = ans.signal2.Data(filterStart+1:end,2);
om_z = ans.signal2.Data(filterStart+1:end,3);

if add_bias == 1
    bias_x = mean(om_x,'omitnan');
    bias_y = mean(om_y,'omitnan'); 
    bias_z = mean(om_z,'omitnan');
end
K_save = [];
diff = [];
for k=1:length(om_y)-1
        
    %Prediction using model
    theta_pred(k+1)=theta_filtered(k)+dt*((om_y(k)-bias_y)*cosd(phi_filtered(k))-(om_z(k)-bias_z)*cosd(phi_filtered(k))); %%IS dt correct
    phi_pred(k+1)=phi_filtered(k)+dt*((om_x(k)-bias_x)+(om_y(k)-bias_y)*sind(phi_filtered(k))*tand(theta_filtered(k))+(om_z(k)-bias_z)*cosd(phi_filtered(k))*tand(theta_filtered(k)));
    P_pred=P+Q;

    %Fusing with measures, optimal  K= 0.0437
    K = P_pred*H'*(H*P_pred*H'+R)^-1;
%     K_save(k) = K;

    theta_filtered(k+1)=theta_pred(k+1)+K(1,1)*(theta_meas(k+1)-H(1,1)*theta_pred(k+1));
    phi_filtered(k+1)=phi_pred(k+1)+K(2,2)*(phi_meas(k+1)-H(2,2)*phi_pred(k+1));
    P=P_pred-K*H*P_pred;
end

figure(5)
plot(filterStart+1:length(ans.signal2.Time), theta_meas')
hold on;
grid on;
plot(filterStart+1:length(ans.signal2.Time), theta_filtered, 'Linewidth', 1.8)

title('\theta data vs \theta filtered')
hold off;
figure(6)
plot(filterStart+1:length(ans.signal2.Time), phi_meas')
hold on;
grid on;
plot(filterStart+1:length(ans.signal2.Time), phi_filtered, 'Linewidth', 1.8)

title('\phi data vs \phi filtered')
hold off;
