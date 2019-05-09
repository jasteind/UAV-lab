%% Kalman for pitch
close all; clear all;
load sensors_e1.mat
acc=ans.signal1.Data(1:end,1:2);
R = eye(4,4);
R(1:2,1:2)=cov(acc);
run pitch_roll.m
 %Set to 1 to include bias term

dt=0.005;
Q= [  0.1,   0, 0, 0;
      0,   0.1, 0, 0;
      0,   0, .05, 0;
      0,   0, 0, .05];

P = eye(4,4); %% obs

phi_filtered(1) = 1;
theta_filtered(1) = 1;
%Pitch rate, used as u(k)
filterStart = a(end);
om_x = ans.signal2.Data(filterStart+1:end,1);
om_y = ans.signal2.Data(filterStart+1:end,2);
om_z = ans.signal2.Data(filterStart+1:end,3);


    bias_x_filtered = 1;
    bias_y_filtered = 1; 

x=[0;0;0;0];
for k=1:length(om_y)-1
        
    %Prediction using model x=(phi,theta)
    x=x+dt*[(om_x(k)-bias_x_filtered(k))+(om_y(k)-bias_y_filtered(k))*sind(phi_filtered(k))*tand(theta_filtered(k))+om_z(k)*cosd(phi_filtered(k))*tand(theta_filtered(k));
            (om_y(k)-bias_y_filtered(k))*cosd(phi_filtered(k))-om_z(k)*sind(phi_filtered(k));
            0;
            0];

    A=[om_y(k)*cosd(x(1))*tand(x(2))-om_z(k)*sind(x(1))*tand(x(2)), (om_y(k)*sind(x(1))-om_z(k)*cosd(x(1)))/(cosd(x(2)))^2, -1, -sind(phi_filtered(k))*tand(theta_filtered(k));
       -om_y(k)*sind(x(1))-om_z(k)*cosd(x(1)) , 0, 0,-cosd(phi_filtered(k));
       0,0,0,0;
       0,0,0,0];  
    P=P+dt*(A*P+P*A'+Q);

    %Fusing with measures, optimal  K= 0.0437
    h=-9.81*[-sind(x(2));
        cosd(x(2))*sind(x(1));
        0;
        0];
    
    H=-9.81.*[0, -cosd(x(2)),0,0;
              cosd(x(2))*cosd(x(1)), -sind(x(1))*sind(x(2)),0,0;
              0,0,0,0;
              0,0,0,0];
    
    K = P*H'*(H*P*H'+R)^-1;

    P=P-K*H*P;
    y=[x_acc(k+1);y_acc(k+1);0;0];
    x=x+K*(y-h);
    theta_filtered(k+1) = x(2);
    phi_filtered(k+1) = x(1);
    bias_x_filtered(k+1) = x(3);
    bias_y_filtered(k+1) = x(4);
end
load states_e1.mat

figure(5)
plot((filterStart+1:length(ans.signal2.Time))./200, theta_meas')
hold on;
grid on;
plot((filterStart+1:length(ans.signal2.Time))./200, theta_filtered, 'Linewidth', 1.8)
plot((1:length(state.time))./200, rad2deg(state.signals.values(:,2)));
title('Pitch Attitude Estimation')
legend('Measurement','Our Estimation','Simulink Model Estimation')
xlabel('seconds')
ylabel('deg')

hold off;
figure(6)
plot((filterStart+1:length(ans.signal2.Time))./200, phi_meas')
hold on;
grid on;
plot((filterStart+1:length(ans.signal2.Time))./200, phi_filtered, 'Linewidth', 1.8)
plot((1:length(state.time))./200, rad2deg(state.signals.values(:,1)));
title('Roll Attitude Estimation')
legend('Measurement','Our Estimation','Simulink Model Estimation')
xlabel('seconds')
ylabel('deg')


figure(7)
hold on;
grid on;
plot((filterStart+1:length(ans.signal2.Time))./200, bias_x_filtered, 'Linewidth', 1.8)
plot((filterStart+1:length(ans.signal2.Time))./200, mean(om_x,'omitnan')+bias_x_filtered*0);
title('Bias Estimation')
legend('Our Estimation','\omega_x Mean')
xlabel('seconds')
ylabel('deg/sec')

figure(8)
hold on;
grid on;
plot((filterStart+1:length(ans.signal2.Time))./200, bias_y_filtered, 'Linewidth', 1.8)
plot((filterStart+1:length(ans.signal2.Time))./200, mean(om_y,'omitnan')+bias_y_filtered*0);
title('Bias Estimation')
legend('Our Estimation','\omega_y Mean')
xlabel('seconds')
ylabel('deg/sec')

hold off;
