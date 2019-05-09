%% Extended Kalman
close all; 
clear all;
load sensors_e1.mat
acc=ans.signal1.Data(1:end,:);
R=cov(acc);
run pitch_roll.m

dt=0.005;
Q= eye(2)*0.001;

P = eye(2,2);

phi_filtered(1) = 1;
theta_filtered(1) = 1;

%Pitch rate, used as u(k)
filterStart = a(end);
om_x = ans.signal2.Data(filterStart+1:end,1);
om_y = ans.signal2.Data(filterStart+1:end,2);
om_z = ans.signal2.Data(filterStart+1:end,3);

x=[0;0];
for k=1:length(om_y)-1
        
    %Prediction using model x=(phi,theta)
    x=x+dt*[om_x(k)+om_y(k)*sind(phi_filtered(k))*tand(theta_filtered(k))+om_z(k)*cosd(phi_filtered(k))*tand(theta_filtered(k));
            om_y(k)*cosd(phi_filtered(k))-om_z(k)*sind(phi_filtered(k))];
           

    A=[om_y(k)*cosd(x(1))*tand(x(2))-om_z(k)*sind(x(1))*tand(x(2)), (om_y(k)*sind(x(1))-om_z(k)*cosd(x(1)))/(cosd(x(2)))^2;
       -om_y(k)*sind(x(1))-om_z(k)*cosd(x(1)),0]; 
    P=P+dt*(A*P+P*A'+Q);

    %Fusing with measures
    h=9.81*[-sind(x(2));
        cosd(x(2))*sind(x(1));
        cosd(x(2))*cosd(x(1))];
    
    H=9.81.*[0, -cosd(x(2));
       cosd(x(2))*cosd(x(1)), -sind(x(1))*sind(x(2));
        -sind(x(1))*cosd(x(2)), -cosd(x(1))*sind(x(2))];
    
    K = P*H'*(H*P*H'+R)^-1;

    P=P-K*H*P;
    y=-[x_acc(k+1);y_acc(k+1);z_acc(k+1)];
    x=x+K*(y-h);
    theta_filtered(k+1) = x(2);
    phi_filtered(k+1) = x(1);
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
hold off
