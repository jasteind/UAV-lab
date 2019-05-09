close all
%% Kalman for pitch
load sensors_b.mat
run pitch_roll.m
theta_pred(1)=0;
dt=0.005;
Q= 0.01;
R= 5;
H=1;
P = eye(1);

theta_filtered = [];
theta_filtered(1) = -0.5;
% Pitch rate, used as u(k)
filterStart = a(end);
om_y=ans.signal2.Data(filterStart+1:end,2);

%Obtaining bias
bias = mean(om_y); 

K_save = [];
diff = [];
for k=1:length(om_y)-1
        
    %Prediction using model
    theta_pred(k+1)=theta_filtered(k)+dt*(om_y(k)-bias);
    P_pred(k+1)=P(k)+Q;

    %Fusing with measures
    K=P_pred(k+1)*H'*(H*P_pred(k+1)*H'+R)^-1;
    K_save(k) = K;
    diff(k) = (theta_dat(k+1)-H*theta_pred(k+1));
    theta_filtered(k+1)=theta_pred(k+1)+K*(theta_meas(k+1)-H*theta_pred(k+1));
    %Based on 'optimal gain'?
    %P(k+1)=P_pred(k+1)-K*H*P_pred(k+1);
    %Based on 'sub-optimal gain'
    P(k+1)=(eye(1)-K)*P_pred(k)*(eye(1)-K)'+K*R*K';
    
end

figure(5)
plot(filterStart+1:length(ans.signal2.Time), theta_meas')
hold on;
grid on;
plot(filterStart+1:length(ans.signal2.Time), theta_filtered, 'Linewidth', 1.8)

title('\theta data vs \theta filtered')
hold off;
mean(theta_filtered)
%figure(6)
%plot(filterStart+1:length(ans.signal2.Time), theta-theta_dat')
%title('Theta filter vs theta_meas difference')

