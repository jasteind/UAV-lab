%% Kalman for pitch, 
close all; clear all;
load sensors_c.mat

bias = 0;
add_bias = 1; %Set to 1 to include bias term
if add_bias == 1,
    bias = -9.844; %mean of the altidute measurements from experiment C
end

altidude_meas = ans.signal5.Data; %% Height measurements
z_acc_u = ans.signal1.Data(:,3); %% Height input

%First timestep in model prediction
%System
%A = [1 0];

height_pred(1)=0;
dt=0.005;
Q= [0.01 0;
    0   0.01];
R= eye(2)*5;
H=[1 0];
P = eye(2);

height_filtered = [];
height_filtered(1) = 0;
K_save = [];
diff = [];

for k=1:length(z_acc_u)-1
        
    %Prediction using model
    height_pred(k+1)=height_filtered(k)+(1/2)*(z_acc_u(k)-bias)*dt^2; %%Subtract the g?
    P_pred(k+1)=P(k)+Q;

    %Fusing with measures, optimal K=0.014
    K=P_pred(k+1)*H'*(H*P_pred(k+1)*H'+R)^-1;
    K_save(k) = K;
    %diff(k) = (altidude_meas(k+1)-H*height_pred(k+1));
    height_filtered(k+1)=height_pred(k+1)+K*(altidude_meas(k+1)-H*height_pred(k+1));
    P(k+1)=P_pred(k+1)-K*H*P_pred(k+1);
end

figure(5)
plot(1:length(ans.signal5.Time), altidude_meas')
hold on;
grid on;
plot(1:length(ans.signal5.Time), height_filtered, 'Linewidth', 1.8)

title('Height data vs Height filtered')
hold off;
%figure(6)
%plot(filterStart+1:length(ans.signal2.Time), theta-theta_dat')
%title('Theta filter vs theta_meas difference')

