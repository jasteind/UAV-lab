function theta_filtered =Kalman_Pitch(phys_gyro,phys_acc)
%Kalman for pitch
% at each sampling time we have a value for om_y and the physical
% accelerations
%refers to phys_acc the equality depends on the way it is introduced into
%the function
persistent init_flag theta_filtered_prev P_prev K
om_y = phys_gyro(2);
x_acc = phys_acc(1);
y_acc = phys_acc(2);
z_acc = phys_acc(3);

Q = 0.01;
R = 5;
H = 1;

dt = 0.005;
if isempty(init_flag)
    %Q and R matrices
    theta_filtered_prev = 0;
    P_prev = eye(1);
    
    init_flag = 1;
end 

if z_acc==0
    theta_filtered=0;
else

phi_meas = atand (y_acc./z_acc);
theta_meas =  atand (-x_acc.*cosd(phi_meas)./z_acc);



%Prediction using model
    theta_pred=theta_filtered_prev+dt*(om_y); %%IS dt correct
    P_pred=P_prev+Q;

    %Fusing with measures
    K=P_pred*H'*(H*P_pred*H'+R)^-1;
    
    theta_filtered=theta_pred+K*(theta_meas-H*theta_pred);
    P_prev=P_pred-K*H*P_pred;

    theta_filtered_prev = theta_filtered;
    
end

end