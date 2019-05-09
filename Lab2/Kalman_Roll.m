function phi_filtered =Kalman_Roll(phys_gyro,phys_acc)
%Kalman for roll
% at each sampling time we have a value for om_x and the physical
% accelerations
%refers to phys_acc the equality depends on the way it is introduced into
%the function
persistent init_flag phi_filtered_prev P_prev K
om_x = phys_gyro(1);
x_acc = phys_acc(1);
y_acc = phys_acc(2);
z_acc = phys_acc(3);

Q = 0.01;
R = 5;
H = 1;

dt = 0.005;
if isempty(init_flag)
    %Q and R matrices
    phi_filtered_prev = 0;
    P_prev = eye(1);
    
    init_flag = 1;
end 

if z_acc==0
    phi_filtered=0;
else

phi_meas = atand (y_acc./z_acc);
theta_meas =  atand (-x_acc.*cosd(phi_meas)./z_acc);



%Prediction using model
    phi_pred=phi_filtered_prev+dt*(om_x); %%IS dt correct
    P_pred=P_prev+Q;

    %Fusing with measures
    K=P_pred*H'*(H*P_pred*H'+R)^-1;
    
    phi_filtered=phi_pred+K*(phi_meas-H*phi_pred);
    P_prev=P_pred-K*H*P_pred;

    phi_filtered_prev = phi_filtered;
    
end

end
