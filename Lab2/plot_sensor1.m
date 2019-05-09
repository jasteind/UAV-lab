clear all; clc; close all
%change letter for plot different experiments
load sensors_d.mat


figure
subplot(3,1,1)
plot(ans.signal2.Time,ans.signal1.Data(:,1))
title('a_x')
xlabel('seconds')
ylabel('m/s^2')
subplot(3,1,2)
plot(ans.signal2.Time,ans.signal1.Data(:,2))
title('a_y')
xlabel('seconds')
ylabel('m/s^2')
subplot(3,1,3)
plot(ans.signal2.Time,ans.signal1.Data(:,3))
title('a_z')
xlabel('seconds')
ylabel('m/s^2')

figure
subplot(3,1,1)
plot(ans.signal2.Time,ans.signal2.Data(:,1))
title('\omega_x (roll rate)')
xlabel('seconds')
ylabel('deg')
subplot(3,1,2)
plot(ans.signal2.Time,ans.signal2.Data(:,2))
title('\omega_y (pitch rate)')
xlabel('seconds')
ylabel('deg')
subplot(3,1,3)
plot(ans.signal2.Time,ans.signal2.Data(:,3))
title('\omega_z (yaw rate)')
xlabel('seconds')
ylabel('deg')
% figure
% plot(ans.signal3)
% title('altitude_vision')
% figure
% plot(ans.signal4)
% title('altitude_vz')
figure
plot(ans.signal5.Time,ans.signal5.Data)
title('altitude raw')
xlabel('seconds')
ylabel('m')
% figure
% plot(ans.signal6)
% title('heading_unwrapped')
% figure
% plot(ans.signal7)
% title('heading_gyro_unwrapped')
% figure
% plot(ans.signal8)
% title('heading_fusion_anwrapped')
% figure
% plot(ans.signal9)
% title('magneto_radius')
% figure
% plot(ans.signal10)
% title('est_z')
% figure
% plot(ans.signal11)
% title('est_zdot')
% figure
% plot(ans.signal12)
% title('theta_a')
% figure
% plot(ans.signal13)
% title('phi_a')
% figure
% plot(ans.signal14)
% title('mx')
% figure
% plot(ans.signal15)
% title('my')
% figure
% plot(ans.signal16)
% title('mz')

