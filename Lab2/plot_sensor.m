clear all; clc; close all
%change letter for plot different experiments
load sensors_a.mat


figure
plot(ans.signal1)
title('phys_acc')
figure
plot(ans.signal2)
title('phys_gyros')
figure
plot(ans.signal3)
title('altitude_vision')
figure
plot(ans.signal4)
title('altitude_vz')
figure
plot(ans.signal5)
title('altitude_raw')
figure
plot(ans.signal6)
title('heading_unwrapped')
figure
plot(ans.signal7)
title('heading_gyro_unwrapped')
figure
plot(ans.signal8)
title('heading_fusion_anwrapped')
figure
plot(ans.signal9)
title('magneto_radius')
figure
plot(ans.signal10)
title('est_z')
figure
plot(ans.signal11)
title('est_zdot')
figure
plot(ans.signal12)
title('theta_a')
figure
plot(ans.signal13)
title('phi_a')
figure
plot(ans.signal14)
title('mx')
figure
plot(ans.signal15)
title('my')
figure
plot(ans.signal16)
title('mz')

