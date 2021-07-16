%% AEROSPACE CONTROL SYSTEMS %%

clc, clear, close all

run Task1
run Task2

%% Pid controller
kp_p = 3;
kp_i = 1;
kp_d = 1;
Tp = 1;

kphi_p = 1;

R_p = pid(kp_p, kp_i, kp_d, Tp);
R_phi = pid(kphi_p, 0, 0);

innerLoop = feedback(R_p * SYSn,1,1,2,-1)

outerLoop = feedback(R_phi * innerLoop,1,1,1,-1)

figure()
pzmap(outerLoop)

tzero(outerLoop)
pole(outerLoop)
