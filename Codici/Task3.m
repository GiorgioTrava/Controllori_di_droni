%% AEROSPACE CONTROL SYSTEMS %%

clc, clear, close all

run Task1
run Task2

%% Pid controller


R_p = tunablePID('rrate','PID');
R_phi = tunablePID('rangle','P');

innerLoop = feedback(R_p * G,1,2,1,-1)

outerLoop = feedback(R_phi * innerLoop,1,1,1,-1)

figure()
pzmap(outerLoop)

tzero(outerLoop)
pole(outerLoop)
