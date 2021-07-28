%% AEROSPACE CONTROL SYSTEMS %%

clc, clear, close all

run Task1
run Task2

%% Pid controller


R_p = tunablePID2('rrate','PID');
R_phi = tunablePID('rangle','P');

innerLoop = feedback(G * R_p,1,2,1,-1);

outerLoop = feedback(innerLoop * R_phi,1,1,1,-1);

figure()
pzmap(outerLoop)

tzero(outerLoop)
pole(outerLoop)
