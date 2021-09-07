%% AEROSPACE CONTROL SYSTEMS %%

clc, clear , close all

run Task1
run Task2
run Task3
run Task4

%% ROBUST PERFORMANCE

%Rob_per=minreal(WR*S+tf(info.W1)*F);
figure(1110)
bodemag(WR*S,info.W1*F);
