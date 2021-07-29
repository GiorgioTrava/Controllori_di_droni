%% AEROSPACE CONTROL SYSTEMS %%

clc, clear, close all

run Task1
run Task2

%% Pid controller


R_p = tunablePID2('rrate','PID');
R_phi = tunablePID('rangle','P');

R_phi.InputName = {'ephi'};
R_phi.OutputName = {'p0'};

R_p.InputName = {'p0','p'};
R_p.OutputName = {'DELTA_{lat}'};

innerloop = connect(R_p,G,{'p0','p'},{'p','phi'});
sum = sumblk('ephi = phi0 - phi');
outerloop = connect(R_phi,innerloop,sum,'phi0','phi');

figure(4)
pzmap(outerloop)

% tzero(outerloop)
% pole(outerloop)
