%% AEROSPACE CONTROL SYSTEMS %%
%dovrebbe essere task 8

%% robust stability with mu analysis

[M , Delta, BlkStruct] = lftdata(CL1_unc);
szDelta = size(Delta);
M11 = M(1:szDelta(2),1:szDelta(1));
omega = logspace(-3,2,500);
M11_g = frd(M11,omega);
mubnds = mussv(M11_g,BlkStruct);

Deltatf = tf(Delta);
M.InputName = {'w1','w2','w3','w4','phi0'};
M.OutputName = {'z1','z2','z3','z4','p','phi'};
Deltatf.InputName = {'z1','z2','z3','z4'};
Deltatf.OutputName = {'w1','w2','w3','w4'};
controllo = connect(M,Deltatf,'phi0',{'p','phi'});

% structured singular value

figure(61)
sigma(mubnds), grid

% controllo M-Delta e sistema incerto

figure(62)
bode(controllo,'b',CL1_unc,'g',[10^-2,10^3])
grid on
title('controllo sistema ricostruito')
legend('controllo','sistema incerto')







