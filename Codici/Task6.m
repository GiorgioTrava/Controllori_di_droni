%% AEROSPACE CONTROL SYSTEMS %%


% robust stability with mu analysis

[M , Delta] = lftdata(CL1_unc);
omega=logspace(-3,2,500);
bounds=mussv(frd(tf(M),omega),[1 0; 1 0]);