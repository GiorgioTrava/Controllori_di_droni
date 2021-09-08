%% AEROSPACE CONTROL SYSTEMS %%


% robust stability with mu analysis

% [M , Delta] = lftdata(CL1_unc);

%M = (-tf(info.W1)*R_p_c*R_phi_c*G(2) - tf(info.W1)*R_p_c*G(1))/([1,1]' + tf(info.W1)*R_p_c*G(1)/tf(info.W1) + tf(info.W1)*R_p_c*R_phi_c*G(2)/tf(info.W1);
G_array_p = usample(SYS(1),60);
[P_p , info_p] = ucover(G_array,SYS(1),5);
W(1,1) = tf(info.W1);
W(1,2) = tf(info.W2);
F_mimo = getIOTransfer(CL1,'phi0',{'p','phi'});
M = F_mimo*W;
omega=logspace(-3,2,500);
bounds=mussv(frd(M,omega),[1 0; 1 0]);

figure(300)
sigma(bounds), grid


