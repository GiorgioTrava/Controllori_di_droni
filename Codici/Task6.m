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


W_strano(1,1) = tf(info.W1);
W_strano(2,2) = tf(info.W2);
W_strano.InputName = {'DELTA_{lat}'};
W_strano.OutputName = {'z1','z2'};
Delta = ultidyn('delta',[2,2]);
Delta.InputName = {'z1','z2'};
Delta.OutputName = {'w1','w2'};
G1 = tf(SYS);
G1.InputName = 'DELTA_{lat}';
G1.OutputName = {'p','phi'};
R_p_c.InputName = {'p'};
R_p_c.OutputName = {'DELTA_{lat}'};
R_phi_c.InputName = {'ephi'};
R_phi_c.OutputName = {'p0'};

sum_inner1 = sumblk{'ep = p - p0'};
sum_unc = sumblk{'g_in = w1 + w2 + DELTA_{lat}'};

M_delta = connect(R_phi_c,R_p_c,W_strano,Delta,G1,sum_inner1,sum_unc,{'w1','w2'},{'z1','z2'});












