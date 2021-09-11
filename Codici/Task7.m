%% AEROSPACE CONTROL SYSTEMS %%

%% montecarlo simulation

n = 500;
for i = 1:n
    
    Y_v_mc(i) = -0.264 + Y_v_sigma*rand(1);
    L_v_mc(i) = -7.349 + L_v_sigma*randn(1);
    Y_delta_mc(i) = 9.568 + Y_delta_sigma*randn(1);
    L_delta_mc(i) = 1079.339 + L_delta_sigma*randn(1);

A = [Y_v_mc(i)  Y_p  g;
     L_v_mc(i)  L_p  0;
      0    1   0];
 
B = [Y_delta_mc(i) L_delta_mc(i) 0]';

C = [0 1 0;
     0 0 1];
D = [0 0]';

SYS_mc = ss(A,B,C,D);
SYS_mc.StateName = {'v','p','phi'};
SYS_mc.InputName = 'DELTA_{lat}';
SYS_mc.OutputName = {'p','phi'};

 

CL1_unc_mc = connect(tf(SYS_mc),R_p_c,R_phi_c,sum_inner,sum_outer,'phi0',{'p','phi'},{'ephi','phi','DELTA_{lat}','ep'});
L_mc = getIOTransfer(CL1_unc_mc,'ephi','phi','phi'); % loop transfer function
[Gm(i),Pm(i)] = margin(L_mc);

t=(0:.01:10);

 y=step(CL1_unc_mc(2),t);
    
    S = stepinfo(y,t);
    
    Sett(i)=S.SettlingTime;
    Over(i)=S.Overshoot;
    

end
 
figure(71),hist(Pm,100), grid, title('Phase margin')

figure(72),hist(Gm,100), grid, title('Gain margin')

figure(73),hist(Sett,100), grid, title('Settling time')

figure(74),hist(Over,100), grid, title('% Overshoot')