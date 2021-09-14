%% AEROSPACE CONTROL SYSTEMS %%


%% ROBUST PERFORMANCE

%Rob_per=minreal(WR*S+tf(info.W1)*F);
x = linspace(0.001,1000,10000);
[mag1] = bode(WR*S,x);
[mag2] = bode(info.W1*F,x);
mag = squeeze(mag1) + squeeze(mag2);
mag = mag2db(mag);

figure(51)
bodemag(tf(1,1),'r');
grid on
hold on
semilogx(x,mag,'g')
legend('bound <1','sum of magnitude')
%% ricostruzione modello incerto controllato

CL1_unc = connect(SYS,R_p_c,R_phi_c,sum_inner,sum_outer,'phi0',{'p','phi'},{'ephi','phi','DELTA_{lat}','ep'});

figure(52)
grid on
step(CL1_unc);










