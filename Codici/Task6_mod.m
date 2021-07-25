%% MONTECARLO ANALYSIS AND CONVERGENCE
close all
% Nominal controller
R1 = connect(Sum_phi,R_phi_C,{'phi_0' 'phi'},'p_0');
R2 = connect(R1,Sum_p,{'phi_0' 'phi' 'p'},'p_error');
R_C = connect(R2,R_p_C,{'phi_0' 'phi' 'p'},'delta_{lat}');

F_nominal = connect(R_C,SYSn,'phi_0',{'phi','p'},{'p','phi'});
% L_nominal = F_nominal/(1-F_nominal);
% 
% [Gm_nominal,Pm_nominal] = margin(L_nominal);


L_nominal = getLoopTransfer(F_nominal,{'p','phi'},-1);
[DM,MM] = diskmargin(L_nominal);

Gm_nominal = MM.DiskMargin;
Pm_nominal = MM.PhaseMargin(2);

infostep_nominal = stepinfo(F_nominal(1)); % to compute information about step response
    
Sett_nominal = infostep_nominal.SettlingTime;
Over_nominal = infostep_nominal.Overshoot;

%samples = [10 20 30];%(10:30:70);
samples=[1:2:50 ];
for i=1:length(samples) 
    N=samples(i)
    Y_v_mc = zeros(N,1); % preallocation of the matrices
    Y_p_mc= zeros(N,1); 
    L_v_mc = zeros(N,1);
    L_p_mc= zeros(N,1);
    Y_delta_mc = zeros(N,1);
    L_delta_mc = zeros(N,1);
%     expected_Pm(N) = 0;
%     expected_Gm(N) = 0;
%     expected_Sett(N) = 0;
%     expected_Over(N) = 0;
    
    for n = 1:N
        % resampling of uncertain parameters
        Y_v_mc(n) = Y_v.NominalValue+Y_v.PlusMinus(2)*randn;
        L_v_mc(n) = L_v.NominalValue+L_v.PlusMinus(2)*randn;
        Y_delta_mc(n) = Y_delta.NominalValue+Y_delta.PlusMinus(2)*randn;
        L_delta_mc(n) = L_delta.NominalValue+L_delta.PlusMinus(2)*randn;
        
        A = [Y_v_mc(n) Y_p_mc(n) g;...
            L_v_mc(n)  L_p_mc(n)  0;...
            0 1 0];
        
        B = [Y_delta_mc(n) L_delta_mc(n) 0]';
        
        C = [0 1 0;...
            0 0 1];
        
        D = [0 0]';
        
   
        SYS_mc = ss(A,B,C,D);
        SYS_mc.OutputName = {'p';'phi'};
        SYS_mc.InputName = 'delta_{lat}';

        
        F_mc = connect(R_C,SYS_mc,'phi_0',{'phi','p'},{'p','phi'});

        
        L_mc = getLoopTransfer(F_mc,{'p','phi'},-1);
        [DM,MM] = diskmargin(L_mc);
        
        Gm(n) = MM.DiskMargin;
        Pm(n) = MM.PhaseMargin(2);

        % Computation of step response
        S = stepinfo(F_mc(1));
        
        Sett(n) = S.SettlingTime;
        Over(n) = S.Overshoot;
    end

    expected_Pm(i) = mean(Pm);
    Pm_min(i) = min(Pm) 
    Pm_max(i) = max(Pm) 

    expected_Gm(i) = mean(Gm);
    Gm_min(i) = min(Gm) 
    Gm_max(i) = max(Gm) 

    expected_Sett(i) = mean(Sett);
    Sett_min(i) = min(Sett) 
    Sett_max(i) = max(Sett) 

    expected_Over(i) = mean(Over);
    Over_min(i) = min(Over) 
    Over_max(i) = max(Over) 
end

%% plot of the results
N = max(samples);


figure(1)
plot(samples,expected_Pm,'-o','LineWidth',1.2)
hold on
plot([samples(1) samples(end)],[Pm_nominal Pm_nominal])
plot(samples,Pm_min,'xr')
plot(samples,Pm_max,'xr')
title('Phase margin')

figure(2)
plot(samples,expected_Gm,'-o','LineWidth',1.2)
hold on 
plot([samples(1) samples(end)],[Gm_nominal Gm_nominal])
plot(samples,Gm_min,'xr')
plot(samples,Gm_max,'xr')
title('Disk margin')

figure(3)
plot(samples,expected_Sett,'-o','LineWidth',1.2)
hold on
plot([samples(1) samples(end)],[Sett_nominal Sett_nominal])
plot(samples,Sett_min,'xr')
plot(samples,Sett_max,'xr')
title('Settling time')

figure(4)
plot(samples,expected_Over,'-o','LineWidth',1.2)
hold on
plot([samples(1) samples(end)],[Over_nominal Over_nominal])
plot(samples,Over_min,'xr')
plot(samples,Over_max,'xr')
title('Percentage overshoot')

figure()
hist(Gm,N)
title('Disk margin')

figure()
hist(Pm,N)
title('Phase margin')

figure()
hist(Sett,N)
title('Settling time')

figure()
hist(Over,N)
title('Percentage overshoot')
