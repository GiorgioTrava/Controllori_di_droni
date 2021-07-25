%% Montecarlo convergence

% Nominal controller
R1 = connect(Sum_phi,R_phi_C,{'phi_0' 'phi'},'p_0');
R2 = connect(R1,Sum_p,{'phi_0' 'phi' 'p'},'p_error');
R_C = connect(R2,R_p_C,{'phi_0' 'phi' 'p'},'delta_{lat}');

F_nominal = connect(R_C,SYSn,'phi_0','phi',{'p','phi'});
% L_nominal = F_nominal/(1-F_nominal);
% 
% [Gm_nominal,Pm_nominal] = margin(L_nominal);

L_nominal = getLoopTransfer(F_nominal,{'p','phi'},-1);
[DM,MM] = diskmargin(L_nominal);

Gm_nominal = MM.GainMargin(1);
Pm_nominal = MM.PhaseMargin(2);

t = (0:.01:1);

y = step(minreal(F_nominal),t);
    
infostep_nominal = stepinfo(y,t,1); % to compute information about step response
    
Sett_nominal = infostep_nominal.SettlingTime;
Over_nominal = infostep_nominal.Overshoot;

samples = (10:30:70);
for N = samples
    N
    Y_v_mc = zeros(N,1); % preallocation of the matrices
    Y_p_mc= zeros(N,1); 
    L_v_mc = zeros(N,1);
    L_p_mc= zeros(N,1);
    Y_delta_mc = zeros(N,1);
    L_delta_mc = zeros(N,1);
    expected_Pm(N) = 0;
    expected_Gm(N) = 0;
    expected_Sett(N) = 0;
    expected_Over(N) = 0;
    
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
        SYS_mc = tf(SYS_mc);
        
        F_mc = connect(R_C,SYS_mc,'phi_0','phi',{'p','phi'});
%         L_mc = F_mc/(1-F_mc);
%         
%         [Gm(n),Pm(n)] = margin(L_mc);
        
        L_mc = getLoopTransfer(F_mc,{'p','phi'},-1);
        [DM,MM] = diskmargin(L_mc);
        
        Gm(n) = MM.GainMargin(1);
        Pm(n) = MM.PhaseMargin(2);

        % Computation of step response
        t=(0:.01:1);
        step(minreal(F_mc,[],false),t);
        
        y = step(minreal(F_mc,[],false),t);
        
        S = stepinfo(y,t);
        
        Sett(n) = S.SettlingTime;
        Over(n) = S.Overshoot;
    end
    Pm_hist = histogram(Pm);
    
    for i =1:size(Pm_hist.Values,2)
        expected_Pm(N) = expected_Pm(N)+(Pm_hist.Values(i)*0.5*(Pm_hist.BinEdges(i)+Pm_hist.BinEdges(i+1)))/N;
    end
    
    Gm_hist = histogram(Gm);
    
    for i =1:size(Gm_hist.Values,2)
        expected_Gm(N) = expected_Gm(N)+(Gm_hist.Values(i)*0.5*(Gm_hist.BinEdges(i)+Gm_hist.BinEdges(i+1)))/N;
    end
    
    Sett_hist = histogram(Sett);
    
    for i =1:size(Sett_hist.Values,2)
        expected_Sett(N) = expected_Sett(N)+(Sett_hist.Values(i)*0.5*(Sett_hist.BinEdges(i)+Sett_hist.BinEdges(i+1)))/N;
    end
    
    Over_hist = histogram(Over);
    
    for i =1:size(Over_hist.Values,2)
        expected_Over(N) = expected_Over(N)+(Over_hist.Values(i)*0.5*(Over_hist.BinEdges(i)+Over_hist.BinEdges(i+1)))/N;
    end
end

%% plot of the results
N = max(samples);

% phase margin
expected_Pm = expected_Pm(samples);
figure(1)
plot(samples,expected_Pm,'-o','LineWidth',1.2)
hold on
plot(Pm_nominal)
plot(1:0.1:N,ones(size(1:0.1:N,2),1)*Pm_nominal,'--','LineWidth',1.2)
legend('Actual behaviour','Expected value')
grid on 
title_fig3 = title('\textbf{Phase margin}','Interpreter','latex');
set(title_fig3,'FontSize',20);
xlabel_fig3 = xlabel('N: number of samples','Interpreter','latex');
ylabel_fig3 = ylabel('Phase margin','Interpreter','latex');
set(xlabel_fig3,'FontSize',15);
set(ylabel_fig3,'FontSize',15);

% Gain margin
expected_Gm=expected_Gm(samples);
figure(2)
plot(samples,expected_Gm,'-o','LineWidth',1.2)
hold on 
plot(1:0.1:N,ones(size(1:0.1:N,2),1)*Gm_nominal,'--','LineWidth',1.2)
legend('Actual behaviour','Expected value')
grid on 
title_fig3 = title('\textbf{Gain margin}','Interpreter','latex');
set(title_fig3,'FontSize',20);
xlabel_fig3 = xlabel('N: number of samples','Interpreter','latex');
ylabel_fig3 = ylabel('Gain margin','Interpreter','latex');
set(xlabel_fig3,'FontSize',15);
set(ylabel_fig3,'FontSize',15);

% Settling time
expected_Sett = expected_Sett(samples);
figure(3)
plot(samples,expected_Sett,'-o','LineWidth',1.2)
hold on
plot(1:0.1:N,ones(size(1:0.1:N,2),1)*Sett_nominal,'--','LineWidth',1.2)
legend('Actual behaviour','Expected value')
grid on 
title_fig3 = title('\textbf{Settling time}','Interpreter','latex');
set(title_fig3,'FontSize',20);
xlabel_fig3 = xlabel('N: number of samples','Interpreter','latex');
ylabel_fig3 = ylabel('Settling time','Interpreter','latex');
set(xlabel_fig3,'FontSize',15);
set(ylabel_fig3,'FontSize',15);

% Overshoot
expected_Over = expected_Over(samples);
figure(4)
plot(samples,expected_Over,'-o','LineWidth',1.2)
hold on
plot(1:0.1:N,ones(size(1:0.1:N,2),1)*Over_nominal,'--','LineWidth',1.2)
legend('Actual behaviour','Expected value')
grid on 
title_fig3 = title('\textbf{Overshoot}','Interpreter','latex');
set(title_fig3,'FontSize',20);
xlabel_fig3 = xlabel('N: number of samples','Interpreter','latex');
ylabel_fig3 = ylabel('Overshoot','Interpreter','latex');
set(xlabel_fig3,'FontSize',15);
set(ylabel_fig3,'FontSize',15);

%% plots of the errors 

% phase margin
Pm_nominal_vect = Pm_nominal*ones(size(expected_Pm));
Pm_error = abs(Pm_nominal_vect-expected_Pm);
figure(5)
plot(samples,Pm_error,'-o','LineWidth',1.2)
grid on 
title_fig3 = title('\textbf{Phase margin}','Interpreter','latex');
set(title_fig3,'FontSize',20);
xlabel_fig3 = xlabel('N: number of samples','Interpreter','latex');
ylabel_fig3 = ylabel('Error','Interpreter','latex');
set(xlabel_fig3,'FontSize',15);
set(ylabel_fig3,'FontSize',15);

% Gain margin
Gm_nominal_vect = Gm_nominal*ones(size(expected_Gm));
Gm_error = abs(Gm_nominal_vect-expected_Gm);
figure(6)
plot(samples,Gm_error,'-o','LineWidth',1.2)
grid on 
title_fig3 = title('\textbf{Gain margin}','Interpreter','latex');
set(title_fig3,'FontSize',20);
xlabel_fig3 = xlabel('N: number of samples','Interpreter','latex');
ylabel_fig3 = ylabel('Error','Interpreter','latex');
set(xlabel_fig3,'FontSize',15);
set(ylabel_fig3,'FontSize',15);

% Settling time
Sett_nominal_vect = Sett_nominal*ones(size(expected_Sett));
Sett_error = abs(Sett_nominal_vect-expected_Sett);
figure(7)
plot(samples,Sett_error,'-o','LineWidth',1.2)
grid on 
title_fig3 = title('\textbf{Settling time}','Interpreter','latex');
set(title_fig3,'FontSize',20);
xlabel_fig3 = xlabel('N: number of samples','Interpreter','latex');
ylabel_fig3 = ylabel('Error','Interpreter','latex');
set(xlabel_fig3,'FontSize',15);
set(ylabel_fig3,'FontSize',15);

% Overshoot
Over_nominal_vect = Over_nominal*ones(size(expected_Over));
Over_error = abs(Over_nominal_vect-expected_Over);
figure(8)
plot(samples,Over_error,'-o','LineWidth',1.2)
grid on 
title_fig3 = title('\textbf{Overshoot}','Interpreter','latex');
set(title_fig3,'FontSize',20);
xlabel_fig3 = xlabel('N: number of samples','Interpreter','latex');
ylabel_fig3 = ylabel('Error','Interpreter','latex');
set(xlabel_fig3,'FontSize',15);
set(ylabel_fig3,'FontSize',15);