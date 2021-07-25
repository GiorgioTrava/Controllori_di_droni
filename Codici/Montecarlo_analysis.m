%% Montecarlo analysis

N = 5; % Number of samples in the MC study

Y_v_mc = zeros(N,1); % preallocation of the matrices
Y_p_mc= zeros(N,1);
L_v_mc = zeros(N,1);
L_p_mc= zeros(N,1);
Y_delta_mc = zeros(N,1);
L_delta_mc = zeros(N,1);

Gm = zeros(N,1);
Pm = zeros(N,1);
Sett = zeros(N,1);
Over = zeros(N,1);

R1 = connect(Sum_phi,R_phi_C,{'phi_0' 'phi'},'p_0');
R2 = connect(R1,Sum_p,{'phi_0' 'phi' 'p'},'p_error');
R_C = connect(R2,R_p_C,{'phi_0' 'phi' 'p'},'delta_{lat}');
% R_C = tf(connect(R_p_C,R_phi_C,Sum_phi,Sum_p,{'phi','p','phi_0'},'delta_{lat}'));

figure()
hold on
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

    C = [0 1 0;
     0 0 1];
 
    D = [0 0]';
    
    SYS_mc = ss(A,B,C,D);
    SYS_mc.OutputName = {'p';'phi'};
    SYS_mc.InputName = 'delta_{lat}';
    SYS_mc = tf(SYS_mc);
%     L_mc = -SYS_mc*R_C;
    F_mc = connect(R_C,SYS_mc,'phi_0','phi');
    L_mc = F_mc/(1-F_mc);
%     L_mc(n) = connect(R_p_C,R_phi_C,SYS_mc,Sum_phi,Sum_p,'phi_0','phi');
%     bode(L_mc/(1+L_mc),F_required), hold on
%     margin(outerLoop_mc(n))
    [Gm(n),Pm(n)] = margin(L_mc);
    
    % Computation of step response
    t=(0:.01:1);
    step(minreal(F_mc,[],false),t);
   
    y = step(minreal(F_mc,[],false),t);
    
    S = stepinfo(y,t);
    
    Sett(n) = S.SettlingTime;
    Over(n) = S.Overshoot;
    
end

figure()
hist(Gm,N)

figure()
hist(Pm,N)

figure()
hist(Sett,N)

figure()
hist(Over,N)

% figure()
% hist(Y_v_mc,N)
% hold on
% y = -0.5:0.001:0;
% mu = -0.264;
% sigma = 3*0.04837*0.264;
% f = exp(-(y-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi));
% plot(y,f,'r','LineWidth',1.5)
% title('Normal distribution - Nominl value: Y_v = -0.264')
% xlabel('Y_v')
% ylabel('Number of samples')