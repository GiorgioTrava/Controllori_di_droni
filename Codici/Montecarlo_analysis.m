%% Montecarlo analysis

N = 20; % Number of samples in the MC study

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
    n
    % resampling of uncertain parameters
    Y_v_mc(n)= Y_v.NominalValue+Y_v.PlusMinus(2)*randn;
    L_v_mc(n)= L_v.NominalValue+L_v.PlusMinus(2)*randn;
    Y_delta_mc(n)= Y_delta.NominalValue+Y_delta.PlusMinus(2)*randn;
    L_delta_mc(n)= L_delta.NominalValue+L_delta.PlusMinus(2)*randn;
    
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
    L_mc = connect(R_C,SYS_mc,'phi_0','phi');
%     L_mc(n) = connect(R_p_C,R_phi_C,SYS_mc,Sum_phi,Sum_p,'phi_0','phi');
%     bode(L_mc/(1+L_mc),F_required), hold on
%     margin(outerLoop_mc(n))
    [Gm(n),Pm(n)] = margin(L_mc);
    
    % Computation of step response
    t=(0:.01:1);
    step(minreal(L_mc/(1+L_mc),[],false),t);
   
    y = step(minreal(L_mc/(1+L_mc),[],false),t);
    
    S = stepinfo(y,t); % to compute information about step response
    
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