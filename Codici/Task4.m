%% AEROSPACE CONTROL SYSTEMS %%


%% output richiesto

% nominal performances: equivalent to a second-order response with
% Wn >= 10 rad/s and damping ratio >= 0.9

zeta = 0.9;
wn = 10;

numerator = wn^2;
denominator = [ 1 , 2*zeta*wn , wn^2];

sys_ref = tf(numerator,denominator); % rappresenta la complementary sensitivity richiesta
S_req = 1 - sys_ref ;


%% design , mixed sensitivity (.weightedGain)
%% A
% nominal performance

zeta_wr = 0.6; 
wn_wr = 10;
numerator_wr = wn_wr^2;
denominator_wr = [ 1 , 2*zeta_wr*wn_wr , wn_wr^2];
Wl = 1;
tf_peso1 = minreal(tf(numerator_wr,denominator_wr));                                 % costruisco la funzione peso sulla base della f_req, poi la modifico opportunamente
s_req_modified = 1.1*(1 - tf_peso1);                                                  % (1 - tf_peso) rappresenta la s_req sulla quale faccio le modifiche
WR = 1/s_req_modified; % peso effettivo
Req1 = TuningGoal.WeightedGain('phi0','ephi',Wl,WR);                                   % qui sto imponendo il vincolo sulla nominal performance tramite la sensitivity

% control sensitivity

%Wr1 = 1.5*tf( 1 , 1 ); % prova con funzione peso costante in frequenza
Wr1 = 0.0727*tf([0.1,1],1)*tf([0.1,1],1)*tf(1,[0.0143,1])*tf(1,[0.0143,1]);           % prova con filtro passa basso
Req2 = TuningGoal.WeightedGain('phi0','DELTA_{lat}',Wl,Wr1);                          % qui sto imponendo il vincolo sul control effort
Req = [ Req1 , Req2 ]; % vettore dei requirements

% design

N=0; %numero di optimizations aggiuntive partendo da valori random
options = systuneOptions('RandomStart',N);
[CL1 , fsoft1] = systune(sys_complete,Req,options); % ottimizzazione

R_p_c = pid(CL1.Blocks.rrate)
R_phi_c = pid(CL1.Blocks.rangle)

R_p_c.InputName = {'ep'};
R_p_c.OutputName = {'DELTA_{lat}'};

R_phi_c.InputName = {'ephi'};
R_phi_c.OutputName = {'p0'};

settlingtime_tuned = stepinfo(CL1(2)).SettlingTime
settlingtime_required = stepinfo(sys_ref).SettlingTime

overshoot_tuned = stepinfo(CL1(2)).Overshoot
overshoot_required = stepinfo(sys_ref).Overshoot

%% nominal stability

L = minreal(getIOTransfer(CL1,'ephi','phi','phi')); % loop transfer function
c1 = 1 + L;
[num,den] = tfdata(c1,'v'); % extract pole polinomial of the closed and open loop, c1(1,1) represent the determinant
disp('check if open loop poles correspond')
pole(L)
roots(den)
disp('------------------------------')
disp('check if closed loop poles correspond')
pole(CL1(2))
roots(num)

figure(43)
bode(L)
figure(44)
nyquistplot(L)

figure(45)
margin(L)

%
L_pi = minreal(getIOTransfer(CL1,'ephi','p','phi'));

figure(444)
nyquistplot(L_pi)

figure(455)
margin(L_pi)
%% C robust stability

G_array = usample(SYS(2),60);
F = minreal(getIOTransfer(CL1,'phi0','phi'));
[P_cover , info] = ucover(G_array,SYS(2),5);
figure(46)
bode(F,1/info.W1)
legend('complementary sensitivity','weight')
figure(47)
bodemag(minreal((G(2)-G_array)/G(2)), 'g', info.W1, 'r') % fare una prova considerando il loop interno come incerto
legend('Relative error array','Weight')
%% plt sensitivity, loop transfer function, 1/WR


% plot sensitivity

S = getIOTransfer( CL1,'phi0','ephi'); % sensitivity

figure(48)

bode(S_req,S,1/WR)
legend('required sensitivity','sensitivity','1/WR')
grid on
figure(49)
asymp(1/WR)
grid on


%risposta a step
 
figure(410)
grid on
step(sys_ref)
hold on
step(CL1(2))
legend('reference system step response','tuned with mixed sensitivity')
title('step response')


% mappa poli e zeri

figure(411)
pzplot(CL1,'r')
legend('tuned')
title('poles and zeros of the tuned system with mixed sensitivity')

% plt control sensitivity

Q = minreal(getIOTransfer(CL1,'phi0','DELTA_{lat}')); 

figure(412)
bode(Q,'c',1/Wr1,'y')
grid on
legend('Control Sensitivity','1/WR1')
title('tuning control effort limitation')

%% B Control effort limitation

% step response control variable

t = linspace(0,6,612);
tpast1 = 2;
tpast2 = 4;
u1 = (10*pi/180)*rectpuls(t - tpast1, 2);
u2 = -(10*pi/180)*rectpuls(t - tpast2, 2);
u = u1 + u2;
u3 = (10)*rectpuls(t - tpast1, 2);
u4 = -(10)*rectpuls(t - tpast2, 2);
u_plot = u3 + u4;

%simulo con questo input

y = lsim(Q,u,t);
y1 = lsim(CL1(2),u,t);

figure(413)
plot(t,u_plot);
hold on
grid on
plot(t,rad2deg(y));
plot(t,rad2deg(y1));
ylabel('amplitude')
xlabel('time')
title('control effort ')
legend('input {u}','output DELTA_{lat}','output phi')
hold off










