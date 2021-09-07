%% AEROSPACE CONTROL SYSTEMS %%

clc, clear , close all

run Task1
run Task2
run Task3

%% output richiesto

% nominal performances: equivalent to a second-order response with
% Wn >= 10 rad/s and damping ratio >= 0.9

zeta = 0.9;
wn = 10;

numerator = wn^2;
denominator = [ 1 , 2*zeta*wn , wn^2];

sys_ref = tf(numerator,denominator); % rappresenta la complementary sensitivity richiesta
S_req = 1 - sys_ref ;


%% Design con steptracking

HardReq = TuningGoal.Poles(0,0.9,50); %bisogna capire quale wmax lasciare, per avere un sistema sensato
SoftReq = TuningGoal.StepTracking('phi0','phi',sys_ref);
Options = systuneOptions('Display','final');
[CL , fsoft , gHard] = systune(sys_complete_n,SoftReq,HardReq,Options);

%risposta a step

figure(5)
step(sys_ref)
hold on
step(CL(2,:))
legend('reference system','tuned system')
title('step response of the tuned system with steptracking function')

% mappa poli e zeri
 
figure(6)
pzplot(CL,'r',G,'g')
legend('tuned S.T.','uncontrolled')


%% design , mixed sensitivity (.weightedGain)

% nominal performance

zeta_wr = 0.6;
wn_wr = 17;

numerator_wr = wn_wr^2;
denominator_wr = [ 1 , 2*zeta_wr*wn_wr , wn_wr^2];
Wl = 1;
tf_peso1 = minreal(tf(numerator_wr,denominator_wr)); % costruisco la funzione peso sulla base della f_req, poi la modifico opportunamente
s_req_modified = 1.1*(1 - tf_peso1);                %*tf([1,0.8],1)*tf(1,[1,0]);                   %*tf(1,[1 , 5])*tf([1 , 13],1)*tf([1 , 13],1)*tf(1,[1 , 23.4]); % (1 - tf_peso) rappresenta la s_req sulla quale faccio le modifiche
WR = 1/s_req_modified; % peso effettivo
%WR = (1/S_req); % costruisco la funzione peso sulla base della sensitivity richiesta
Req1 = TuningGoal.WeightedGain('phi0','ephi',Wl,WR); % qui sto imponendo il vincolo sulla nominal performance tramite la sensitivity

% control sensitivity

% wb1 = 8; %regolo la banda di frequenze
% M1 = 0.3;  %regolo l'overshoot, diminuendo il valore si riscontra una diminuzione nei valori di picco di delta_lat per l'ingresso u

% numerator_WR1 = [1/M1 , wb1];
% denominator_WR1 = [1 , 0];
WR1 = 6*tf( 1 , 1 ); % prova con funzione peso costante in frequenza
% WR1 = tf( numerator_WR1 , denominator_WR1 );
Req2 = TuningGoal.WeightedGain('phi0','DELTA_{lat}',Wl,WR1); % qui sto imponendo il vincolo sul control effort
Req = [ Req1 , Req2 ]; % vettore dei requirements



% design

N=0; %numero di optimizations aggiuntive partendo da valori random
options = systuneOptions('RandomStart',N);
[CL1 , fsoft1] = systune(sys_complete_n,Req,options); % ottimizzazione

pid(CL1.Blocks.rrate)
pid(CL1.Blocks.rangle)

settlingtime_tuned = stepinfo(CL1(2)).SettlingTime
settlingtime_required = stepinfo(sys_ref).SettlingTime

overshoot_tuned = stepinfo(CL1(2)).Overshoot
overshoot_required = stepinfo(sys_ref).Overshoot

%% nominal stability

G_n = getNominal(G);
L = minreal(getIOTransfer(CL1,'ephi','phi','phi')); % loop transfer function
% L = G_n*tf(CL1.Blocks.rrate)*tf(CL1.Blocks.rangle); % funzione di trasferimento in anello aperto
% L1 = tf(L);
% L2 = [ 1 , 0 ];
% L_n = L1*L2;
% I = eye(2);
c1 = 1 + L;
[num,den] = tfdata(c1,'v'); % extract pole polinomial of the closed and open loop, c1(1,1) represent the determinant
disp('check if open loop poles correspond')
pole(L)
roots(den)
disp('------------------------------')
disp('check if closed loop poles correspond')
pole(CL1(2))
roots(num)

figure(7)
bode(c1)
figure(70)
nyquistplot(c1)

figure(71)
margin(c1)


%% robust stability

G_array = usample(SYS(2),60);
F = minreal(getIOTransfer(CL1,'phi0','phi'));
[P , info] = ucover(G_array,SYS(2),1);
figure(30)
bode(F,info.W1)
legend('complementary sensitivity','weight')


%% plt sensitivity, loop transfer function, 1/WR


% plot sensitivity

S = minreal(getIOTransfer( CL1,'phi0','ephi')); % sensitivity

figure(8)

bode(S_req,S,1/WR)
legend('required sensitivity','sensitivity','1/WR')
grid on
figure(80)
asymp(1/WR)
grid on



%risposta a step
 
figure(9)
grid on
step(sys_ref)
hold on
step(CL1(2))
legend('reference system step response','tuned with mixed sensitivity')
title('step response')


% mappa poli e zeri

figure(10)
pzplot(CL1,'r',G,'g')
legend('tuned','plant')
title('poles and zeros of the tuned system with mixed sensitivity')

% plt control sensitivity

Q = minreal(getIOTransfer(CL1,'phi0','DELTA_{lat}')); 
R = minreal(getIOTransfer(CL1,'ep','DELTA_{lat}'));

figure(11)
bode(Q,'c',R,'g',1/WR1,'y')
grid on
legend('Control Sensitivity','Controller','1/WR1')
title('tuning control effort limitation')

% step response control variable

t = linspace(0,6,612);
tpast1 = 2;
tpast2 = 4;
u1 = 10*rectpuls(t - tpast1, 2);
u2 = -10*rectpuls(t - tpast2, 2);
u = u1 + u2;

%simulo con questo input

y = lsim(Q,u,t);
y1 = lsim(CL1(2),u,t);

figure(12)
plot(t,u);
hold on
grid on
plot(t,y);
plot(t,y1);
ylabel('amplitude')
xlabel('time')
title('control effort ')
legend('input {u}','output DELTA_{lat}','output phi')
hold off










