%% AEROSPACE CONTROL SYSTEMS %%

clc, clear , close all

run Task1
run Task2
run Task3

%% nominal design (A)

% nominal performances: equivalent to a second-order response with
% Wn >= 10 rad/s and damping ratio >= 0.9

zeta = 0.9;
wn = 10;

numerator = wn^2;
denominator = [ 1 , 2*zeta*wn , wn^2];

sys_ref = tf(numerator,denominator);

%% Design con steptracking

HardReq = TuningGoal.Poles(0,0.9,30); %bisogna capire quale wmax lasciare, per avere un sistema sensato
SoftReq = TuningGoal.StepTracking('phi0','phi',sys_ref);
Options = systuneOptions('Display','final');
[CL , fsoft , gHard] = systune(sys_complete,SoftReq,HardReq,Options);

%risposta a step

figure(5)
step(sys_ref)
hold on
step(CL(1,:))
legend('reference system','tuned system')
title('step response of the tuned system with steptracking function')

% mappa poli e zeri

figure(6)
pzplot(CL,'r',G,'g')
legend('tuned S.T.','uncontrolled')


%% design con la sensitivity function(.weightedGain)


wb = 10; %regolo la banda di frequenze
M = 1;  %regolo l'overshoot
numerator_WR = [1/M , wb];
denominator_WR = [1 , 0];
Wl = 1;
WR = tf(numerator_WR,denominator_WR); % costruisco la funzione peso
Req = TuningGoal.WeightedGain('phi0','ephi',Wl,WR); % qui sto imponendo il vincolo sulla nominal performance tramite la sensitivity
%options = systuneOptions('RandomStart','UseParallel'); %uso il random start essendo il problema non convesso (structured mixed sensitivity)
[CL1 , fsoft1] = systune(sys_complete,Req); % ottimizzazione



%% plt sensitivity, loop transfer function, 1/WR


S = getIOTransfer( CL1,'phi0','ephi');
L = getLoopTransfer(CL1,'phi');

figure(8)
bode(S)
hold on
bode(L)
bode(1/WR)
legend('Sensitivity function','loop transfer function','1/WR')
title('tuned system with mixed sensitivity')

%risposta a step
 
figure(9)
step(sys_ref)
hold on
step(CL1(2))
legend('reference system step response','tuned with mixed sensitivity')
title('step response')

% mappa poli e zeri

figure(10)
pzplot(CL1,'r',G,'g')
legend('tuned','uncontrolled')
title('poles and zeros of the tuned system with mixed sensitivity')









