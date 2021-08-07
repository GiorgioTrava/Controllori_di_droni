%% AEROSPACE CONTROL SYSTEMS %%

clc, clear , close all

run Task1
run Task2
run Task3

%% nominal design (A)

% nominal performances: equivalent to a second-order response with
% Wn >= 10 rad/s and damping ratio >= 0.9

% qui abbiamo uilizzato .Poles e Step Tracking

zeta = 0.9;
wn = 10;

numerator = wn^2;
denominator = [ 1 , 2*zeta*wn , wn^2];

sys_ref = tf(numerator,denominator);

HardReq = TuningGoal.Poles(0,0.9,30); %bisogna capire quale wmax lasciare, per avere un sistema sensato
SoftReq = TuningGoal.StepTracking('phi0','phi',sys_ref);
Options = systuneOptions('Display','final');
[CL , fsoft , gHard] = systune(outerloop,SoftReq,HardReq,Options);


figure(5)
step(sys_ref)
hold on
step(CL(1,:))
legend('reference system step response','tuned system response')


figure(6)
pzplot(CL,'r',outerloop,'g')
legend('tuned S.T.','not tuned')


% qui utilizziamo la complementary sensitivity (.weightedGain)

figure(7)
bode(sys_ref)
legend('desired complementary sensitivity function')

Wl = 1;
WR = sys_ref;
Req = TuningGoal.WeightedGain('phi0','phi',Wl,WR);
[CL1 , fsoft1] = systune(outerloop,Req);

figure(8)
pzplot(CL1,'r',outerloop,'g')
legend('tuned W.G.','not tuned')





















