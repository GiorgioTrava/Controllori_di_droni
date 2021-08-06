%% AEROSPACE CONTROL SYSTEMS %%

clc, clear , close all

run Task1
run Task2
run Task3

%% nominal design (A)

% nominal performances: equivalent to a second-order response with
% Wn >= 10 rad/s and damping ratio >= 0.9

zeta = 0.9;
wn = 100;

numerator = wn^2;
denominator = [ 1 , 2*zeta*wn , wn^2];

sys_ref = tf(numerator,denominator);

Req = TuningGoal.StepTracking('phi0','phi',sys_ref);


viewGoal(Req)
hold on
step(sys_ref)


