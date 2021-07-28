%% AEROSPACE CONTROL SYSTEMS %%

clc, clear , close all

run Task1
run Task2
run Task3

%% nominal design (A)

% nominal performances: equivalent to a second-order response with
% Wn >= 10 rad/s and damping ratio >= 0.9


HardReq = TuningGoal.ControllerPoles('outerLoop', 0 , 0.9 , Inf);
SoftReq = TuningGoal.ControllerPoles('outerLoop', 0 , 0 , Inf);

outer_tuned = systune(outerLoop,SoftReq,HardReq);

