%% Mixed-Sensitivity Synthesis
% Design a mixed-sensitivity controller for the following plant, augmented
% by the following loop-shaping filters (see <docid:robust_ref.f10-71923>).
%

clear,clc
close all

% Design requirements

M=1.35;

omb=12; % [rad/s]

A=1e-3;

%% 
% Define the plant, weighting filters, and augmented plant.
s = zpk('s');
G=5/s/(0.1*s+1)^2/(0.01*s+1);

W1inv = (s+A*omb)/(s/M+omb); 

W1=1/W1inv;

W2inv=tf(200,1);
W2=1/W2inv; 

W3 = [];

P = augw(G,W1,W2,W3);
%%
% Synthesize the controller.

% Unstructured synthesis
% [K,CL,GAM] = hinfsyn(P,'METHOD','lmi');


% Structured synthesis
K0 = tunablePID('C','pid');
opt = hinfstructOptions('Display','final','RandomStart',5);
K = hinfstruct(P,K0,opt);

%% Examine the closed-loop result (frequency domain).

figure(1),bode(G,K,G*K),grid,legend('G','K','G*K')
figure(2),bode(1/(1+G*K),W1inv),grid,legend('S','1/W1')
figure(3),bode(K/(1+G*K),W2inv),grid,legend('Q','1/W2')
%% Examine the closed-loop result (time domain).
figure(4)
step(G*K/(1+G*K)),grid

figure(5)
step(K/(1+G*K)),grid
