%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LONGITUDINAL SAS/CAS TUNING - STRUCTURED H-Infinity               %
% Authors:  Marco Lovera (marco.lovera@polimi.it)                     %
% Date: 28/05/2021                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clearvars
close all
clc

%% Generate model

Airplane_B747_data

DerivativeConversionS2B

Long_SLJ

KQ=0.528;

Gqdele_sp_sas=Gqdele_sp/(1+KQ*Gqdele_sp);

Gqdele_sp_sas.u = '\deltae_0'; Gqdele_sp_sas.y = 'q';

%% Tunable regulators
R_q0 = tunablePID('R_q','pid');                                             % tunable PID
R_q0.Kp.Value = 2;%KP_Q;                                                    % initialize Kp
R_q0.Ki.Value = 2;%KI_Q;                                                    % initialize Ki
R_q0.Kd.Value = 1;%KD_Q;                                                    % initialize Kd
R_q0.Tf.Value = 0.01;                                              % initialize N
R_q0.Tf.Free = false;                                                     % fix parameter Tf to this value
R_q0.u = 'e_{q}'; R_q0.y = '\deltae_0';


%% Shaping functions

WFinv=tf(1,[1 2*0.6 1]);

WSinv=1-WFinv;

WQinv=10;

%% Connect blocks
Sum_outer = sumblk('e_{q} = q_0 - q');

% T0 = connect(Gqdele_sp_sas, R_q0, Sum_outer, ...
%     {'q_0'},{'q', '\deltae_0'});

T0 = connect(Gqdele_sp_sas, R_q0, Sum_outer, ...
    {'q_0'},{'e_{q}', '\deltae_0'});

%% Structured H-Infinity
rng('default');

N_TESTS = 10;

Req = [
    TuningGoal.WeightedGain('q_0', '\deltae_0', 1/WQinv, 1);
    TuningGoal.WeightedGain('q_0', 'e_{q}', 1/WSinv, 1)...
    ];

% opt = systuneOptions('RandomStart', N_TESTS, 'Display','final','SoftTol',1e-5);
opt = systuneOptions('RandomStart', N_TESTS,'SoftTol',1e-7,'Display','iter');
[T, J, ~] = systune(T0, Req, opt);


%%
R_q = T.blocks.R_q;

pid(R_q)

loops = loopsens(Gqdele_sp_sas, R_q);

S = loops.Si;
F = loops.Ti;
Q = loops.CSo;

%% END OF CODE