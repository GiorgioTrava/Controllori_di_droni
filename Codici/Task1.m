%% AEROSPACE CONTROL SYSTEMS %%



%% Uncertain model state space representation

% The stability derivatives of the system are implemented with the required
% level of uncertainty, the standard deviation on
% Y_v, L_v, Y_delta, L_delta

Y_v_sigma = 0.04837*0.264;
Y_v = ureal('Y_v',-0.264,'PlusMinus',[-3*Y_v_sigma,3*Y_v_sigma]);
Y_p= 0; 
L_v_sigma = 0.04927*7.349;
L_v = ureal('L_v',-7.349,'PlusMinus',[-3*L_v_sigma,3*L_v_sigma]);
L_p= 0;
Y_delta_sigma = 0.04647*9.568;
Y_delta = ureal('Y_delta',9.568,'PlusMinus',[-3*Y_delta_sigma,3*Y_delta_sigma]);
L_delta_sigma = 0.02762*7;
L_delta = ureal('L_delta',1079.339,'PlusMinus',[-3*L_delta_sigma,3*L_delta_sigma]);

g = 9.807;

A = [Y_v  Y_p  g;
     L_v  L_p  0;
      0    1   0];
 
B = [Y_delta L_delta 0]';

C = [0 1 0;
     0 0 1];
D = [0 0]';

SYS = ss(A,B,C,D);

SYS.StateName = {'v','p','phi'};
SYS.InputName = 'DELTA_{lat}';
SYS.OutputName = {'p','phi'};

%% Transfer function

% Transfer function of the nominal model

G = tf(SYS);
G.InputName = 'DELTA_{lat}';
G.OutputName = {'p','phi'};


