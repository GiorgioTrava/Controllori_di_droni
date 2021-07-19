%% Data and state-space matrices
g =9.81; % m/s^2

Ts=0.004; % sampling time

% Y_v, Y_p, L_v, L_p, Y_delta, L_delta are uncertain parameters. We use an
% uncertainty equal to 3*sigma (for robust analysis)
Y_v = ureal('Y_v',-0.264,'Percentage',3*4.837);
Y_p= 0; 
L_v = ureal('L_v',-7.349,'Percentage',3*4.927);
L_p= 0;
Y_delta = ureal('Y_delta',9.568,'Percentage',3*4.647);
L_delta = ureal('L_delta',1079.339,'Percentage',3*2.762);

A = [Y_v  Y_p  g;
     L_v  L_p  0;
      0    1   0];
 
B = [Y_delta L_delta 0]';

C = [0 1 0;
     0 0 1];
D = [0 0]';

%% create the uncertain state-space model 
G = uss(A,B,C,D);
G.InputName = 'delta_{lat}'; % name of input signal of G
G.OutputName = {'p';'phi'}; % name of output signal of G