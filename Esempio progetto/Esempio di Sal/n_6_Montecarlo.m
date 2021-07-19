clear, clc, close all

%% DATA
g =9.81; % m/s^2
Ts = 0.004; % s
% definition of the uncertain parameters
Y_v = ureal('Y_v',-0.264,'Percentage',4.837);
Y_p= 0; 
L_v = ureal('L_v',-7.349,'Percentage',4.927);
L_p= 0;
Y_delta = ureal('Y_delta',9.568,'Percentage',4.647);
L_delta = ureal('L_delta',1079.339,'Percentage',2.762);

%% Tuned Controller
% tuned parameters of the controllers
D_phi = 0.170549175453535;
b = 1.620895607321319;
c1 = 0.086200237029275;
c2 = 0.103218330013736;
d1 = 0.194734070587707;
d2 = -2.483324671681038;

% State-Space Matrices
Ap=[1 0;0 0];
Bp=[b -b*D_phi; 0 0.5];
Cp=[c1 c2];
Dp=[d1 d2*D_phi];

% State-Space model
R= ss(Ap,Bp,Cp,Dp,Ts);
R.InputName={'e_phi';'p'};
R.OutputName={'delta_{lat}'};
R = d2c(R,'tustin'); 

%% MonteCarlo analysis

N=10; % Number of samples in the MC study

Y_v_mc = zeros(N,1); % preallocation of the matrices
Y_p_mc= zeros(N,1); 
L_v_mc = zeros(N,1);
L_p_mc= zeros(N,1);
Y_delta_mc = zeros(N,1);
L_delta_mc = zeros(N,1);

figure(1)
hold on
for n=1:N
    % resampling of uncertain parameters
    Y_v_mc(n)=Y_v.NominalValue + Y_v.PlusMinus(2)*randn(1,1);
    L_v_mc(n)=L_v.NominalValue + L_v.PlusMinus(2)*randn(1,1);
    Y_delta_mc(n)=Y_delta.NominalValue + Y_delta.PlusMinus(2)*randn(1,1);
    L_delta_mc(n)=L_delta.NominalValue + L_delta.PlusMinus(2)*randn(1,1);
    
    A = [Y_v_mc(n)  Y_p_mc(n)  g;
     L_v_mc(n)  L_p_mc(n)  0;
      0    1   0];
 
    B = [Y_delta_mc(n) L_delta_mc(n) 0]';

    C = [0 1 0;
     0 0 1];
 
    D = [0 0]';
    
    % Construction of resampled uncertain plant
    G=ss(A,B,C,D);
    G.OutputName={'p';'phi'};
    G.InputName='delta_{lat}';
    
    % Computation of gain and phase margins
    L(n)=connect(G,R,R.InputName(1),G.OutputName(2));
    [Gm(n),Pm(n)] = margin(L(n));
    
    % Computation of step response 
    t=(0:.01:1);
    step(minreal(L(n)/(1+L(n)),[],false),t);
   
    y=step(minreal(L(n)/(1+L(n)),[],false),t);
    
    S = stepinfo(y,t,1); % to compute information about step response
    
    Sett(n)=S.SettlingTime;
    Over(n)=S.Overshoot;  
end

%% Step response
title_fig1 = title('Step Response','Interpreter','latex');
set(title_fig1,'FontSize',20);
grid on
xlabel_fig1 = xlabel('Time','Interpreter','latex');
ylabel_fig1 = ylabel('Amplitude','Interpreter','latex');
set(xlabel_fig1,'FontSize',15);
set(ylabel_fig1,'FontSize',15);



%% Analysis of results

% Plot the histogram of phase margin
figure(2)
hist(Pm,N)
title_fig2 = title('\textbf{Phase margin}','Interpreter','latex');
set(title_fig2,'FontSize',20);
grid on
xlabel_fig2 = xlabel('Value of phase margin','Interpreter','latex');
ylabel_fig2 = ylabel('Number of occurrence','Interpreter','latex');
set(xlabel_fig2,'FontSize',15);
set(ylabel_fig2,'FontSize',15);

% Plot the histogram of gain margin
figure(3)
hist(Gm)
title_fig3 = title('\textbf{Gain margin}','Interpreter','latex');
set(title_fig3,'FontSize',20);
grid on
xlabel_fig3 = xlabel('Value of gain margin','Interpreter','latex');
ylabel_fig3 = ylabel('Number of occurrence','Interpreter','latex');
set(xlabel_fig3,'FontSize',15);
set(ylabel_fig3,'FontSize',15);

% Plot the histogram of settling time
figure(4)
hist(Sett,N)
title_fig4 = title('\textbf{Settling time}','Interpreter','latex');
set(title_fig4,'FontSize',20);
grid on
xlabel_fig4 = xlabel('Value of settling time','Interpreter','latex');
ylabel_fig4 = ylabel('Number of occurrence','Interpreter','latex');
set(xlabel_fig4,'FontSize',15);
set(ylabel_fig4,'FontSize',15);

% Plot the histogram of overshoot
figure(5)
hist(Over,N)
title_fig5 = title('\textbf{Overshoot}','Interpreter','latex');
set(title_fig5,'FontSize',20);
grid on
xlabel_fig5 = xlabel('Value of overshoot','Interpreter','latex');
ylabel_fig5 = ylabel('Number of occurrence','Interpreter','latex');
set(xlabel_fig5,'FontSize',15);
set(ylabel_fig5,'FontSize',15);

%% study of the worst case in term of settling time 

% computing the settling time for the nominal value of the parameters
run main.m

L_nominal = connect(G.NominalValue,R,R.InputName(1),G.OutputName(2));

y_nominal=step(minreal(L_nominal/(1+L_nominal)),t);
    
S_nominal = stepinfo(y,t,1); % to compute information about step response
    
Sett_nominal=S_nominal.SettlingTime;

% find the worst settling time 
max_Sett=max(Sett);
% find the position of the worst case 
worst=find(Sett==max_Sett);
% find the worst case of the uncertain parameters 

Y_v_worst = Y_v_mc(worst);
Y_p= 0; 
L_v_worst = L_v_mc(worst);
L_p= 0;
Y_delta_worst = Y_delta_mc(worst);
L_delta_worst = L_delta_mc(worst);

% build the plant with the worst uncertain parameters
A_worst = [Y_v_worst  Y_p  g;
     L_v_worst  L_p  0;
      0    1   0];
 
B_worst = [Y_delta_worst L_delta_worst 0]';

C_worst = [0 1 0;
           0 0 1];
D_worst = [0 0]';

G_worst = ss(A_worst,B_worst,C_worst,D_worst);
G_worst.InputName = 'delta_{lat}';
G_worst.OutputName = {'p';'phi'};

% creation of the loop TF
L_worst = connect(G_worst,R,R.InputName(1),G.OutputName(2));

% analysis of the margins of L
figure(6)
margin(L_worst), grid on 

% also studying the worst case in terms of settling time we obtain goo
% results in terms os stability margins 