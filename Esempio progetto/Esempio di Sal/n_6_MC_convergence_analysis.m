clear, clc

%% Nominal Values, they should be the expected ones

g =9.81; % m/s^2
Ts = 0.004; % s
% definition of the uncertain parameters
Y_v = ureal('Y_v',-0.264,'Percentage',4.837);
Y_p= 0; 
L_v = ureal('L_v',-7.349,'Percentage',4.927);
L_p= 0;
Y_delta = ureal('Y_delta',9.568,'Percentage',4.647);
L_delta = ureal('L_delta',1079.339,'Percentage',2.762);

A = [Y_v  Y_p  g;
     L_v  L_p  0;
      0    1   0];
 
B = [Y_delta L_delta 0]';

C = [0 1 0;
     0 0 1];
D = [0 0]';
    
G = uss(A,B,C,D);
G.InputName = 'delta_{lat}'; % name of input signal of G
G.OutputName = {'p';'phi'}; % name of output signal of G

% Tuned Controller
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

L_nominal =connect(G.NominalValue,R,R.InputName(1),G.OutputName(2));

% gain and phase margin
[Gm_nominal,Pm_nominal] = margin(L_nominal);

% settling time and overshoot
t=(0:.01:1);

y=step(minreal(L_nominal/(1+L_nominal)),t);
    
infostep_nominal = stepinfo(y,t,1); % to compute information about step response
    
Sett_nominal=infostep_nominal.SettlingTime;
Over_nominal=infostep_nominal.Overshoot;

vect = [1,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160];
for N = vect
    
Y_v_mc = zeros(N,1); % preallocation of the matrices
Y_p_mc= zeros(N,1); 
L_v_mc = zeros(N,1);
L_p_mc= zeros(N,1);
Y_delta_mc = zeros(N,1);
L_delta_mc = zeros(N,1);
expected_Pm(N) =0;
expected_Gm(N) =0;
expected_Sett(N) =0;
expected_Over(N) =0;

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
   
    y=step(minreal(L(n)/(1+L(n)),[],false),t);
    
    S = stepinfo(y,t,1); % to compute information about step response
    
    Sett(n)=S.SettlingTime; 
    Over(n)=S.Overshoot;  
end

Pm_hist = histogram(Pm);

for i =1:size(Pm_hist.Values,2)
    expected_Pm(N) = expected_Pm(N)+(Pm_hist.Values(i)*0.5*(Pm_hist.BinEdges(i)+Pm_hist.BinEdges(i+1)))/N;
end

Gm_hist = histogram(Gm);

for i =1:size(Gm_hist.Values,2)
    expected_Gm(N) = expected_Gm(N)+(Gm_hist.Values(i)*0.5*(Gm_hist.BinEdges(i)+Gm_hist.BinEdges(i+1)))/N;
end

Sett_hist = histogram(Sett);

for i =1:size(Sett_hist.Values,2)
    expected_Sett(N) = expected_Sett(N)+(Sett_hist.Values(i)*0.5*(Sett_hist.BinEdges(i)+Sett_hist.BinEdges(i+1)))/N;
end

Over_hist = histogram(Over);

for i =1:size(Over_hist.Values,2)
    expected_Over(N) = expected_Over(N)+(Over_hist.Values(i)*0.5*(Over_hist.BinEdges(i)+Over_hist.BinEdges(i+1)))/N;
end
end

%% plot of the results
N = max(vect);

% phase margin
expected_Pm=expected_Pm(vect);
figure(1)
plot(vect,expected_Pm,'-o','LineWidth',1.2)
hold on 
plot(1:0.1:N,ones(size(1:0.1:N,2),1)*Pm_nominal,'--','LineWidth',1.2)
legend('Actual behaviour','Expected value')
grid on 
title_fig3 = title('\textbf{Phase margin}','Interpreter','latex');
set(title_fig3,'FontSize',20);
xlabel_fig3 = xlabel('N: number of samples','Interpreter','latex');
ylabel_fig3 = ylabel('Phase margin','Interpreter','latex');
set(xlabel_fig3,'FontSize',15);
set(ylabel_fig3,'FontSize',15);

% Gain margin
expected_Gm=expected_Gm(vect);
figure(2)
plot(vect,expected_Gm,'-o','LineWidth',1.2)
hold on 
plot(1:0.1:N,ones(size(1:0.1:N,2),1)*Gm_nominal,'--','LineWidth',1.2)
legend('Actual behaviour','Expected value')
grid on 
title_fig3 = title('\textbf{Gain margin}','Interpreter','latex');
set(title_fig3,'FontSize',20);
xlabel_fig3 = xlabel('N: number of samples','Interpreter','latex');
ylabel_fig3 = ylabel('Gain margin','Interpreter','latex');
set(xlabel_fig3,'FontSize',15);
set(ylabel_fig3,'FontSize',15);

% Settling time
expected_Sett = expected_Sett(vect);
figure(3)
plot(vect,expected_Sett,'-o','LineWidth',1.2)
hold on
plot(1:0.1:N,ones(size(1:0.1:N,2),1)*Sett_nominal,'--','LineWidth',1.2)
legend('Actual behaviour','Expected value')
grid on 
title_fig3 = title('\textbf{Settling time}','Interpreter','latex');
set(title_fig3,'FontSize',20);
xlabel_fig3 = xlabel('N: number of samples','Interpreter','latex');
ylabel_fig3 = ylabel('Settling time','Interpreter','latex');
set(xlabel_fig3,'FontSize',15);
set(ylabel_fig3,'FontSize',15);

% Overshoot
expected_Over = expected_Over(vect);
figure(4)
plot(vect,expected_Over,'-o','LineWidth',1.2)
hold on
plot(1:0.1:N,ones(size(1:0.1:N,2),1)*Over_nominal,'--','LineWidth',1.2)
legend('Actual behaviour','Expected value')
grid on 
title_fig3 = title('\textbf{Overshoot}','Interpreter','latex');
set(title_fig3,'FontSize',20);
xlabel_fig3 = xlabel('N: number of samples','Interpreter','latex');
ylabel_fig3 = ylabel('Overshoot','Interpreter','latex');
set(xlabel_fig3,'FontSize',15);
set(ylabel_fig3,'FontSize',15);

%% plots of the errors 

% phase margin
Pm_nominal_vect = Pm_nominal*ones(size(expected_Pm));
Pm_error = abs(Pm_nominal_vect-expected_Pm);
figure(5)
plot(vect,Pm_error,'-o','LineWidth',1.2)
grid on 
title_fig3 = title('\textbf{Phase margin}','Interpreter','latex');
set(title_fig3,'FontSize',20);
xlabel_fig3 = xlabel('N: number of samples','Interpreter','latex');
ylabel_fig3 = ylabel('Error','Interpreter','latex');
set(xlabel_fig3,'FontSize',15);
set(ylabel_fig3,'FontSize',15);

% Gain margin
Gm_nominal_vect = Gm_nominal*ones(size(expected_Gm));
Gm_error = abs(Gm_nominal_vect-expected_Gm);
figure(6)
plot(vect,Gm_error,'-o','LineWidth',1.2)
grid on 
title_fig3 = title('\textbf{Gain margin}','Interpreter','latex');
set(title_fig3,'FontSize',20);
xlabel_fig3 = xlabel('N: number of samples','Interpreter','latex');
ylabel_fig3 = ylabel('Error','Interpreter','latex');
set(xlabel_fig3,'FontSize',15);
set(ylabel_fig3,'FontSize',15);

% Settling time
Sett_nominal_vect = Sett_nominal*ones(size(expected_Sett));
Sett_error = abs(Sett_nominal_vect-expected_Sett);
figure(7)
plot(vect,Sett_error,'-o','LineWidth',1.2)
grid on 
title_fig3 = title('\textbf{Settling time}','Interpreter','latex');
set(title_fig3,'FontSize',20);
xlabel_fig3 = xlabel('N: number of samples','Interpreter','latex');
ylabel_fig3 = ylabel('Error','Interpreter','latex');
set(xlabel_fig3,'FontSize',15);
set(ylabel_fig3,'FontSize',15);

% Overshoot
Over_nominal_vect = Over_nominal*ones(size(expected_Over));
Over_error = abs(Over_nominal_vect-expected_Over);
figure(8)
plot(vect,Over_error,'-o','LineWidth',1.2)
grid on 
title_fig3 = title('\textbf{Overshoot}','Interpreter','latex');
set(title_fig3,'FontSize',20);
xlabel_fig3 = xlabel('N: number of samples','Interpreter','latex');
ylabel_fig3 = ylabel('Error','Interpreter','latex');
set(xlabel_fig3,'FontSize',15);
set(ylabel_fig3,'FontSize',15);