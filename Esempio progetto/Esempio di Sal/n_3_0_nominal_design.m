clear,clc,close all

run main.m;
% run n_3_1_design_weight_functions.m

s=tf('s');

% R = [R1 R2]

%                            d1*(s-1)+ b*c1
% R1 = d1 + (b*c1)/(s - 1) = --------------
%                                s-1

%                                      d2*s*(s-1) + 5*c2*(s-1)- b*c1*s                                             
% R2 = d2 + (5*c2)/s - (b*c1)/(s - 1)= -------------------------------
%                                                  s*(s-1) 

Ts=0.004;

G_ss = c2d(LAT_DYN.NominalValue,Ts);
G = tf(G_ss);
G.InputName = 'delta_lat';
G.OutputName = {'phi';'p'};


%% definition of Rp
Rp = tunableSS('Rp',2,1,2,Ts); % 2 states, 1 output, 2 inputs
% assegno le caratteristiche alle matrici che compongono Rp. Per vedere come
% si fa guarda la sezione proprietà qui: https://it.mathworks.com/help/control/ref/tunabless.html

% matrice Ap
Rp.A.Free(1,1) = 0; % =0 parametro bloccato, =1 parametro libero
Rp.A.Value(1,1) = 1;
Rp.A.Free(1,2) = 0;
Rp.A.Value(1,2) = 0;
Rp.A.Free(2,1) = 0;
Rp.A.Value(2,1) = 0;
Rp.A.Free(2,2) = 0;
Rp.A.Value(2,2) = 0;

% matrice Bp
Rp.B.Free(1,1) = 1; % =0 parametro bloccato, =1 parametro libero 
Rp.B.Free(1,2) = 0;
Rp.B.Value(1,2) = -Rp.B.Value(1,1); % B(1,2) dovrebbe essere uguale a -B(1,1)
Rp.B.Free(2,1) = 0;
Rp.B.Value(2,1) = 0;
Rp.B.Free(2,2) = 0;
Rp.B.Value(2,2) = 0.5;


% poichè di default le matrici C e D hanno componenti tunable non c'è
% bisogno di definirle

% assign inputs and output name 
Rp.InputName = {'p0';'p'};
Rp.OutputName = 'delta_lat';

%% definition of Rphi
Rphi = tunableGain('D_phi',1,1);
%Rphi = tunablePID('Rphi','P');

Rphi.Ts=Ts;
Rphi.InputName = 'e_phi';
Rphi.OutputName = 'p0';

%% Build the complete system (needed for systyne)

% Summing junctions
S1 = sumblk('%e_phi = phi0-%phi', Rphi.InputName, G.OutputName(1));
        
% % build all the feedback system
opt_connect = connectOptions('Simplify',false);
T0 = connect(G,Rp,Rphi,S1,'phi0',{'phi','p','delta_lat'},opt_connect);

%% use systune to tune T
% per usare systune si devono scrivere i punti A.i. e A.ii come
% requirements 

% Requirement 1
% target is a second order TF with:
omega_min=10; %[rad/s]
csi_min=0.9;
% so: (definizioni inutili)
tA = 4.5*1/(csi_min*omega_min); % max settling time
oversh = exp( (-csi_min*pi) / (sqrt(1-csi_min^2)) ); %max overshoot

refsys1 = omega_min^2/(s^2+2*csi_min*omega_min*s+omega_min^2);
refsys1 = c2d(refsys1,Ts);
Req1 = TuningGoal.Transient('phi0','phi',refsys1);


% Requirement 2 
% Req2 = TuningGoal.StepTracking('phi','delta_lat',refsys) 
            %credo che sia così:
            
refsys2 = tf(0.003*0.05,[1 0.05]); % gain = 0.003
refsys2 = c2d(refsys2,Ts);

% Req2 = TuningGoal.StepTracking('phi0','delta_lat',3);
Req2 = TuningGoal.Transient('phi0',Rp.y,refsys2,'step');

opt = systuneOptions('Display','final','RandomStart',5); %ho messo le stesse di hinf
[T,fSoft,gHard,info_systune] = systune(T0,[],Req1,opt);


%% use hinfsys to tune T

% R0=connect(Rp,Rphi,{'e_phi','p'},'delta_lat');
% % Non sarebbe meglio così?
% %       R0=connect(Rp,Rphi,{Rphi.u,Rp.u(2)},Rp.y);
% 
% Wp_d=c2d(Wp,Ts);
% Wc_d=c2d(Wc,Ts);
% 
% P=augw(G,Wp_d,Wc_d,[]);
% 
% opt = hinfstructOptions('Display','final','RandomStart',5);
% R = hinfstruct(P,R0,opt);
% 
% %% Plot
% figure(1)
% bodeplot(R)
% title('R')
% grid on
% 
% % step response
% L=R*G;
% F=L/(1+L);
% S=1/(1+L);
% Q=R/(1+L);
% 
% % step response compared to F2
% F2=omega_min^2/(s^2+2*csi*omega_min*s+omega_min^2);
% figure(2)
% step(F,F2)
% legend('F','F2');   % the shape should be similar
% grid on
% 
% % Q and control effort
% figure(3)
% bodeplot(Q,Wc_inv)  % |Q|<Wc_inv
% legend('Q','Wc_inv');
% grid on
% 
% % S and performance weight
% figure(4)
% bodeplot(S,Wp_inv)  % |S|<Wp_inv
% legend('S','Wp_inv');
% grid on
% 
% % F and F2 in freuency domain
% figure(5)
% bodeplot(F,F2)  % |S|<Wp_inv
% legend('F','F2');
% grid on
