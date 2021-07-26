%% SCELTA GUADAGNI TUNING E VERIFICA NP, NS %%

% clc, clear, close all
 
% run Task1
% run Task2
% run Task3

 %% requisiti NP: funzione di trasferimento tra phi_0 e phi 
 
omega_n_systune = 16;
ksi_systune = 0.9;
numeratore_systune = omega_n_systune^2;
denominatore_systune = [1, 2*omega_n_systune*ksi_systune, omega_n_systune^2];
F_required_systune = tf(numeratore_systune, denominatore_systune);

omega_n_H_1 = 39;
ksi_H_1 = 0.9;
numeratore_H_1 = omega_n_H_1^2;
denominatore_H_1 = [1, 2*omega_n_H_1*ksi_H_1, omega_n_H_1^2];
F_required_H_1 = tf(numeratore_H_1, denominatore_H_1);

%% controllo------------------------------------------------------
tipo_controllo = ['systune'] %possibilità: systune ; Hinf 

switch tipo_controllo
    case 'systune'

%         Req1 = TuningGoal.Transient('phi_0','phi',F_required_systune);
%         Req2 = TuningGoal.Transient('phi_0','phi',F_required, 'step');      
        Req1 = TuningGoal.Tracking('phi_0','phi',2/(omega_n-3),0.0001,hinfnorm(S_required));
%         Req4 = TuningGoal.Overshoot('phi_0','phi',2); 
%         Req5 = TuningGoal.Gain('phi_error','delta_{lat}',0.34);
%         Req6 = TuningGoal.LoopShape('phi',F_required);
        Req2 = TuningGoal.WeightedGain('phi_error','delta_{lat}',makeweight(0.01,[25 1],3.5),[]);%25

        figure(11)
        bodemag(1/makeweight(0.01,[25 1],3.5))
        grid
        legend('1/W_L')
        
        [outerLoop_n_C,fSoft] = systune(outerLoop_n,[Req1],[Req2]);
        
        figure(12)
        viewGoal(Req1,outerLoop_n_C)
        hold on

        figure(13)
        viewGoal(Req2,outerLoop_n_C)
        hold on

        
        R_p_C = pid(outerLoop_n_C.Blocks.R_p)
        R_p_C.InputName = {'p_error'};       
        R_p_C.OutputName = {'delta_{lat}'};

        R_phi_C = pid(outerLoop_n_C.Blocks.R_phi)
        R_phi_C.InputName = {'phi_error'};       
        R_phi_C.OutputName = {'p_0'};

        
    case 'Hinf' 
 
        s = tf('s');

        L_req = makeweight(1000,omega_n_H_1,0.01);
        L_req.InputName = {'phi_error'}; 
        L_req.OutputName = {'phi_E_req'};
        
        Wn = 1/L_req;
        Wn.InputName = {'n_w'}; 
        Wn.OutputName = {'n'};

        Sum_phi_2 = sumblk('phi_error = phi_0 - phi_n');
        Sum_n = sumblk('phi_n = phi + n');

        R_H_inf = connect(R_p,R_phi,{'phi_error','p_error'},{'delta_{lat}','p_0'});
        P_H_inf = connect(SYSn,L_req,Wn,Sum_phi_2,Sum_n,Sum_p,{'phi_0','n_w','delta_{lat}','p_0'},...
            {'phi','p','phi_E_req','phi_error','p_error'});
        SisTot = connect(R_H_inf,P_H_inf,{'phi_0','n_w'},{'phi','p','phi_E_req'});
        
        rng('default')
        opt = hinfstructOptions('Display','final','RandomStart',5);
        [SisTot_C,gamma,info] = hinfstruct(SisTot,opt);
        
%         figure(11)
%         impulse(F_required_H_1,'--r')
%         hold on
%         impulse(L_req/(1+L_req),'--k')
% 
%         figure(12)
%         step(F_required_H_1,'--r')
%         hold on
%         step(L_req/(1+L_req),'--k')
        
        R_p_C = pid(SisTot_C.Blocks.R_p) 
        R_p_C.InputName = {'p_error'};       
        R_p_C.OutputName = {'delta_{lat}'};

        R_phi_C = pid(SisTot_C.Blocks.R_phi) 
        R_phi_C.InputName = {'phi_error'};       
        R_phi_C.OutputName = {'p_0'};
            
end
%% ricostruzione sistema

outerLoop_n_C = connect(R_p_C,R_phi_C,SYSn,Sum_phi,Sum_p,'phi_0',{'p','phi'},{'phi_error','delta_{lat}','p_0','phi','p'});

%% NP 
%plot sensitivities
S_n_C = getSensitivity(outerLoop_n_C,'phi_error');
figure(20)
bodemag(S_n_C,S_required)
grid
legend('controlled system sensitivity','required sensitivity')

%plot step response of inner loop
% figure(21)
% step(connect(SYSn,R_p_C,Sum_p,'p_0',{'p','phi'}))

figure(21)
subplot(2,1,1)
impulse(outerLoop_n_C(1))
subplot(2,1,2)
impulse(F_required,'g',outerLoop_n_C(2))
legend('F_required','controlled system')

figure(22)
subplot(2,1,1)
step(outerLoop_n_C(1))
subplot(2,1,2)
step(F_required,'g',outerLoop_n_C(2))
legend('F_required','controlled system')

figure(23)
subplot(2,1,1)
bode(outerLoop_n_C(1))
grid
subplot(2,1,2)
bode(F_required,'g',outerLoop_n_C(2))
grid
legend('F_required','controlled system')

%% nominal stability NS
%poles and zeros
figure(24)
pzmap(outerLoop_n_C)
grid
legend('controlled system')

% disk margin
L = getLoopTransfer(outerLoop_n_C,{'p','phi'},-1);
[DM,MM] = diskmargin(L);
% DM(1)   %gain e phase margin con in loop aperto alla volta
% DM(2)
MM      %unico affidabile-->controlla anche le combinazioni

%MIMO Nyquist criterion , Bode, poles and zeros(no cancellations)

figure(26)
nyquist((1+L(1,1))*(1+L(2,2))-L(1,2)*L(2,1))
legend('open loop controlled system')
ylim([-300 300])
figure(27)
bode((1+L(1,1))*(1+L(2,2))-L(1,2)*L(2,1))
legend('open loop controlled system')
figure(28)
pzmap((1+L(1,1))*(1+L(2,2))-L(1,2)*L(2,1))
legend('open loop controlled system')
%% step responce
step_controlled = stepinfo(outerLoop_n_C);
step_controlled(1)
step_controlled(2)
%% verifica control effort per doublet di phi_0 %NOTA: da fare dopo il tuning!!!!

t_f = 7;
timestep = 0.001;
t = [0:timestep:t_f];
doublet_phi_0 = interp1([0,1-timestep,1,3-timestep,3,5-timestep,5,t_f],[0,0,10,10,-10,-10,0,0],t);%deg!!!!!
figure(30)
plot(t,doublet_phi_0,'--')
hold on
plot([t(1) t(end)],[5 5],'-r')
plot([t(1) t(end)],[-5 -5],'-r')
%funzione di trasferimento phi_0 delta_lat
H1 = getIOTransfer(outerLoop_n_C,'phi_0','delta_{lat}');
H2 = outerLoop_n_C(2);
y = lsim(H1,doublet_phi_0*pi/180,t);
y_phi = lsim(H2,doublet_phi_0*pi/180,t);
y_req = lsim(F_required,doublet_phi_0*pi/180,t);
plot(t,y*180/pi)
plot(t,y_phi*180/pi,'--k')
plot(t,y_req*180/pi,'--g')

