%% SCELTA GUADAGNI E VERIFICA %%

% clc, clear, close all
 
% run Task1
% run Task2
% run Task3

 %% requisiti NP: funzione di trasferimento tra phi e phi_0 
omega_n = 10;
epsilon = 0.9;
numeratore = omega_n^2;
denominatore = [1, 2*omega_n*epsilon, omega_n^2];
F_required = tf(numeratore, denominatore);
 
omega_n_systune = 16;
epsilon_systune = 0.97;
numeratore_systune = omega_n_systune^2;
denominatore_systune = [1, 2*omega_n_systune*epsilon_systune, omega_n_systune^2];
F_required_systune = tf(numeratore_systune, denominatore_systune);

omega_n_H_1 = 39;
epsilon_H_1 = 0.9;
numeratore_H_1 = omega_n_H_1^2;
denominatore_H_1 = [1, 2*omega_n_H_1*epsilon_H_1, omega_n_H_1^2];
F_required_H_1 = tf(numeratore_H_1, denominatore_H_1);

omega_n_H_2 = 39;
epsilon_H_2 = 0.9;
numeratore_H_2 = omega_n_H_2^2;
denominatore_H_2 = [1, 2*omega_n_H_2*epsilon_H_2, omega_n_H_2^2];
F_required_H_2 = tf(numeratore_H_2, denominatore_H_2);

%% controllo------------------------------------------------------
tipo_controllo=['systune'] %possibilità: systune ; Hinf_1 ; Hinf_2 ; Open loop

switch tipo_controllo
    case 'systune'

        Req1 = TuningGoal.Transient('phi_0','phi',F_required_systune);
        Req2 = TuningGoal.Transient('phi_0','phi',F_required_systune, 'step');
        %TuningGoal.Margins
        %structured H_inf-->ucover-->sensitivity

        [outerLoop_n_C,fSoft] = systune(outerLoop_n,[Req1, Req2]);
        
        figure(20)
        viewGoal(Req1,outerLoop_n_C)
        hold on

        figure(21)
        viewGoal(Req2,outerLoop_n_C)
        hold on

        R_p_C=pid(outerLoop_n_C.Blocks.R_p)
        R_p_C.InputName = {'p_error'};       
        R_p_C.OutputName = {'delta_{lat}'};

        R_phi_C=pid(outerLoop_n_C.Blocks.R_phi)
        R_phi_C.InputName = {'phi_error'};       
        R_phi_C.OutputName = {'p_0'};

    case 'Hinf_1' 
        %S_phi = getIOTransfer(outerLoop,'phi_0','phi_error');
        %%tf([1, 2*omega_n_2*epsilon_2, 0 ],[1, 2*omega_n_2*epsilon_2, omega_n_2^2]);%
        s = tf('s');

        L_req=makeweight(1000,omega_n_H_1,0.1);%(1+0.001*s/omega_n_H)/(0.001+s/omega_n_H);
        L_req.InputName = {'phi_error'}; 
        L_req.OutputName = {'phi_E_req'};

        Wn=1/L_req;
        Wn.InputName = {'n_w'}; 
        Wn.OutputName = {'n'};

        Sum_phi_2 = sumblk('phi_error = phi_0 - phi_n');
        Sum_n = sumblk('phi_n = phi + n');

        R_H_inf=connect(R_p,R_phi,{'phi_error','p_error'},{'delta_{lat}','p_0'});

        P_H_inf=connect(SYSn,L_req,Wn,Sum_phi_2,Sum_n,Sum_p,{'phi_0','n_w','delta_{lat}','p_0'},...
            {'phi','p','phi_E_req','phi_error','p_error'});
        %opt_connect = connectOptions('Simplify',false);
        SisTot=connect(R_H_inf,P_H_inf,{'phi_0','n_w'},{'phi','p','phi_E_req'});
        rng('default')
        opt = hinfstructOptions('Display','final','RandomStart',5);
        [SisTot_C,gamma,info]=hinfstruct(SisTot,opt);
        
        figure(20)
        impulse(F_required_H_1,'--r')
        hold on
        impulse(L_req/(1+L_req),'--k')

        figure(21)
        step(F_required_H_1,'--r')
        hold on
        step(L_req/(1+L_req),'--k')
        
        
        R_p_C=pid(SisTot_C.Blocks.R_p) 
        R_p_C.InputName = {'p_error'};       
        R_p_C.OutputName = {'delta_{lat}'};

        R_phi_C=pid(SisTot_C.Blocks.R_phi) 
        R_phi_C.InputName = {'phi_error'};       
        R_phi_C.OutputName = {'p_0'};
    case 'Hinf_2'
        %prova struttura differente
        s=zpk('s');
        W1_inv=(s+1e-3*(omega_n_H_2-20))/(s/1.47+(omega_n_H_2-20));
        W1=(s/1.35+12)/(s+1e-3*12);%makeweight(1000,30,0.1);%1/W1_inv;%
        W1.InputName = {'p_error'};       
        W1.OutputName = {'z1'};

        W2=1/tf(200,1);
        W2.InputName = {'delta_{lat}'};       
        W2.OutputName = {'z2'};
        
        %     W3=;%makeweight(0.1,30,1000);
        %     W3.InputName = {'phi'};       
        %     W3.OutputName = {'z3'};
        
        figure(1000)
        bode(W1)
        hold on

        P=connect(SYSn,Sum_phi,Sum_p,W1,W2,{'phi_0','p_0','delta_{lat}'},... %ricordarsi di aggiungere W3 e z3 se necessari 
        {'z1','z2','phi_error','p_error'});
        [K_C,gamma,info]=hinfstruct(P,R_H_inf,opt);

        %plot
        figure(20)
        impulse(F_required_H_2,'--r')
        hold on
        %impulse(L_req/(1+L_req),'--k')

        figure(21)
        step(F_required_H_2,'--r')
        hold on
        %step(L_req/(1+L_req),'--k')
        
        R_p_C=pid(K_C.Blocks.R_p)
        R_p_C.InputName = {'p_error'};       
        R_p_C.OutputName = {'delta_{lat}'};
        
        R_phi_C=pid(K_C.Blocks.R_phi)
        R_phi_C.InputName = {'phi_error'};       
        R_phi_C.OutputName = {'p_0'};
        
    case 'Open loop'
        
        
end
%% ricostruzione sistema

outerLoop_n_C= connect(R_p_C,R_phi_C,SYSn,Sum_phi,Sum_p,'phi_0',{'p','phi'},{'phi_error','delta_{lat}'});
    
%     %%nota come recuperare la TF da "Fixed-Structure H-infinity Synthesis
%     %%with hinfstruct"
%     T = hinfstruct(T0,opt);
%     showTunable(T)
%     C = getBlockValue(T,'C');
%     F = getValue(F0,T.Blocks);  % propagate tuned parameters from T to F


%% plot
figure(20)
impulse(F_required,'g')
hold on
impulse(outerLoop_n_C(2))
legend

figure(21)
step(F_required,'g')
hold on
step(outerLoop_n_C(2))
legend


figure(22) 
bode(F_required,'g')
hold on
bode(outerLoop_n_C(2))
legend

figure(23) 
bode(outerLoop_n_C)
legend

figure(24)
pzmap(outerLoop_n_C)
legend

%% verifica control effort per doublet di phi_0 %NOTA: da fare dopo il tuning!!!!

t_f=20;
timestep=0.001;
t=[0:timestep:t_f];
doublet_phi_0=interp1([0,1-timestep,1,3-timestep,3,5-timestep,5,t_f],[0,0,10,10,-10,-10,0,0],t);%deg!!!!!
figure(8)
plot(t,doublet_phi_0,'--')
hold on
plot([t(1) t(end)],[5 5],'-r')
plot([t(1) t(end)],[-5 -5],'-r')
%funzione di trasferimento phi_0 delta_lat
H1 = getIOTransfer(outerLoop_n_C,'phi_0','delta_{lat}');
H2 = outerLoop_n_C(2);
y=lsim(H1,doublet_phi_0*pi/180,t);
y_phi=lsim(H2,doublet_phi_0*pi/180,t);
y_req=lsim(F_required,doublet_phi_0*pi/180,t);
plot(t,y*180/pi)
plot(t,y_phi*180/pi,'--k')
plot(t,y_req*180/pi,'--g')

