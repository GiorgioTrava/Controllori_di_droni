%% SCELTA GUADAGNI E VERIFICA %%
% clc, clear, close all
% 
% run Task1
% run Task2
% run Task3

 %% requisiti NP: funzione di trasferimento tra phi e phi_0 
omega_n_2 = 10;
epsilon_2 = 0.9;
numeratore_2 = omega_n_2^2;
denominatore_2 = [1, 2*omega_n_2*epsilon_2, omega_n_2^2];
G_required_2 = tf(numeratore_2, denominatore_2);


omega_n = 15;
epsilon = 0.98;
numeratore = omega_n^2;
denominatore = [1, 2*omega_n*epsilon, omega_n^2];
G_required = tf(numeratore, denominatore);
%% controllo
tipo_controllo=['systune'] %possibilità: Hinf; systune;

switch tipo_controllo
    case 'systune'
   
    Req1 = TuningGoal.Transient('phi_0','phi',G_required);
    Req2 = TuningGoal.Transient('phi_0','phi',G_required, 'step');
    %TuningGoal.Margins
    %structured H_inf-->ucover-->sensitivity
    figure(20)
    viewGoal(Req1)
    hold on
    impulse(G_required_2,'--r')
    figure(21)
    viewGoal(Req2)
    hold on
    step(G_required_2,'--r')

    [outerLoop_n_C,fSoft] = systune(outerLoop_n,[Req1, Req2]);
    case 'Hinf' 
%     S_phi = getIOTransfer(outerLoop,'phi_0','phi_error');
    WP=tf([1, 2*omega_n_2*epsilon_2, omega_n_2^2],[1, 2*omega_n_2*epsilon_2, 0 ])
    WP.InputName = {'phi_error'}; 
    WP.OutputName = {'phi_model'};
    
    R_H_inf=connect(R_p,R_phi,Sum_p,{'phi_error','p'},'delta_{lat}');
   
    P_H_inf=connect(SYSn,Sum_phi,WP,{'phi_0','delta_{lat}'},{'phi_error','p','phi_model'},'phi');
    %P_H_inf=connect(G_required_2,Sum_e,outerLoop_n,'phi_0',);
    opt = hinfstructOptions('Display','final','RandomStart',1);
    [R_H_C]=hinfstruct(P_H_inf,R_H_inf,opt);
    pid(R_H_C.Blocks.R_p)
    pid(R_H_C.Blocks.R_phi)
    outerLoop_n_C = connect(R_H_C,SYSn,Sum_phi,'phi_0',{'p','phi'},{'phi_error','delta_{lat}'});
%getIOTransfer(lft(P_H_inf,R_H_C),'phi_0',{'phi'});
    
    
end
pid(outerLoop_n_C.Blocks.R_p)
pid(outerLoop_n_C.Blocks.R_phi)
%% plot
figure(22) 
bode(G_required,'g')
hold on
bode(outerLoop_n_C(2))
figure(23) 
%bode(G_required_2, 'r')
bode(outerLoop_n_C)

%viewGoal(Req,CL)
figure(24)
% pzmap(outerLoop_n)
% hold on
pzmap(outerLoop_n_C)

figure(20)
hold on
impulse(outerLoop_n_C(2))
figure(21)
hold on
step(outerLoop_n_C(2))




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
plot(t,y*180/pi)
plot(t,y_phi*180/pi,'--g')

