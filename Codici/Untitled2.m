%% Hinf norm

omega_weight=10;        %[rad/s] minimum cross-over frequency
csi_weight=0.9;         % minimum damping ratio

s=tf('s');
F_weight=omega_weight^2/(s^2+2*csi_weight*omega_weight*s+omega_weight^2);
S_weight=1-F_weight;
Wp=1/S_weight;          % performance weight
Wp.InputName='phi_error'; % input signal of W1
Wp.OutputName='z_p'; % output signal of W1
Wp_inv=S_weight;

Wc = [];                % control weight

%% Controller
Sum_p = sumblk('p_error = p_0 - p');
R = connect(R_phi,Sum_p,R_p,{'phi_error','p'},'delta_{lat}');

%% Plant
Sum_phi = sumblk('phi_error = phi_0 - phi');
P = connect(SYSn,Sum_phi,Wp,{'phi_0','delta_{lat}'},{'z_p','phi','p','phi_error'});

%% Hinf
opt=hinfstructOptions('Display','final','RandomStart',5);
R_c = hinfstruct(P,R,opt);

R_p_C=pid(R_c.Blocks.R_p)
R_p_C.InputName = {'p_error'};       
R_p_C.OutputName = {'delta_{lat}'};

R_phi_C=pid(R_c.Blocks.R_phi)
R_phi_C.InputName = {'phi_error'};       
R_phi_C.OutputName = {'p_0'};
outerLoop_n_C= connect(R_p_C,R_phi_C,SYSn,Sum_phi,Sum_p,'phi_0',{'p','phi'},{'phi_error','delta_{lat}'});


%% Plot
figure(1)
% pzmap(outerLoop_n)
% hold on
pzmap(outerLoop_n_C)
legend

figure(2)
impulse(F_required,'g')
hold on
impulse(outerLoop_n_C(2))
legend

figure(3)
hold on
step(outerLoop_n_C(2))
step(F_required,'g')
legend

t_f=20;
timestep=0.001;
t=[0:timestep:t_f];
doublet_phi_0=interp1([0,1-timestep,1,3-timestep,3,5-timestep,5,t_f],[0,0,10,10,-10,-10,0,0],t);%deg!!!!!
figure(4)
plot(t,doublet_phi_0,'--')
hold on
plot([t(1) t(end)],[5 5],'-r')
plot([t(1) t(end)],[-5 -5],'-r')
H1 = getIOTransfer(outerLoop_n_C,'phi_0','delta_{lat}');
H2 = outerLoop_n_C(2);
y=lsim(H1,doublet_phi_0*pi/180,t);
y_phi=lsim(H2,doublet_phi_0*pi/180,t);
y_req=lsim(F_required,doublet_phi_0*pi/180,t);
plot(t,y*180/pi)
plot(t,y_phi*180/pi,'--k')
plot(t,y_req*180/pi,'--g')

figure(5) 
bode(F_required,'g')
hold on
bode(outerLoop_n_C(2))
legend