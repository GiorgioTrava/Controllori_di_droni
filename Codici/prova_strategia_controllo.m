clear
close all
clc
%%
run Task1
% run Task2
run Task3

%%
s=tf('s');
omega_n = 10;
epsilon = 0.9;
% numeratore = omega_n^2;
% denominatore = [1, 2*omega_n*epsilon, omega_n^2];
F_required = omega_n^2/(s^2+2*epsilon*omega_n*s+omega_n^2);%tf(numeratore, denominatore)

y=step(F_required);
stepinfo(y)
% omega_n_2 = 15;
% epsilon_2 = 0.9;
% numeratore_2 = omega_n_2^2;
% denominatore_2 = [1, 2*omega_n_2*epsilon_2, omega_n_2^2];
% F_required_2 = tf(numeratore_2, denominatore_2);
% 
% omega_n_3 = 15;
% epsilon_3 = 0.95;
% numeratore_3= omega_n_3^2;
% denominatore_3 = [1, 2*omega_n_3*epsilon_3, omega_n_3^2];
% F_required_3 = tf(numeratore_3, denominatore_3);

t_a=5/(epsilon*omega_n)
perc_overshoot=exp(-pi*epsilon/sqrt(1-epsilon^2))*100

W3_inv=makeweight(1,[(omega_n) 0.9],0.00001);%makeweight(1.1,omega_n,0.00001)

figure(1)
bode(F_required,W3_inv)
hold on, grid on
% bode(F_required_2)
% bode(F_required_3)
legend

S_required = 1 - F_required;
% S_required_2 = 1 - F_required_2;
% S_required_3 = 1 - F_required_3;
figure(2)
bode(S_required)
grid on, hold on
% bode(S_required_2)
% bode(S_required_3)
%  prova=tf([1, 2*omega_n*epsilon, 0 ],[1, 2*omega_n*epsilon, omega_n^2]);
%  bode(prova)
s=zpk('s');
M=hinfnorm(S_required)%db2mag(1.47)
A=0.0001
W1_inv=(s+A*omega_n)/(s/M+omega_n)%(s+1e-3*(omega_n-20))/(s/1.47+(omega_n-20))

bode(W1_inv)
legend

L_required = F_required/(1-F_required);
[G,fase] = margin(L_required)
% L_required_2 = F_required_2/(1-F_required_2);
% L_required_3 = F_required_3/(1-F_required_3);
figure(3)
bode(L_required)
grid on, hold on
%bode(L_required_2)
%bode(L_required_3)
legend

figure
margin(5/(s*(0.1*s+1)))
% s = tf('s');
% LS = (1+0.001*s/omega_n)/(0.001+s/omega_n);
% bode(LS)
% WH = makeweight(1000,omega_n,0.0001);
% bode(WH)
% tf(WH)
