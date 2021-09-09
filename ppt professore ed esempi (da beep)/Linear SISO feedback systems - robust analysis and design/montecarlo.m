clc
clear
close all

gamma = ureal('gamma',2,'Perc',30);  % uncertain gain

tau = ureal('tau',1,'Perc',30);      % uncertain time-constant

% parameters of second order mode
wn = 50; 
xi = ureal('xi',0.5,'Range',[0.1 0.8]);      % uncertain damping ratio

% uncertain plant
P = tf(gamma,[tau 1]) * tf(wn^2,[1 2*xi*wn wn^2]);

figure(1),step(P,5)

figure(2),bode(P)

Parray = usample(P,100);

Pn = P.NominalValue;

Wt=tf(0.4*[1 1],[1/60 1]);

figure(3), bodemag((Pn-Parray)/Pn,Wt,'r')

Cn=tf([1, 1],[1 0]);

figure(4), margin(Pn*Cn)

figure(5), bode(minreal(Pn*Cn/(1+Pn*Cn)),1/Wt)

N=500;

for n=1:N
    n;
    gamma_mc(n)= 0.7*2 + (1.3-0.7)*2*rand(1,1);
    tau_mc(n)= 0.7*1 + (1.3-0.7)*1*rand(1,1);
    xi_mc(n)= 0.1 + (0.8-0.1)*rand(1,1);
    P_mc(n) = tf(gamma_mc(n),[tau_mc(n) 1]) * tf(wn^2,[1 2*xi_mc(n)*wn wn^2]);
    
    [Gm(n),Pm(n)] = margin(P_mc(n)*Cn);
    
    t=(0:.01:10);
    
    y=step(minreal(P_mc(n)*Cn/(1+P_mc(n)*Cn)),t);
    
    S = stepinfo(y,t,1);
    
    Sett(n)=S.SettlingTime;
    Over(n)=S.Overshoot;
    
end

figure(6),hist(Pm,100), grid, title('Phase margin')

figure(7),hist(Gm,100), grid, title('Gain margin')

figure(8),hist(Sett,100), grid, title('Settling time')

figure(9),hist(Over,100), grid, title('% Overshoot')

