%% n2_1_poles_and_zeros.m 
% Poles and zeros maps of Lateral Dynamics

run n_1_main.m
%% Nominal Poles 
P = pole(G); % compute nominal poles of the system 

%% Nominal Zeros
Z = tzero(G); % compute nominal zeros of the system 

%% poles and zeros maps

% nominal values
figure(1)
plot(real(P),imag(P),'x','color','[0 0 0]','linewidth',2)
hold on 
plot(real(Z),imag(Z),'o','color','[0 0 0]','linewidth',2) 
grid on
legend('pole', 'zero','Location', 'NorthWest')
title_fig1 = title('\textbf{Nominal Poles-Zeros}','Interpreter','latex');
set(title_fig1,'FontSize',20);
axis([-5.5 3 -4 4])
xlabel_fig1 = xlabel('Real Axis [$1/s$]','Interpreter','latex');
ylabel_fig1 = ylabel('Immaginary Axis [$1/s$]','Interpreter','latex');
set(xlabel_fig1,'FontSize',15);
set(ylabel_fig1,'FontSize',15);

% uncertain values
figure(2)
option_figure2 = pzoptions;
option_figure2.Title.String = '\textbf{Uncertain Poles-Zeros}';
option_figure2.Title.Interpreter = 'latex';
option_figure2.Title.FontSize = 20;
option_figure2.Xlabel.String = 'Real Axis';
option_figure2.Xlabel.FontSize = 15;
option_figure2.Ylabel.String = 'Imaginary Axis';
option_figure2.Ylabel.FontSize = 15;
pzplot(G,option_figure2)
% write the nominal value of pole 1 in figure 2
text(-4.35,0.3,'( -4.25; 0)')
% write the nominal value of pole 2 in figure 2 
text(1.3,3.6,'( 1.99; 3.60)')
% write the nominal value of pole 3 in figure 2
text(1.3,-3.6,'( 1.99; -3.60)')
% write the nominal value of zero in figure 2
text(-0.4,0.3,'( -0.198; 0)')