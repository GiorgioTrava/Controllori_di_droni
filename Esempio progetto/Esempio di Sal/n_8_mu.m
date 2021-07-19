clear,clc

run n_4_robust_analysis.m
close all

%% Singular Values of M

% for RS the maximum singular values of M should be less than 1(0 dB)
figure (1) % Analysis of singular values of M
sigma(M)
grid on
title_fig1 = title('\textbf{Singular Values of M}','Interpreter','latex');
set(title_fig1,'FontSize',20);
grid on
xlabel_fig1 = xlabel('Frequency','Interpreter','latex');
ylabel_fig1 = ylabel('Singular Values','Interpreter','latex');
set(xlabel_fig1,'FontSize',15);
set(ylabel_fig1,'FontSize',15);
set(findall(gcf,'type','line'),'linewidth',1.3)

%% Structured Singular Values-mu analysis

omega=logspace(-2,3,500); % vector of logaritmically spaced values 
bounds=mussv(frd(M,omega),[1,0]); % Real Delta, as required

% Because the test of RS was passed before we expect to pass it also in
% this case because this is a less conservative method
figure(2) % Analysis of the structured singular values of mu
sigma(bounds)
grid on
title_fig2 = title('\textbf{Structured Singular Values}','Interpreter','latex');
set(title_fig2,'FontSize',20);
grid on
xlabel_fig2 = xlabel('Frequency','Interpreter','latex');
ylabel_fig2 = ylabel('Singular Values','Interpreter','latex');
set(xlabel_fig2,'FontSize',15);
set(ylabel_fig2,'FontSize',15);
set(findall(gcf,'type','line'),'linewidth',1.3)
