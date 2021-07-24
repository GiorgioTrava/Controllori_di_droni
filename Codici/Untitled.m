%% Robust performance

S_nominal=1-F_nominal;
bode(Wp*S_nominal + W*F_nominal,tf(1))