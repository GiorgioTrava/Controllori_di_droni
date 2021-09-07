%% AEROSPACE CONTROL SYSTEMS %%


%% Pid controller


R_p = tunablePID('rrate','PID');
R_phi = tunablePID('rangle','P');

R_p.InputName = {'ep'};
R_p.OutputName = {'DELTA_{lat}'};

R_phi.InputName = {'ephi'};
R_phi.OutputName = {'p0'};


%% costruisco il sistema a blocchi


sum_inner = sumblk('ep = p0 - p',1);
sum_outer = sumblk('ephi = phi0 - phi',1);
sys_complete = connect(G,R_p,R_phi,sum_inner,sum_outer,'phi0',{'p','phi'},{'ephi','phi','DELTA_{lat}','ep'});
sys_complete.Inputname = 'phi0';
sys_complete.OutputName = {'p','phi'};
sys_complete_n = getNominal(sys_complete); %extract nominal system

    
