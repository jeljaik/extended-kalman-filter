

%% Model Parameters
model.I   = diag([0.05 0.02 0.03]);
model.m   = 7;
model.dtInvDyn = 0.0001;
model.dtForDyn = 0.001;
model.dtKalman = 0.001;%0.01;
model.g   = 9.81;
model.bck = false;
t_min = 59;%42;
t_max = 64;%43;


%[a, a_filt] = extractViconData(model.dtKalman,1,59,64);

[yMeas,tMeas,model] = realMeasurement(model.dtKalman,model,1,t_min,t_max);