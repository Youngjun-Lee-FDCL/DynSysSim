clear; close all; clc; 


% add path: if you want to used functions/scripts located in different
% folders, please add thoes folder's path.
addpath(genpath("../src"))


s0 = zeros(3, 1);
logOn = true;
taus = [0.01; 0.02; 0.03]; % Time constant of each system
sys = FirstOrderSystem("1st-order", s0, logOn).setParams(taus);

t0 = 0;
dt = 0.005;
tf = 0.1;
tspan = t0:dt:tf;
input = @(t) stepCmd(t, 0, 1); % set an exogenous input applied into our system
sim = Simulator(sys).propagate(tspan, input);
sim.report(); % Show an elapsed simulation time
log = sim.log(); % Extract data saved during simulation.
log.state.plot(1, "states"); % The "state" field is determined at line 18 in FirstOrderSystem class file.
legend({"state 1 (\tau="+num2str(taus(1))+")", ...
        "state 2 (\tau="+num2str(taus(2))+")", ...
        "state 3 (\tau="+num2str(taus(3))+")",});


% Remove path: if you add path, remove those folders after scripts end.
rmpath(genpath("../src"))
