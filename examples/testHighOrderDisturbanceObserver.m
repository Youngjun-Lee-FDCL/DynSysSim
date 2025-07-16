close all; clear; clc;

% set parameter
T = 100;
m0 = 1;
mc = 0.02;
t0 = 0;

f = @(x,u,t) T/(m0-mc * (t - t0));
F = @(t, u) T*m0/(m0-mc*(t-t0))^2;
d = @(t) 0.1;
Gamma0 = 10;
name = "Test";
s0 = zeros(6, 1);
logOn = true;
sys = HighOrderDisturbanceObserver(name, s0, logOn).setParams(f,F,d,Gamma0);
sim = Simulator(sys).propagate(tspan, u);
sim.report();
log = sim.log();

% plot
fig = figure(Name="Dist");
log.d.plot(fig.Number); hold on;
log.d_hat.plot(fig.Number);
