% This is second-order system class which is originated from source code of Prof. Shin
classdef NonlinearSecondOrderSystem < SecondOrderSystem        
    methods
        function obj = NonlinearSecondOrderSystem(name, s0, logOn)  
            obj = obj@SecondOrderSystem(name, s0, logOn);
        end
        
        function sDot = dynEqns(obj, ~, s, u) 
            x = s(1);
            x_dot = s(2);
%             x_ddot = - obj.sat(x, obj.xLim) * obj.omega^2 ...
%                      - 2 * obj.xi*obj.omega*obj.sat(x_dot, obj.xDotLim)...
%                      + u * obj.omega^2;
            x_ddot = -2 * obj.xi * obj.omega * x_dot + obj.omega^2 *obj.sat( obj.sat(u, obj.xLim)-x, 2*obj.xDotLim/obj.omega);
            sDot = [x_dot; x_ddot];
            if obj.logData
                obj.data.state = s;
                obj.data.stateDot = sDot;
                obj.data.cmd = u;
            end
        end                     
    end
    
    methods (Static)
        function test()
            % add path
            addpath("../simulator/")
            addpath("../utils/")
            addpath("../solvers/")
            addpath("../datalogger/")

            xi = 0.7;
            omega = 50; % [rad/s]
            xLim =  deg2rad(30); %[rad]
            xDotLim = deg2rad(500); % [rad/s]
            s0 = [0;0];
            logOn = true;
            sys = NonlinearSecondOrderSystem('sys', s0, logOn).setParams(xi, omega, xLim, xDotLim);   
            t0 = 0;
            dt = 0.0005;
            tf = 2;
            tspan = t0:dt:tf;
            command = deg2rad(29);
            input = @(t) command*sin(100*t);

            sim = Simulator(sys).propagate(tspan, input);
            sim.report();
            log = sim.log;

            % analytic solution
%             c = @(t, xi, omega, v) v*(1 - exp(-xi*omega*t)/sqrt(1-xi^2)...
%                 .*sin( omega*sqrt(1-xi^2)*t + atan( sqrt(1-xi^2)/xi)));
%             traj = c(sim.tspan, xi, omega, command);
             
            % Plots
            log.cmd.plot(1); hold on;
            log.state.plot(1, 1);
%             plot(sim.tspan, traj,"--")
            plot(tspan, sys.xLim*ones(length(tspan),1), "--k");
            plot(tspan, -sys.xLim*ones(length(tspan),1), "--k");
            legend("Command","Ours");%,"Analytic solutions")
            
            log.state.plot(2, 2); hold on;
            plot(tspan, sys.xDotLim*ones(length(tspan),1), "--k");
            plot(tspan, -sys.xDotLim*ones(length(tspan),1), "--k");

            % Remove path
            rmpath("../simulator/")
            rmpath("../utils/")
            rmpath("../solvers/")
            rmpath("../datalogger/")
        end
        function out = sat(x, lim)
            out = min(max(x, -lim),lim);
        end
    end
end