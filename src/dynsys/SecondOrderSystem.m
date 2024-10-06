classdef SecondOrderSystem < DynSystems
    properties (Hidden)
        xi
        omega
        xLim = inf 
        xDotLim = inf      
        dataNames = {["x","xdot"],["xdot","xddot"],"Command"}
    end    
        
    methods
        function obj = SecondOrderSystem(name, s0, logOn)
            obj = obj@DynSystems(name, s0, logOn);
        end
        
        function obj = setParams(obj, xi, omega, xLim, xDotLim)
            obj.xi = xi;
            obj.omega = omega; % [rad/s]
            obj.xLim = xLim;
            obj.xDotLim = xDotLim;
        end

        function sDot = dynEqns(obj, ~, s, u) 
            if isempty(obj.omega) || isempty(obj.xi)
                error("Natural frequency or damping ratio is not set, please check if you call setParams before evaluating the propgate method")
            end
            s1 = s(1);
            s2 = s(2);
            A = [0, 1; -obj.omega^2, -2*obj.xi*obj.omega];
            B = [0; obj.omega^2];
            sDot = A * s(:) + B* u;
            
            if abs(s2) >= obj.xDotLim
                si = sign(s2);
                sDot(2) = si * min(si * sDot(2), 0);
                obj.state(2) = si * obj.xDotLim;
                sDot(1) = obj.state(2);
            end
            
            if abs(s1) >= obj.xLim
                si = sign(s1);
                obj.state(1) = si * obj.xLim;
                sDot(1) = si * min(si * sDot(1), 0);
                obj.state(2) = sDot(1);
                if sDot(1) == 0
                    sDot(2) = si * min(si * sDot(2), 0);
                end               
            end
            
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
            omega = 40;
            xLim =  deg2rad(10); %[rad]
            xDotLim = deg2rad(200); % [rad/s]
            logOn = true;
            sys = SecondOrderSystem('sys', [0;0], logOn).setParams(xi, omega, xLim, xDotLim);
            t0 = 0;
            dt = 0.0025;
            tf = 2;
            tspan = t0:dt:tf;
            v = deg2rad([10,-10,10]);
            input = @(t) stepCmd(t, [0, 0.2,0.4], v);
            
            sim = Simulator(sys).propagate(tspan, input);
            sim.report();
            log = sim.log;

            fig = figure(Name="state");
            log.state.plot(fig.Number, 1);
            plot(tspan, xLim*ones(length(tspan), 1), "--k")
            plot(tspan, -xLim*ones(length(tspan), 1), "--k")
            hold on;
            log.cmd.plot();
            fig = figure(Name = "state derivative");
            log.state.plot(fig.Number, 2); hold on;
            plot(tspan, xDotLim*ones(length(tspan),1), "--k")
            
            
            % Remove path
            rmpath("../simulator/")
            rmpath("../utils/")
            rmpath("../solvers/")
            rmpath("../datalogger/")
        end
    end
end