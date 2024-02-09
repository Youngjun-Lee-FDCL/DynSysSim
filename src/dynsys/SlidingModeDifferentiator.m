% [Ref] M. Chen, P. Shi and C. -C. Lim, "Robust Constrained Control for 
% MIMO Nonlinear Systems Based on Disturbance Observer," in IEEE 
% Transactions on Automatic Control, vol. 60, no. 12, pp. 3281-3286, 
% Dec. 2015, doi: 10.1109/TAC.2015.2450891.
classdef SlidingModeDifferentiator < DynSystems
    properties
        eps0
        eps1
        dataNames = {["rho01", "rho02"], ["rho11","rho12"], ["zeta01", "zeta02"]}
    end
    methods
        function obj = SlidingModeDifferentiator(name, s0, logOn)
            obj = obj@DynSystems(name, s0, logOn);
        end
        
        function obj = setParams(obj, eps0, eps1)
            obj.eps0 = eps0(:);
            obj.eps1 = eps1(:);
            assert( length(obj.eps0) == length(obj.state)/2, "The number of eps0 must be matched with the size of the state")
            assert( length(obj.eps1) == length(obj.state)/2, "The number of eps1 must be matched with the size of the state")
        end

        function ds = dynEqns(obj, ~, s, u)
            u = u(:);
            n = length(u);
            rho0 = s(1:n);
            rho1 = s(n+1:2*n);

            d_rho0 = -obj.eps0 .* abs( rho0 - u ).^(1/2) .* sign( rho0 - u ) + rho1;
            d_rho1 = -obj.eps1 .* sign( rho1 - d_rho0);

            ds = [d_rho0; d_rho1];
            if obj.logData
                obj.data.rho0 = rho0;
                obj.data.rho1 = rho1;
                obj.data.zeta0 = d_rho0;
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

            name = "test";
            s0 = [0;1;0;0];
            logOn = true;
            eps0 = [7.5; 7.5];
            eps1 = [0.0001; 0.0001];
            sys = SlidingModeDifferentiator(name, s0, logOn).setParams(eps0, eps1);

            f = @(t) [sin(2*t); cos(2*t)];
            d_f = @(t) [2*cos(2*t); -2*sin(2*t)];
            tspan = 0:0.01:10;
            sim = Simulator(sys).propagate(tspan, f);
            sim.report();
            log = sim.log;
            
            log.rho0.plot(1); 
            plot(tspan, f(tspan));
            ylabel("\rho_{0}");
            log.rho1.plot(2); 
            ylabel("\rho_{1}");
            log.zeta0.plot(3); 
            plot(tspan, d_f(tspan));ylabel("\zeta_{0}");
            
            % Remove path
            rmpath("../simulator/")
            rmpath("../utils/")
            rmpath("../solvers/")
            rmpath("../datalogger/")
        end
    end
end