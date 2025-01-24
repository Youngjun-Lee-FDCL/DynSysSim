% [Ref] M. Chen, P. Shi and C. -C. Lim, "Robust Constrained Control for 
% MIMO Nonlinear Systems Based on Disturbance Observer," in IEEE 
% Transactions on Automatic Control, vol. 60, no. 12, pp. 3281-3286, 
% Dec. 2015, doi: 10.1109/TAC.2015.2450891.
classdef SlidingModeDifferentiator < DynSystems
    properties
        lambda0 = 1.1
        lambda1 = 1.5
        lambda2 = 2
        lambda3 = 3
        L
        dataNames = {["z01", "z02"], ["z01", "z02"]}
    end
    methods
        function obj = SlidingModeDifferentiator(name, s0, logOn)
            obj = obj@DynSystems(name, s0, logOn);
        end
        
        function obj = setParams(obj, L)
            obj.L = L(:); % Can be time varying such that |u(t)^{order+1}| < L(t)
%             assert( length(obj.eps0) == length(obj.state)/2, "The number of eps0 must be matched with the size of the state")
%             assert( length(obj.eps1) == length(obj.state)/2, "The number of eps1 must be matched with the size of the state")
        end

        function ds = dynEqns(obj, ~, s, u)
            u = u(:);
            n = length(u);
            z0 = s(1:n);
            z1 = s(n+1:2*n);
            order = 1;

            d_z0 = -obj.lambda1 * (obj.L).^(1/(order+1)) .* abs( z0 - u ).^(order/(order+1)) .* sign( z0 - u ) + z1;
            d_z1 = -obj.lambda0 * obj.L .* sign( z1 - d_z0);

            ds = [d_z0; d_z1];
            if obj.logData
                obj.data.z0 = z0;
                obj.data.z1 = z1;
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
            omega = 2;
            L = 6*[1;1];
%             eps0 = 1.1*[7.5; 7.5];
%             eps1 = [0.0001; 0.0001];
            sys = SlidingModeDifferentiator(name, s0, logOn)...
                .setParams(L);

            f = @(t) [sin(omega*t); cos(omega*t)];
            d_f = @(t) [omega*cos(omega*t); -omega*sin(omega*t)];
            tspan = 0:0.01:10;
            sim = Simulator(sys).propagate(tspan, f);
            sim.report();
            log = sim.log;
            
            % plot
            fig=figure(Name="original signal"); hold on;
            log.z0.plot(fig.Number); 
            plot(tspan, f(tspan), "k");
            ylabel("z_{0} and u");

            fig = figure(Name="1st derivative");
            plot(tspan, d_f(tspan), "k");
            log.z1.plot(fig.Number); 
            ylabel("z_{1}");
            
            % Remove path
            rmpath("../simulator/")
            rmpath("../utils/")
            rmpath("../solvers/")
            rmpath("../datalogger/")
        end
    end
end