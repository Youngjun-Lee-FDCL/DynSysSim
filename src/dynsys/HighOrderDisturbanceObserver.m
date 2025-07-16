classdef HighOrderDisturbanceObserver < DynSystems
    % Consider a class of nonlinear system depicted by
    % x_dot = f(x,u,t) + F*d(t)
    % where d(t) is the disterbance vector
    properties
        f
        F
        d
        Gamma0
        dataNames
    end
    methods
        function obj = HighOrderDisturbanceObserver(name, s0, logOn)
            obj = obj@DynSystems(name, s0, logOn);
        end

        function obj = setParams(obj, f, F, d, Gamma0)
            obj.f = f;
            obj.F = F;
            obj.d = d;
            obj.Gamma0 = Gamma0;
        end

        function ds = dynEqns(obj, t, s, u)
            F_sinv = pinv(obj.F(t));
            dim_x = size(obj.F(t), 1);
            x = s(1:dim_x);
            z = s(dim_x+1:end);

            dx = obj.f(x, u , t) + obj.F(t)*obj.d(t);
            dz = F_sinv * obj.f(x, u, t) + obj.Gamma0 * (F_sinv*x - z);
            ds = vertcat(dx, dz);

            if obj.logData
                obj.data.x = x;
                obj.data.z = z;
                obj.data.d = obj.d(t);
                obj.data.d_hat = obj.get_d_hat(t, x, z);
            end
        end

        function out = get_d_hat(obj, t, x, z)
            F_sinv = pinv(obj.F(t));
            out = obj.Gamma0 * (F_sinv*x - z);
        end
    end

    methods (Static)
        function test()
            f = @(x, u, t) -2*x;
            F = @(t) 1;
            d = @(t) stepCmd(t, [0, 0.5, 1, 1.5], [1, 0, 1, 0]);
            Gamma0 = 50;
            
            % set system
            name = "Test";
            s0 = [0; 0];
            logOn = true;
            sys = HighOrderDisturbanceObserver(name, s0, logOn);
            sys = sys.setParams(f, F, d, Gamma0);

            % run sim
            tspan = 0:0.01:2;
            input = 0;
            sim = Simulator(sys).propagate(tspan, input);
            sim.report();
            log = sim.log();

            % plot
            fig = figure(Name="dist");
            log.d.plot(fig.Number); hold on;
            log.d_hat.plot(fig.Number, lineStyle="--");
        end
    end
end
