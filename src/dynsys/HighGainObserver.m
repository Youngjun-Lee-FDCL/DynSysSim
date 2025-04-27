classdef HighGainObserver < DynSystems
    properties
        n
        dataNames
        alpha
        epsilon
    end
    methods
        function obj = HighGainObserver(name, s0, logOn)
            obj = obj@DynSystems(name, s0, logOn);
        end

        function obj = setParams(obj, param)
            obj.n = param.n;
            for i = 1:param.n
                obj.dataNames = [obj.dataNames, "x"+num2str(i)];
                obj.dataNames = [obj.dataNames, "dx"+num2str(i)];
            end
            obj.dataNames = cellstr(obj.dataNames);
            if length(param.poles) ~= obj.n
                error("Number of poles must match with the order")
            end
            coeff=poly(param.poles);
            obj.alpha=coeff(2:end);% disregard the leading coefficient
            obj.epsilon = param.epsilon;
        end
        
        function ds = dynEqns(obj, ~, s, y)
            e = y-s(1);
            if obj.n == 1
                ds = obj.alpha(end) / obj.epsilon^(obj.n) * e;
            else                
                for k = 1:obj.n-1
                    ds(k) = s(k+1) + (obj.alpha(k)/obj.epsilon^(k))*e;
                end
                ds_n = obj.alpha(end) / obj.epsilon^(obj.n) * e;
                ds = [ds(:); ds_n];
            end
            if obj.logData
                for i = 1:obj.n
                    obj.data.("x"+num2str(i)) = s(i);
                    obj.data.("dx"+num2str(i)) = ds(i);
                end
            end
        end
        
        function out = get_x(~, state, num)
            out = state(num);
        end
    end

    methods (Static)
        function test()
            addpath("../simulator/")
            addpath("../utils/")
            addpath("../solvers/")
            addpath("../datalogger/")

            % set parameters
            param.n = 2;
            param.poles = [-0.5 ,-0.5];
            param.epsilon = 0.01;
            s0 = zeros(param.n, 1);
            % set system
            logOn = true;
            
            sys = HighGainObserver("test",s0, logOn).setParams(param);
            
            % set first order system
            s0_1st = 0;
            taus = -param.epsilon/sum(param.poles);
            sys_1st = FirstOrderSystem("1st", s0_1st, logOn).setParams(taus);

            % set reference
            omega = 10;
            f = @(t) sin(omega*t);
            df = @(t) omega*cos(omega*t);
            ddf = @(t) -omega^2*sin(omega*t);
            dddf = @(t) -omega^3*cos(omega*t);

            % run sim
            ts = 0:0.0001:1;
            sim = Simulator(sys).propagate(ts, f);
            sim.report();

            % run 1st-order sim
            sim_1st = Simulator(sys_1st).propagate(ts, f);
            sim_1st.report();

            % plot
            log = sim.log();
            log_1st = sim_1st.log();
            fig = figure(Name="x1"); hold on;
            log.x1.plot(fig.Number, color="r",lineStyle="--");
            log_1st.state.plot(fig.Number,color="b",lineStyle="--");
            plot(ts, f(ts),"-k"); legend("Estimate",  "1st", "True");

            fig = figure(Name="x2");
            if param.n > 1
                log.x2.plot(fig.Number, Color = "r", lineStyle="--");
            else
                log.dx1.plot(fig.Number, Color = "r", lineStyle="--");
            end
            log_1st.ds.plot(fig.Number, Color = "b", lineStyle="--");
            plot(ts, df(ts),"--k"); legend("Estimate",  "1st", "True");
            
            if param.n >= 3
                fig = figure(Name="x3");
                log.x3.plot(fig.Number);
                plot(ts, ddf(ts),"--k"); legend("Estimate", "True");
            end

            if param.n >=4
                fig = figure(Name="x4");
                log.x4.plot(fig.Number);
                plot(ts, dddf(ts),"--k"); legend("Estimate", "True");
            end
            rmpath("../simulator/")
            rmpath("../utils/")
            rmpath("../solvers/")
            rmpath("../datalogger/")
        end
    end
end