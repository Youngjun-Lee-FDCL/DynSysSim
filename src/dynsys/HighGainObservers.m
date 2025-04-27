classdef HighGainObservers < DynSystems
    properties
        n_obs
        dataNames
    end
    methods
        function obj = HighGainObservers(name, s0_cell, logOn, n_obs)
            obs = cell(n_obs, 1);
            for i = 1:n_obs
                obs{i} = HighGainObserver(name, s0_cell{i}, logOn);
            end
            obj = obj@DynSystems(name, obs, logOn);
            obj.n_obs = n_obs;
        end

        function obj = setParams(obj, params)
            for i = 1:obj.n_obs
                obj.subSysCell{i}.setParams(params(i));
            end
        end

        function ds = dynEqns(obj, t, s, signal)
            stateCells = obj.splitStates_cell(s);
            ds = cell(obj.n_obs, 1);
            for i = 1:obj.n_obs
                subState = stateCells{i};                
                ds{i} = obj.subSysCell{i}.dynEqns(t, subState, signal(i));
            end
            ds = vertcat(ds{:});
            if obj.logData
                for i = 1:obj.n_obs
                    subsys = obj.subSysCell{i};
                    obj.data.("sys"+num2str(i)+"_x1") = subsys.data.x1;
                    if subsys.n > 1
                        obj.data.("sys"+num2str(i)+"_x2") = subsys.data.x2;
                    else
                        obj.data.("sys"+num2str(i)+"_x2") = subsys.data.dx1;
                    end
                end
            end
        end

        function out = get_x1(obj, state)
            subState_cell = obj.splitStates_cell(state);
            for i = 1:obj.n_obs
                subState = subState_cell{i};
                out(i) = obj.subSysCell{i}.get_x(subState, 1);
            end
            out = out(:);
        end

        function out = get_x2(obj, state, input)
            subState_cell = obj.splitStates_cell(state);
            for i = 1:obj.n_obs
                subState = subState_cell{i};
                dx1 = obj.subSysCell{i}.dynEqns(0, subState, input(i));
                out(i) = dx1(1);
            end
            out = out(:);
        end
    end

    methods (Static)
        function test()
            addpath("../simulator/")
            addpath("../utils/")
            addpath("../solvers/")
            addpath("../datalogger/")
            % set parameter
            param1 = struct("n", 1, "pole", -10, "epsilon", 0.1);
            param2 = struct("n", 2, "pole", -10, "epsilon", 0.1);
            param3 = struct("n", 3, "pole", -10, "epsilon", 0.1);
            params = [param1, param2, param3];

            % set initial state
            s0 = {zeros(param1.n, 1), zeros(param2.n, 1), zeros(param3.n, 1)};
            
            % set system
            logOn = true;
            n_obs = length(params);
            sys = HighGainObservers("test", s0, logOn, n_obs).setParams(params);

            % set reference
            A = 0.001;
            omega = 100;
            f = @(t) A*sin(omega*t);
            df = @(t) A*omega*cos(omega*t);
            ddf = @(t) -A*omega^2*sin(omega*t);
            dddf = @(t) -A*omega^3*cos(omega*t);
            fs = @(t) [f(t); f(t); f(t)];

            % run sim
            ts = 0:0.001:1;
            sim = Simulator(sys).propagate(ts, fs);
            sim.report();

            % plot
            log = sim.log();
            fig=figure(Name="x1"); hold on;
            plot(ts, f(ts),"--k"); 
            log.sys1_x1.plot(fig.Number);
            log.sys2_x1.plot(fig.Number);
            log.sys3_x1.plot(fig.Number);
            legend("True", "sys1", "sys2", "sys3");
            
            fig = figure(Name="x2"); hold on;
            plot(ts, df(ts), "--k");
            log.sys1_x2.plot(fig.Number);
            log.sys2_x2.plot(fig.Number);
            log.sys3_x2.plot(fig.Number);
            legend("True", "sys1", "sys2", "sys3");

            % remove path
            rmpath("../simulator/")
            rmpath("../utils/")
            rmpath("../solvers/")
            rmpath("../datalogger/")
        end
    end
end