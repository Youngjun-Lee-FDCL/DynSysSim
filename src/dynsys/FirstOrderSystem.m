classdef FirstOrderSystem < DynSystems
    properties
        taus
        dataNames = {"s1","s2","s3"}
    end
    methods
        function obj = FirstOrderSystem(name, s0, logOn)
            obj = obj@DynSystems(name, s0, logOn);
        end

        function obj = setParams(obj, taus)
            obj.taus = taus(:);
        end

        function ds = dynEqns(obj, ~, s, u)
            ds = 1./obj.taus .* (u - s);
             if obj.logData
                obj.data.state = s;
            end
        end
    end

    methods (Static)
        function test()
            addpath("../simulator/")
            addpath("../utils/")
            addpath("../solvers/")
            addpath("../datalogger/")

            s0 = zeros(3, 1);
            logOn = true;
            taus = [0.01; 0.01; 0.01];
            den = [taus(1) 1];
            num = 1;
            order = 1;
            sys = FirstOrderSystem("1st-order", s0, logOn).setParams(taus);
            sys_tr = TransferFunction("1st-order transfer", 0, logOn).setParams(order, den, num);
            
            t0 = 0;
            dt = 0.01;
            tf = 0.1;
            tspan = t0:dt:tf;
            input = @(t) stepCmd(t, 0, 1);
            sim1 = Simulator(sys).propagate(tspan, input);
            sim2 = Simulator(sys_tr).propagate(tspan, input);

            sim1_log = sim1.log();
            sim2_log = sim2.log();
            sim1_log.state.subplots(1, "states", "-","k",1);
            sim2_log.state.subplots(1, "states", "--", "r", 1);
            

            rmpath("../simulator/")
            rmpath("../utils/")
            rmpath("../solvers/")
            rmpath("../datalogger/")
        end
    end
end