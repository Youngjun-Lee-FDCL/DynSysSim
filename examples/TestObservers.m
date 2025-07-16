classdef TestObservers < DynSystems
    properties
        HGO
        dataNames
    end
    methods
        function obj = TestObservers(name, s0_cell, logOn)
            HGO = HighGainObserver("HGO", s0_cell.hgo, logOn);
            obj = obj@DynSystems(name, {HGO, s0_cell.example}, logOn);
            obj.HGO = HGO;
        end

        function obj = setParams(obj, gains_hgo)
            obj.HGO.setParams( ...
                gains_hgo.n, ...
                gains_hgo.pole, ...
                gains_hgo.epsilon);
        end

        function ds = dynEqns(obj, t, s)
            [s_hgo, s_example]= obj.splitStates(s);
            ds_hgo = obj.HGO.dynEqns(t, s, s_example(1));
            A = [0 1 0;
                 0 0 1;
                 0 0 0];
            B = [0; 0; 1];
            ds_example = A*s_example + B*obj.b(s_example);
            ds = [ds_hgo; ds_example];
        end

        function out = b(~, s_example)
            [x1, x2, x3] = disperse(s_example);
            alpha = 2 * (x1^2 + x2^2 + x3^2)/(1 + x1^2 + x2^2 + x3^2);
            gs = -54*x1 -36*x2 -9*x3;
            gu = 54*x1 -36*x2 + 9*x3;
            out = alpha*gs + (1-alpha)*gu;
        end
    end
    
    methods (Static)
        function test()
            s0_cell.s_example = [0;-1;2];
            n = 3;
            epsilon = 0.064;
            sys = TestObservers();

        end
    end
end