classdef PointMass3D < DynSystems
    methods
        function obj = PointMass3D(name, pos0, vel0, logOn)
            assert(length(pos0)==3, "Dimension mismatched: the inital position must be 3 by 1 vector")
            assert(length(vel0)==3, "Dimension mismatched: the inital velocity must be 3 by 1 vector")
            s0 = [pos0(:); vel0(:)];
            obj = obj@DynSystems(name, s0, logOn);
        end

        function ds = dynEqns(~, ~, s, u)
            ds = [s(4:6); u(:)];            
        end
    end
    methods (Static)
        function test()
            pos0 = [0;0;0];
            vel0 = [1;0;0];
            logOn = true;
            sys = PointMass3D("Target", pos0, vel0, logOn);
            sim = Simulator(sys);

            t0 = 0;
            dt = 0.01;
            tf = 2;
            tspan = t0:dt:tf;
            sim.propagate(tspan, @(t) [0; 0; 0]);
            sim.report();
        end
    end
end