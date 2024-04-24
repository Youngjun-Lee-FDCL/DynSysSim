% [Ref.1] S. H. Mousavi, and A. Khaytian, Dead-zone model based Adaptive Backstepping Control for a Class of
% Uncertain Saturated System, IFAC WC, Milano, Italy, 2011.
classdef Mousavi_2nd_Order_Mass_Spring < DynSystems
    properties (Constant)
        m = 1.25
        c = 2
        k = 8
        u_sat = 12
        delta_r = 0.01
        M = 1200
        c1 = 10 % paragraph below Eq. (37)
        c2 = 10 % paragraph below Eq. (37)
    end
    properties
        dataNames = {{"x1", "x2", "hat a1", "hat a2", "hat \beta", "u_sc"},
                     {"input"},
                     {"x_d", "x_d dot", "x_d ddot"}}
    end
    methods
        function obj = Mousavi_2nd_Order_Mass_Spring(name, s0, logOn)
            obj = obj@DynSystems(name, s0, logOn);
        end

        function obj = setParams(obj)
            
        end

        function ds = dynEqns(obj, t, s, input)
            x1 = s(1);
            x2 = s(2);
            a1_hat = s(3);
            a2_hat = s(4);
            beta_hat = s(5);
            u_sc = s(6);
            
            % control input
            xd = input(1);
            x1d = input(2);
            x2d = input(3);

            z1 = x1 - xd; % Eq. (12)
            alpha1 = -obj.c1*z1; % Eq. (13)
            d_alpha1 = -obj.c1 * (x2 - x1d);
            z2 = x2 - x1d - alpha1; % Eq. (12)
            
            a1_hat = -obj.k/obj.m;
            a2_hat = -obj.c/obj.m;
            u_sc  = 0;
            u1 = -z1 + x2d + d_alpha1 - obj.c2 * z2 + a1_hat * x1 + a2_hat * x2 - u_sc;
            beta_hat = 1 / obj.m;
            u = beta_hat * u1;
            
            d_x1 = x2;
            d_x2 = -obj.k / obj.m * x1 - obj.c / obj.m * x2 + 1/obj.m * obj.sat(u); % Eq. (30)
            
            % adaptive laws
            d_a1_hat = 0.5*x1*z2; % Eq.(34)
            d_a2_hat = 0.5*x2*z2; % Eq.(34)
            d_beta_hat = -4*u1*z2; % Eq.(34)
            d_u_sc = 0;
            for i = 1:obj.M
                r = i * obj.delta_r;
                d_u_sc = d_u_sc + 0.001 * obj.dz(u, r)^2 * z2 * obj.delta_r;
            end
            ds = [d_x1; d_x2; d_a1_hat; d_a2_hat;
                  d_beta_hat; d_u_sc];
            if obj.logData
                obj.data.state = s;
                obj.data.input = obj.sat(u);
                obj.data.ref = input;
            end
        end

        function out = sat(obj, u)
            if u >= obj.u_sat
                out = obj.u_sat;
            elseif u <= -obj.u_sat
                out = -obj.u_sat;
            else
                out = u;
            end
        end

        function out = u_sc(obj, u)
            rho = 1/6;
            out = 0;
            for i = 1:obj.M
                r_i = i * obj.delta_r;
                out = out + rho * obj.dz(u, r_i)* obj.delta_r;
            end
        end
    end
    
    methods (Static)
        function test()
            addpath("../src/simulator/")
            addpath("../src/utils/")
            addpath("../src/solvers/")
            addpath("../src/datalogger/")

            x0 = [8;8]; % paragraph below Eq. (37)
            s0 = [x0; 0; 0; 1/1.25; 0];
            logOn = true;
            sys = Mousavi_2nd_Order_Mass_Spring("test", s0, logOn).setParams();

            input = @(t) [sin(t); cos(t); -sin(t)];
            tspan = 0:0.01:40;
            sim = Simulator(sys).propagate(tspan, input);
            log = sim.log();
            
            log.ref.subplots(1, "x_d", "--", "k"); hold on;
            log.state.subplots(1, "x", "-", "r"); 
            log.input.plot(2,"control input");
            log.state.subplots(3, "a_1 and a_2 estimates", "-","k");
            log.state.subplots(4, "U_sc", "-","k");

            rmpath("../src/simulator/")
            rmpath("../src/utils/")
            rmpath("../src/solvers/")
            rmpath("../src/datalogger/")
        end

        function out = dz(u, r)
            out = max(u - r, min(0, u + r));
        end
    end
end