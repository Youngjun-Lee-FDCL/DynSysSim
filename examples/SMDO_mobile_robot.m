% TODO: fine-tuning is needed.
classdef SMDO_mobile_robot < DynSystems
    properties (Constant)
        c1 = -2.384
        c2 = -0.065
        c3 = -3.384
        c4 = 9.060
        b1 = 0.086
        b2 = -1.206
    end

    properties
        K1 = diag([3,3])
        K2 = diag([3,3])
        L11 = diag([10,10])
        L12 = diag([10,10])
        L21 = diag([50,50])
        L22 = diag([50,50])
        eps210 = 2.5
        eps211 = 1
        eps220 = 2.5
        eps221 = 1
        q211 = 50
        q221 = 50
        u10 = 10 % symmetric saturation level of v
        u20 = 10 % symmetric saturation level of omega
        rho0
        smdiff_zeta_20
        smdiff_zeta_s10
        smdiff_zeta_s20
        dataNames = {["X", "psi"], ["v","omega"], ["u1","u2"], ...
                     ["s11","s12"],["\zeta_{11}","\zeta_{12}"], ["\rho_{s101}", "\rho_{s102}"], ...
                     ["s21","s22"],["\zeta_{21}","\zeta_{22}"], ["\rho_{s201}", "\rho_{s202}"], ...
                     ["\alpha_{11}", "\alpha_{12}"],  ["\rho_{21}", "\rho_{22}"], ...
                     ["\zeta_{21}",  "\zeta_{22}"], ...
                     ["D_{11} estimate", "D_{12} estimate"], ["D_{21} estimate", "D_{22} estimate"], ...
                     ["D_{21}", "D_{22}"],...
                     ["z_{11}", "z_{12}"],...
                     ["z_{21}", "z_{22}"],...
                     ["s_{211}", "s_{212}"]}
    end
    methods
        function obj = SMDO_mobile_robot(name, s0, logOn)
            obj = obj@DynSystems(name, s0, logOn);
            obj.smdiff_zeta_20 = SlidingModeDifferentiator("zeta_20", [0;0;0;0], false);
            obj.smdiff_zeta_s10 = SlidingModeDifferentiator("zeta_s10", [0;0;0;0], false);
            obj.smdiff_zeta_s20 = SlidingModeDifferentiator("zeta_s20", [0;0;0;0], false);
        end
        
        function obj = setParams(obj)
            obj.rho0 = 1;
            obj.smdiff_zeta_20.setParams([5, 5], [1,1]);
            obj.smdiff_zeta_s10.setParams([obj.eps210;obj.eps220], [obj.eps211;obj.eps221]);
            obj.smdiff_zeta_s20.setParams([15;15], [20;1]);
        end

        function ds = dynEqns(obj, t, s, u)
                % inputs 
                x1d = u(1:2);
                d_x1d = u(3:4);

                % states
                x1 = s(1:2);
                x2 = s(3:4);
                d_Xm = x2(1);
                d_psi = x2(2);

                % states (for controllers)
                z1 = s(5:6);
                z2 = s(7:8);
                D1_hat = s(9:10);
                D2_hat = s(11:12);
                int_sign_s11 = s(13:14);
                int_sign_s21 = s(15:16);
                rho20 = s(17:20);
                rho_s10 = s(21:24);
                rho_s20 = s(25:28);
                
                % model info
                F1 = [0;0];
                G1 = eye(2);
                F2 = [obj.c1 * d_Xm + obj.c2 * d_psi^2;
                     obj.c3 * d_psi + obj.c4 * d_Xm * d_psi];
                G2 = diag([obj.b1, obj.b2]);

                % virtual control input of backstepping control law
                alpha1 = inv(G1) * ( -obj.K1 * z1 - F1 - D1_hat - d_x1d);

                % state errors and sliding modes
                e1 = x1 - x1d;
                e2 = x2 - alpha1;
                s10 = z1 - e1;
                s20 = z2 - e2;
               
                % dynamic equations of the sliding mode differentiators
                d_rho20 = obj.smdiff_zeta_20.dynEqns(t, rho20, alpha1);
                zeta_20 = d_rho20(1:2);
                
                % control  input
                v = -inv(G2*obj.rho0) * ( obj.K2 * z2  + F2 + D2_hat - zeta_20 + G1.' * e1); % Eq. (43)
                u = sign(v).*min([obj.u10; obj.u20], abs(v(:)));% saturated control input
                
                % dynamic systems for contorller

                % actual disturbances
                D2 = [0.1*sin(t)+0.1*d_psi^2;
                     0.1*cos(t)-0.1*d_Xm*d_psi]; % pp.3285
                
                % dynamic equations of plant
                d_x1 = x2; % Eq. (49)
                d_x2 = F2 + G2*u + D2; % Eq. (49)
                
                % dynamic equation of sliding mode estimator
                d_rho_s10 = obj.smdiff_zeta_s10.dynEqns(t, rho_s10, s10);
                d_rho_s20 = obj.smdiff_zeta_s20.dynEqns(t, rho_s20, s20);
                zeta_s10 = d_rho_s10(1:2); % + omega_s1, Eq.(15)
                zeta_s20 = d_rho_s20(1:2); % + omega_s2, 
                s11 = s10 + zeta_s10; % Eq.(16)
                s21 = s20 + zeta_s20; % Eq.(41)
                d_z1 = F1 + G1 * (e2 + alpha1) + D1_hat - d_x1d; % Eq.(20)
                d_z2 = F2 + G2 * obj.rho0 * v + D2_hat- zeta_20; % Eq.(42)
                d_D1_hat = -zeta_s10 - obj.L11 * abs( s11 ).^(1/2) .* sign( s11 ) - obj.L12 * int_sign_s11; % Eq. (16)
                d_D2_hat = -zeta_s20 - obj.L21 * abs( s21 ).^(1/2) .* sign( s21 ) - obj.L22 * int_sign_s21; % Eq. (41) with n = 2
                d_int_sign_s11 = sign(s11);
                d_int_sign_s21 = sign(s21);
                
                ds = [d_x1; d_x2; d_z1; d_z2; d_D1_hat; d_D2_hat; 
                      d_int_sign_s11; d_int_sign_s21; d_rho20;
                      d_rho_s10; d_rho_s20];
                if obj.logData
                    obj.data.x1 = x1;
                    obj.data.x2 = x2;
                    obj.data.u = u;
                    obj.data.s10 = s10;
                    obj.data.zeta_s10 = zeta_s10;
                    obj.data.rho_s10 = rho_s10(1:2);
                    obj.data.s20 = s20;
                    obj.data.zeta_s20 = zeta_s20;
                    obj.data.rho_s20 = rho_s20(1:2);
                    obj.data.alpha1 = alpha1;
                    obj.data.rho_20 = rho20(1:2);
                    obj.data.zeta_20 = zeta_20;
                    obj.data.D1_hat = D1_hat;
                    obj.data.D2_hat = D2_hat;
                    obj.data.D2 = D2;
                    obj.data.z1 = z1;
                    obj.data.z2 = z2;
                    obj.data.s21 = s21;
                end
        end

        function out = dead_zone_operator(obj, v)
            r = [obj.u10; obj.u20];
            out = max(v(:) - r, min(0, v(:) + r));
        end
    end
    methods (Static)
        function test()
            addpath("../src/simulator/")
            addpath("../src/utils/")
            addpath("../src/solvers/")
            addpath("../src/datalogger/")
            
            x1 = [0;0];
            x2 = [0;0];
            refmag = [0.1; 0.1];
            z1 = x1 - refmag; % equal to e1
            K1 = diag([3,3]);
            alpha1 = -K1*z1;
            z2 = x2 - alpha1;
            s0 = [x1; x2; z1; z2; zeros(20,1)];
            logOn = true;
            sys = SMDO_mobile_robot("test", s0, logOn).setParams();
            tspan = 0:0.01:10;
            input = @(t) [refmag;0;0];
            sim = Simulator(sys).propagate(tspan, input);
            log = sim.log();
            
            log.x1.subplots(1);
            log.x2.subplots(2);
            log.alpha1.subplots(2); hold on;
            log.z2.subplots(2);
            log.rho_20.subplots(2, "\alpha_{1} estimates"); legend(["actual", "z_{2}", "command", "command estimate"]);
            log.u.subplots(3, "Control input");
            log.s10.subplots(4); hold on;
            log.rho_s10.subplots(4, "s_{10}"); legend(["actual", "estimate"]);
            log.zeta_s10.subplots(5, "s_{10} derivative estimates");

            log.s20.subplots(6); hold on;
            log.rho_s20.subplots(6, "s_{20} estimate"); legend(["actual", "estimate"]);
            log.zeta_s20.subplots(7, "s_{20} derivative estimates");
            log.D1_hat.subplots(8, "D1 estimate");
            log.D2.subplots(9, "D2");
            log.D2_hat.subplots(9, "D2 estimate");
            log.zeta_20.subplots(10,"\alpha_{1} derivative estimate")
            
            log.s21.subplots(12,"s_{21}")
            
            % remove path
            rmpath("../src/simulator/")
            rmpath("../src/utils/")
            rmpath("../src/solvers/")
            rmpath("../src/datalogger/")
        end
    end
end