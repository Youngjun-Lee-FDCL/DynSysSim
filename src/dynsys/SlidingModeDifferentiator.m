% [Ref] M. Chen, P. Shi and C. -C. Lim, "Robust Constrained Control for 
% MIMO Nonlinear Systems Based on Disturbance Observer," in IEEE 
% Transactions on Automatic Control, vol. 60, no. 12, pp. 3281-3286, 
% Dec. 2015, doi: 10.1109/TAC.2015.2450891.
classdef SlidingModeDifferentiator < DynSystems
    properties
        lambdas = [12, 8, 5, 3, 1.5, 1.1]
        L
        n
        dim
        dataNames
    end
    methods
        function obj = SlidingModeDifferentiator(name, s0, logOn)
            obj = obj@DynSystems(name, s0, logOn);
        end
        
        function obj = setParams(obj, L, n, dim)
            if nargin == 2
                n = 1;
                dim = 1;
            end
            obj.n = n;
            obj.L = L(:); % Can be time varying such that |u(t)^{order+1}| < L(t)
            obj.dim = dim;
            % set data names
            for i = 1:n+1
                for j = 1:dim
                    names{i}(j) = "z"+num2str(i-1)+num2str(j);
                end
            end
            obj.dataNames = names;

            if numel(obj.state) ~= obj.dim*(obj.n+1)
                error("Check dimension, order or the size of initial state")
            end
        end

        function ds = dynEqns(obj, ~, s, u)
            u = u(:);            
            z0 = s(1:obj.dim);
            if obj.n == 1
                z1 = s(obj.dim+1:end);
                zn = z1;
            else
                z1 = s(obj.dim + 1: obj.dim*2);
                zn = s(obj.n * obj.dim + 1 : (obj.n+1)*obj.dim);
            end
            lam_0 = obj.lambdas(end-obj.n);
            lam_n = obj.lambdas(end);
            d_z0 = -lam_0 * (obj.L).^(1/(obj.n+1)) .* abs( z0 - u ).^(obj.n/(obj.n+1)) .* sign( z0 - u ) + z1;
            ds = nan(numel(s), 1);

            for i = 1:obj.n-1

                % set d_zi_1
                if i == 1
                    d_zi_prev = d_z0;
                else
                    zi_idxes = obj.get_zi_idx(i-1);
                    d_zi_prev = ds(zi_idxes);
                end

                % set z_next
                if i == obj.n -1
                    z_next = zn;
                else
                    zi_next_idxes = obj.get_zi_idx(i+1);
                    z_next = s(zi_next_idxes);
                end

                zi_idxes = obj.get_zi_idx(i);
                zi = s(zi_idxes);
                ds(zi_idxes) = -obj.lambdas(end-i) * obj.L.^(1/2) .* (obj.L).^(i/obj.n) .* abs( zi - d_zi_prev ).^((obj.n-i)/obj.n) .* sign( zi - d_zi_prev) + z_next;
            end
            if obj.n > 1
                d_zn_1 = ds(zi_idxes);
                d_zn = -lam_n * obj.L .* sign( zn - d_zn_1);
                ds(1:obj.dim) = d_z0;
                ds(obj.dim*obj.n+1:end) = d_zn;
            elseif obj.n == 1
                d_zn_1 = d_z0;
                d_zn = -lam_n * obj.L .* sign( zn - d_zn_1);
                ds = [d_z0; d_zn];
            else
                error("Invalid order")
            end
            
            if obj.logData
                for i = 1:obj.n+1
                    if i == 1
                        obj.data.z0 = z0;
                    elseif i == obj.n+1
                        obj.data.("z"+num2str(i-1)) = zn;
                    else
                        zi_idxes = obj.get_zi_idx(i-1);
                        obj.data.("z"+num2str(i-1)) = s(zi_idxes);
                    end
                end
            end
        end

        function out = get_zi_idx(obj, i)
            out = obj.dim*i + 1 : obj.dim*(i+1);
        end
        
        function out = get_zeroDeriv_esti(obj, state)
            out = state(1:obj.dim);
        end
        
        function out = get_1stDeriv_esti(obj, state)
            out = state(obj.dim+1:2*obj.dim);
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
            n = 2;
            omega = 2;
            M = 0.01;
            if n == 1
                L = 6*[1;1] * M;
            elseif n == 2
                L = 2.5*omega^(n+1)*[1;1] * M;
            elseif n ==3
                L = 10*[1;1]* M;
            end
            
            dim = numel(L);
            s0 = zeros((n+1)*dim, 1);
            logOn = true;

            file = readtable("./table.csv");
            data = table2array(file);
            tspan = file.Var1_1;
            dTable = DataTable(data(:, 2), tspan);
            f =  @(t) dTable.interpolate(t);
            s0 = [0; 0; 0];
            n = 2; dim =1;
            L = 1000;
            sys = SlidingModeDifferentiator(name, s0, logOn)...
                .setParams(L, n, dim);

%             f = @(t) [M*sin(omega*t); M*cos(omega*t)];
%             d_f = @(t) [M*omega*cos(omega*t); -M*omega*sin(omega*t)];
%             tspan = 0:0.01:10;
            sim = Simulator(sys).propagate(tspan, f);
            sim.report();
            log = sim.log;
            
            % plot
            fig=figure(Name="original signal"); hold on;
            log.z0.plot(fig.Number); 
            plot(tspan, f(tspan), "k");
            ylabel("z_{0} and u");
            
            d_f = [NaN; central_difference(data(:, 2), tspan(2)-tspan(1)); NaN];
            fig = figure(Name="1st derivative");
            plot(tspan, d_f, "k"); hold on;
            log.z1.plot(fig.Number);
%             fig = figure(Name="1st derivative");
%             plot(tspan, d_f(tspan), "k");
%             log.z1.plot(fig.Number); 
%             ylabel("z_{1}");
            
%             if n >= 2
%                 dd_f = @(t) [-M*omega^2*sin(omega*t); -M*omega^2*cos(omega*t)];
%                 fig = figure(Name = "2nd derivative");
%                 plot(tspan, dd_f(tspan), "k");
%                 log.z2.plot(fig.Number);
%                 ylabel("z_{2}");
%             end
            % Remove path
            rmpath("../simulator/")
            rmpath("../utils/")
            rmpath("../solvers/")
            rmpath("../datalogger/")
        end
    end
end