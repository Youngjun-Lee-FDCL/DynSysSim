classdef Simulator < matlab.mixin.Copyable
    properties
        system
        log = []
        isStopCondSatisfied = false
        psim = false
    end
    properties (Hidden)
        INIT = 1
        elapsedTime = 0
        logSwitches
        logHist
        fieldNames
        tspan
    end
  
    methods
        function obj = Simulator(system, logHist, psim)
            if nargin <= 1
                logHist = false;
            end
            if nargin == 3
                obj.psim = psim;
            end
            classes = superclasses(system);
            if ~strcmp(classes{end-1}, 'DynSystems')
                error('invalid system')
            end
            obj.system = system;
            obj.logHist = logHist;
        end

        function obj = propagate(obj, tspan, u)
            if nargin == 2
                u = @(t) NaN;
            end
            if isempty(obj.system.state)
                error('state should be initialized before calling propagate method');
            end
     
            if isnumeric(u)
               u = @(t) u; 
            end
            
            % set a timer of the simulator
            obj.tspan = round(tspan, 6);
            numiter = length(tspan);
            dt = tspan(2) - tspan(1);
            obj.system.updateTimes(tspan(1));

            % check if the dynEqns method designed well
            f = obj.system.setODEfun(u(tspan(1)));
            dim_bools = size(f(obj.system.time, obj.system.state)) == size(obj.system.state);
            assert(and(dim_bools(1), dim_bools(2)), "Check the input/output of your dynEqns method: the sizes of state and its derivative does not match")
            
            tic
            for i = 1:numiter
                t = obj.system.time();
                s = obj.system.state();
                s_next = obj.system.step(t, s, u(t), dt);
                t_next = round(t + dt ,6);  
                obj.stackdata();
                obj.system.updateState(s_next);
                obj.system.updateTimes(t_next);
                if obj.system.stopConds(t_next, s_next)
                    break;
                end
            end
            obj.elapsedTime = toc;                    
            if obj.INIT 
                obj.INIT = false;
            end

            if obj.psim == true
                obj.log = DataInventory.obj2str(obj.log);
            end
        end
        

        function [obj, log] = propagate_parallel(obj, tspan, u)
            if isempty(obj.system.state)
                error('state should be initialized before calling propagate method');
            end
            if isnumeric(u)
               u = @(t) u; 
            end

            obj.tspan = round(tspan, 6);
            numiter = length(tspan);
            dt = tspan(2) - tspan(1);
            obj.system.updateTimes(tspan(1));
            
            obj.log = DataInventory.str2obj(obj.log);
            tic
            for i = 1:numiter
                t = obj.system.time();
                s = obj.system.state();
                s_next = obj.system.step(t, s, u(t), dt);
                t_next = round(t + dt ,6);  
                obj.stackdata_parallel();
                obj.system.updateState(s_next);
                obj.system.updateTimes(t_next);
                if obj.system.stopConds(t_next, s_next)
                    break;
                end
            end
            obj.elapsedTime = toc;
            if obj.INIT 
                obj.INIT = false;
            end
            log = DataInventory.obj2str(obj.log);
        end

        function stackdata(obj)
            data = obj.system.data;
            if isempty(obj.log) && ~isempty(data)
                fnames = fieldnames(data);
                fieldCellLength = length(fnames);
                [~, obj.log] = DataInventory(fnames, obj.system.dataNames, obj.tspan);
                for i = 1:fieldCellLength
                    obj.log.(fnames{i}).append(data.(fnames{i}));
                end
                obj.fieldNames = fnames;
            elseif ~isempty(data)
                fnames = obj.fieldNames;
                for i = 1:length(fnames)
                    obj.log.(fnames{i}).append(data.(fnames{i}));
                end
            end
        end
        
        function stackdata_parallel(obj)
            data = obj.system.data;
            fnames = fieldnames(data);
            for i = 1:length(fnames)
                obj.log.(fnames{i}).append(data.(fnames{i}));
            end
        end
        
        function report(obj)
            fprintf('[Elapsed simulation time] %.2f seconds \n',obj.elapsedTime)
        end
        
        function f = set_f(obj, u)
            if nargin == 1
                f = @(t, s) obj.system.stateSpaceEqn(t, s);
            elseif isa(u, 'function_handle')
                f = @(t, s) obj.system.stateSpaceEqn(t, s, u(t));
            else
                f = @(t, s) obj.system.stateSpaceEqn(t, s, u);
            end
        end

        function out = copyLog(obj)
            n = length(obj.fieldNames);
            out = cell(n, 1);
            for i = 1:n
                out{i} = obj.log.(obj.fieldNames{i}).copy();
            end
        end
    end                
end