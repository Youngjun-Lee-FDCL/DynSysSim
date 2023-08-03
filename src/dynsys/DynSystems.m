classdef DynSystems < handle
    properties   
        name
        time = 0
        state 
        data
    end

    properties (Hidden)
        stateDeriv
        stateSize
        subSysCell
        subSysNum = 0
        subSysSize 
        isLogOn = false
        logData = false
        holder
    end
        
    methods
        function obj = DynSystems(name, in, logOn) 
            if nargin == 3
                obj.isLogOn = logOn;
            end
            assert(or(iscell(in), isnumeric(in)), 'invalid input');
            if iscell(in)              
                obj.subSysCell = in;
                obj.subSysNum = numel(in);   
                startIdx = 1;
                for i = 1:obj.subSysNum
                    stSize = in{i}.stateSize;
                    obj.subSysSize(i) = stSize;
                    stateArr(startIdx:startIdx + stSize -1, 1) = in{i}.state;
                    startIdx = startIdx  + stSize;
                end                   
                obj.updateState(stateArr);
                obj.stateSize = startIdx - 1;
            else
                obj.updateState(in);   
                obj.stateSize = numel(in);
            end
            obj.name = name;
        end

        function out = dynEqns(obj, t, ~, u)
            if obj.subSysNum == 0
                error("Create the 'dynEqns' method in the "+obj.name+" dynamical system")
            end
            out = nan(sum(obj.subSysSize), 1);
            idx = 1;
            for i = 1:obj.subSysNum
                subsys = obj.subSysCell{i};
                s = subsys.state;
                stSize = subsys.stateSize;
                if obj.logData == true
                    subsys.logData = true;
                    out(idx:idx+stSize-1) = subsys.dynEqns(t, s, u{i});
                    obj.data.(subsys.name) = subsys.data;
                    subsys.logData = false;
                else
                    out(idx:idx+stSize-1) = subsys.dynEqns(t, s, u{i});
                end
                idx = idx + stSize;
            end
        end

        function s_next = step(obj, u, dt)
            obj.holder = [];
            t = obj.time;
            s = obj.state;
            f = obj.setODEfun(u);
            
            if obj.isLogOn
                obj.logData = true;
                k1 = f(t, s) * dt;
                obj.logData = false;
            else
                k1 = f(t, s) * dt;
            end
            k2 = f(t + dt/2, s + k1/2) * dt;
            k3 = f(t + dt/2, s + k2/2) * dt;
            k4 = f(t + dt/2, s + k3/2) * dt;
            s_next = s + 1/6 * (k1 + 2*k2 + 2*k3 + k4);
        end

        function [varargout] = ZeroOrderHold(obj, varargin)
            if isempty(obj.holder)
                varargout = varargin;
                obj.holder = varargin;
            else
                varargout = obj.holder;
            end
        end

        function f = setODEfun(obj, u)
            if nargin == 1
                f = @(t, s) obj.dynEqns(t, s);
            else
                f =  @(t, s) obj.dynEqns(t, s, u);
            end
        end

        function [varargout]= getSubStates(obj)
            varargout = cell(obj.subSysNum, 1);
            for i = 1:obj.subSysNum
                varargout{i} = obj.subSysCell{i}.state;
            end
        end

        function updateState(obj, state)
            startIdx = 1;
            for i = 1: obj.subSysNum
                sysSize = obj.subSysSize(i);
                obj.subSysCell{i}.updateState(state(startIdx:startIdx + sysSize - 1));
                startIdx = startIdx + sysSize;
            end            
            obj.state = state;
        end
        
        function updateTimes(obj, t)
            obj.time = t;
            for i = 1:obj.subSysNum
                obj.subSysCell{i}.updateTimes(t);
            end
        end

        function reset(obj, state)
            obj.state = state;
            startIdx = 1;
            for i = 1:obj.subSysNum
                subSysLen = obj.subSysSize(i);
                subState = state(startIdx: startIdx + subSysLen - 1);
                obj.subSysCell{i}.reset(subState);
                startIdx = startIdx + subSysLen;
            end
            obj.time = 0;
            obj.data = [];
        end

%                
%         function updateStateAndTime(obj, state, t)
%             startIdx = 1;
%             for i = 1: obj.subSysNum
%                 sysSize = obj.subSysSize(i);
%                 obj.subSysCell{i}.updateStateAndTime(state(startIdx:startIdx + sysSize - 1), t);
%                 startIdx = startIdx + sysSize;
%             end            
%             obj.state = state;
%             obj.time = t;
%         end
%         
%         function updateStateDerivAndTime(obj, state, stateDot, t)
%             startIdx = 1;
%             for i = 1: obj.subSysNum
%                 sysSize = obj.subSysSize(i);
%                 st = state(startIdx:startIdx + sysSize - 1);
%                 stDot =  stateDot(startIdx:startIdx + sysSize - 1);
%                 obj.subSysCell{i}.updateStateDerivAndTime(st, stDot, t);
%                 startIdx = startIdx + sysSize;
%             end            
%             obj.state = state;
%             obj.stateDeriv = stateDot;
%             obj.time = t;
%         end
%         
%         function updateStateDeriv(obj, stateDot)
%             startIdx = 1;
%             for i = 1: obj.subSysNum
%                 sysSize = obj.subSysSize(i);
%                 obj.subSysCell{i}.updateStateDeriv(stateDot(startIdx:startIdx + sysSize - 1));
%                 startIdx = startIdx + sysSize;
%             end
%             obj.stateDeriv = stateDot;
%         end        
%         
        
        
%         function preallocateAll(obj, dt, tf)
%             if obj.isLogOn 
%                 obj.preallocate(dt, tf);
%             end
%             for i = 1:obj.subSysNum
%                 obj.subSysCell{i}.preallocateAll(dt, tf);   
%             end
%         end
        
%         function preallocate(obj, dt, tf)
%         end
        
%         function logSwitches = loggerSwitchOn(obj, logSwitches)
%             if ~isscalar(logSwitches)
%                 logSwitch = logSwitches(1);
%                 logSwitches(1) = [];
%             else
%                 logSwitch = logSwitches;
%             end
%             if obj.subSysNum > 1
%                 for i = 1:obj.subSysNum
%                     logSwitches = obj.subSysCell{i}.loggerSwitchOn(logSwitches);
%                 end
%             end
%             obj.isLogOn = logSwitch;                               
%         end
        
%         function updateIndex(obj, index)
%             if obj.isLogOn == true
%                 for i = 1:obj.subSysNum                    
%                     subSysLog = obj.subSysCell{i}.log;
%                     if isempty(subSysLog)
%                         continue
%                     end
%                     obj.subSysCell{i}.log = DataInventory.updateIdx(subSysLog, index);
%                 end
%                 obj.log = DataInventory.updateIdx(obj.log, index);
%             end
%         end
% %             
%         function updateFromStates(obj)
%             obj.updateFromState;
%             for i = 1:obj.subSysNum
%                 subSys = obj.subSysCell{i};
%                 subSys.updateFromStates;
%             end
%         end
%         
%         function updateFromState(obj)
%         end
%         
%         function updateFromStateDerivs(obj)
%             obj.updateFromStateDeriv;
%             for i = 1:obj.subSysNum
%                 subSys = obj.subSysCell{i};
%                 subSys.updateFromStateDeriv;
%             end
%         end
%         
%         function updateFromStateDeriv(obj)
%         end
%         
%         function out = stopCondSatisfied(obj)
%             out = false;
%         end
%         
%         function [fieldCellArr, valueCellArr, numCellArr] = mergeLog(obj)   
%             if isempty(obj.log)
%                 fieldCellArr = [];
%                 valueCellArr = [];
%                 numCellArr = 0;              
%             else
%                 obj.postProcessing();
%                 fieldCellArr = {obj.name};
%                 valueCellArr = {obj.log};
%                 numCellArr = 1;            
%             end
%             for i = 1: obj.subSysNum              
%                 subSys = obj.subSysCell{i};                
%                 [fca, vca, nca] = subSys.mergeLog;
%                 fieldCellArr = [fieldCellArr; fca];
%                 valueCellArr = [valueCellArr; vca];
%                 numCellArr = numCellArr + nca;
%             end              
%         end
        
%         function out = logData(obj, t)
%             if isempty(obj.timer)
%                 out = false;
%                 return
%             end
%             obj.timer.update(t);
%             out = and(obj.isLogOn, obj.timer.updateFlag);
%         end
    

    end
end