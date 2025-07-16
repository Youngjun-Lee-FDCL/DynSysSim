classdef DataInventory < matlab.mixin.Copyable
    properties
        fieldName
        dataName        
        data
        dataNum
        dataLen        
        lastAppenedIdx = 0
        indepVar
        xlabelName = "Time (sec)"
    end
    
    properties (Hidden)
        interval
        startval
        axes
    end
    methods
        function [obj, str] = DataInventory(fieldNames, dataNames, indepVarSpan)           
            if nargin < 1
                return
            end
            if ischar(fieldNames)
                    fieldNames = {fieldNames};
            end
            numFieldNames = numel(fieldNames);
            if numFieldNames == 1
                obj.fieldName = fieldNames;
                obj.dataLen = numel(indepVarSpan);
                obj.startval = indepVarSpan(1);
                obj.interval = diff(indepVarSpan(1:2));
                obj.dataNum = length(dataNames);
                obj.dataName = dataNames;
                obj.data = nan(length(indepVarSpan), obj.dataNum);
                obj.indepVar = indepVarSpan;
                str.(fieldNames{1}) = obj;
            else                
                assert(length(fieldNames)==length(dataNames), "fieldnams does not match with dataNames: Check dataNames property")
                objs = cell(1, numFieldNames);
                for i = 1:numFieldNames
                    if ischar(dataNames{i})
                        dataNames{i} = string(dataNames{i});
                    end
                    objs{i} = DataInventory(fieldNames(i), dataNames{i}, indepVarSpan);
                end
                arguments = [fieldNames(:).';objs];
                arguments = arguments(:).';
                str = struct(arguments{:});
            end
        end
             
        function append(obj, data)
            startIdx = obj.lastAppenedIdx + 1;          
            obj.dynamicAllocation(data);
            if obj.dataNum ~= numel(data)
                error("[Mismatched size] "+"Name :"+obj.fieldName{1} +" / Size of data: "+num2str(numel(obj.dataName))+" / Size of input: "+num2str(numel(data)))
            end
            obj.data(startIdx, :) = data;
            obj.lastAppenedIdx = startIdx;
            if obj.lastAppenedIdx > obj.dataLen
                obj.data(obj.lastAppenedIdx+1:2*obj.lastAppenedIdx, :) = nan;
                obj.dataLen = 2*obj.lastAppenedIdx;
            end
        end
        
        function dynamicAllocation(obj, data)
            if obj.dataNum == 0
                obj.dataNum = numel(data);
                obj.data = nan(numel(obj.indepVar), obj.dataNum);
                obj.dataName = repmat("undef", 1, numel(data));
            end
        end

        function [ps, axes] = subplots(obj, fignum, Title, linestyle, color, idx, layout, arrange, axes)
            switch nargin
                case 1
                    figure();
                    idx = 1:1:obj.dataNum;
                    linestyle = "";
                    color = "default";
                    layout = [length(idx), 1];
                    arrange = num2cell(idx);
                    obj.axes = cell(length(idx), 1);
                case 2
                    figure(fignum);
                    idx = 1:1:obj.dataNum;
                    linestyle = "";
                    color = "default";
                    layout = [length(idx), 1];
                    arrange = num2cell(idx);
                    obj.axes = cell(length(idx), 1);
                case 3
                    fig = figure(fignum);
                    fig.Name = Title;
                    idx = 1:1:obj.dataNum;
                    linestyle = "";
                    color = "default";
                    layout = [length(idx), 1];
                    arrange = num2cell(idx);
                    obj.axes = cell(length(idx), 1);
                case 4
                    fig = figure(fignum);
                    fig.Name = Title;
                    idx = 1:1:obj.dataNum;
                    color = "default";
                    layout = [length(idx), 1];
                    arrange = num2cell(idx);
                    obj.axes = cell(length(idx), 1);
                case 5
                    fig = figure(fignum);
                    fig.Name = Title;
                    idx = 1:1:obj.dataNum;
                    layout = [length(idx), 1];
                    arrange = num2cell(idx);
                    obj.axes = cell(length(idx), 1);
                case 6
                    fig = figure(fignum);
                    fig.Name = Title;
                    layout = [length(idx), 1];
                    arrange = num2cell(idx);
                    obj.axes = cell(length(idx), 1);
                case 7
                    fig = figure(fignum);
                    fig.Name = Title;
                    arrange = num2cell(idx);
                    obj.axes = cell(length(idx), 1);
                case 8
                    fig = figure(fignum);
                    fig.Name = Title;
                    obj.axes = cell(length(idx), 1);
                case 9
                    fig = figure(fignum);
                    fig.Name = Title;
                    if isempty(axes)
                        obj.axes = cell(length(idx), 1);
                    else
                        obj.axes = axes;
                    end
            end
            obj.postProcessData;
            ps = cell(length(idx) , 1); % preallocation
            for i = 1:1:length(idx)
                k = idx(i);
                if isempty(obj.axes{i})
                    obj.axes{i} = subplot(layout(1), layout(2), arrange{i}); hold on;
                else
                    subplot(layout(1), layout(2), arrange{i}, obj.axes{i}); hold on;
                end
                if i == 1 && nargin > 2
                    sgtitle(Title);
                end
                if isempty(obj.dataName{k})
                    obj.dataName{k} = '';
                end
                ndata = size(obj.data, 1);
                ps{i} = plot(obj.indepVar(1:ndata), obj.data(:, k),linestyle, color=color, DisplayName=obj.dataName{k});
                ylabel(obj.dataName{k});
                box on; grid on; hold on;
                xlabel(obj.xlabelName);
            end
            axes = obj.axes;
        end
        
        function [Axes, ps] = subplot(obj, fignum, axes_or_idx_1, axes_or_idx_2, varargin)
            default_axes = (obj.dataNum * 100 + 10 + 1):(obj.dataNum * 100 + 10 + obj.dataNum);
            default_idx = 1:1:obj.dataNum;
            if nargin == 1
                figure('Color', 'white');
                axes = default_axes;
                idx = default_idx;
            elseif nargin == 2
                fig = figure(fignum);
                fig.Color = 'white';
                axes = (obj.dataNum * 100 + 10 + 1):(obj.dataNum * 100 + 10 + obj.dataNum);
                idx = 1:1:obj.dataNum;
            elseif nargin == 3
                if isa(fignum, 'string')
                    figure('Color', 'white');
                    varargin = [{fignum}, {axes_or_idx_1}];
                    axes = (obj.dataNum * 100 + 10 + 1):(obj.dataNum * 100 + 10 + obj.dataNum);
                    idx = default_idx;
                else
                    fig = figure(fignum);
                    fig.Color = 'white';
                    if isa(axes_or_idx_1, 'matlab.graphics.axis.Axes')
                        axes = axes_or_idx_1;
                        idx = default_idx;
                    else
                        n = length(axes_or_idx_1);
                        idx = axes_or_idx_1;
                        axes = (n * 100 + 10 + 1):(n * 100 + 10 + n);
                    end
                    varargin = {};
                end
            elseif nargin == 4
                fig = figure(fignum);
                fig.Color = 'white';
                if isa(axes_or_idx_1, 'string')
                    varargin = [{axes_or_idx_1}, {axes_or_idx_2}, varargin(:)'];
                    axes = default_axes;
                    idx = default_idx;
                elseif isa(axes_or_idx_1, 'matlab.graphics.axis.Axes')
                    varargin = {};
                    axes = axes_or_idx_1;
                    idx = axes_or_idx_2;
                else
                    varargin = {};
                    axes = axes_or_idx_2;
                    idx = axes_or_idx_1;
                end
            elseif nargin == 5
                if isa(fignum, 'string')
                    figure('Color', 'white');
                    varargin = [{fignum}, {axes_or_idx_1}];
                    axes = default_idx;
                    idx = default_idx;
                else
                    fig = figure(fignum);
                    fig.Color = 'white';
                    if isa(axes_or_idx_1, 'matlab.graphics.axis.Axes')
                        axes = axes_or_idx_1;
                        idx = default_idx;
                    else
                        axes = default_axes;
                        idx = axes_or_idx_1;
                    end
                    varargin = [{axes_or_idx_2}, varargin(:)'];
                end
            elseif mod(nargin - 6, 2) ==0 && nargin >= 6
                fig = figure(fignum);
                fig.Color = 'white';
                if isa(axes_or_idx_1, 'string')
                    varargin = [{axes_or_idx_1}, {axes_or_idx_2}, varargin(:)'];
                    axes = default_axes;
                    idx = default_idx;
                elseif isa(axes_or_idx_1, 'matlab.graphics.axis.Axes')
                    axes = axes_or_idx_1;
                    idx = axes_or_idx_2;
                else
                    axes = axes_or_idx_2;
                    idx = axes_or_idx_1;
                end
            elseif mod(nargin - 6, 2) ==1 && nargin >= 7
                if isa(fignum, 'string')
                    figure('Color', 'white');
                    varargin = [{fignum}, {axes_or_idx_1}, {axes_or_idx_2}, varargin(:)'];
                    axes = default_idx;
                    idx = default_idx;
                else
                    fig = figure(fignum);
                    fig.Color = 'white';
                    if isa(axes_or_idx_1, 'matlab.graphics.axis.Axes')
                        axes = axes_or_idx_1;
                        idx = default_idx;
                    else
                        idx = axes_or_idx_1;
                        axes  = default_axes;
                    end
                    varargin = [{axes_or_idx_2}, varargin(:)'];
                end
            end


            obj.postProcessData;
            nsubfig = length(idx);
            Axes = [];
            for i = 1:1:nsubfig
                k = idx(i);
                ndata = size(obj.data, 1);
                if nsubfig == 1
                    Axes = plot(obj.indepVar(1:ndata), obj.data(:, k), varargin{:});
                else
                    plt = subplot(axes(i));
                    Axes = [Axes; plt]; hold on;
                    if isempty(obj.dataName{k})
                        obj.dataName{k} = '';
                    end
                    
                    ps{i} = plot(obj.indepVar(1:ndata), obj.data(:, k), varargin{:});
                end
                ylabel(obj.dataName{k});
                box on; grid on;
                xlabel('Time [sec]');
            end
        end

        function p = plot(obj, fignum, idx, varargin)
            if nargin < 2
                figure('Color', 'white');
            elseif isa(fignum, 'double')
                fig = figure(fignum);
                fig.Color = "white";
            else
                figure(Color="white");
                varargin = [{fignum}, {idx}, varargin(:)'];
                idx = 1:1:obj.dataNum;
            end
            if nargin < 3
                idx = 1:1:obj.dataNum;
            end
            if mod(nargin-4, 2) == 0 && nargin >= 4
                varargin = [{idx}, varargin(:)'];
                idx = 1:1:obj.dataNum;
            end

            obj.postProcessData;
            p = cell(length(idx) , 1); % preallocation
            hold on;
            xlabel('Time [sec]');box on; grid on;
            ylabel(obj.fieldName);
            ndata = size(obj.data, 1);
            for i = 1:length(idx)                
                p{i} = plot(obj.indepVar(1:ndata), obj.data(:, idx(i)),'DisplayName', obj.dataName{idx(i)}, varargin{:});
            end           
        end
     
        function p = plot3(obj, fignum, order, varargin)
            if nargin < 2
                figure('Color', 'white');
            elseif isa(fignum, 'double')
                fig = figure(fignum);
                fig.Color = "white";
            else
                figure(Color="white");
                varargin = [{fignum}, {order}, varargin(:)'];
                order = [1,2,3];
            end
            if nargin < 3
                order = [1, 2, 3];
            end
            if nargin == 3
                order = [1, 2, 3];
            end
            if mod(nargin-4, 2) == 0 && nargin >= 4
                varargin = [{order}, varargin(:)'];
                order = [1, 2, 3];
            end
            
            % extract unitConverter from varargin
            [varargin, a2b] = obj.extractUnitConverter(varargin);

            % post processing
            obj.postProcessData;
            shownData = obj.data*a2b;
            p = plot3(shownData(:,order(1)), shownData(:,order(2)), shownData(:,order(3)), varargin{:});
            xlabel(obj.dataName{order(1)});
            ylabel(obj.dataName{order(2)});
            zlabel(obj.dataName{order(3)})
            grid on; box on; 
        end
        
        function erase(obj, idx) 
            if isnan(idx)
                obj.data(obj.lastAppenedIdx+1:end, :) = [];
                return
            end
            obj.indepVar(idx) = [];
            obj.data(idx, :) = []; 
            obj.dataLen = size(obj.data, 1);
        end           
        
        function str = saveobj(obj)
            str.fieldName = obj.fieldName;
            str.dataName = obj.dataName;
            str.indepVar = obj.indepVar;
            str.data = obj.data;
            str.dataNum = obj.dataNum;
            str.dataLen = obj.dataLen;
            str.lastAppenedIdx = obj.lastAppenedIdx;
            str.interval = obj.interval;
            str.startval = obj.startval;
        end
        
        function postProcessData(obj)
            len = obj.lastAppenedIdx;
            obj.data(len+1:end, :) = [];
        end
        
        function [updatedCellArray, value] = extractUnitConverter(~, cellArray)
            bool = cellfun(@(x) isequal(x, "unitConverter"), cellArray);
            idx = find(bool==1, 1);
            if ~isempty(idx)
                value = cellArray{idx+1};
                cellArray([idx, idx+1]) = [];  % Erase the string at the found index
            else
                value = 1;
            end
            updatedCellArray = cellArray;
        end

        function savecsv(obj, savePath)
            postProcessData(obj);
            cellData = num2cell([obj.indepVar(:) obj.data], 1);
            T = table(cellData{:}, 'VariableNames', cellstr([obj.xlabelName obj.dataName]));
            if ~exist(savePath, 'dir')
                mkdir(savePath)
            end
            writetable(T, fullfile(savePath,obj.fieldName+".csv"));
        end

        function out = get_RAE(obj)
            len = size(obj.data, 1);
            L1_norm = sum(abs(obj.data), 1); % column-wise
            out = sqrt(L1_norm / len);
            if numel(out) ~= size(obj.data, 2)
                error("Invalid dimension of RAE calculation")
            end
        end

        function out = get_RMS(obj)
            squared_error = obj.data.^2;
            mse = mean(squared_error, 1);
            out = sqrt(mse);
            if numel(out) ~=  size(obj.data, 2)
                error("Invalid dimension of RMS calculation")
            end
        end

        function out = get_dominant_frequency(obj, Fs)
            for i = 1:size(obj.data,2)
                signal = obj.data(:, i);
                N = length(signal);
                fft_signal = fft(signal);
                freq = (0:N-1)*(Fs/N); % frequency vector

                fft_magnitude = abs(fft_signal);
                [~, index] = max(fft_magnitude);
                out(i) = freq(index);
            end
        end
    end
    
    methods (Static)
        function test()
            timeSpan = 0:0.1:10;
            % singular data case
            fieldNames = {'aa'};
            dataNames = {'a1','a2'};
            aa = DataInventory(fieldNames, dataNames, timeSpan);
            
            for i = 1:length(timeSpan)
                adata = [sin(timeSpan(i)); cos(timeSpan(i))];               
                aa.save(adata, i);
            end
            aa.multiPlot(1, 'a'); hold on;
            
            %% multiple data case
            fieldNames = {'aa','bb','cc'};
            dataNames = {{'a1','a2'},{'b1','b2'},{'c1','c2'}};
            [~, log] = DataInventory(fieldNames, dataNames, timeSpan);
            
            for i = 1:length(timeSpan)
                adata = [sin(timeSpan(i)); cos(timeSpan(i))];
                bdata = 2*adata;
                cdata = 3*adata;
                log.aa.save(adata, i);
                log.bb.save(bdata, i);
                log.cc.save(cdata, i);
            end
           
            log.aa.multiPlot(2, 'a'); hold on;
            log.bb.multiPlot(2, 'b');
            log.cc.multiPlot(2, 'c')
            log.aa.plot2(3);
        end
        
        function str = obj2str(datastr)
            fieldNames = fieldnames(datastr);
            for i = 1:length(fieldNames)
                object = datastr.(fieldNames{i});
                dataStr = object.saveobj();
                str.(fieldNames{i}) =  dataStr;
            end            
        end
        function strs = correctName(str)
            fieldNames = fieldnames(str);
            nfield = length(fieldNames);
            for i = 1:nfield
                fieldName = fieldNames{i}; 
                strs.(fieldName).fieldName = fieldName;
                strs.(fieldName).dataName = str.(fieldName).dataName;
                strs.(fieldName).indepVar = str.(fieldName).indepVar;
                strs.(fieldName).data = str.(fieldName).data;
                strs.(fieldName).dataNum = str.(fieldName).dataNum;
                strs.(fieldName).dataLen = str.(fieldName).dataLen;
                strs.(fieldName).lastAppenedIdx = str.(fieldName).lastAppenedIdx;
                strs.(fieldName).interval = str.(fieldName).interval;
                strs.(fieldName).startval = str.(fieldName).startval;
            end
        end
        function objs = str2obj(str)
            if isa(str, "DataInventory")
                return
            end
            fieldNames = fieldnames(str);
            nfield = length(fieldNames);
            for i  = 1:nfield
                fieldName = fieldNames{i};
                objs.(fieldName) = DataInventory;
                objs.(fieldName).fieldName = fieldName;
                objs.(fieldName).dataName = str.(fieldName).dataName;
                objs.(fieldName).indepVar = str.(fieldName).indepVar;
                objs.(fieldName).data = str.(fieldName).data;
                objs.(fieldName).dataNum = str.(fieldName).dataNum;
                objs.(fieldName).dataLen = str.(fieldName).dataLen;
                objs.(fieldName).lastAppenedIdx = str.(fieldName).lastAppenedIdx;
                objs.(fieldName).interval = str.(fieldName).interval;
                objs.(fieldName).startval = str.(fieldName).startval;
            end
            
        end   
             
        function str = addData(obj1, obj2, operator, fieldName, dataName)
            assert(obj1.dataNum == obj2.dataNum, 'the length of entered two data are not equal');
            if nargin == 3
                fieldName = 'addedData';
                dataName = cell(obj1.dataNum, 1);
            end
            if nargin == 4
                dataName = cell(obj1.dataNum, 1);
            end
            
            
            [~, str] = DataInventory(fieldName, dataName, obj1.indepVar);
            switch operator
                case '+'
                    str.(fieldName).append(obj1.data + obj2.data);
                case '-'
                    str.(fieldName).append(obj1.data - obj2.data);
            end
            
        end
        
    end
end