% This class is an implementation of a time delay function
classdef TimeDelay < handle
    properties
        tau % delayed time
        simulationTimeInterval
        time = -inf
        n = 1
        y_hist        
    end
    methods
        function obj = TimeDelay(tau, dt)
            assert(tau >= dt, 'delay must be greater than or equal to simulatio time interval')
            obj.tau = tau;
            obj.simulationTimeInterval = dt;            
        end

        function y_delay = delay(obj, t, y)  
            assert(t >= obj.time-eps(), 'Current time must not be smaller than previous time')
%             if t > 0
%                 assert(round(t-obj.time, 6) == obj.simulationTimeInterval, 'Time interval must be the same with the predefined simulation time interval')
%             end
            if round(t - obj.tau,6) < 0
                obj.y_hist(:, obj.n) = y;                
                obj.n = obj.n + 1;
                y_delay = zeros(size(y));
            else
                y_delay = obj.y_hist(:, 1);
                obj.y_hist(:, 1:end-1) = obj.y_hist(:, 2:end);
                obj.y_hist(:, end) = y;
            end
        obj.time = t;
        end
    end

    methods (Static)
        function test()
            disp('[Test TimeDelay class]');
            dt = 0.01;
            tau = 0.03;
            t = 0:dt:1;
            y = [sin(2*pi*t); cos(2*pi*t)];
            
            d = TimeDelay(tau, dt);
            y_delay = nan(2, length(t));
            for i = 1:length(t)
                y_delay(:, i) = d.delay(t(i), y(:, i));
            end

            % plot
            disp(['Simulation time interval : ',num2str(dt)])
            disp(['Delay : ',num2str(tau)])
            plot(t, y(1, :),'DisplayName','y1 true'); hold on
            plot(t, y_delay(1, :),'DisplayName','y1 delayed')
            plot(t, y(2, :),'DisplayName','y2 true'); hold on
            plot(t, y_delay(2, :),'DisplayName','y2 delayed')
            legend();
            xlabel('Time (sec)');
            ylabel('y')
            grid on        
        end
    end
end