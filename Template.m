classdef Template < DynSystems % Must be inherited by "DynSystems"
    properties
        some_variable_you_want % (Optional)
    end
    methods
        % (Mandatory) Constructor
        function obj = Template(name, s0, logOn)
            % ===== INPUT ARGUMENTS ====
            % name: name of the system, string. ex.) "MyDynSys"
            % s0: an initial state of the system, array. ex.) [1, 1] for two dimensional dynamical system
            % logOn: if you want save intermediate variables generated
            % during integration process, please set this value true. boolean. ex.) true, false

            % ===== OUTPUT ARGUMENTS ====
            % obj: instance of a class
            
            % Case 1: Create a new dynamical system
            obj = obj@DynSystems(name, s0, logOn); % Mandatory

            % Case 2: Combine several systems written in advance.
            obj_example_sys1 = ExampleSys1("Example_System_1", s0(1:2), logOn);
            obj_example_sys2 = ExampleSys2("Example_System_2", s0(3:4), logOn);
            obj = obj@DynSystems(name, {obj_example_sys1, obj_example_sys2});
        end
        
        % (Mandatory) Equations of our dynamical systems
        function ds = dynEqns(obj, t, s, u)
            % ===== INPUT ARGUMENTS ====
            % obj: instance of a class
            % t: time, scalar
            % s: states, vector
            % u: input, vector

            % ===== INPUT ARGUMENTS ====
            % ds: derivatives of states which have the same length as the state s, vector. 

            % Write here on your dynamical system.
            some_variable_you_want = obj.some_variable_you_want;
            ds = some_function(t, s, u);

            % (Optional) if you want to save variables you want, please
            % write the below codes.
            if obj.logData
                obj.data.some_variable_name_you_want_to_save = some_var_you_want_to_save;
                % Examples
                % obj.data.s = s;
                % obj.data.time = t;
            end
        end

        % (Optional) if you want to set variable in this class from running
        % script, please use this method. The name of this method is just
        % convention. Feel free to used another name if you want.
        function setParams(obj, some_variable_you_want)
            obj.some_variable_you_want = some_variable_you_want;
        end
    end
end