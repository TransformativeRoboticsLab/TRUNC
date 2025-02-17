classdef robotArm
    % Attributes
    properties
        port;
        channels = 0:8;
        comp = load("./state/comp.mat").comp;
        comp_max = load("./state/comp_max.mat").comp_max;
        pause_length = 3;
        min_motor = -250;
        max_motor = 150;
        nnc
    end

    methods
        % Constructor
        function obj = robotArm()
            addpath('./util/NatNet_SDK_4.1/NatNetSDK/Samples/Matlab');
            addpath('./util/')
            if ~exist('obj.port', 'var')
                obj.port = serialport("COM4", 9600);
                initialize_servos(obj.port, obj.channels, obj.pause_length, obj.comp);
                pause(5);
            end
            
            % Connect to motive
            if ~exist('nnc','var')
                obj.nnc = connect_to_natnet();
            end
            
        end
        function tool = get_pose(obj)
            % Obtain rigid body state
            tool = obj.nnc.getFrame().RigidBodies(1); %End Effector
        end
        
        % Method to set the position
        function set_pos(obj, motor_pos)
            
            if min(motor_pos) < obj.min_motor || max(motor_pos) > obj.max_motor
                disp('Motor value out of safe bounds!!!')
                disp(min(motor_pos))
                disp(max(motor_pos))
                res = input();
            else
                final_pos = zeros(1,8);
                current_pos = zeros(1,8);
                threshold = 1;
                reached = false;
                
                % Sends commands to servo motors
                for idx = 0:8
                    target = set_servo_position_auxarm(obj.port, idx, motor_pos(idx+1));
                    final_pos(idx+1) = target;
                end
                
                % Record initial time
                startTime = tic;
    
                % Waits until all servo motors are done moving
                while ~reached
                    % Read positions
                    current_pos = zeros(1, 9); % Preallocate array for current positions
                    for idx = 0:8
                        current = get_servo_position(obj.port, idx);
                        current_pos(idx+1) = current;
                    end
            
                    % Calculate error
                    e = sum((final_pos - current_pos).^2);
            
                    % Check threshold
                    if e <= threshold
                        reached = true;
                    end
            
                    % Check if timeout (15 seconds) is exceeded
                    if toc(startTime) > 20
                        fprintf('Timeout: Servos did not reach the desired position within 20 seconds.\n');
                        disp(e)
                        break;
                    end
                end
            end
        end

        % Sends arm to compressed state and then cycles a known trajectory
        function reset_arm(obj)
            fprintf('Resetting arm.\n')
            obj.set_pos(obj.comp)
            for rep = 1:5
                pause(.5)
                obj.set_pos(obj.comp_max)
                pause(.5)
                obj.set_pos(obj.comp)
            end
        end

        % Method to turn off the motors
        function stop_motors(obj)
            stop_servos(obj.port, obj.channels);
        end
    end
end
