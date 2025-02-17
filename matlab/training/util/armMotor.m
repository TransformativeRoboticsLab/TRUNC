classdef armMotor
    properties
        s;           % Serial object
        motor_pin = 'D7'; % Pin where the motor relay is connected, if needed for reference
    end
    
    methods
        % Constructor method to create the arm_motor object
        function obj = armMotor()
            
            % Clean up ports
            objs = instrfind;
            if ~isempty(objs)
                fclose(objs);
                delete(objs);
            end

            obj.s = serial('COM6', 'BaudRate', 9600); % Create serial object
            fopen(obj.s); % Open serial connection
            pause(2); % Allow some time for Arduino to reset and establish a serial connection
        end
        
        % Method to turn on the relay
        function turnOnRelay(obj)
            fprintf(obj.s, '%c', '1'); % Send character '1' to Arduino for turning on the relay
        end
        
        % Method to turn off the relay
        function turnOffRelay(obj)
            fprintf(obj.s, '%c', '0'); % Send character '0' to Arduino for turning off the relay
        end
        
        % Method to pulse the motor for a fixed duration
        function pulse(obj, t)
            if t > 0
                obj.turnOnRelay();
                pause(t);
                obj.turnOffRelay();
            end
        end
        
        % Destructor method to close serial port when object is deleted
        function delete(obj)
            fclose(obj.s); % Close serial connection
            delete(obj.s); % Delete serial object
        end
    end
end
