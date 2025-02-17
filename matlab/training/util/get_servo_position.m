function currentPosition = get_servo_position(port, channel)
    
% Command byte for 'Get Position'
    command = 0x90;

    % Send the command to the Maestro
    write(port, [command, channel], 'uint8');

    % Read the response from the Maestro
    response = read(port, 2, 'uint8');

    % Convert the response to position
    currentPosition = response(1) + 256 * response(2);
end
