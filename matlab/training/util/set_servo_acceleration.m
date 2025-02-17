function set_servo_acceleration(port, channel, acceleration)
    
    command = 0x89;
    bits = [binvec2dec(bitget(acceleration, 1:7)), binvec2dec(bitget(acceleration, 8:13))];
    
%     fprintf("Set channel %d acceleration to %.2f\n", channel, acceleration);
    send_servo_command(port, channel, command, bits);
end