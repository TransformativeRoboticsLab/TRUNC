function set_servo_speed(port, channel, speed)

    command = 0x87;
    bits = [binvec2dec(bitget(speed, 1:7)), binvec2dec(bitget(speed, 8:13))];
    
%     fprintf("Set channel %d speed to %.2f\n", channel, speed);
    send_servo_command(port, channel, command, bits);
end