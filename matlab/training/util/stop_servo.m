
% References:
% * https://www.pololu.com/docs/0J40/5.e
function stop_servo(port, channel)

    command = 0x84;
    bits = [0x0, 0x0];
    send_servo_command(port, channel, command, bits);
    
end