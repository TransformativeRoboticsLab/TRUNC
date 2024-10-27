function send_servo_command(port, channel, command, bits)
    write(port, [command, channel, bits], 'uint8');
end