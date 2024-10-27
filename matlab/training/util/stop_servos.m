function stop_servos(port, channels)
    for channel = channels
        stop_servo(port, channel)
    end
end

