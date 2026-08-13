from jq_eskin_serial import JQESkinSerial

with JQESkinSerial("/dev/ttyUSB0") as skin:
    while True:
        frame = skin.read_frame()

        if frame and frame.sensor_name == "LH":
            left_hand = frame.pressure
            print(left_hand)
