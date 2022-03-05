from typing import ByteString
import evdev
import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("10.0.0.2",7777))

deviceID = "usb-Logitech_Extreme_3D_pro_00000000002A-event-joystick"

device = evdev.InputDevice(f"/dev/input/by-id/{deviceID}")

def map(value, bittage):
    from_min = 0 if bittage != 1 else -1
    from_max = 2 ** bittage -1
    to_min = -32768
    to_max = 32767
    return int((value - from_min) * ((to_max - to_min) / (from_max - from_min)) + to_min)

funmap = {0:10, 1:10, 5:7, 6:8, 16:1, 17:1}
axismap = {0:0, 1:1, 5:2, 6:3, 16:4, 17:5}

for event in device.read_loop():
    message = 0x80000000
    if(event.type == 1): # button pressed/released
        message = message | ((event.code - 288) << 24)
        message = message | 0x00010000
        message = message | (0x1 if event.value else 0x0)
        s.send(message.to_bytes(4, byteorder="little"))
    elif(event.type == 3): # absolute axis event
        message = message | (axismap[event.code] << 24)
        message = message | 0x00020000
        message = message | (map(event.value, funmap[event.code]) & 0xFFFF)
        s.send(message.to_bytes(4, byteorder="little"))

s.close()