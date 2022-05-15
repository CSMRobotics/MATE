import rclpy
from rclpy.node import Node
from rov_interfaces.srv import CreateContinuousServo
from rov_interfaces.msg import PWM

rclpy.init()
node = rclpy.create_node("thruster_tester_3000")
current_thruster = 0
print("Welcome to the best thruster tester you've ever experienced")
print("Creating thruster servos")

servo_srv = node.create_client(CreateContinuousServo, "create_continuous_servo")
print("Waiting for thruster servo creator service")
servo_srv.wait_for_service()
print("Service found!  Creating thruster servos")
servos = [0, 1, 2, 3, 12, 13, 14, 15]

for srv in servos:
    res = servo_srv.call(CreateContinuousServo.Request(channel=srv))
    # future = servo_srv.call_async(CreateContinuousServo.Request(channel=srv))
    # rclpy.spin_until_future_complete(node, future)
    # res = future.result()
    print(f"Thruster Servo: {res.channel}, Result: {res.result}")


print("Created thrusters at channels " + ", ".join(servos) + "!")

pwm_pub = node.create_publisher(PWM, "pwm", 3)

while True:
    print(f"Current Thruster: T{current_thruster}")
    command = input("Enter a thruster (T0), a throttle (1.2), or quit (quit): ")
    if command[0] == "T":
        try:
            current_thruster = int(command[1:])
        except ValueError:
            print("You entered an invalid thruster number you dimwit")
    elif command == "quit":
        print("Ciao")
        break
    else:
        thrust = 0
        try:
            thrust = float(command)
        except ValueError:
            print("Why are you so bad at entering numbers? Defaulting to zero")
        if not (-1 <= thrust <= 1):
            print("Thrust out of range, you numb nut.")
            continue
        pwm_pub.publish(PWM(is_continuous_servo=True, channel=current_thruster, angle_or_throttle=thrust))
    rclpy.spin_once(node, timeout_sec=1)
rclpy.shutdown()