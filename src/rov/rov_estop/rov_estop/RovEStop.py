import Jetson.GPIO as GPIO
import dbus

def main():
    # setup GPIO names in Board mode
    GPIO.setmode(GPIO.BOARD)
    CHANNEL = 12
    GPIO.setup(CHANNEL, GPIO.IN)
    print("GPIO setup")
    try:
        while True:
            pinVoltage = GPIO.input(CHANNEL)
            if (pinVoltage == GPIO.HIGH):
                #shutdown
                sys_bus = dbus.SystemBus()
                ck_srv = sys_bus.get_object('org.freedesktop.login1',
                                            '/org/freedesktop/login1')
                ck_iface = dbus.Interface(ck_srv, 'org.freedesktop.login1.Manager')
                ck_iface.get_dbus_method("PowerOff")(False)
    finally:
        GPIO.cleanup(CHANNEL)

if __name__ == '__main__':
    main()