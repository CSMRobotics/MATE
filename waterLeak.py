#pin 12

from logging import shutdown
import Jetson.GPIO as GPIO

def main():
    GPIO.setmode(GPIO.BOARD)
    channel = 12
    GPIO.setup(channel, GPIO.IN)
    input = GPIO.input(channel)
    try:
        while True:
            if (input == GPIO.HIGH):
                #shutdown
                os.system("shutdown 0")
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()