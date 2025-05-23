import RPi.GPIO as GPIO
import time
from collections import deque

PWM_FREQ = 50
PWM_MIN_US = 1100
PWM_MAX_US = 1900
PERIOD_US = 20000

def us_to_duty(us):
    return (us / PERIOD_US) * 100

PWM_PINS = [12, 13, 18]
SWITCH_PIN = 17

GPIO.setmode(GPIO.BCM)
for pin in PWM_PINS:
    GPIO.setup(pin, GPIO.OUT)
GPIO.setup(SWITCH_PIN, GPIO.IN)

pwms = [GPIO.PWM(pin, PWM_FREQ) for pin in PWM_PINS]
for pwm in pwms:
    pwm.start(us_to_duty(PWM_MIN_US))

WINDOW_SIZE = 10
switch_window = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)

try:
    while True:
        switch_state = GPIO.input(SWITCH_PIN)
        switch_window.append(switch_state)
        filtered_state = sum(switch_window) > (WINDOW_SIZE // 2)
        if filtered_state:
            for pwm in pwms:
                pwm.ChangeDutyCycle(us_to_duty(PWM_MAX_US))
        else:
            for pwm in pwms:
                pwm.ChangeDutyCycle(us_to_duty(PWM_MIN_US))
        time.sleep(0.01) 
except KeyboardInterrupt:
    pass
finally:
    for pwm in pwms:
        pwm.stop()
    GPIO.cleanup()
