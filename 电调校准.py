import pigpio
import time

PWM_PINS = [13, 12, 18]
SERVO_MIN_PULSE = 1100
SERVO_MAX_PULSE = 1900
SWITCH_PIN = 17

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("无法连接到 pigpio daemon")

for pin in PWM_PINS:
    pi.set_mode(pin, pigpio.OUTPUT)
pi.set_mode(SWITCH_PIN, pigpio.INPUT)

for pin in PWM_PINS:
    pi.set_servo_pulsewidth(pin, SERVO_MAX_PULSE)
print(f"Init: PWM set to {SERVO_MAX_PULSE}us")

try:
    last_state = None
    while True:
        switch_state = pi.read(SWITCH_PIN)
        if switch_state != last_state:
            last_state = switch_state
            pulse = SERVO_MAX_PULSE if switch_state else SERVO_MIN_PULSE
            for pin in PWM_PINS:
                pi.set_servo_pulsewidth(pin, pulse)
            print(f"PWM set to {pulse}us (switch: {switch_state})")
        time.sleep(0.05)
except KeyboardInterrupt:
    pass
finally:
    for pin in PWM_PINS:
        pi.set_servo_pulsewidth(pin, 0)
    print("程序已停止")

