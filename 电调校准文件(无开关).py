import pigpio

PWM_PINS = [13, 12, 18]
SERVO_MIN_PULSE = 1100
SERVO_MAX_PULSE = 1900

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("无法连接到 pigpio daemon")

for pin in PWM_PINS:
    pi.set_mode(pin, pigpio.OUTPUT)

for pin in PWM_PINS:
    pi.set_servo_pulsewidth(pin, SERVO_MAX_PULSE)
print(f"Init: PWM set to {SERVO_MAX_PULSE}us")

try:
    state = True
    while True:
        user_input = input("输入1切换频率，其他键退出：")
        if user_input.strip() == "1":
            state = not state
            pulse = SERVO_MAX_PULSE if state else SERVO_MIN_PULSE
            for pin in PWM_PINS:
                pi.set_servo_pulsewidth(pin, pulse)
            print(f"PWM set to {pulse}us")
        else:
            break
except KeyboardInterrupt:
    pass
finally:
    for pin in PWM_PINS:
        pi.set_servo_pulsewidth(pin, 0)
    print("程序已停止")

