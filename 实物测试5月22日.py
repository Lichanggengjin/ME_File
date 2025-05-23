import time
import pigpio
import numpy as np
import serial

SERVO_MIN_PULSE = 1100
SERVO_MAX_PULSE = 1900

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("无法连接到 pigpio daemon")

PWM_PIN = 18 
PWM_PIN1 = 12 
pi.set_mode(PWM_PIN, pigpio.OUTPUT)
latest_accel = [0.0, 0.0, 0.0]
latest_gyro = [0.0, 0.0, 0.0]
data_ready = False
ser_port = "/dev/ttyUSB0"
ser_baudrate = 115200
ser_timeout = 2
ser = serial.Serial(ser_port, ser_baudrate, timeout=ser_timeout)
CmdPacket_Begin = 0x49
CmdPacket_End = 0x4D
CmdPacketMaxDatSizeRx = 73
CS = 0
i = 0
RxIndex = 0
buf = bytearray(5 + CmdPacketMaxDatSizeRx)
cmdLen = 0

def Cmd_RxUnpack(buf, DLen):
    global latest_accel, latest_gyro, data_ready
    scaleAccel = 0.00478515625
    scaleAngleSpeed = 0.06103515625
    if buf[0] == 0x11:
        ctl = (buf[2] << 8) | buf[1]
        L = 7
        if (ctl & 0x0001):
            latest_accel[0] = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            latest_accel[1] = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            latest_accel[2] = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
        if (ctl & 0x0004):
            latest_gyro[0] = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2
            latest_gyro[1] = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2
            latest_gyro[2] = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2
        data_ready = True
        return True
    return False

def Cmd_GetPkt(byte):
    global CS, i, RxIndex, buf, cmdLen
    CS += byte
    if RxIndex == 0:
        if byte == CmdPacket_Begin:
            i = 0
            buf[i] = CmdPacket_Begin
            i += 1
            CS = 0
            RxIndex = 1
    elif RxIndex == 1:
        buf[i] = byte
        i += 1
        RxIndex = 2 if byte != 255 else 0
    elif RxIndex == 2:
        buf[i] = byte
        i += 1
        if 0 < byte <= CmdPacketMaxDatSizeRx:
            cmdLen = byte
            RxIndex = 3
        else:
            RxIndex = 0
    elif RxIndex == 3:
        buf[i] = byte
        i += 1
        if i >= cmdLen + 3:
            RxIndex = 4
    elif RxIndex == 4:
        CS -= byte
        if (CS & 0xFF) == byte:
            buf[i] = byte
            i += 1
            RxIndex = 5
        else:
            RxIndex = 0
    elif RxIndex == 5:
        RxIndex = 0
        if byte == CmdPacket_End:
            buf[i] = byte
            i += 1
            return Cmd_RxUnpack(buf[3:i-2], i-5)
    return False

def Cmd_PackAndTx(pDat, DLen):
    if DLen == 0 or DLen > 19:
        return -1
    buf = bytearray([0x00]*46) + bytearray([0x00, 0xff, 0x00, 0xff, 0x49, 0xFF, DLen]) + bytearray(pDat[:DLen])
    CS = sum(buf[51:51+DLen+2]) & 0xFF
    buf.append(CS)
    buf.append(0x4D)
    ser.write(buf)
    return 0

def init_sensor():
    params = [0x12,5,255,0,((2 & 3) << 1) | (0 & 1),30,1,3,5,0xFF,0x0F]
    Cmd_PackAndTx(params, len(params))
    time.sleep(0.2)
    Cmd_PackAndTx([0x03], 1)
    time.sleep(0.2)
    Cmd_PackAndTx([0x19], 1)

def read_data():
    global data_ready
    while not data_ready:
        data = ser.read(1)
        if data:
            Cmd_GetPkt(data[0])
    data_ready = False
    return latest_accel.copy(), latest_gyro.copy()

class LowPassFilter:
    def __init__(self, alpha=0.5, initial_value=None):
        self.alpha = alpha
        self.state = initial_value
    def filter(self, value):
        if self.state is None:
            self.state = value.copy() if isinstance(value, list) else value
        else:
            if isinstance(value, list):
                self.state = [self.alpha * v + (1 - self.alpha) * s for v, s in zip(value, self.state)]
            else:
                self.state = self.alpha * value + (1 - self.alpha) * self.state
        return self.state

class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.last_error = 0.0
        self.integral = 0.0
    def update(self, measurement, dt):
        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return output

accel_filter = LowPassFilter(alpha=0.3, initial_value=[0.0, 0.0, 0.0])
gyro_filter = LowPassFilter(alpha=0.3, initial_value=[0.0, 0.0, 0.0])

def get_accelerometer_data():
    accel = [0.0, 0.0, 0.0]
    accel, _ = read_data()
    filtered_accel = accel_filter.filter(accel)
    return [filtered_accel[0], filtered_accel[1], filtered_accel[2]]

def get_gyroscope_data():
    gyro = [0.0, 0.0, 0.0]
    _, gyro = read_data()
    filtered_gyro = gyro_filter.filter(gyro)
    return [filtered_gyro[0], filtered_gyro[1], filtered_gyro[2]]

x_position_pid = PIDController(kp=1.0, ki=0.1, kd=0.05, setpoint=0.0)
x_position = 0.0
x_velocity = 0.0

def update_x_position(dt):
    global x_position, x_velocity
    ax = get_accelerometer_data()[1]
    x_velocity += ax * dt
    x_position += x_velocity * dt
    return x_position  

class SlidingWindowFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.values = []
    def filter(self, value):
        self.values.append(value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
        return sum(self.values) / len(self.values)

pwm_filter = SlidingWindowFilter(window_size=5)

def output_to_pwm(output, min_output=-1.0, max_output=1.0):
    output = max(min(output, max_output), min_output)
    if output == 0:
        pulse = 1500
    elif output > 0:
        # 映射output从0~max_output到1500~1700
        pulse = 1500 + (output / 20) * (200)
    else:
        # 映射output从min_output~0到1100~1500
        pulse = 1500 + (output /20) * (400)
    filtered_pulse = pwm_filter.filter(pulse)
    pi.set_servo_pulsewidth(PWM_PIN, int(filtered_pulse))
    return int(filtered_pulse)

if __name__ == "__main__":
    BUTTON_PIN = 17
    pi.set_mode(BUTTON_PIN, pigpio.INPUT)
    pi.set_pull_up_down(BUTTON_PIN, pigpio.PUD_UP)
    dt = 0.02
    try:
        while True:
            pos = update_x_position(dt)
            button_pressed = pi.read(BUTTON_PIN) == 0
            if button_pressed:
                output = x_position_pid.update(pos, dt)
                pwm_val = output_to_pwm(output)
                pi.set_servo_pulsewidth(PWM_PIN, pwm_val)
                pi.set_servo_pulsewidth(PWM_PIN1, pwm_val)
                print(f"Button pressed | pos={pos:.3f} | output={output:.3f} | pwm={pwm_val}")
            else:
                pi.set_servo_pulsewidth(PWM_PIN, 0)
                pi.set_servo_pulsewidth(PWM_PIN1, 0)
                print(f"Button not pressed | pos={pos:.3f}")
            time.sleep(dt)
    except KeyboardInterrupt:
        pi.set_servo_pulsewidth(PWM_PIN, 0)
        print("程序已停止")

