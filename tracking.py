import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

TRIG = 1
ECHO = 0
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13
TrackSensorLeftPin1 = 3  # 定义左边第一个循迹红外传感器引脚为3号口
TrackSensorLeftPin2 = 5  # 定义左边第二个循迹红外传感器引脚为5号口
TrackSensorRightPin1 = 4  # 定义右边第一个循迹红外传感器引脚为4号口
TrackSensorRightPin2 = 18  # 定义右边第二个循迹红外传感器引脚为18号口

GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)
GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(TrackSensorLeftPin1, GPIO.IN)
GPIO.setup(TrackSensorLeftPin2, GPIO.IN)
GPIO.setup(TrackSensorRightPin1, GPIO.IN)
GPIO.setup(TrackSensorRightPin2, GPIO.IN)

pwm_ENA = GPIO.PWM(ENA, 2000)
pwm_ENB = GPIO.PWM(ENB, 2000)
pwm_ENA.start(0)
pwm_ENB.start(0)

def voltage_set(signal):
    IN1 = 20
    IN2 = 21
    IN3 = 19
    IN4 = 26
    GPIO.output(IN1, signal[0])
    GPIO.output(IN2, signal[1])
    GPIO.output(IN3, signal[2])
    GPIO.output(IN4, signal[3])
    pass

def flexible_run(rate, delay_time=None):
    signal = [GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.HIGH]
    voltage_set(signal)
    pwm_ENA.ChangeDutyCycle(int((1 - rate[0]) * rate[1] / 2 + rate[2]))
    pwm_ENB.ChangeDutyCycle(int((1 + rate[0]) * rate[1] / 2 + rate[2]))
    if delay_time is not None:
        time.sleep(delay_time)
    else:
        pass
    brake()

def forward(speed, delay_time=None):
    signal = [GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.HIGH]
    voltage_set(signal)
    pwm_ENA.ChangeDutyCycle(speed)
    pwm_ENB.ChangeDutyCycle(speed)
    if delay_time is not None:
        time.sleep(delay_time)
    else:
        pass
    brake()

def left(speed, delay_time=None):
    signal = [GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.HIGH]
    voltage_set(signal)
    pwm_ENA.ChangeDutyCycle(0)
    pwm_ENB.ChangeDutyCycle(speed)
    if delay_time is not None:
        time.sleep(delay_time)
    else:
        pass
    brake()

def right(speed, delay_time=None):
    signal = [GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.LOW]
    voltage_set(signal)
    pwm_ENA.ChangeDutyCycle(speed)
    pwm_ENB.ChangeDutyCycle(0)
    if delay_time is not None:
        time.sleep(delay_time)
    else:
        pass
    brake()

def brake(speed=100, delay_time=None):
    signal = [GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.LOW]
    voltage_set(signal)
    pwm_ENA.ChangeDutyCycle(speed)
    pwm_ENB.ChangeDutyCycle(speed)
    if delay_time is not None:
        time.sleep(delay_time)
    else:
        pass


def read_sensors():
    # 读取左侧两个循迹红外传感器的状态
    left1 = GPIO.input(TrackSensorLeftPin1)
    left2 = GPIO.input(TrackSensorLeftPin2)
    # 读取右侧两个循迹红外传感器的状态
    right1 = GPIO.input(TrackSensorRightPin1)
    right2 = GPIO.input(TrackSensorRightPin2)

    # 返回各个传感器的状态
    return left1, left2, right1, right2
    


def Distance_Ultrasound():
    emitTime = 0
    while emitTime == 0:
        GPIO.output(TRIG, GPIO.LOW)
        time.sleep(0.000002)
        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIG, GPIO.LOW)
        while GPIO.input(ECHO) == 0:
            emitTime = time.time()
        while GPIO.input(ECHO) == 1:
            acceptTime = time.time()
        totalTime = acceptTime - emitTime
        distanceReturn = totalTime * 340 / 2 * 100
    return distanceReturn



def Line_Tracking():
    while True:
        dis = Distance_Ultrasound()
        left1, left2, right1, right2 = read_sensors()
        if ((left2) == False) and ((right1) == False):
            while dis > 30:
                forward(5, delay_time=0.1)
                print("向前CAN GO:距离", dis, "cm")
                break
            while dis < 30:
                print("CAN'T GO:距离", dis, "cm")
                brake(delay_time=0.1)
                time.sleep(0.5)
                break
        elif ((left2) == False ) and ((right2) == True):
            while dis > 30:
                left(5, delay_time=2)
                print("向左CAN GO:距离", dis, "cm")
                break
            while dis < 30:
                print("CAN'T GO:距离", dis, "cm")
                brake(delay_time=0.1)
                time.sleep(0.5)
                break
        elif ((left2) == True) and ((right1) == False):
            while dis > 30:
                right(5, delay_time=2)
                print("向右CAN GO:距离", dis, "cm")
                break
            while dis < 30:
                print("CAN'T GO:距离", dis, "cm")
                brake(delay_time=0.1)
                time.sleep(0.5)
                break
        else:
            brake(delay_time=0.1)
            print("停车")
            print(f"Left1: {left1}, Left2: {left2}, Right1: {right1}, Right2: {right2}")
            time.sleep(0.1)
            break

def Obstacle_Avoidance():
    while True:
        dis = Distance_Ultrasound()
        while dis > 30:
            forward(20, delay_time=0.1)
            print("CAN GO:距离", dis, "cm")
            break
        while dis < 30:
            print("CAN'T GO:距离", dis, "cm")
            left(20, delay_time=0.5)
            time.sleep(0.5)
            break

print("巡线与超声波避障系统运行中，按Ctrl+C退出...")
try:
    Line_Tracking()
    Obstacle_Avoidance()

except KeyboardInterrupt:
    pass

GPIO.cleanup()