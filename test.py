import sensor, image, time

from pid import PID
from pyb import Servo, UART

pan_servo=Servo(1)
tilt_servo=Servo(2)

pan_servo.calibration(500,2500,500)
tilt_servo.calibration(500,2500,500)


pan_servo.angle(97)
tilt_servo.angle(0)

uart = UART(3, 115200)  # 修改为您实际使用的串口号和波特率

red_threshold  = (0, 0, -128, 127, -1, 127)
#pan_pid = PID(p=0.07, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
#tilt_pid = PID(p=0.05, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
pan_pid = PID(p=0.4, i=0, imax=90)#在线调试使用这个PID
tilt_pid = PID(p=0.15, i=0, imax=90)#在线调试使用这个PID
z
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
clock = time.clock() # Tracks FPS.

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob

ax,ay,bx,by,cx,cy,dx,dy = None,None,None,None,None,None,None,None
b_pan_error = 20
b_tilt_error = 20
b_pan_error = 20
b_tilt_error = 20
b_pan_error = 20
b_tilt_error = 20

while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.

    blobs = img.find_rects(threshold=30000)
    a = [0] * 16
    i = 0
    for r in img.find_rects(threshold=30000):
        for p in r.corners():
            img.draw_circle(p[0], p[1], 5, color=(0, 255, 0))
            a[i] = p[0]
            a[i + 1] = p[1]
            i = i + 2

    if blobs:
        max_blob = find_max(blobs)
        a_pan_error = a[6] - img.width() / 2
        a_tilt_error = a[7] - img.height() / 2
        img.draw_rectangle(max_blob.rect())
        img.draw_cross(a[6], a[7])

        pan_output = pan_pid.get_pid(a_pan_error, 1) / 2
        tilt_output = tilt_pid.get_pid(a_tilt_error, 1)


        print("pan_output",pan_output)
        pan_servo.angle(pan_servo.angle()-pan_output)
        tilt_servo.angle(tilt_servo.angle()-tilt_output)
        print("67")
        print(a_pan_error,a_tilt_error)
        #print(max_blob.corners())
        #print(a[0],a[1],a[2],a[3],a[4],a[6],a[7],a[8])
        if abs(a_pan_error) < 20 and abs(a_tilt_error) < 20:
            time.sleep(1)
            while abs(b_pan_error)>10 a abs(b_tilt_error)>10:
                b_pan_error = a[4] - a[6]
                b_tilt_error = a[5] - a[7]

                pan_output = pan_pid.get_pid(b_pan_error, 1) / 2
                tilt_output = tilt_pid.get_pid(b_tilt_error, 1)

                print("pan_output",pan_output)
                pan_servo.angle(pan_servo.angle()-pan_output)
                tilt_servo.angle(tilt_servo.angle()-tilt_output)
                print("45")
                print(b_pan_error,b_tilt_error)
                if abs(b_pan_error) > 10 or abs(b_tilt_error) > 10:
                    continue
                elif abs(b_pan_error) < 10 and abs(b_tilt_error) < 10:

                    break

            if abs(b_pan_error) < 10 and abs(b_tilt_error) < 10:
                time.sleep(1)
                while abs(c_pan_error)>10 and abs(c_tilt_error)>10:
                    b_pan_error = a[2] - a[4]
                    b_tilt_error = a[3] - a[5]

                    pan_output = pan_pid.get_pid(c_pan_error, 1) / 2
                    tilt_output = tilt_pid.get_pid(c_tilt_error, 1)

                    print("pan_output",pan_output)
                    pan_servo.angle(pan_servo.angle()-pan_output)
                    tilt_servo.angle(tilt_servo.angle()-tilt_output)
                    #print("23")
                    print(c_pan_error,c_tilt_error)
                    if abs(c_pan_error) > 10 or abs(c_tilt_error) > 10:
                        continue
                    elif abs(c_pan_error) < 10 and abs(c_tilt_error) < 10:
                        break


                if abs(c_pan_error) < 10 and abs(c_tilt_error) < 10:
                    time.sleep(1)
                    while abs(d_pan_error)>10 and abs(d_tilt_error)>10:
                        b_pan_error = a[0] - a[2]
                        b_tilt_error = a[1] - a[3]

                        pan_output = pan_pid.get_pid(d_pan_error, 1) / 2
                        tilt_output = tilt_pid.get_pid(d_tilt_error, 1)

                        print("pan_output",pan_output)
                        pan_servo.angle(pan_servo.angle()-pan_output)
                        tilt_servo.angle(tilt_servo.angle()-tilt_output)
                        #print("01")
                        print(d_pan_error,d_tilt_error)
                        if abs(d_pan_error) > 10 or abs(d_tilt_error) > 10:
                            continue
                        elif abs(d_pan_error) < 10 and abs(d_tilt_error) < 10:
                            break
