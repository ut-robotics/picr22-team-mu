import time
import cv2
import numpy as np
import pyrealsense2 as rs
import os
import serial
import struct

class Robot:
    def __init__(self):
        super().__init__()
        self.ser = serial.Serial('/dev/ttyACM0')
        self.speed = 5 / 32767


    def setSpeed(self, speed):
        self.speed = speed


    def stop(self):
        self.move(0, 0, 0)

    # self.speed - from 0 to 1, float, out of maximum speed
    # 32767 on max valid
    # -32768 on min valid
    def forward(self):
        speed = int(self.speed * 32767)
        self.move(0, speed, -speed)


    def backward(self):
        speed = int(self.speed * 32767)
        self.move(0, -speed, speed)


    # portion - turning / driving straight: 1 - spin // 0 - straight
    def orbit(self, portion, forward=True, left=True):
        if left:
            speed1 = portion
            if forward:
                speed2 = 1
                speed3 = 2 * portion - 1
            else:
                speed2 = 2 * portion - 1
                speed3 = 1
        else:
            speed1 = -portion
            if forward:
                speed2 = 1 - 2 * portion
                speed3 = -1
            else:
                speed2 = -1
                speed3 = 1 - 2 * portion
        
        speed1 = int(speed1 * 32767 * self.speed)
        speed2 = int(speed2 * 32767 * self.speed)
        speed3 = int(speed3 * 32767 * self.speed)
        self.move(speed1, speed2, speed3)

    
    def spinLeft(self):
        speed = int(self.speed * 32767)
        self.move(speed, speed, speed)


    def spinRight(self):
        speed = int(self.speed * 32767)
        self.move(-speed, -speed, -speed)

    
    # speed1 - tagumine
    # speed2 - parem
    # speed3 - vasak
    def move(self, speed1 : int, speed2 : int, speed3 : int, throwerSpeed=0, disableFailsafe=0, delimiter=0xAAAA) -> None:
        print(speed1, speed2, speed3)
        self.ser.write(struct.pack("<hhhHBH", speed1, speed2, speed3, throwerSpeed, disableFailsafe, delimiter))
        # self.ser.write(bytes(bytearray.fromhex('100010001000000000AAAA')))



class RealsenseCamera:
    def __init__(self,
                rgb_width = 848, 
                rgb_height = 480,
                rgb_framerate = 30,
                depth_width = 848, 
                depth_height = 480,
                depth_framerate = 30,
                exposure = 200, 
                white_balace = 3500,
                depth_enabled = False):
        
        self.rgb_width = rgb_width
        self.rgb_height = rgb_height
        self.rgb_framerate = rgb_framerate
        self.exposure = exposure
        self.white_balace = white_balace

        self.depth_width = depth_width
        self.depth_height = depth_height
        self.depth_framerate = depth_framerate

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.rgb_width, self.rgb_height, rs.format.bgr8, self.rgb_framerate)
        
        self.depth_enabled = depth_enabled
        if self.depth_enabled:
            self.config.enable_stream(rs.stream.depth, self.depth_width, self.depth_height, rs.format.z16, self.depth_framerate)
            
        self.align = rs.align(rs.stream.color)
        self.depth_scale = -1

        profile = self.pipeline.start(self.config)
        color_sensor = profile.get_device().query_sensors()[1]
        color_sensor.set_option(rs.option.enable_auto_exposure, False)
        color_sensor.set_option(rs.option.enable_auto_white_balance, False)
        color_sensor.set_option(rs.option.white_balance, self.white_balace)
        color_sensor.set_option(rs.option.exposure, self.exposure)

        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
    
    
    def close(self):
        self.pipeline.stop()

    
    def get_color_frame(self):
        frames = self.pipeline.wait_for_frames()
        return np.asanyarray(frames.get_color_frame().get_data())


    def has_depth_capability(self) -> bool:
        return self.depth_enabled

    
    def get_frames(self, aligned = False):
        frames = self.pipeline.wait_for_frames()
        if aligned:
            frames = self.align.process(frames)
        return np.asanyarray(frames.get_color_frame().get_data()), np.asanyarray(frames.get_depth_frame().get_data())


cam = RealsenseCamera()
startTime = time.time()
os.mkdir(f"data/{startTime}")

r = Robot()
r.move(0, -50, 50, disableFailsafe=1)
while time.time() < startTime + 3:
    frame = cam.get_color_frame()
    cv2.imshow("Aken", frame)
    curTime = time.time()
    with open(f"data/{startTime}/{curTime}.npy", "wb") as f:
        np.save(f, frame)
    if (cv2.waitKey(1) & 0xFF) == ord('q'):
        break
r.move(0, 0, 0)