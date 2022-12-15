import cv2
import json
import numpy as np
import pyrealsense2 as rs
import numpy as np
from enum import Enum

class Camera:
    def __init__(self, id=0):
        self.cap = cv2.VideoCapture(id)
    
    def readFrame(self):
        ret, frame = self.cap.read()
        return frame

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
                depth_enabled = True):
        
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

    
    def get_frames(self, aligned = True):
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)
        return np.asanyarray(frames.get_color_frame().get_data()), frames.get_depth_frame()


class Mode(Enum):
    RGB = "ŕgb"
    HSV = "hsv"


class Thresholder:
    def __init__(self, mode):
        if mode.value not in ['rgb', 'hsv']:
            raise ValueError(f"Parameter \"{mode.value}\" is not one of allowed types for mode.")
        self.mode = mode
        

class FileThresholder(Thresholder):
    def __init__(self, path="thres.json", mode = Mode.RGB):
        super().__init__(mode)
        self.path = path
        self.reload()
    

    def get_low(self):
        return self.low


    def get_high(self):
        return self.high

    
    def reload(self):
        with open(self.path, 'r') as f:
            fileData = json.load(f)
            self.high = np.array(fileData[self.mode.value]["high"])
            self.low = np.array(fileData[self.mode.value]["low"])
                
    
    def save(self):
        current_file = {}
        with open(self.path, 'r') as f:
            current_file = json.load(f)
        
        current_file[self.mode.value]["high"] = self.get_high().tolist()
        current_file[self.mode.value]["low"] = self.get_low().tolist()
        with open(self.path, "w") as f:
            json.dump(current_file, f)


class EditableThresholder(Thresholder):
    def __init__(self, mode=Mode.RGB, file_thresholder: FileThresholder = None, name = "Thresholder"):
        super().__init__(mode)
        self.file_thresholder = file_thresholder
        if file_thresholder == None:
            self.low = np.array([0, 0, 0])
            self.high = np.array([255, 255, 255])
        else:
            self.low = file_thresholder.get_low()
            self.high = file_thresholder.get_high()

        cv2.namedWindow(name)
        cv2.createTrackbar(f"low {self.mode.value[0].upper()}", name, self.low[0], 255, lambda x: self.change_value("l", 0, x))
        cv2.createTrackbar(f"low {self.mode.value[1].upper()}", name, self.low[1], 255, lambda x: self.change_value("l", 1, x))
        cv2.createTrackbar(f"low {self.mode.value[2].upper()}", name, self.low[2], 255, lambda x: self.change_value("l", 2, x))
        cv2.createTrackbar(f"high {self.mode.value[0].upper()}", name, self.high[0], 255, lambda x: self.change_value("h", 0, x))
        cv2.createTrackbar(f"high {self.mode.value[1].upper()}", name, self.high[1], 255, lambda x: self.change_value("h", 1, x))
        cv2.createTrackbar(f"high {self.mode.value[2].upper()}", name, self.high[2], 255, lambda x: self.change_value("h", 2, x))


    def get_low(self):
        return self.low


    def get_high(self):
        return self.high


    def reload(self):
        if self.file_thresholder != None:
            self.file_thresholder.reload()
            self.low = self.file_thresholder.get_low()
            self.high = self.file_thresholder.get_high()
        else:
            self.low = np.array([0, 0, 0])
            self.high = np.array([255, 255, 255])

    def save(self):
        if self.file_thresholder != None:
            self.file_thresholder.high = self.get_high()
            self.file_thresholder.low = self.get_low()   
            self.file_thresholder.save()

    
    def change_value(self, lh, i, x):
        if lh == 'l':
            self.low[i] = x
        elif lh == 'h':
            self.high[i] = x
        else:
            raise ValueError("Bad parameter")
        

def main():
    e = EditableThresholder(file_thresholder=FileThresholder(path="thres.json"))
    cap = RealsenseCamera()
    while True:
        frame = cap.get_color_frame()
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        cv2.imshow("Window", frame_hsv)
        if cv2.waitKey(1) == ord('q') & 0xFF:
            e.save()
            break
    #    f = FileThresholder(mode=Mode.RGB, path="programming/simpleSolution/thres.json")
    #    f.save()


if __name__ == "__main__":
    main()