import cv2

class Camera:
    
    def __init__(self, id=0):
        self.cap = cv2.VideoCapture(id)
    
    def readFrame(self):
        ret, frame = self.cap.read()
        return frame