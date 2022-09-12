import cv2
import json
import numpy as np

class Camera:
    def __init__(self, id=0):
        self.cap = cv2.VideoCapture(id)
    
    def readFrame(self):
        ret, frame = self.cap.read()
        return frame


class Thresholder:
    def __init__(self, mode):
        if mode.lower() not in ['rgb', 'hsv']:
            raise ValueError(f"Parameter \"{mode}\" is not one of allowed types for mode.")
        self.mode = mode.lower()
        

class FileThresholder(Thresholder):
    def __init__(self, path="thres.json", mode = "RGB"):
        super().__init__(mode)
        self.path = path
        self.reload()
    

    def getLow(self):
        return self.low


    def getHigh(self):
        return self.high

    
    def reload(self):
        with open(self.path, 'r') as f:
            fileData = json.load(f)
            self.high = np.array(fileData[self.mode]["high"])
            self.low = np.array(fileData[self.mode]["low"])
                
    
    def save(self):
        currentFile = {}
        with open(self.path, 'r') as f:
            currentFile = json.load(f)
        
        currentFile[self.mode]["high"] = self.getHigh().tolist()
        currentFile[self.mode]["low"] = self.getLow().tolist()
        with open(self.path, "w") as f:
            json.dump(currentFile, f)


class EditableThresholder(Thresholder):
    def __init__(self, mode="rgb", fileThresholder: FileThresholder = None):
        super().__init__(mode)
        self.fileThresholder = fileThresholder
        if fileThresholder == None:
            self.low = np.array([0, 0, 0])
            self.high = np.array([255, 255, 255])
        else:
            self.low = fileThresholder.getLow()
            self.high = fileThresholder.getHigh()

        cv2.namedWindow("Thresholding")
        cv2.createTrackbar(f"low {self.mode[0].upper()}", "Thresholding", self.low[0], 255, lambda x: self.changeValue("l", 0, x))
        cv2.createTrackbar(f"low {self.mode[1].upper()}", "Thresholding", self.low[1], 255, lambda x: self.changeValue("l", 1, x))
        cv2.createTrackbar(f"low {self.mode[2].upper()}", "Thresholding", self.low[2], 255, lambda x: self.changeValue("l", 2, x))
        cv2.createTrackbar(f"high {self.mode[0].upper()}", "Thresholding", self.high[0], 255, lambda x: self.changeValue("h", 0, x))
        cv2.createTrackbar(f"high {self.mode[1].upper()}", "Thresholding", self.high[1], 255, lambda x: self.changeValue("h", 1, x))
        cv2.createTrackbar(f"high {self.mode[2].upper()}", "Thresholding", self.high[2], 255, lambda x: self.changeValue("h", 2, x))


    def getLow(self):
        return self.low


    def getHigh(self):
        return self.high


    def reload(self):
        if self.fileThresholder != None:
            self.fileThresholder.reload()
            self.low = self.fileThresholder.getLow()
            self.high = self.fileThresholder.getHigh()
        else:
            self.low = np.array([0, 0, 0])
            self.high = np.array([255, 255, 255])

    def save(self):
        if self.fileThresholder != None:
            self.fileThresholder.high = self.getHigh()
            self.fileThresholder.low = self.getLow()   
            self.fileThresholder.save()

    
    def changeValue(self, lh, i, x):
        if lh == 'l':
            self.low[i] = x
        elif lh == 'h':
            self.high[i] = x
        else:
            raise ValueError("Bad parameter")
        

def main():
    e = EditableThresholder(fileThresholder=FileThresholder(path="programming/simpleSolution/thres.json"))
    while True:
        if cv2.waitKey(1) == ord('q') & 0xFF:
            e.save()
            break
    #    f = FileThresholder(mode="rgb", path="programming/simpleSolution/thres.json")
    #    f.save()


if __name__ == "__main__":
    main()