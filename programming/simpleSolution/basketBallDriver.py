from robot import Robot
from imageDetection import Camera, EditableThresholder
import cv2


def main():
    robot = Robot()
    cap = Camera()
    thresholder = EditableThresholder()
    while True:
        frame = cap.readFrame()
        cv2.imshow("Frame", frame)
        
        print(thresholder.getHigh(), thresholder.getLow())
        if ord('q') == cv2.waitKey(1) & 0xFF:
            break


if __name__ == "__main__":
    main()