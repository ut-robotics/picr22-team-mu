from robot import Robot
from imageDetection import Camera
import cv2


def main():
    robot = Robot()
    cap = Camera()
    while True:
        frame = cap.readFrame()
        cv2.imshow("Frame", frame)
        
        
        
        if ord('q') == cv2.waitKey(1) & 0xFF:
            break


if __name__ == "__main__":
    main()