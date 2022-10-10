from robot import Robot
from imageDetection import Camera, EditableThresholder, RealsenseCamera, FileThresholder
import cv2
import numpy as np
from constants import *

def getBlobDetectorParams():
    blobDetectorParams = cv2.SimpleBlobDetector_Params()

    blobDetectorParams.filterByArea = True
    blobDetectorParams.maxArea = 200000
    blobDetectorParams.minArea = 0

    blobDetectorParams.filterByCircularity = False
    blobDetectorParams.maxCircularity = 0
    blobDetectorParams.minCircularity = 0

    blobDetectorParams.filterByConvexity = False
    blobDetectorParams.maxConvexity = 0
    blobDetectorParams.minConvexity = 0

    blobDetectorParams.filterByInertia = False
    blobDetectorParams.maxInertiaRatio = 0
    blobDetectorParams.minInertiaRatio = 0

    return blobDetectorParams


def otsi_palli(robot):
    pass


def main():
    robot = Robot()
    cap = RealsenseCamera()
    thresholder = EditableThresholder("hsv", FileThresholder(mode="hsv"))
    detector = cv2.SimpleBlobDetector_create()

    while True:
        frame = cap.get_color_frame()
        
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        thresholded = cv2.inRange(frameHSV, thresholder.getLow(), thresholder.getHigh())
        thresholded = 255 - thresholded
        cv2.imshow("Thresholded", thresholded)

        keypoints = list(detector.detect(thresholded))
        keypoints.sort(key=lambda x: -x.size)  #  Suurim keypoint esimeseks

        frame = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        if len(keypoints) == 0:
            robot.spinLeft()
        else:
            kp = keypoints[0]
            x_loc = kp.pt[0]
            
            if (x_loc < WIDTH // 2 - 30):
                robot.spinLeft()
            elif (x_loc > WIDTH // 2 + 30):
                robot.spinRight()
            else:
                # robot vaatab palli poole
                print(kp.size)
                if kp.size > 45:
                    robot.stop()
                else:
                    robot.forward()
                

            print(x_loc)

        cv2.imshow("Frame", frame)
        # print(thresholder.getHigh(), thresholder.getLow())
        if ord('q') == cv2.waitKey(1) & 0xFF:
            break
    thresholder.save()


if __name__ == "__main__":
    main()