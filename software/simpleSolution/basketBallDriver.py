from robot import Robot
from imageDetection import Camera, EditableThresholder, RealsenseCamera, FileThresholder
import cv2
import numpy as np
from constants import *

def getBlobDetectorParams():
    blobDetectorParams = cv2.SimpleBlobDetector_Params()

    blobDetectorParams.filterByArea = True
    blobDetectorParams.maxArea = 2000000000000000000000
    blobDetectorParams.minArea = 100

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
    ballThresholder = EditableThresholder("hsv", FileThresholder(mode="hsv"), name="Ball")
    detector = cv2.SimpleBlobDetector_create(getBlobDetectorParams())

    poleThresholder = EditableThresholder("hsv", FileThresholder(mode="hsv", path="pole.json"), name="Pole")

    while True:
        frame = cap.get_color_frame()
        
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frameHSV = cv2.erode(frameHSV, np.ones((3, 3)), iterations=1)
        thresholded = cv2.inRange(frameHSV, ballThresholder.getLow(), ballThresholder.getHigh())
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
                if kp.size > 60:
                    robot.backward()
                elif kp.size > 45:
                    poleThresholded = cv2.inRange(frameHSV, poleThresholder.getLow(), poleThresholder.getHigh())
                    poleThresholded = 255 - poleThresholded
                    poleThresholded = cv2.morphologyEx(poleThresholded, cv2.MORPH_OPEN, np.ones((5, 5)))
                    poleThresholded = cv2.rectangle(poleThresholded, (0, 0), (WIDTH, HEIGHT), (255), 5)

                    cv2.imshow("PoleThresholded", poleThresholded)
                    poleKeypoints = list(detector.detect(poleThresholded))
                    frame = cv2.drawKeypoints(frame, poleKeypoints, np.array([]), (255, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                    poleKeypoints.sort(key=lambda x: -x.size)  #  Suurim keypoint esimeseks

                    if len(poleKeypoints) == 0:
                        # Ei näe posti, keerleme ümber palli. NB kuidas me aru saame, et mis pool meie post on?
                        robot.move(-25, 0, 0)
                    else:
                        pole = poleKeypoints[0]
                        pole_x = pole.pt[0]
                        if (pole_x < WIDTH // 2 - 30):
                            robot.spinLeft()
                        elif (pole_x > WIDTH // 2 + 30):
                            robot.spinRight()
                        else:
                            robot.stop() # TODO, siin peaks viskamise tegema
                            break
                else:
                    robot.forward()
                

        cv2.imshow("Frame", frame)
        # print(ballThresholder.getHigh(), ballThresholder.getLow())
        if ord('q') == cv2.waitKey(1) & 0xFF:
            break
    ballThresholder.save()
    poleThresholder.save()


if __name__ == "__main__":
    main()