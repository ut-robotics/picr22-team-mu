from robot import Robot
from imageDetection import Camera, EditableThresholder, RealsenseCamera, FileThresholder
import cv2
import numpy as np
from constants import *
import time

def getBallBlobDetectorParams():
    blobDetectorParams = cv2.SimpleBlobDetector_Params()

    blobDetectorParams.filterByArea = True
    blobDetectorParams.maxArea = 4500
    blobDetectorParams.minArea = 75

    blobDetectorParams.filterByCircularity = True
    blobDetectorParams.maxCircularity = 1
    blobDetectorParams.minCircularity = 0.5

    blobDetectorParams.filterByConvexity = False
    blobDetectorParams.maxConvexity = 0
    blobDetectorParams.minConvexity = 0

    blobDetectorParams.filterByInertia = False
    blobDetectorParams.maxInertiaRatio = 0
    blobDetectorParams.minInertiaRatio = 0

    return blobDetectorParams


def getPoleBlobDetectorParams():
    blobDetectorParams = cv2.SimpleBlobDetector_Params()

    blobDetectorParams.filterByArea = True
    blobDetectorParams.maxArea = 450000
    blobDetectorParams.minArea = 1500

    blobDetectorParams.filterByCircularity = True
    blobDetectorParams.maxCircularity = 1
    blobDetectorParams.minCircularity = 0.5

    blobDetectorParams.filterByConvexity = False
    blobDetectorParams.maxConvexity = 0
    blobDetectorParams.minConvexity = 0

    blobDetectorParams.filterByInertia = False
    blobDetectorParams.maxInertiaRatio = 0
    blobDetectorParams.minInertiaRatio = 0

    return blobDetectorParams


def rect(frame):
    sh = frame.shape
    return cv2.rectangle(frame, (0, 0), (sh[1], sh[0]), (255), 1)


# returns distance in mm
def getDistance(depthFrame, loc):
    return depthFrame.get_distance(loc[0], loc[1]) * 1000


def mapToMax(s1, s2, s3, maxVal):
    curMax = max(s1, s2, s3)
    return int(s1 * maxVal / curMax), int(s2 * maxVal / curMax), int(s3 * maxVal / curMax)


def main():
    robot = Robot()
    cap = RealsenseCamera()
    ballThresholder = EditableThresholder("hsv", FileThresholder(mode="hsv"), name="Ball")
    ballDetector = cv2.SimpleBlobDetector_create(getBallBlobDetectorParams())
    
    poleThresholder = EditableThresholder("hsv", FileThresholder(mode="hsv", path="blue_pole.json"), name="Pole")
    poleDetector = cv2.SimpleBlobDetector_create(getPoleBlobDetectorParams())
    prevTime = time.time()
    while True:
        try:
            frame, depth_frame = cap.get_frames()
            newTime = time.time()
            print(f"FPS: {round(1 / (newTime - prevTime), 2)}")
            prevTime = newTime
            
            frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            frameHSV = cv2.erode(frameHSV, np.ones((3, 3)), iterations=1)
            thresholded = cv2.inRange(frameHSV, ballThresholder.getLow(), ballThresholder.getHigh())
            thresholded = 255 - thresholded
            # thresholded = rect(thresholded)
            cv2.imshow("Thresholded", thresholded)

            keypoints = list(ballDetector.detect(thresholded))
            keypoints.sort(key=lambda x: -x.size)  #  Suurim keypoint esimeseks

            frame = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            
            if len(keypoints) == 0 and False:
                robot.spinLeft()
            else:
                #kp = keypoints[0]
                #x_loc = kp.pt[0]
                x_loc = 400

                # otse liikumine
                s1 = 0
                s2 = -1
                s3 = 1

                err = P_const * (x_loc - (WIDTH / 2))
                s1 += err
                s2 += err
                s3 += err

                s1, s2, s3 = mapToMax(s1, s2, s3, 100)

                # speed1 - tagumine
                # speed2 - parem
                # speed3 - vasak
                robot.move(s1, s2, s3)
                raise StopIteration()
                
                if (x_loc < WIDTH // 2 - 30):
                    robot.spinLeft()
                elif (x_loc > WIDTH // 2 + 30):
                    robot.spinRight()
                else:
                    # robot vaatab palli poole
                    raise StopIteration()
                    if kp.size > 60:
                        robot.backward()
                    elif kp.size > 45:
                        poleThresholded = cv2.inRange(frameHSV, poleThresholder.getLow(), poleThresholder.getHigh())
                        poleThresholded = 255 - poleThresholded
                        poleThresholded = cv2.morphologyEx(poleThresholded, cv2.MORPH_OPEN, np.ones((5, 5)))
                        poleThresholded = cv2.rectangle(poleThresholded, (0, 0), (WIDTH, HEIGHT), (255), 5)

                        cv2.imshow("PoleThresholded", poleThresholded)
                        poleKeypoints = list(poleDetector.detect(poleThresholded))
                        frame = cv2.drawKeypoints(frame, poleKeypoints, np.array([]), (255, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                        poleKeypoints.sort(key=lambda x: -x.size)  #  Suurim keypoint esimeseks

                        if len(poleKeypoints) == 0:
                            # Ei näe posti, keerleme ümber palli. NB kuidas me aru saame, et mis pool meie post on?
                            robot.move(-25, 0, 0)
                        else:
                            pole = poleKeypoints[0]
                            pole_x = pole.pt[0]
                            if (pole_x < WIDTH // 2 - 30):
                                robot.move(-25, 0, 0)
                            elif (pole_x > WIDTH // 2 + 30):
                                robot.move(-25, 0, 0)
                            else:
                                robot.stop() # TODO, siin peaks viskamise tegema
                                break
                    else:
                        robot.forward()
        except StopIteration as e:
            pass

        cv2.imshow("Frame", frame)
        # print(ballThresholder.getHigh(), ballThresholder.getLow())
        if ord('q') == cv2.waitKey(1) & 0xFF:
            break
    ballThresholder.save()
    poleThresholder.save()


if __name__ == "__main__":
    main()