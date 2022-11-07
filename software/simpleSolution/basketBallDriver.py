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
    curMax = max(abs(s1), abs(s2), abs(s3))
    return int(s1 * maxVal / curMax), int(s2 * maxVal / curMax), int(s3 * maxVal / curMax)


def orbitLeft(robot, maxVal):
    # speed1 - tagumine
    # speed2 - parem
    # speed3 - vasak
    
    # liikumine vasakule
    # NB TODO Vajab testimist ja kalibreerimist, proovi hiljem uuesti
    return
    s = [-1, 0.5, 0.5]
    or_s = [-1, -1, -1]
    
    a = 0.5
    s1, s2, s3 = [((s[i] * a) + (or_s[i] * (1 - a)))for i in range(len(s))]
    s1, s2, s3 = mapToMax(s1, s2, s3, maxVal)
    robot.move(s1, s2, s3)


def oribtRight(robot, maxVal):
    # NB TODO Vajab testimist ja kalibreerimist, proovi hiljem uuesti
    return
    s = [1, -0.5, -0.5]
    or_s = [1, 1, 1]
    
    a = 0.5
    s1, s2, s3 = [((s[i] * a) + (or_s[i] * (1 - a)))for i in range(len(s))]
    s1, s2, s3 = mapToMax(s1, s2, s3, maxVal)
    robot.move(s1, s2, s3)


def getThrowerSpeed(distance):
    # TODO, implementeeri see meetod
    return 1000


def main():
    robot = Robot()
    cap = RealsenseCamera()
    ballThresholder = EditableThresholder("hsv", FileThresholder(mode="hsv"), name="Ball")
    ballDetector = cv2.SimpleBlobDetector_create(getBallBlobDetectorParams())
    
    poleThresholder = EditableThresholder("hsv", FileThresholder(mode="hsv", path=POLE_FILE), name="Pole")
    poleDetector = cv2.SimpleBlobDetector_create(getPoleBlobDetectorParams())
    prevTime = time.time()
    
    while True:
        try:
            frame, depth_frame = cap.get_frames()
            newTime = time.time()
            print(f"FPS: {round(1 / (newTime - prevTime), 2)}")
            prevTime = newTime
            
            frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            thresholded = cv2.inRange(frameHSV, ballThresholder.getLow(), ballThresholder.getHigh())
            thresholded = 255 - thresholded
            # thresholded = rect(thresholded)
            cv2.imshow("Thresholded", thresholded)

            keypoints = list(ballDetector.detect(thresholded))
            keypoints.sort(key=lambda x: -x.size)  #  Suurim keypoint esimeseks

            frame = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            
            if len(keypoints) == 0:
                robot.setSpeed(10 / 32767)
                robot.spinLeft()
            else:
                kp = keypoints[0]
                if kp.size < 40:
                    x_loc = kp.pt[0]
                    
                    # otse liikumine
                    s1 = 0
                    s2 = 1
                    s3 = -1

                    err = -P_const * (x_loc - (WIDTH / 2))
                    s1 += err
                    s2 += err
                    s3 += err

                    s1, s2, s3 = mapToMax(s1, s2, s3, 20)

                    # speed1 - tagumine
                    # speed2 - parem
                    # speed3 - vasak
                    robot.move(s1, s2, s3)
                elif kp.size < 60:
                    
                    # have to orbit the ball and get the location of post
                    
                    poleThresholded = cv2.inRange(frame, poleThresholder.getLow(), poleThresholder.getHigh())
                    poleThresholded = 255 - poleThresholded
                    poleThresholded = rect(poleThresholded)
                    cv2.imshow("Pole thresholded", thresholded)
                    
                    poleKeypoints = list(poleDetector.detect(poleThresholded))
                    poleKeypoints.sort(key = lambda x: -x.size)  #  Suurim keypoint esimeseks

                    if len(poleKeypoints) == 0:
                        orbitLeft(robot, 50)
                    else:
                        pole_kp = poleKeypoints[0]
                        pole_x = int(pole_kp.pt[0])
                        pole_y = int(pole_kp.pt[1])
                        # TODO, siin vb tahame teha nii, et ta läheb hästi aeglaselt palli poole, kui pall on keskmele lähemal
                        if pole_x < (WIDTH // 2 - 30):
                            oribtRight(robot, 20)
                        elif pole_x >  (WIDTH // 2 + 30):
                            orbitLeft(robot, 20)
                        else:
                            # Sõidame otse palli poole, ilmselt tuleks teha mingi palli keskel hoidmine ka, aga selleks tagumist mootorit vaja
                            pole_dist = getDistance(depth_frame, (pole_x, pole_y))
                            throwerSpeed = getThrowerSpeed(pole_dist)
                            startTime = time.time()
                            while time.time() - startTime < 3:
                                robot.move(0, 5, -5, throwerSpeed)

                else:
                    robot.backward()
        except StopIteration as e:
            pass

        cv2.imshow("Frame", frame)
        # print(ballThresholder.getHigh(), ballThresholder.getLow())
        if ord('q') == cv2.waitKey(1) & 0xFF:
            break
    ballThresholder.save()
    poleThresholder.save()


def thresh():
    cap = RealsenseCamera()
    ballThresholder = EditableThresholder("hsv", FileThresholder(mode="hsv"), name="Ball")
    ballDetector = cv2.SimpleBlobDetector_create(getBallBlobDetectorParams())
    
    poleThresholder = EditableThresholder("hsv", FileThresholder(mode="hsv", path=POLE_FILE), name="Pole")
    poleDetector = cv2.SimpleBlobDetector_create(getPoleBlobDetectorParams())
    prevTime = time.time()
    while True:
        frame, depth_frame = cap.get_frames()
        newTime = time.time()
        print(f"FPS: {round(1 / (newTime - prevTime), 2)}")
        prevTime = newTime
        
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        thresholded = cv2.inRange(frameHSV, ballThresholder.getLow(), ballThresholder.getHigh())
        thresholded = 255 - thresholded
        # thresholded = rect(thresholded)
        cv2.imshow("Thresholded", thresholded)

        poleThresholded = cv2.inRange(frameHSV, poleThresholder.getLow(), poleThresholder.getHigh())
        poleThresholded = 255 - poleThresholded
        poleThresholded = rect(poleThresholded)
        cv2.imshow("PoleThresholded", poleThresholded)

        keypoints = list(ballDetector.detect(thresholded))
        
        poleKeypoints = list(poleDetector.detect(poleThresholded))
        
        frame = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        frame = cv2.drawKeypoints(frame, poleKeypoints, np.array([]), (255, 0, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        if len(poleKeypoints) > 0:
            frame = cv2.circle(frame, (int(poleKeypoints[0].pt[0]), int(poleKeypoints[0].pt[1])), 3, (255, 255, 0), 1)
        
        cv2.imshow("Frame", frame)
        
        if ord('q') == cv2.waitKey(1) & 0xFF:
            break
    poleThresholder.save()
    ballThresholder.save()


if __name__ == "__main__":
    thresh()
    #main()