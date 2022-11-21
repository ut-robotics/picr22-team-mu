from robot import Robot
from imageDetection import Camera, EditableThresholder, RealsenseCamera, FileThresholder
import cv2
import numpy as np
from constants import *
import time


def find_goal_location(thresd, top_width_threshold = 10, bottom_width_threshold = 5, kas_plot_y = False, kas_plot_x = False):
    if np.sum(255 - thresd[0]) > 255 * top_width_threshold:
        # #find x coordinate
        # indecies = np.array(range(len(thresd[0])))
        # masses = 255-thresd[0]
        # x_mean = np.sum(masses*indecies)/sum(masses)


        # goal_radius = np.sqrt(3)*np.std(masses)

        #find y coordinate
        y_bottom = 0
        sums = []
        for i in range(len(thresd)):
            sums += [np.sum(255-thresd[i])]
        for i in range(len(thresd)):
            if np.sum(255-thresd[i]) < bottom_width_threshold*255:
                y_bottom = i
                break
        
        if kas_plot_x or kas_plot_y:
            import matplotlib.pyplot as plt

        if kas_plot_y:
            plt.plot(range(len(sums)), sums)  
            plt.scatter([y_bottom], [sums[y_bottom]] , color = "r")
            plt.show()
        
        if kas_plot_x:
            plt.plot(range(len(thresd[0])), 255-thresd[0])  
            plt.scatter([np.sum((255-thresd[0])*np.array(range(len(thresd[0]))))/sum((255-thresd[0]))], [255] , color = "r")
            plt.show()
        
        return y_bottom
    else:
        return None


def getThrowerSpeedYCoord(x, a1 = -2.70683525e+02, a2 = 5.86756139e+01, dx = -4.24677858e-02, c = 1.03808178e+03):
    return a1/(x-dx) + a2/(x-dx)**2 +c


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
    blobDetectorParams.minArea = 1000

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


def orbitLeft(robot, maxVal=35, a=0.75):
    # speed1 - tagumine
    # speed2 - parem
    # speed3 - vasak
    # -35, 4, 4
    
    # liikumine vasakule
    # NB TODO Vajab testimist ja kalibreerimist, proovi hiljem uuesti
    s = [-1, 0.5, 0.5]
    or_s = [-1, -1, -1]
    
    s1, s2, s3 = [((s[i] * a) + (or_s[i] * (1 - a)))for i in range(len(s))]
    s1, s2, s3 = mapToMax(s1, s2, s3, maxVal)
    robot.move(s1, s2, s3)


def oribtRight(robot, maxVal=35, a=0.75):
    # NB TODO Vajab testimist ja kalibreerimist, proovi hiljem uuesti
    # 35, -4, -4
    s = [1, -0.5, -0.5]
    or_s = [1, 1, 1]
    
    s1, s2, s3 = [((s[i] * a) + (or_s[i] * (1 - a)))for i in range(len(s))]
    s1, s2, s3 = mapToMax(s1, s2, s3, maxVal)
    robot.move(s1, s2, s3)


def getThrowerSpeed(distance):
    return int(0.397 * distance + 221)


def delayCamera(dt, cam):
    startTime = time.time()
    while time.time() < startTime + dt:
        frame, depth_frame = cam.get_frames()
    
    lastFrame, depth_frame = cam.get_frames()
    return lastFrame


def main(robot = Robot(), basket = "magenta"):
    print("Basketball driver starting")
    cap = RealsenseCamera()
    ballThresholder = EditableThresholder("hsv", FileThresholder(mode="hsv"), name="Ball")
    ballDetector = cv2.SimpleBlobDetector_create(getBallBlobDetectorParams())
    
    poleThresholder = EditableThresholder("hsv", FileThresholder(mode="hsv", path=f"{basket}.json"), name="Pole")
    poleDetector = cv2.SimpleBlobDetector_create(getPoleBlobDetectorParams())
    prevTime = time.time()
    
    STATE = "BALL" # , "FINAL"

    while True:
        print(STATE)
        frame, depth_frame = cap.get_frames()
        newTime = time.time()
        #print(f"FPS: {round(1 / (newTime - prevTime), 2)}")
        prevTime = newTime
        
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if (STATE == "BALL"):
            thresholded = cv2.inRange(frameHSV, ballThresholder.getLow(), ballThresholder.getHigh())
            thresholded = 255 - thresholded
            # thresholded = rect(thresholded)
            cv2.imshow("Thresholded", thresholded)

            keypoints = list(ballDetector.detect(thresholded))
            keypoints.sort(key=lambda x: -x.size)  #  Suurim keypoint esimeseks

            frame = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            
            if len(keypoints) == 0:
                robot.setSpeed(8 / 32767)
                robot.spinLeft()
            else:
                kp = keypoints[0]
                x_loc = kp.pt[0]
                if kp.size < 45:                        
                    # otse liikumine
                    s1 = 0
                    s2 = 1
                    s3 = -1

                    err = -P_const * (x_loc - HALF_WIDTH)
                    s1 += err
                    s2 += err
                    s3 += err

                    s1, s2, s3 = mapToMax(s1, s2, s3, 20)

                    # speed1 - tagumine
                    # speed2 - parem
                    # speed3 - vasak
                    robot.move(s1, s2, s3)
                elif kp.size < 60:
                    # kontrollime et pall oleks keskel
                    if x_loc < (HALF_WIDTH - 15):
                        robot.spinLeft()
                    elif x_loc >  (HALF_WIDTH + 15):
                        robot.spinRight()
                    else:
                        startTime = time.time()
                        while time.time() - startTime < 2:
                            robot.move(0, 20, -20)
                        STATE = "FINAL"
                else:
                    robot.backward()
        elif STATE == "FINAL":
            poleThresholded = cv2.inRange(frameHSV, poleThresholder.getLow(), poleThresholder.getHigh())
            poleThresholded = 255 - poleThresholded
            poleThresholded = rect(poleThresholded)
            cv2.imshow("Pole thresholded", poleThresholded)
            poleKeypoints = list(poleDetector.detect(poleThresholded))
            frame = cv2.drawKeypoints(frame, poleKeypoints, np.array([]), (255, 0, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            if len(poleKeypoints) == 0:
                orbitLeft(robot, 15, 0.57)
            else:
                pole_kp = poleKeypoints[0]
                x_loc = pole_kp.pt[0]
                loc = (int(x_loc), int(pole_kp.pt[0]))
                if x_loc < HALF_WIDTH - 8:
                    oribtRight(robot, 8, 0.57)
                elif x_loc > HALF_WIDTH + 8:
                    orbitLeft(robot, 8, 0.57)
                else:
                    robot.move(0, -20, 20, disableFailsafe=1)
                    delayCamera(0.25, cap)
                    robot.move(0, 0, 0)
                    
                    y_coord = find_goal_location(poleThresholded[1:,loc[0] - 20 : loc[0] + 20])
                    speed = getThrowerSpeedYCoord(y_coord / HEIGHT)
                    print(y_coord, speed)

                    # dist = getDistance(depth_frame, loc)
                    # speed = getThrowerSpeed(dist)
                    # print(dist, speed)
                    
                    startTime = time.time()
                    if speed:
                        while time.time() - startTime < 1:
                            frame, depth_frame = cap.get_frames()
                            frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                            poleThresholded = cv2.inRange(frameHSV, poleThresholder.getLow(), poleThresholder.getHigh())
                            poleThresholded = 255 - poleThresholded
                            poleThresholded = rect(poleThresholded)
                            cv2.imshow("Pole thresholded", poleThresholded)
                            poleKeypoints = list(poleDetector.detect(poleThresholded))
                            if len(poleKeypoints) > 0:
                                pole_kp = poleKeypoints[0]
                                x_loc = pole_kp.pt[0]
                                loc = (int(x_loc), int(pole_kp.pt[0]))
                                y_coord = find_goal_location(poleThresholded[1:,loc[0] - 20 : loc[0] + 20])
                                speed = getThrowerSpeedYCoord(y_coord / HEIGHT)   # TODO See on katki debuggi pärast
                                print(y_coord, speed)
                            robot.move(0, 20, -20, int(speed))
                    STATE = "BALL"
        else:
            raise ValueError("Unexpected state")

        cv2.imshow("Frame", frame)
        if ord('q') == cv2.waitKey(1) & 0xFF:
            break
    ballThresholder.save()
    poleThresholder.save()
    yield


def thresh(basket = "magenta"):
    cap = RealsenseCamera()
    ballThresholder = EditableThresholder("hsv", FileThresholder(mode="hsv"), name="Ball")
    ballDetector = cv2.SimpleBlobDetector_create(getBallBlobDetectorParams())
    
    poleThresholder = EditableThresholder("hsv", FileThresholder(mode="hsv", path=f"{basket}.json"), name="Pole")
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
            print(getDistance(depth_frame, (int(poleKeypoints[0].pt[0]), int(poleKeypoints[0].pt[1]))))
        
        cv2.imshow("Frame", frame)
        
        if ord('q') == cv2.waitKey(1) & 0xFF:
            break
    poleThresholder.save()
    ballThresholder.save()


def competition(basket="magenta"):
    cap = RealsenseCamera()
    
    poleThresholder = EditableThresholder("hsv", FileThresholder(mode="hsv", path=f"{basket}.json"), name="Pole")
    poleDetector = cv2.SimpleBlobDetector_create(getPoleBlobDetectorParams())
    prevTime = time.time()
    while True:
        frame, depth_frame = cap.get_frames()
        newTime = time.time()
        #print(f"FPS: {round(1 / (newTime - prevTime), 2)}")
        prevTime = newTime
        
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        poleThresholded = cv2.inRange(frameHSV, poleThresholder.getLow(), poleThresholder.getHigh())
        poleThresholded = 255 - poleThresholded
        poleThresholded = rect(poleThresholded)
        cv2.imshow("PoleThresholded", poleThresholded)

        poleKeypoints = list(poleDetector.detect(poleThresholded))
        frame = cv2.drawKeypoints(frame, poleKeypoints, np.array([]), (255, 0, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        if (len(poleKeypoints) > 0):
            pole = poleKeypoints[0]
            loc = (int(pole.pt[0]), int(pole.pt[1]))
            frame = cv2.circle(frame, loc, 3, (255, 255, 0), 1)
            dist = getDistance(depth_frame, loc)
            y_loc = find_goal_location(poleThresholded[1:,loc[0] - 20 : loc[0] + 20])
            if y_loc:
                print(y_loc)
                frame = cv2.line(frame, (0, y_loc), (WIDTH, y_loc), (0, 255, 0), 2)
   
            cv2.imshow("frame", frame)
        if ord('q') == cv2.waitKey(1) & 0xFF:
            break
    

if __name__ == "__main__":
    # startTime = time.time()
    # r = Robot()
    # while time.time() - startTime < 10:
    #     oribtRight(r, 35, 0.57)
    #thresh()
    next(main())
    #competition()