from robot import Robot
from imageDetection import Camera, EditableThresholder, RealsenseCamera, FileThresholder
import cv2
import numpy as np
from constants import *
import time
from state import StateHandler


def find_goal_location(thresd, top_width_threshold = 10, bottom_width_threshold = 5, kas_plot_y = False, kas_plot_x = False):
    if np.sum(255 - thresd[0]) > 255 * top_width_threshold:
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
    blobDetectorParams.maxArea = 600000
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


# returns distance in mm
def getDistance(depthFrame, loc):
    dist = 0
    for i in range(-2, 3):
        for j in range(-2, 3):
            dist += depthFrame.get_distance(loc[0] + i, loc[1] + j) * 1000
    return dist / 25


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


def orbitRight(robot, maxVal=35, a=0.75):
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


def goForward(cap, robot, poleThresholder, poleDetector):
    startTime = time.time()
    s1 = 0
    s2 = 1
    s3 = -1
    while time.time() - startTime < 0.5:
        frame, depth_frame = cap.get_frames()
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        poleThresholded = cv2.inRange(frameHSV, poleThresholder.getLow(), poleThresholder.getHigh())
        poleThresholded = 255 - poleThresholded 
        poleThresholded = rect(poleThresholded)
        cv2.imshow("Pole thresholded", poleThresholded)
        poleKeypoints = list(poleDetector.detect(poleThresholded))

        if len(poleKeypoints) == 0:
            robot.move(s1, s2, s3)
        else:
            x_loc = poleKeypoints[0].pt[0]
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
    startTime = time.time()
    thr = 0
    while time.time() - startTime < 0.5:
        frame, depth_frame = cap.get_frames()
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        poleThresholded = cv2.inRange(frameHSV, poleThresholder.getLow(), poleThresholder.getHigh())
        poleThresholded = 255 - poleThresholded 
        poleThresholded = rect(poleThresholded)
        cv2.imshow("Pole thresholded", poleThresholded)
        poleKeypoints = list(poleDetector.detect(poleThresholded))

        if len(poleKeypoints) == 0:
            robot.move(0, 20, -20, thr)
        else:
            x_loc = int(poleKeypoints[0].pt[0])
            y_loc = int(poleKeypoints[0].pt[1])
            y_coord = find_goal_location(poleThresholded[1:,x_loc - 20 : x_loc + 20])
            speed = int(getThrowerSpeedYCoord(y_coord / HEIGHT))
            thr = speed
            robot.move(0, 20, -20, speed)


def main(controller=False, robot = Robot(), basket = "magenta"):
    print("Basketball driver starting")
    cap = RealsenseCamera()
    ballThresholder = EditableThresholder("hsv", FileThresholder(mode="hsv"), name="Ball")
    ballDetector = cv2.SimpleBlobDetector_create(getBallBlobDetectorParams())
    
    poleThresholder = EditableThresholder("hsv", FileThresholder(mode="hsv", path=f"{basket}.json"), name="Pole")
    poleDetector = cv2.SimpleBlobDetector_create(getPoleBlobDetectorParams())
    prevTime = time.time()

    oppBasket = None
    if basket == 'magenta':
        oppBasket = 'blue'
    else:
        oppBasket = 'magenta'
    oppThresholder = EditableThresholder("hsv", FileThresholder(mode="hsv", path=f"{oppBasket}.json"), name="OPP pole")
    
    STATE = StateHandler(cap, robot, oppThresholder, poleThresholder, ballThresholder, ballDetector, poleDetector, mapToMax, orbitLeft, orbitRight, delayCamera, goForward)
    while True:
        if controller:
            yield
        
        STATE.state_handler()
        
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
            print(dist)
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
    # thresh()
    next(main(basket="blue"))
    #competition()