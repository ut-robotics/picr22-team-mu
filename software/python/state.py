from enum import Enum
import cv2
import time
from constants import *
import numpy as np


class State(Enum):
    OPP_BASKET = 0
    BALL = 1
    FINAL = 2


class StateHandler:
    def __init__(self, cap, robot, oppThresholder, poleThresholder, ballThresholder, ballDetector, poleDetector, mapToMax, orbitLeft, orbitRight, delayCamera, goForward):
        self.cap = cap
        self.robot = robot
        self.oppThresholder = oppThresholder
        self.poleThresholder = poleThresholder
        self.ballThresholder = ballThresholder
        self.ballDetector = ballDetector
        self.poleDetector = poleDetector
        self.setState(State.BALL)
        self.stateStartTime = time.time()
        self.mapToMax = mapToMax
        self.orbitLeft = orbitLeft
        self.orbitRight = orbitRight
        self.delayCamera = delayCamera
        self.goForward = goForward
    

    def imgProcessing(self):
        frame, depth_frame = self.cap.get_frames()
        self.frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.depth_frame = depth_frame
    

    def state_handler(self):
        self.imgProcessing()
        if (self.getState() == State.BALL):
            self.state_ball()
        elif (self.getState() == State.OPP_BASKET):
            self.state_opp_basket()
        elif (self.getState() == State.FINAL):
            self.state_final()
        else:
            raise Exception("Unexpected state")
        
        cv2.imshow("Aken", self.frameHSV)


    def getState(self):
        return self.state

    
    def setState(self, state):
        self.state = state
        self.stateStartTime = time.time()


    def state_opp_basket(self):
        if time.time() - self.stateStartTime > STATE_OVERFLOW_TIME:
            self.setState(State.BALL)
        thresholded = self.createThresImgForBlobDetection(self.oppThresholder)
        cv2.imshow("Thresholded", thresholded)

        kps = list(self.poleDetector.detect(thresholded))

        self.frameHSV = cv2.drawKeypoints(self.frameHSV, kps, np.array([]), (0, 255, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        if len(kps) == 0:
            self.robot.setSpeed(8 / 32767)
            self.robot.spinLeft()
        else:
            self.robot.forward()
    

    def state_ball(self):
        if time.time() - self.stateStartTime > STATE_OVERFLOW_TIME:
            self.setState(State.OPP_BASKET)
            self.stateStartTime = time.time()
        thresholded = self.createThresImgForBlobDetection(self.ballThresholder)
        cv2.imshow("Thresholded", thresholded)

        keypoints = list(self.ballDetector.detect(thresholded))
        keypoints.sort(key=lambda x: -x.size)  #  Biggest ball as first

        self.frameHSV = cv2.drawKeypoints(self.frameHSV, keypoints, np.array([]), (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        if len(keypoints) == 0:
            self.robot.setSpeed(8 / 32767)
            self.robot.spinLeft()
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

                s1, s2, s3 = self.mapToMax(s1, s2, s3, 20)

                # speed1 - tagumine
                # speed2 - parem
                # speed3 - vasak
                self.robot.move(s1, s2, s3)
            elif kp.size < 60:
                if x_loc < (HALF_WIDTH - 15):
                    self.robot.spinLeft()
                elif x_loc >  (HALF_WIDTH + 15):
                    self.robot.spinRight()
                else:
                    startTime = time.time()
                    while time.time() - startTime < 1:
                        self.robot.move(0, 20, -20)
                    self.setState(State.FINAL)
            else:
                self.robot.backward()


    def state_final(self):
        if time.time() - self.stateStartTime > 5:
            self.setState(State.BALL)
        thresholded = self.createThresImgForBlobDetection(self.poleThresholder)
        cv2.imshow("Pole thresholded", thresholded)
        poleKeypoints = list(self.poleDetector.detect(thresholded))
        self.frameHSV = cv2.drawKeypoints(self.frameHSV, poleKeypoints, np.array([]), (255, 0, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        if len(poleKeypoints) == 0:
            self.orbitLeft(self.robot, 15, 0.57)
        else:
            pole_kp = poleKeypoints[0]
            x_loc = pole_kp.pt[0]
            loc = (int(x_loc), int(pole_kp.pt[1]))
            if x_loc < HALF_WIDTH - 8:
                self.orbitRight(self.robot, 8, 0.57)
            elif x_loc > HALF_WIDTH + 8:
                self.orbitLeft(self.robot, 8, 0.57)
            else:
                self.robot.move(0, -20, 20, disableFailsafe=1)
                self.delayCamera(0.25, cap)
                self.goForward(self.cap, self.robot, self.poleThresholder, self.poleDetector)
                self.setState(State.BALL)


    def createThresImgForBlobDetection(self, thresholder):
        thresholded = cv2.inRange(self.frameHSV, thresholder.getLow(), thresholder.getHigh())
        thresholded = 255 - thresholded
        sh = thresholded.shape
        return cv2.rectangle(thresholded, (0, 0), (sh[1], sh[0]), (255), 1)