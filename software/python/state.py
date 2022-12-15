from enum import Enum
import cv2
import time
from constants import *
import numpy as np


class State(Enum):
    OPP_BASKET = 0
    BALL = 1
    THROW_GOAL = 2


class StateHandler:
    def __init__(self, cap, robot, opp_thresholder, pole_thresholder, ball_thresholder, ball_detector, pole_detector, map_to_max, orbit_left, orbit_right, delay_camera, go_forward):
        self.cap = cap
        self.robot = robot
        self.opp_thresholder = opp_thresholder
        self.pole_thresholder = pole_thresholder
        self.ball_thresholder = ball_thresholder
        self.ball_detector = ball_detector
        self.pole_detector = pole_detector
        self.set_state(State.BALL)
        self.state_start_time = time.time()
        self.map_to_max = map_to_max
        self.orbit_left = orbit_left
        self.orbit_right = orbit_right
        self.delay_camera = delay_camera
        self.go_forward = go_forward
    

    def img_processing(self):
        frame, depth_frame = self.cap.get_frames()
        self.frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.depth_frame = depth_frame
    

    def state_handler(self):
        self.img_processing()
        if (self.get_state() == State.BALL):
            self.state_ball()
        elif (self.get_state() == State.OPP_BASKET):
            self.state_go_to_opponent_basket()
        elif (self.get_state() == State.THROW_GOAL):
            self.state_throw_to_goal()
        else:
            raise Exception("Unexpected state")
        
        cv2.imshow("Aken", self.frame_hsv)


    def get_state(self):
        return self.state

    
    def set_state(self, state):
        self.state = state
        self.state_start_time = time.time()


    def state_go_to_opponent_basket(self):
        if time.time() - self.state_start_time > STATE_OVERFLOW_TIME:
            self.set_state(State.BALL)
        thresholded = self.create_thres_img_for_blob_detection(self.opp_thresholder)
        cv2.imshow("Thresholded", thresholded)

        kps = list(self.pole_detector.detect(thresholded))

        self.frame_hsv = cv2.drawKeypoints(self.frame_hsv, kps, np.array([]), (0, 255, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        if len(kps) == 0:
            self.robot.set_speed(8 / 32767)
            self.robot.spin_left()
        else:
            self.robot.forward()
    

    def state_ball(self):
        if time.time() - self.state_start_time > STATE_OVERFLOW_TIME:
            self.set_state(State.OPP_BASKET)
            self.state_start_time = time.time()
        thresholded = self.create_thres_img_for_blob_detection(self.ball_thresholder)
        cv2.imshow("Thresholded", thresholded)

        keypoints = list(self.ball_detector.detect(thresholded))
        keypoints.sort(key=lambda x: -x.size)  #  Biggest ball as first

        self.frame_hsv = cv2.drawKeypoints(self.frame_hsv, keypoints, np.array([]), (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        if len(keypoints) == 0:
            self.robot.set_speed(8 / 32767)
            self.robot.spin_left()
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

                s1, s2, s3 = self.map_to_max(s1, s2, s3, 20)

                # speed1 - tagumine
                # speed2 - parem
                # speed3 - vasak
                self.robot.move(s1, s2, s3)
            elif kp.size < 60:
                if x_loc < (HALF_WIDTH - 15):
                    self.robot.spin_left()
                elif x_loc >  (HALF_WIDTH + 15):
                    self.robot.spin_right()
                else:
                    start_time = time.time()
                    while time.time() - start_time < 1:
                        self.robot.move(0, 20, -20)
                    self.set_state(State.THROW_GOAL)
            else:
                self.robot.backward()


    def state_throw_to_goal(self):
        if time.time() - self.state_start_time > 5:
            self.set_state(State.BALL)
        thresholded = self.create_thres_img_for_blob_detection(self.pole_thresholder)
        cv2.imshow("Pole thresholded", thresholded)
        pole_keypoints = list(self.pole_detector.detect(thresholded))
        self.frame_hsv = cv2.drawKeypoints(self.frame_hsv, pole_keypoints, np.array([]), (255, 0, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        if len(pole_keypoints) == 0:
            self.orbit_left(self.robot, 15, 0.57)
        else:
            pole_kp = pole_keypoints[0]
            x_loc = pole_kp.pt[0]
            loc = (int(x_loc), int(pole_kp.pt[1]))
            if x_loc < HALF_WIDTH - 8:
                self.orbit_right(self.robot, 8, 0.57)
            elif x_loc > HALF_WIDTH + 8:
                self.orbit_left(self.robot, 8, 0.57)
            else:
                self.robot.move(0, -20, 20, disable_failsafe=1)
                self.delay_camera(0.25, self.cap)
                self.go_forward(self.cap, self.robot, self.pole_thresholder, self.pole_detector)
                self.set_state(State.BALL)


    def create_thres_img_for_blob_detection(self, thresholder):
        thresholded = cv2.inRange(self.frame_hsv, thresholder.get_low(), thresholder.get_high())
        thresholded = 255 - thresholded
        sh = thresholded.shape
        return cv2.rectangle(thresholded, (0, 0), (sh[1], sh[0]), (255), 1)