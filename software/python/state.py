from enum import Enum
import cv2
import time
from constants import *
import numpy as np

from img_processor import ImgProcessor, ThresholderTypes


class State(Enum):
    OPP_BASKET = 0
    BALL = 1
    THROW_GOAL = 2


class StateHandler:
    def __init__(self, cap, robot, opp_thresholder, pole_thresholder, ball_thresholder, ball_detector, pole_detector, map_to_max, orbit_left, orbit_right, go_forward):
        self.robot = robot
        self.img_processor = ImgProcessor(cap, opp_thresholder, pole_thresholder, ball_thresholder, ball_detector, pole_detector)
        self.set_state(State.BALL)
        self.state_start_time = time.time()
        self.map_to_max = map_to_max
        self.orbit_left = orbit_left
        self.orbit_right = orbit_right
        self.go_forward = go_forward
    

    def state_handler(self):
        self.img_processor.read_frame()
        if (self.get_state() == State.BALL):
            self.state_ball()
        elif (self.get_state() == State.OPP_BASKET):
            self.state_go_to_opponent_basket()
        elif (self.get_state() == State.THROW_GOAL):
            self.state_throw_to_goal()
        else:
            raise Exception("Unexpected state")
        
        cv2.imshow("Aken", self.img_processor.frame_hsv)


    def get_state(self):
        return self.state

    
    def set_state(self, state):
        self.state = state
        self.state_start_time = time.time()


    def state_go_to_opponent_basket(self):
        if time.time() - self.state_start_time > STATE_OVERFLOW_TIME:
            self.set_state(State.BALL)
            self.state_start_time = time.time()
        
        kps_opponent = self.img_processor.get_keypoints_by_type(ThresholderTypes.OPPONENT)
        kps_ball = self.img_processor.get_keypoints_by_type(ThresholderTypes.BALL)

        if len(kps_ball) > 0:
            self.set_state(State.BALL)
            self.state_start_time = time.time()
        elif len(kps_opponent) == 0:
            self.robot.spin_left()
        else:
            self.robot.forward()
    

    def state_ball(self):
        if time.time() - self.state_start_time > STATE_OVERFLOW_TIME:
            self.set_state(State.OPP_BASKET)
            self.state_start_time = time.time()
        
        keypoints = self.img_processor.get_keypoints_by_type(ThresholderTypes.BALL)

        if len(keypoints) == 0:
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
                    self.state_start_time = time.time()
            else:
                self.robot.move_backward()


    def state_throw_to_goal(self):
        if time.time() - self.state_start_time > 5:
            self.set_state(State.BALL)
            self.state_start_time = time.time()
        
        pole_keypoints = self.img_processor.get_keypoints_by_type(ThresholderTypes.OUR_POLE)

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
                time.sleep(0.25)
                self.go_forward(self.img_processor.cap, self.robot, self.img_processor.pole_thresholder, self.img_processor.pole_detector)
                self.set_state(State.BALL)
                self.state_start_time = time.time()