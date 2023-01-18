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
    THROWING = 3
    NO_BALL_THROWING = 4
    HACK_BEGIN = 5


class StateHandler:
    def __init__(self, cap, robot, opp_thresholder, pole_thresholder, ball_thresholder, ball_detector, pole_detector, map_to_max, orbit_left, orbit_right):
        self.robot = robot
        self.img_processor = ImgProcessor(cap, opp_thresholder, pole_thresholder, ball_thresholder, ball_detector, pole_detector)
        self.set_state(State.HACK_BEGIN)
        self.map_to_max = map_to_max
        self.orbit_left = orbit_left
        self.orbit_right = orbit_right
        self.prev_ball_loc = None
        self.helper = False
    

    def state_handler(self):
        self.img_processor.read_frame()
        print(self.state)
        if (self.get_state() == State.BALL):
            self.state_ball()
        elif (self.get_state() == State.OPP_BASKET):
            self.state_go_to_opponent_basket()
        elif (self.get_state() == State.THROW_GOAL):
            self.state_throw_to_goal()
        elif (self.get_state() == State.THROWING):
            self.state_throw()
        elif (self.get_state() == State.NO_BALL_THROWING):
            self.state_no_ball_throwing()
        elif (self.get_state() == State.HACK_BEGIN):
            self.state_hack_begin()
        else:
            raise Exception("Unexpected state")
        
        cv2.imshow("Aken", self.img_processor.frame_hsv)
        print(self.img_processor.frame_hsv.shape)


    def get_state(self):
        return self.state

    
    def set_state(self, state):
        self.state = state
        self.state_start_time = time.time()


    def get_thrower_speed(self, distance):
        return 2047 # int(3.64*distance - 3927)


    def state_hack_begin(self):
        kps_other = self.img_processor.get_keypoints_by_type(ThresholderTypes.OPPONENT_POLE)
        if (len(kps_other) > 0):
            kp = kps_other[0]
            if (kp.pt[0] < HALF_WIDTH - 10):
                self.robot.move_omni(0, 0, 3)
            elif (kp.pt[0] > HALF_WIDTH + 10):
                self.robot.move_omni(0, 0, -3)
            else:
                for i in range(100):
                    self.robot.move_omni(-np.pi * 0.71, 10 * i, 0.02)
                    time.sleep(0.02)
                self.robot.move_omni(-np.pi * 0.71, 1000, 0.02, disable_failsafe=1)
                time.sleep(3.9)
                self.robot.stop()
                self.set_state(State.BALL)
        else:
            self.robot.move_omni(0, 0, 3)



    def state_no_ball_throwing(self):
        if (self.state_start_time + 2 > time.time()):
            kps_our = self.img_processor.get_keypoints_by_type(ThresholderTypes.OUR_POLE)
            if len(kps_our) > 0:
                pole = kps_our[0]
                speed = self.get_thrower_speed(self.img_processor.calc_dist(pole.pt))
                #print(f"Distance: {self.img_processor.calc_dist(pole.pt)}, speed: {speed}")
                self.robot.move_omni(0, 1000, 0, speed)
        else:
            self.set_state(State.BALL)

    
    def state_throw_sub(self, kps_ball, kps_our):
        #self.robot.stop()
        #return
        ball = kps_ball[0]
        pole = kps_our[0]
        pole_x = pole.pt[0]
        ball_x = ball.pt[0]

        ball_Const = -0.003
        pole_Const = -0.008

        ball_error = (ball_x - HALF_WIDTH) * ball_Const
        pole_error = (pole_x - HALF_WIDTH - 20) * pole_Const

        speed = self.get_thrower_speed(self.img_processor.calc_dist(pole.pt))

        self.robot.move_omni(ball_error, 750, pole_error, speed)
        print("tere")


    def dist(self, loc1, loc2):
        return (((loc1[0] - loc2[0]) ** 2) + ((loc1[1] - loc2[1]) ** 2)) ** 0.5


    def state_throw(self):
        if time.time() - self.state_start_time > STATE_OVERFLOW_TIME:
            self.set_state(State.BALL)
        # self.img_processor.frame_hsv = self.img_processor.frame_hsv[200:]
        kps_ball = self.img_processor.get_keypoints_by_type(ThresholderTypes.BALL, True)
        kps_our = self.img_processor.get_keypoints_by_type(ThresholderTypes.OUR_POLE)
        print(len(kps_ball), len(kps_our))
        if len(kps_ball) == 0 or len(kps_our) == 0:
            self.set_state(State.NO_BALL_THROWING)
        else:
            self.state_throw_sub(kps_ball, kps_our)

            


    def state_go_to_opponent_basket(self):
        if time.time() - self.state_start_time > STATE_OVERFLOW_TIME:
            self.set_state(State.BALL)

        kps_opponent = self.img_processor.get_keypoints_by_type(ThresholderTypes.OPPONENT_POLE)
        kps_ball = self.img_processor.get_keypoints_by_type(ThresholderTypes.BALL)

        if len(kps_ball) > 0:
            
            self.set_state(State.BALL)
        elif len(kps_opponent) == 0:
            self.robot.move_omni(0, 0, 5)
        else:
            self.robot.move_omni(0, 5, 0)
    

    def state_ball(self):
        if time.time() - self.state_start_time > STATE_OVERFLOW_TIME:
            self.set_state(State.OPP_BASKET)
        
        keypoints = self.img_processor.get_keypoints_by_type(ThresholderTypes.BALL)

        if len(keypoints) == 0:
            self.robot.move_omni(0, 0, 4)
        else:
            kp = keypoints[0]
            x_loc = kp.pt[0]
            if kp.size < 45:
                err = (x_loc - HALF_WIDTH) * P_const * 2
                self.robot.move_omni(0, 1400, -err)
            else:
                start_time = time.time()
                self.robot.stop()
                self.set_state(State.THROW_GOAL)


    def state_throw_to_goal(self):
        if time.time() - self.state_start_time > STATE_OVERFLOW_TIME:
            self.set_state(State.BALL)
        
        pole_keypoints = self.img_processor.get_keypoints_by_type(ThresholderTypes.OUR_POLE)

        CONST = 1

        if len(pole_keypoints) == 0:
            self.robot.move_omni(-np.pi / 2, 300, CONST)
        else:
            pole_kp = pole_keypoints[0]
            x_loc = pole_kp.pt[0]
            loc = (int(x_loc), int(pole_kp.pt[1]))
            if x_loc < HALF_WIDTH - 8:
                self.robot.move_omni(-np.pi / 2, 250, CONST)
            elif x_loc > HALF_WIDTH + 8:
                self.robot.move_omni(np.pi / 2, 250, -CONST)
            else:
                self.set_state(State.THROWING)