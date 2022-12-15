import cv2
import numpy as np
from enum import Enum


class ThresholderTypes(Enum):
    OPPONENT_POLE = 0
    OUR_POLE = 1
    BALL = 2


class ImgProcessor:
    def __init__(self, cap, opp_thresholder, pole_thresholder, ball_thresholder, ball_detector, pole_detector):
        self.cap = cap
        self.opp_thresholder = opp_thresholder
        self.pole_thresholder = pole_thresholder
        self.ball_thresholder = ball_thresholder
        self.ball_detector = ball_detector
        self.pole_detector = pole_detector


    def read_frame(self):
        frame, depth_frame = self.cap.get_frames()
        self.frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.depth_frame = depth_frame
    

    def create_thres_img_for_blob_detection(self, thresholder):
        thresholded = cv2.inRange(self.frame_hsv, thresholder.get_low(), thresholder.get_high())
        thresholded = 255 - thresholded
        sh = thresholded.shape
        return cv2.rectangle(thresholded, (0, 0), (sh[1], sh[0]), (255), 1)


    def get_keypoints(self, thresholder, detector, color):
        thresholded = self.create_thres_img_for_blob_detection(thresholder)
        cv2.imshow("Thresholded", thresholded)

        keypoints = list(detector.detect(thresholded))
        keypoints.sort(key=lambda x: -x.size)  #  Biggest ball as first

        self.frame_hsv = cv2.drawKeypoints(self.frame_hsv, keypoints, np.array([]), color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        return keypoints


    def get_keypoints_by_type(self, thres_type):
        if (thres_type == ThresholderTypes.BALL):
            return self.get_keypoints(self.ball_thresholder, self.ball_detector, (0, 255, 0))
        elif thres_type == ThresholderTypes.OPPONENT_POLE:
            return self.get_keypoints(self.opp_thresholder, self.pole_detector, (0, 0, 255))
        elif thres_type == ThresholderTypes.OUR_POLE:
            return self.get_keypoints(self.pole_thresholder, self.pole_detector, (255, 0, 0))
        else:
            raise ValueError("Shouldn't happen")

