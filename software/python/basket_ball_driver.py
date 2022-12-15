from robot import Robot
from image_detection import Camera, EditableThresholder, RealsenseCamera, FileThresholder, Mode
import cv2
import numpy as np
from constants import *
import time
from state import StateHandler
from basket_enum import Basket


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


def get_thrower_speed_y_coord(x, a1 = -2.70683525e+02, a2 = 5.86756139e+01, dx = -4.24677858e-02, c = 1.03808178e+03):
    return a1/(x-dx) + a2/(x-dx)**2 +c


def get_ball_blob_detector_params():
    blob_detector_params = cv2.SimpleBlobDetector_Params()

    blob_detector_params.filterByArea = True
    blob_detector_params.maxArea = 4500
    blob_detector_params.minArea = 75

    blob_detector_params.filterByCircularity = True
    blob_detector_params.maxCircularity = 1
    blob_detector_params.minCircularity = 0.5

    blob_detector_params.filterByConvexity = False
    blob_detector_params.maxConvexity = 0
    blob_detector_params.minConvexity = 0

    blob_detector_params.filterByInertia = False
    blob_detector_params.maxInertiaRatio = 0
    blob_detector_params.minInertiaRatio = 0

    return blob_detector_params


def get_pole_blob_detector_params():
    blob_detector_params = cv2.SimpleBlobDetector_Params()

    blob_detector_params.filterByArea = True
    blob_detector_params.maxArea = 600000
    blob_detector_params.minArea = 1000

    blob_detector_params.filterByCircularity = True
    blob_detector_params.maxCircularity = 1
    blob_detector_params.minCircularity = 0.5

    blob_detector_params.filterByConvexity = False
    blob_detector_params.maxConvexity = 0
    blob_detector_params.minConvexity = 0

    blob_detector_params.filterByInertia = False
    blob_detector_params.maxInertiaRatio = 0
    blob_detector_params.minInertiaRatio = 0

    return blob_detector_params


# returns distance in mm
def get_distance(depth_frame, loc):
    return np.sum(depth_frame[loc[0] - 2 : loc[0] + 3, loc[1] - 2 : loc[1] + 3]) / 25


def map_to_max(s1, s2, s3, max_val):
    cur_max = max(abs(s1), abs(s2), abs(s3))
    return int(s1 * max_val / cur_max), int(s2 * max_val / cur_max), int(s3 * max_val / cur_max)


def orbit_left(robot, max_val=35, a=0.75):
    # speed1 - tagumine
    # speed2 - parem
    # speed3 - vasak
    # -35, 4, 4
    
    # liikumine vasakule
    # NB TODO Vajab testimist ja kalibreerimist, proovi hiljem uuesti
    s = [-1, 0.5, 0.5]
    or_s = [-1, -1, -1]
    
    s1, s2, s3 = [((s[i] * a) + (or_s[i] * (1 - a)))for i in range(len(s))]
    s1, s2, s3 = map_to_max(s1, s2, s3, max_val)
    robot.move(s1, s2, s3)


def orbit_right(robot, max_val=35, a=0.75):
    # NB TODO Vajab testimist ja kalibreerimist, proovi hiljem uuesti
    # 35, -4, -4
    s = [1, -0.5, -0.5]
    or_s = [1, 1, 1]
    
    s1, s2, s3 = [((s[i] * a) + (or_s[i] * (1 - a)))for i in range(len(s))]
    s1, s2, s3 = map_to_max(s1, s2, s3, max_val)
    robot.move(s1, s2, s3)


def get_thrower_speed(distance):
    return int(0.397 * distance + 221)


def delay_camera(dt, cam):
    start_time = time.time()
    while time.time() < start_time + dt:
        frame, depth_frame = cam.get_frames()
    
    lastFrame, depth_frame = cam.get_frames()
    return lastFrame


def go_forward(cap, robot, pole_thresholder, pole_detector):
    start_time = time.time()
    s1 = 0
    s2 = 1
    s3 = -1
    while time.time() - start_time < 0.5:
        frame, depth_frame = cap.get_frames()
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        pole_thresholded = cv2.inRange(frame_hsv, pole_thresholder.get_low(), pole_thresholder.get_high())
        pole_thresholded = 255 - pole_thresholded 
        pole_thresholded = rect(pole_thresholded)
        cv2.imshow("Pole thresholded", pole_thresholded)
        pole_keypoints = list(pole_detector.detect(pole_thresholded))

        if len(pole_keypoints) == 0:
            robot.move(s1, s2, s3)
        else:
            x_loc = pole_keypoints[0].pt[0]
            # otse liikumine
            s1 = 0
            s2 = 1
            s3 = -1

            err = -P_const * (x_loc - HALF_WIDTH)
            s1 += err
            s2 += err
            s3 += err

            s1, s2, s3 = map_to_max(s1, s2, s3, 20)

            # speed1 - tagumine
            # speed2 - parem
            # speed3 - vasak
            robot.move(s1, s2, s3)
    start_time = time.time()
    thr = 0
    while time.time() - start_time < 0.5:
        frame, depth_frame = cap.get_frames()
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        pole_thresholded = cv2.inRange(frame_hsv, pole_thresholder.get_low(), pole_thresholder.get_high())
        pole_thresholded = 255 - pole_thresholded 
        pole_thresholded = rect(pole_thresholded)
        cv2.imshow("Pole thresholded", pole_thresholded)
        pole_keypoints = list(pole_detector.detect(pole_thresholded))

        if len(pole_keypoints) == 0:
            robot.move(0, 20, -20, thr)
        else:
            x_loc = int(pole_keypoints[0].pt[0])
            y_loc = int(pole_keypoints[0].pt[1])
            y_coord = find_goal_location(pole_thresholded[1:,x_loc - 20 : x_loc + 20])
            speed = int(get_thrower_speed_y_coord(y_coord / HEIGHT))
            thr = speed
            robot.move(0, 20, -20, speed)


def main(controller=False, robot = Robot(), basket = Basket.MAGENTA):
    print("Basketball driver starting")
    cap = RealsenseCamera()
    ball_thresholder = EditableThresholder(Mode.HSV, FileThresholder(mode=Mode.HSV), name="Ball")
    ball_detector = cv2.SimpleBlobDetector_create(get_ball_blob_detector_params())
    
    pole_thresholder = EditableThresholder(Mode.HSV, FileThresholder(mode=Mode.HSV, path=f"{basket.value}.json"), name="Pole")
    pole_detector = cv2.SimpleBlobDetector_create(get_pole_blob_detector_params())

    opp_basket = None
    if basket == Basket.MAGENTA:
        opp_basket = Basket.BLUE
    else:
        opp_basket = Basket.MAGENTA
    opp_thresholder = EditableThresholder(Mode.HSV, FileThresholder(mode=Mode.HSV, path=f"{opp_basket.value}.json"), name="OPP pole")
    
    STATE = StateHandler(cap, robot, opp_thresholder, pole_thresholder, ball_thresholder, ball_detector, pole_detector, map_to_max, orbit_left, orbit_right, delay_camera, go_forward)
    while True:
        if controller:
            yield
        
        STATE.state_handler()
        
        if ord('q') == cv2.waitKey(1) & 0xFF:
            break
    ball_thresholder.save()
    pole_thresholder.save()
    yield


def thresh(basket = Basket.MAGENTA):
    cap = RealsenseCamera()
    ball_thresholder = EditableThresholder(Mode.HSV, FileThresholder(mode=Mode.HSV), name="Ball")
    ball_detector = cv2.SimpleBlobDetector_create(get_ball_blob_detector_params())
    
    pole_thresholder = EditableThresholder(Mode.HSV, FileThresholder(mode=Mode.HSV, path=f"{basket.value}.json"), name="Pole")
    pole_detector = cv2.SimpleBlobDetector_create(get_pole_blob_detector_params())
    prev_time = time.time()
    while True:
        frame, depth_frame = cap.get_frames()
        new_time = time.time()
        print(f"FPS: {round(1 / (new_time - prev_time), 2)}")
        prev_time = new_time
        
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        thresholded = cv2.inRange(frame_hsv, ball_thresholder.get_low(), ball_thresholder.get_high())
        thresholded = 255 - thresholded
        # thresholded = rect(thresholded)
        cv2.imshow("Thresholded", thresholded)

        pole_thresholded = cv2.inRange(frame_hsv, pole_thresholder.get_low(), pole_thresholder.get_high())
        pole_thresholded = 255 - pole_thresholded
        pole_thresholded = rect(pole_thresholded)
        cv2.imshow("pole_thresholded", pole_thresholded)

        keypoints = list(ball_detector.detect(thresholded))
        
        pole_keypoints = list(pole_detector.detect(pole_thresholded))
        
        frame = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        frame = cv2.drawKeypoints(frame, pole_keypoints, np.array([]), (255, 0, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        if len(pole_keypoints) > 0:
            frame = cv2.circle(frame, (int(pole_keypoints[0].pt[0]), int(pole_keypoints[0].pt[1])), 3, (255, 255, 0), 1)
            print(get_distance(depth_frame, (int(pole_keypoints[0].pt[0]), int(pole_keypoints[0].pt[1]))))
        
        cv2.imshow("Frame", frame)
        
        if ord('q') == cv2.waitKey(1) & 0xFF:
            break
    pole_thresholder.save()
    ball_thresholder.save()


if __name__ == "__main__":
    # start_time = time.time()
    # r = Robot()
    # while time.time() - start_time < 10:
    #     oribtRight(r, 35, 0.57)
    # thresh()
    next(main(basket="blue"))