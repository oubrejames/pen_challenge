from fileinput import close
from tkinter.tix import Y_REGION
import cv2
import numpy as np
import modern_robotics as mr
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
# The robot object is what you use to control the robot
from move_arm import robot

class centroid:
    def __init__(self):
        self.x = None
        self.y = None
        self.cnts = []
        self.thresh_im = None
        self.cntrd = [None] * 2
        self.depth = None
        self.x_end_eff_to_base_robot_frame = None
        self.y_end_eff_to_base_robot_frame = None
        self.z_end_eff_to_base_robot_frame = None

    def get_threshold(self, img):
        hsvImage = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
        purple_min = np.array([110, 106, 8],np.uint8)
        purple_max = np.array([142, 255, 255],np.uint8)
        hsvblr = cv2.GaussianBlur(hsvImage, (15,15),5)
        frame_threshold = cv2.inRange(hsvblr, purple_min, purple_max)
        self.thresh_im = cv2.morphologyEx(frame_threshold, cv2.MORPH_CLOSE, (15,15))

    def find_centroid(self):
        self.cnts, hierarchy = cv2.findContours(self.thresh_im, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(self.cnts) > 1:
            areas = [cv2.contourArea(c) for c in self.cnts]
            max_index = np.argmax(areas)
            main_body = self.cnts[max_index]

            M = cv2.moments(self.cnts[max_index])
            if M['m00'] > 5:
                self.x = int(M['m10']/M['m00'])    
                self.y = int(M['m01']/M['m00'])
            
            self.cntrd = [self.x, self.y]
            print("Self.centroid ", self.cntrd)

    def get_depth(self, d_im):
        
        if self.cntrd[0] != None:
            if d_im[self.y, self.x] > 0 and d_im[self.y, self.x] < 400:         
                self.depth = d_im[self.y, self.x]
                print("Depth: ", self.depth)

    def find_y_base(self):
        # Get end effector rotation matrix
        joints = robot.arm.get_joint_commands()
        T = mr.FKinSpace(robot.arm.robot_des.M, robot.arm.robot_des.Slist, joints)
        [R, p] = mr.TransToRp(T) # get the rotation matrix and the displacement
        # P = position of end effector
        self.x_end_eff_to_base_robot_frame = p[0]
        self.y_end_eff_to_base_robot_frame = p[1]
        self.z_end_eff_to_base_robot_frame = p[2]
