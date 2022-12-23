
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
from tkinter import N
from tkinter.messagebox import NO
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import find_pen
#import move_arm 
    
# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
config.enable_record_to_file("color_detection.mp4")

#config.enable_device_from_file("pen_move.mp4")



# Get intrinsics
cfg = pipeline.start(config) # Note in the example code, cfg is misleadingly called "profile" but cfg is a better name
profile = cfg.get_stream(rs.stream.color)
intr = profile.as_video_stream_profile().get_intrinsics()

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = cfg.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
# print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)




# Streaming loop
try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        # Render images:
        #   depth align to color on left
        #   depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))
        
        ################ What I added ################
        # Get depth to pen
        centroid = find_pen.centroid()
        centroid.get_threshold(color_image)
        centroid.find_centroid()
        # if centroid.cntrd[0] != None:
        #     print("Centroid: ", centroid.cntrd)
        centroid.get_depth(depth_image)
        
        # if centroid.depth != None:
        #     depth_pen_to_arm = centroid.depth - 302 #261 # 261 = depth to arm
        
        #     # Get distance from pen to base
        #     # print("Centroid.depth: ", centroid.depth)
        #     centroid.find_y_base()
            
            
        #     end_eff_pos_sleep_cam_frame = [87.73471069335938, 1.7764190435409546, 302.0]#[119.62161254882812, 97.28886413574219, 296.0] #[108.07705688476562, -22.648998260498047, 265.0] 
            
        #     # Centroid in cam frame
        #     pen_pose_camera_frame = rs.rs2_deproject_pixel_to_point(intr, centroid.cntrd, centroid.depth) 
        #     print(pen_pose_camera_frame)
        #     # Centroid in robot frame
            
        #     #x_cent_RF = 
            
        #     x_base_2_pen_robot_frame = abs(end_eff_pos_sleep_cam_frame[0] - pen_pose_camera_frame[0]) + end_eff_pos_sleep_cam_frame[0]
        #     z_base_2_pen_robot_frame = abs(end_eff_pos_sleep_cam_frame[1] - pen_pose_camera_frame[1]) + end_eff_pos_sleep_cam_frame[1]
        #     phi = np.arctan(depth_pen_to_arm/x_base_2_pen_robot_frame)
           
        #     if centroid.x_end_eff_to_base_robot_frame < 0.02 and centroid.y_end_eff_to_base_robot_frame <0.02:
        #         print("IN SLEEP")
        #         #move_arm.turn_to_pen(phi,(x_base_2_pen_robot_frame*depth_scale - centroid.x_end_eff_to_base_robot_frame-95),(z_base_2_pen_robot_frame*depth_scale - centroid.z_end_eff_to_base_robot_frame - 35), depth_scale)
 
            
            
        #     z_base_2_pen_robot_frame = abs(centroid.z_end_eff_to_base_robot_frame - pen_pose_camera_frame[1]) + centroid.z_end_eff_to_base_robot_frame
            
            # if z_base_2_pen_robot_frame*depth_scale < centroid.z_end_eff_to_base_robot_frame:
            #     move_arm.turn_to_pen(phi,(x_base_2_pen_robot_frame*depth_scale - centroid.x_end_eff_to_base_robot_frame),(centroid.z_end_eff_to_base_robot_frame - z_base_2_pen_robot_frame*depth_scale), depth_scale)
            #     if abs(abs(centroid.x_end_eff_to_base_robot_frame)-abs(x_base_2_pen_robot_frame*depth_scale)) < 0.03: # and abs(abs(centroid.z_end_eff_to_base_robot_frame)-abs(z_base_2_pen_robot_frame*depth_scale)) < 0.15:
            #         move_arm.robot.gripper.grasp()
            #         move_arm.stop_seq()               
            # else:
            #     move_arm.turn_to_pen(phi,(x_base_2_pen_robot_frame*depth_scale - centroid.x_end_eff_to_base_robot_frame),(z_base_2_pen_robot_frame*depth_scale - centroid.z_end_eff_to_base_robot_frame), depth_scale)
            #     if abs(abs(centroid.x_end_eff_to_base_robot_frame)-abs(x_base_2_pen_robot_frame*depth_scale)) < 0.03: # and abs(abs(centroid.z_end_eff_to_base_robot_frame)-abs(z_base_2_pen_robot_frame*depth_scale)) < 0.15:
            #         move_arm.robot.gripper.grasp()
            #         move_arm.stop_seq()     
            # print("Test: ", abs(abs(centroid.z_end_eff_to_base_robot_frame)-abs(z_base_2_pen_robot_frame*depth_scale)))

        #else:
            #move_arm.robot.arm.go_to_sleep_pose()
        
        im_w_centroid = cv2.circle(color_image.copy(), centroid.cntrd,7, (0,255,69), -1)
        #rec_im = np.hstack((im_w_centroid, centroid.thresh_im))
        cv2.namedWindow('Pen Detection', cv2.WINDOW_NORMAL)
        cv2.imshow('Pen Detection', im_w_centroid) 
        cv2.imshow('Thresholding', centroid.thresh_im ) 
        key = cv2.waitKey(1)

        
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
        
finally:
    pipeline.stop()

    