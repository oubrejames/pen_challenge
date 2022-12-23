from tkinter import Scale
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import modern_robotics as mr

# The robot object is what you use to control the robot
robot = InterbotixManipulatorXS("px100", "arm", "gripper")

def turn_to_pen(phi,x,z, scale):
    if x < 0:
        robot.arm.set_ee_cartesian_trajectory(x=(x),wp_accel_time=0.05)

        if phi > -3.14 and phi < 3.14:
            robot.arm.set_single_joint_position("waist", -phi)
    
        robot.arm.set_ee_cartesian_trajectory(z=(z),wp_accel_time=0.05)
    elif phi > -3.14 and phi < 3.14:
            robot.arm.set_single_joint_position("waist", -phi)
        
            robot.arm.set_ee_cartesian_trajectory(z=(z),wp_accel_time=0.05)

            robot.arm.set_ee_cartesian_trajectory(x=(x),wp_accel_time=0.05)

    #If phi equals phi, (if you are at phi)
    
def stop_seq():
    robot.arm.go_to_sleep_pose()
    robot.gripper.release(1)

