#!/usr/bin/python3

import rospy
import copy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import *
from moveit_commander import *
# from moveit_commander.conversions import pose_to_list
from yumi_motion_api.srv import *

# TODO : Finish this pipeline, and add parameters (x,y,z), check scale, curr_arm 
def cartesian_move(curr_arm, targetPose, scale=1):
    """
    
    """
    waypoints = []

    waypoints.append(curr_arm.get_current_pose().pose) # current pose
    waypoints.append(targetPose) # desired pose

    wpose = targetPose # The Reach Point
    wpose.position.z -= scale * 0.1  # First move up (z)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, _) = curr_arm.compute_cartesian_path(
        waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    # return plan, fraction
    curr_arm.execute(plan, wait=True)

def pick_place(req):
    # CB
    cartesian_move(req.arm, req.)
    return YuMiPickPlaceCmdResponse(True)

def pick_place_server():
    s = rospy.Service('pick_place', YuMiPickPlaceCmd, pick_place)
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('yumi_pick_place_server')
        rospy.loginfo('=== Yumi_pick_place node Initialized! ===')
        pick_place_server()

    except rospy.ROSInterruptException:
        rospy.logerr("yumi_pick_plcae node has died!")