#!/usr/bin/env python3

import rospy
from yumi_motion_api.srv import *

# #=========== Joint =============
from sensor_msgs.msg import JointState
rospy.wait_for_service('/move_joints')

move_joints = rospy.ServiceProxy("/move_joints", YuMiJointsCmd)

jointsL = JointState()
jointsL.position = [0.0, -2.2689, 2.3562, 0.5236, 0.0, 0.6981, 0.0]
jointsL.name = ['yumi_robl_joint_1', 'yumi_robl_joint_2', 'yumi_robl_joint_3', 'yumi_robl_joint_4',
        'yumi_robl_joint_5', 'yumi_robl_joint_6', 'yumi_robl_joint_7']

jointsR = JointState()
jointsR.position = [0.0, -2.2689,-2.3562, 0.5236, 0.0, 0.6981, 0.0]
jointsR.name = ['yumi_robr_joint_1', 'yumi_robr_joint_2', 'yumi_robr_joint_3', 'yumi_robr_joint_4',
        'yumi_robr_joint_5', 'yumi_robr_joint_6', 'yumi_robr_joint_7']

jointsB = JointState()
jointsB.position = jointsL.position + jointsR.position
# jointsB.position = [-0.3904, -1.863, 1.6118, -0.4216, -0.2341, 1.4111, -0.6769] + [0.1534, -1.9199, -1.8037, 0.0169, 0.2081, 1.2114, 0.4893]
jointsB.name = jointsL.name + jointsR.name 

# #=========== Pick_Place =============
# rospy.wait_for_service('/pick_place')
# pick_place = rospy.ServiceProxy("/pick_place", YuMiPickPlace)

pickL = JointState()
pickL.name = jointsL.name
pickL.position = [-1.3798, -0.3942, 2.1109, 0.3581, 1.9072, 0.3485, 0.3353]  # [-1.3861, -0.4906, 1.9939, 0.2006, -1.4505, -0.3759, 0.6383]

placeL = JointState()
placeL.name = jointsL.name
placeL.position = [-0.5016, -0.7581, 1.7548, 0.8245, -1.8507, 1.1507, 0.4268] # [-1.8872, -0.4387, 2.3964, -0.1373, -1.9644, -0.4261, 1.0246]

placeR = JointState()
placeR.name = jointsR.name
placeR.position =[2.0411, -2.0423, -0.7361, 0.4753, -2.9977, 2.0556, 2.6983]

# pick_place(pickL, placeL, 1)
# move_joints(pickL, 1)

# #=========== Pose =============
from moveit_commander.conversions import list_to_pose
from geometry_msgs.msg import Pose
move_pose = rospy.ServiceProxy("/move_pose", YuMiPoseCmd)

pose_L = Pose()
# # (x,y,z,r,p,y) or (x,y,z,qx,qy,qz,qw)
pose_L = list_to_pose([-0.0069473880000000005, 0.18092670000000002, 0.1978614,
0.06901466, 0.844258, -0.09030558, 0.5237464]) # RWS

# pose_L = list_to_pose([0.0666091247545823, 0.16982066475831367, 0.26272234453225224,
#   0.8402183848537118, -0.10762470399490534, 0.5279347173994628, 0.06111401646249685]) # ROS

pose_R = Pose()
# # (x,y,z,r,p,y) or (x,y,z,qx,qy,qz,qw)
# pose_R = list_to_pose([-0.005278209000000001, -0.1740314, 0.1975962,
# 0.0601522, -0.8363719, -0.1202634, -0.5314136]) # RWS

pose_R = list_to_pose([ 0.06521628659600023, -0.15157665169207687, 0.2614716442279221,
-0.8377765973295542, -0.11183994779767356, -0.5302164767824513, 0.06702750772579635]) # ROS

# move_pose(pose_L, 1)
# move_pose(pose_R, 2)

# # xxxxxxxxxxxxxxxxxxxxxxxxxx

# move_joints(jointsL, 1)
# move_joints(jointsR, 2)
move_joints(jointsB, 3)

# pick_place(pickL, placeL, 1)
# 
# move_joints(placeL, 1)
# move_joints(placeR, 2)

# move_pose(pose_L, 1)
# move_pose(pose_R, 2)