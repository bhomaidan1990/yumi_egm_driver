#!/usr/bin/env python3

import sys
from math import pi as PI
import time
from time import sleep
from math import degrees
import rospy
import tf
# from std_msgs.msg import Float64
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from moveit_msgs.msg import *
from moveit_commander import *
# from rospy_message_converter import message_converter
from abb_rapid_sm_addin_msgs.srv import *
from abb_robot_msgs.srv import *
from abb_robot_msgs.msg import *
from controller_manager_msgs.srv import *

global group_l  # :The move group for the left arm
global group_r  # :The move group for the right arm
global group_both  # :The move group for using both arms at once
global robot  # :The RobotCommander() from MoveIt!
global scene  # :The PlanningSceneInterface from MoveIt!
global system_info 

LEFT  = 1  # :ID of the left arm
RIGHT = 2  # :ID of the right arm
BOTH  = 3  # :ID of both_arms

def changeEGMSettings():
    """
    
    """
    rospy.wait_for_service('/yumi/rws/sm_addin/get_egm_settings')
    rospy.wait_for_service('/yumi/rws/sm_addin/set_egm_settings')
    rospy.wait_for_service("/yumi/rws/sm_addin/stop_egm")

    stop_egm = rospy.ServiceProxy("/yumi/rws/sm_addin/stop_egm", TriggerWithResultCode)

    get_egm_settings = rospy.ServiceProxy("/yumi/rws/sm_addin/get_egm_settings", GetEGMSettings)
    set_egm_settings = rospy.ServiceProxy("/yumi/rws/sm_addin/set_egm_settings", SetEGMSettings)

    current_settings_L = get_egm_settings(task='T_ROB_L')
    current_settings_R = get_egm_settings(task='T_ROB_R')

    settings_L = current_settings_L.settings
    settings_R = current_settings_R.settings

    # print ("********** EGM Settings Before **********\n\n", settings_L, '\n\n', settings_R)

    # max_speed_deviation is in deg/s, we convert from rad/s
    settings_L.activate.max_speed_deviation = degrees(7.0)
    settings_R.activate.max_speed_deviation = degrees(7.0)

    # settings.activate.cond_min_max

    # settings.run.cond_time
    settings_L.run.cond_time = 120.0
    settings_R.run.cond_time = 120.0
    # posCorrgain = 0.0
    settings_L.run.pos_corr_gain = 0.0
    settings_R.run.pos_corr_gain = 0.0
    
    stop_egm()
    
    # print ("********** EGM Settings After **********\n\n", settings_L, '\n\n', settings_R)

    taskname = "T_ROB_L"
    set_egm_settings(task=taskname, settings=settings_L)

    taskname = "T_ROB_R"
    set_egm_settings(task=taskname, settings=settings_R)

    print("===================== EGM Settings Updated =====================")

def calibrateGrippers():
    """
    
    """
    rospy.wait_for_service('/yumi/rws/sm_addin/set_sg_command')
    rospy.wait_for_service('/yumi/rws/sm_addin/run_sg_routine')

    set_sg_command = rospy.ServiceProxy("/yumi/rws/sm_addin/set_sg_command", SetSGCommand)
    run_sg_routine = rospy.ServiceProxy("/yumi/rws/sm_addin/run_sg_routine", TriggerWithResultCode)

    """
    uint8 SG_COMMAND_INITIALIZE   = 3
    uint8 SG_COMMAND_CALIBRATE    = 4
    uint8 SG_COMMAND_MOVE_TO      = 5
    uint8 SG_COMMAND_GRIP_IN      = 6
    uint8 SG_COMMAND_GRIP_OUT     = 7
    """
    task_L = 'T_ROB_L'
    task_R = 'T_ROB_R'
    fin_pos = 0.0
    # # Close Grippers
    cmd = 6
    set_sg_command(task=task_L, command=cmd, target_position=fin_pos)
    set_sg_command(task=task_R, command=cmd, target_position=fin_pos)
    run_sg_routine()
    sleep(1)
    # # Calibrate Grippers
    cmd = 4
    set_sg_command(task=task_L, command=cmd, target_position=fin_pos)
    set_sg_command(task=task_R, command=cmd, target_position=fin_pos)
    run_sg_routine()
    sleep(2)
    # # Open Grippers
    # cmd = 7
    # fin_pos = 0.0
    # set_sg_command(task=task_L, command=cmd, target_position=fin_pos)
    # set_sg_command(task=task_R, command=cmd, target_position=fin_pos)
    # run_sg_routine()
    print('===================== Grippers: Calibrated =====================')

def sys_init():
    """
    Initialize the system, auto_mode, motors_on, rapid_running, egm_settings_adjusted
    """
    # system_info = rospy.wait_for_message("/yumi/rws/system_states", SystemState, timeout=5)
    # TODO: check this
    auto_mode = True # system_info.auto_mode # 
    # rapid_running = system_info.rapid_running
    motors_on = False # system_info.motors_on #  
    # print(auto_mode, motors_on)

    if(not auto_mode):
        print("xxxxxxxxxxxxx Robot in Manual Mode! Can't Initialize! xxxxxxxxxxxxx")
        sys.exit()
    else:
        # Reset Program Pointer to main and Start RAPID
        rospy.wait_for_service('/yumi/rws/pp_to_main')
        rospy.wait_for_service('/yumi/rws/stop_rapid')
        rospy.wait_for_service('/yumi/rws/start_rapid')
        # set_motors_off= rospy.ServiceProxy("/yumi/rws/set_motors_off",TriggerWithResultCode)
        stop_rapid    = rospy.ServiceProxy("/yumi/rws/stop_rapid"   , TriggerWithResultCode)
        pp_to_main    = rospy.ServiceProxy("/yumi/rws/pp_to_main"   , TriggerWithResultCode)
        start_rapid   = rospy.ServiceProxy("/yumi/rws/start_rapid"  , TriggerWithResultCode)
        # Call the functionalities
        stop_rapid()
        pp_to_main()
        time.sleep(1)
        start_rapid()
        # Turn Motors On
        if(not motors_on):
            rospy.wait_for_service('/yumi/rws/set_motors_on')
            set_motors_on = rospy.ServiceProxy("/yumi/rws/set_motors_on", TriggerWithResultCode)
            set_motors_on()

        # Calibrate Grippers
        calibrateGrippers()

        print('===================== YuMi Initialized! =====================')

def clamp(num, min_value, max_value):
    """
    
    """
    return max(min(num, max_value), min_value)

# Set the gripper to an pos value
def grasp(gripper_id, pos):
    """Set gripper pos

    Sends a pos command to the selected gripper. Should be in the range of
    20.0 (fully open) to 0.0 (fully closed)

    :param gripper_id: The ID of the selected gripper (LEFT or RIGHT)
    :param pos: The pos value for the gripper (0.0 to 20.0)
    :type gripper_id: int
    :type pos: float
    :returns: Nothing
    :rtype: None
    """
    pos = clamp(pos, 0, 20)

    rospy.loginfo("Setting gripper " + str(gripper_id) + " to " + str(pos))
    rospy.loginfo('Setting gripper pos to ' + str(pos) + ' for arm ' + str(gripper_id))

    rospy.wait_for_service('/yumi/rws/sm_addin/set_sg_command')
    rospy.wait_for_service('/yumi/rws/sm_addin/run_sg_routine')

    set_sg_command = rospy.ServiceProxy("/yumi/rws/sm_addin/set_sg_command", SetSGCommand)
    run_sg_routine = rospy.ServiceProxy("/yumi/rws/sm_addin/run_sg_routine", TriggerWithResultCode)

    task_L = 'T_ROB_L'
    task_R = 'T_ROB_R'
    cmd = 5
    if gripper_id   == RIGHT:
        set_sg_command(task=task_R, command=cmd, target_position=pos)
        run_sg_routine()
        time.sleep(1)
    elif gripper_id == LEFT:
        set_sg_command(task=task_L, command=cmd, target_position=pos)
        run_sg_routine()
        time.sleep(1)
    elif gripper_id == BOTH:
        set_sg_command(task=task_L, command=cmd, target_position=pos)
        set_sg_command(task=task_R, command=cmd, target_position=pos)
        run_sg_routine()
        time.sleep(1)
    else:
        print("Worng arm option, please choose LEFT, RIGHT. or BOTH")
        rospy.logwarn("Wrong arm option!")

    rospy.sleep(1.0)

def motion_init(planner="RRTConnect", planning_attempts=100, planning_time=50):
    """
    Initializes the package to interface with MoveIt!
    """
    tic = time.time()
    
    global group_l
    global group_r
    global group_both
    global robot
    global scene
    print("===================== Start Motion Initialization =====================")
    roscpp_initialize(sys.argv)

    robot = RobotCommander()
    scene = PlanningSceneInterface()
    mpr = MotionPlanRequest()
    rospy.sleep(1.0)

    # publish a demo scene
    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()

    # TODO: Check table position, and size.
    # add a table
    p.pose.position.x = 0.625
    p.pose.position.y = 0.0
    p.pose.position.z = 0.5 # Table Height
    scene.add_box("table", p, (1, 2, 0.1))
    #===========
    # Left arm |
    #===========    
    # group_l = moveit_commander.MoveGroupCommander("left_arm")
    group_l = MoveGroupCommander("left_arm")
    # Type of planner
    group_l.set_planner_id(planner)
    group_l.set_pose_reference_frame("yumi_body")

    # Replanning
    group_l.allow_replanning(True)
    group_l.set_goal_tolerance(0.005)
    group_l.set_num_planning_attempts(planning_attempts)
    group_l.set_planning_time(planning_time)
    group_l.set_max_velocity_scaling_factor(1)
    group_l.set_max_acceleration_scaling_factor(1)
    # print('For the Left arm the end effector link is')
    # print(group_l.get_end_effector_link())
    # print(group_l.get_planning_frame())

    #============
    # Right arm |
    #============
    # group_r = moveit_commander.MoveGroupCommander("right_arm")
    group_r = MoveGroupCommander("right_arm")
    # Type of planner
    group_r.set_planner_id(planner)
    group_r.set_pose_reference_frame("yumi_body")

    # Replanning
    group_r.allow_replanning(True)
    group_r.set_goal_tolerance(0.005)
    group_r.set_num_planning_attempts(planning_attempts)
    group_r.set_planning_time(planning_time)
    group_r.set_max_velocity_scaling_factor(1)
    group_r.set_max_acceleration_scaling_factor(1) 
    # print('For the Right arm the end effector link is')
    # print(group_r.get_end_effector_link())
    # print(group_l.get_planning_frame())

    #============
    # Both arms |
    #============
    group_both = MoveGroupCommander("both_arms")
    # Type of planner
    group_both.set_planner_id(planner)
    group_both.set_pose_reference_frame("yumi_body")
    # Replanning
    group_both.allow_replanning(True)
    group_both.set_goal_tolerance(0.005)
    group_both.set_num_planning_attempts(planning_attempts)    
    group_both.set_planning_time(planning_time)
    group_both.set_max_velocity_scaling_factor(1)
    group_both.set_max_acceleration_scaling_factor(1)
    
    # display_trajectory_publisher = rospy.Publisher(
    #     '/move_group/display_planned_path', DisplayTrajectory, queue_size=20)
    rospy.sleep(3)
    print("####################################     Finished Initialization     ####################################")
    toc = time.time()
    dur = round(toc - tic, 3)
    sys.stdout.write('\nYuMi MoveIt! motion initialized within {} seconds!\n\n\n'.format(dur))

# Make a plan and move within the joint space
def go_to_joints(positions, arm):
    """Set joint values
    Moves the selected arm to make the joint positions match the given values
    :param positions: The desired joint values [j1-j7] (or [j1l-j7l,j1r-j7r] for both arms at once)
    :param arm: The selected arm (LEFT, RIGHT or BOTH)
    :type positions: float[]
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    global group_l
    global group_r
    global group_both
    
    rospy.wait_for_service("/yumi/rws/sm_addin/start_egm_joint")
    rospy.wait_for_service("/yumi/rws/sm_addin/stop_egm")
    rospy.wait_for_service("/yumi/egm/controller_manager/switch_controller")

    start_egm = rospy.ServiceProxy("/yumi/rws/sm_addin/start_egm_joint", TriggerWithResultCode)
    stop_egm = rospy.ServiceProxy("/yumi/rws/sm_addin/stop_egm", TriggerWithResultCode)
    # switch_controller = rospy.ServiceProxy("/yumi/egm/controller_manager/switch_controller", SwitchController)

    stop_egm()

    if arm == LEFT:
        cur_arm = group_l
        controller_name="left_arm_controller"
    elif arm == RIGHT:
        cur_arm = group_r
        controller_name="right_arm_controller"
    elif arm == BOTH:
        cur_arm = group_both
        controller_name="both_arms_controller"
    else:
        print("Wrong Arms Selection, Left, Right, or BOTH are only accepted")
        sys.exit()
    
    tic = time.time()

    cur_arm.set_joint_value_target(positions)
    cur_arm.plan()
    
    toc = time.time()
    dur1 = round(toc - tic, 3)
    print("\nPlanning Time is: {} seconds!".format(dur1))
    start_egm()
    changeEGMSettings()
    # time.sleep(1)
    
    # switch_controller(start_controllers=controller_name, stop_controllers="", strictness=1, start_asap=False, timeout=0.0)
    controller_conf = "start_controllers: [{}] \nstop_controllers: [''] \nstrictness: 1 \nstart_asap: false \ntimeout: 0.0".format(controller_name)
    import subprocess
    subprocess.run(["rosservice", "call", "/yumi/egm/controller_manager/switch_controller", controller_conf])

    cur_arm.go(wait=True)
    rospy.sleep(3)

if(__name__ == "__main__"):
    rospy.init_node('yumi_init')
    sys_init()
    # motion_init()
    # grasp(3, 0) # Both

