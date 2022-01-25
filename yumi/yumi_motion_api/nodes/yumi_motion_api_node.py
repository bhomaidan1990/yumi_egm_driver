#!/usr/bin/python3

# import sys
import rospy
import time
import copy
from math import degrees
# from std_msgs.msg import Float64
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from moveit_msgs.msg import *
from moveit_commander import *
# from rospy_message_converter import message_converter
# from moveit_commander.conversions import list_to_pose
from abb_rapid_sm_addin_msgs.srv import *
from abb_robot_msgs.srv import *
from abb_robot_msgs.msg import *
from controller_manager_msgs.srv import *
# import tf
# from std_msgs.msg import Float64
# from utils import sys_init, motion_init, go_to_joints, grasp
from yumi_motion_api.srv import *

#====================================================================
global group_l  # :The move group for the left arm
global group_r  # :The move group for the right arm
global group_both  # :The move group for using both arms at once
global robot  # :The RobotCommander() from MoveIt!
global scene  # :The PlanningSceneInterface from MoveIt!

LEFT  = 1  # :ID of the left arm
RIGHT = 2  # :ID of the right arm
BOTH  = 3  # :ID of both_arms

def changeEGMSettings():
    """
    
    """
    rospy.wait_for_service('/yumi/rws/sm_addin/get_egm_settings')
    rospy.wait_for_service('/yumi/rws/sm_addin/set_egm_settings')
    # rospy.wait_for_service("/yumi/rws/sm_addin/stop_egm")

    # stop_egm = rospy.ServiceProxy("/yumi/rws/sm_addin/stop_egm", TriggerWithResultCode)

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
    settings_L.run.cond_time = 60.0
    settings_R.run.cond_time = 60.0
    # posCorrgain = 0.0
    settings_L.run.pos_corr_gain = 0.0
    settings_R.run.pos_corr_gain = 0.0
    
    # stop_egm()
    
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
    time.sleep(1)
    # # Calibrate Grippers
    cmd = 4
    set_sg_command(task=task_L, command=cmd, target_position=fin_pos)
    set_sg_command(task=task_R, command=cmd, target_position=fin_pos)
    run_sg_routine()
    time.sleep(2)
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

"""
- SBL
- EST
- LBKPIECE
- BKPIECE
- KPIECE
- RRT
- RRTConnect
- RRTstar
- TRRT
- PRM
- PRMstar
- FMT
- BFMT
- PDST
- STRIDE
- BiTRRT
- LBTRRT
- BiEST
- ProjEST
- LazyPRM
- LazyPRMstar
- SPARS
- SPARStwo
"""
def motion_init(planner="RRTConnect", planning_attempts=100, planning_time=20, replan = False):
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

    rospy.sleep(2.0)
    # scene.remove_world_object("table1")
    # scene.remove_world_object("table2")
    scene.remove_world_object("table")

    # p1 = PoseStamped()
    # p1.header.frame_id = robot.get_planning_frame()
    # p1.header.stamp = rospy.Time.now()
    # # TODO: Check table position, and size.
    # # add a table
    # p1.pose.position.x = 0.3
    # p1.pose.position.y = 0.2
    # p1.pose.position.z = 0.14 # Table Height
    # p1.pose.orientation.x = 0.0
    # p1.pose.orientation.y = 0.0
    # p1.pose.orientation.z = 0.0
    # p1.pose.orientation.w = 1.0
    # scene.add_box("table1", p1, (0.5, 0.5, 0.05))

    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.header.stamp = rospy.Time.now()
    # TODO: Check table position, and size.
    # add a table
    p.pose.position.x = 0.3
    p.pose.position.y = 0.0
    p.pose.position.z = 0.12 # Table Height
    p.pose.orientation.x = 0.0
    p.pose.orientation.y = 0.0
    p.pose.orientation.z = 0.0
    p.pose.orientation.w = 1.0
    scene.add_box("table", p, (0.55, 0.95, 0.05))
    #===========
    # Left arm |
    #===========    
    # group_l = moveit_commander.MoveGroupCommander("left_arm")
    group_l = MoveGroupCommander("left_arm")
    # Type of planner
    group_l.set_planner_id(planner)
    group_l.set_pose_reference_frame("yumi_body")

    # Replanning
    group_l.allow_replanning(replan)
    group_l.set_goal_tolerance(0.005)
    group_l.set_num_planning_attempts(planning_attempts)
    group_l.set_planning_time(planning_time)
    group_l.set_max_velocity_scaling_factor(1)
    group_l.set_max_acceleration_scaling_factor(1)
    # print('For the Left arm the end effector link is')
    # print(group_l.get_end_effector_link())
    # print(group_l.get_planning_frame())
    # print("left: --> ", group_l.get_current_pose().pose)
    # print("left: --> ", group_l.get_current_joint_values())
    #============
    # Right arm |
    #============
    # group_r = moveit_commander.MoveGroupCommander("right_arm")
    group_r = MoveGroupCommander("right_arm")
    # Type of planner
    group_r.set_planner_id(planner)
    group_r.set_pose_reference_frame("yumi_body")

    # Replanning
    group_r.allow_replanning(replan)
    group_r.set_goal_tolerance(0.005)
    group_r.set_num_planning_attempts(planning_attempts)
    group_r.set_planning_time(planning_time)
    group_r.set_max_velocity_scaling_factor(1)
    group_r.set_max_acceleration_scaling_factor(1) 
    # print('For the Right arm the end effector link is')
    # print(group_r.get_end_effector_link())
    # print(group_l.get_planning_frame())
    # print("right --> ", group_r.get_current_pose().pose)
    # print("right --> ", group_r.get_current_joint_values())
    #============
    # Both arms |
    #============
    group_both = MoveGroupCommander("both_arms")
    # Type of planner
    group_both.set_planner_id(planner)
    group_both.set_pose_reference_frame("yumi_body")
    # Replanning
    group_both.allow_replanning(replan)
    group_both.set_goal_tolerance(0.005)
    group_both.set_num_planning_attempts(planning_attempts)    
    group_both.set_planning_time(planning_time)
    group_both.set_max_velocity_scaling_factor(1)
    group_both.set_max_acceleration_scaling_factor(1)
    # print('both --> ',group_both.get_current_joint_values())
    
    # display_trajectory_publisher = rospy.Publisher(
    #     '/move_group/display_planned_path', DisplayTrajectory, queue_size=20)
    rospy.sleep(3)
    print("===================== Motion Initialization Successful! =====================")
    toc = time.time()
    dur = round(toc - tic, 3)
    sys.stdout.write('\nYuMi MoveIt! motion initialized within {} seconds!\n\n\n'.format(dur))

# Make a plan and move within the joint space
def go_to_joints(joints_, arm):
    """Set joint values
    Moves the selected arm to make the joints match the given values
    :param joints_: The desired joint states [j1-j7] (or [j1l-j7l,j1r-j7r] for both arms at once)
    :param arm: The selected arm (LEFT, RIGHT or BOTH)
    :type joints_: instance of JointState[]
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    global group_l
    global group_r
    global group_both
    
    rospy.wait_for_service("/yumi/rws/sm_addin/start_egm_joint")
    rospy.wait_for_service("/yumi/rws/sm_addin/stop_egm")
    # rospy.wait_for_service("/yumi/egm/controller_manager/switch_controller")

    start_egm = rospy.ServiceProxy("/yumi/rws/sm_addin/start_egm_joint", TriggerWithResultCode)
    stop_egm = rospy.ServiceProxy("/yumi/rws/sm_addin/stop_egm", TriggerWithResultCode)
    # switch_controller = rospy.ServiceProxy("/yumi/egm/controller_manager/switch_controller", SwitchController)

    stop_egm()

    if arm == LEFT:
        cur_arm = group_l
        controller_name = "left_arm_vel_controller"

    elif arm == RIGHT:
        cur_arm = group_r
        controller_name ="right_arm_vel_controller"

    elif arm == BOTH:
        cur_arm = group_both
        controller_name ="both_arms_vel_controller"

    else:
        print("Wrong Arms Selection, Left, Right, or BOTH are only accepted")
        sys.exit()
    
    tic = time.time()

    cur_arm.set_joint_value_target(joints_)
    cur_arm.plan()
    
    toc = time.time()
    dur1 = round(toc - tic, 3)
    print("\nPlanning Time is: {} seconds!".format(dur1))
    # TODO: check if this sleep is necessary.
    time.sleep(1)
    changeEGMSettings()
    start_egm()

    # print("Controller_Name:", controller_name)

    # switch_controller(start_controllers=list(controller_name), stop_controllers=[""], strictness=1, start_asap=False, timeout=0.0)
    controller_conf = "start_controllers: [{}] \nstop_controllers: [''] \nstrictness: 1 \nstart_asap: false \ntimeout: 0.0".format(controller_name)
    import subprocess
    subprocess.run(["rosservice", "call", "/yumi/egm/controller_manager/switch_controller", controller_conf])

    cur_arm.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    cur_arm.stop()

    rospy.sleep(3)

# Make a plan and move within the cartesian space
def go_to_pose(pose_, arm):
    """Set pose values
    Moves the selected arm to a specific pose.
    :param pose_: The desired end-effector pose.
    :param arm: The selected arm (LEFT, RIGHT or BOTH)
    :type pose_: Pose
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    global group_l
    global group_r
    global group_both
    # TODO: check pose or joint egm
    rospy.wait_for_service("/yumi/rws/sm_addin/start_egm_joint")
    rospy.wait_for_service("/yumi/rws/sm_addin/stop_egm")
    # rospy.wait_for_service("/yumi/egm/controller_manager/switch_controller")

    start_egm = rospy.ServiceProxy("/yumi/rws/sm_addin/start_egm_joint", TriggerWithResultCode)
    stop_egm = rospy.ServiceProxy("/yumi/rws/sm_addin/stop_egm", TriggerWithResultCode)
    
    # switch_controller = rospy.ServiceProxy("/yumi/egm/controller_manager/switch_controller", SwitchController)

    stop_egm()

    if arm == LEFT:
        cur_arm = group_l
        controller_name="left_arm_vel_controller"
    elif arm == RIGHT:
        cur_arm = group_r
        controller_name="right_arm_vel_controller"
    elif arm == BOTH:
        cur_arm = group_both
        controller_name="both_arms_vel_controller"
    else:
        print("Wrong Arms Selection, Left, Right, or BOTH are only accepted")
        sys.exit()
    
    tic = time.time()

    cur_arm.set_pose_target(pose_)
    cur_arm.plan()
    
    toc = time.time()
    dur1 = round(toc - tic, 3)
    print("\nPlanning Time is: {} seconds!".format(dur1))
    time.sleep(1)
    changeEGMSettings()
    start_egm()

    # time.sleep(0.5)
    # switch_controller(start_controllers=list(controller_name), stop_controllers=[""], strictness=1, start_asap=False, timeout=0.0)

    controller_conf = "start_controllers: [{}] \nstop_controllers: [''] \nstrictness: 1 \nstart_asap: false \ntimeout: 0.0".format(controller_name)
    import subprocess
    subprocess.run(["rosservice", "call", "/yumi/egm/controller_manager/switch_controller", controller_conf])

    cur_arm.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    cur_arm.stop()

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    cur_arm.clear_pose_targets()

    rospy.sleep(3)

def MoveL(scale, arm):
    if arm == LEFT:
        cur_arm = group_l
    elif arm == RIGHT:
        cur_arm = group_r
    else:
        print("Wrong Arms Selection, Left, Right, or BOTH are only accepted")
        sys.exit()

    waypoints = []

    wpose = cur_arm.get_current_pose().pose
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.z += scale * 0.1  # First move up (z)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = cur_arm.compute_cartesian_path(
        waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    print('Linear_Movement with scale: {}'.format(scale))
    cur_arm.execute(plan, wait=True)
    cur_arm.stop()

def pick(joints_, arm, scale=0.016):
    grasp(arm, 20)
    time.sleep(2)
    go_to_joints(joints_, arm)
    time.sleep(2)
    MoveL(scale, arm)
    time.sleep(1)
    grasp(arm, 15)
    # time.sleep(2)
    # MoveL(-scale, arm)

def place(joints_, arm, scale=+0.016):
    go_to_joints(joints_, arm)
    time.sleep(2)
    MoveL(scale, arm)
    time.sleep(1)
    grasp(arm, 20)
    time.sleep(2)
    MoveL(-scale, arm)
#====================================================================

def move_joints(req):
    """
    
    """
    go_to_joints(req.joints, req.arm)
    return YuMiJointsCmdResponse(True)

def move_pose(req):
    """
    
    """
    go_to_pose(req.goalPose, req.arm)
    return YuMiPoseCmdResponse(True)

def pick_place(req):
    pick(req.pick_joints, req.arm)
    # time.sleep(5)
    # place(req.place_joints, req.arm)
    # time.sleep(5)
    return YuMiPickPlaceResponse(True)

if __name__ == '__main__':
    try:
        rospy.init_node('yumi_motion_api')
        sys_init()
        motion_init()
        rospy.Service('move_joints', YuMiJointsCmd, move_joints)
        rospy.Service('move_pose', YuMiPoseCmd, move_pose)
        # rospy.Service('pick_place', YuMiPickPlace, pick_place)
        rospy.spin()
        rospy.loginfo('=== Yumi_motion_API node Initialized! ===')
        
            
    except rospy.ROSInterruptException:
        rospy.logerr("yumi_motion_api node has died!")