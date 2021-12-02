#!/usr/bin/env python3

from math import degrees
from time import sleep

import rospy

from abb_rapid_sm_addin_msgs.srv import *
from abb_robot_msgs.srv import *

def changeEGMSettings():
    """
    
    """
    rospy.wait_for_service('/yumi/rws/sm_addin/get_egm_settings')
    rospy.wait_for_service('/yumi/rws/sm_addin/set_egm_settings')

    get_egm_settings = rospy.ServiceProxy("/yumi/rws/sm_addin/get_egm_settings", GetEGMSettings)
    set_egm_settings = rospy.ServiceProxy("/yumi/rws/sm_addin/set_egm_settings", SetEGMSettings)

    current_settings_L = get_egm_settings(task='T_ROB_L')
    current_settings_R = get_egm_settings(task='T_ROB_R')

    settings_L = current_settings_L.settings
    settings_R = current_settings_R.settings

    # print ("Before:", settings_L, '\n\n', settings_R)

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
    
    print ("After", settings_L, '\n\n', settings_R)

    taskname = "T_ROB_L"
    set_egm_settings(task=taskname, settings=settings_L)

    taskname = "T_ROB_R"
    set_egm_settings(task=taskname, settings=settings_R)

    print("EGM Settings Updated")

def init_rapid():
    """
    
    """
    rospy.wait_for_service('/yumi/rws/set_motors_on')
    # rospy.wait_for_service('/yumi/rws/set_motors_off')
    rospy.wait_for_service('/yumi/rws/pp_to_main')
    rospy.wait_for_service('/yumi/rws/stop_rapid')
    rospy.wait_for_service('/yumi/rws/start_rapid')

    # set_motors_off= rospy.ServiceProxy("/yumi/rws/set_motors_off",TriggerWithResultCode)
    stop_rapid    = rospy.ServiceProxy("/yumi/rws/stop_rapid"   , TriggerWithResultCode)
    set_motors_on = rospy.ServiceProxy("/yumi/rws/set_motors_on", TriggerWithResultCode)
    pp_to_main    = rospy.ServiceProxy("/yumi/rws/pp_to_main"   , TriggerWithResultCode)
    start_rapid   = rospy.ServiceProxy("/yumi/rws/start_rapid"  , TriggerWithResultCode)

    # set_motors_off()
    stop_rapid()
    set_motors_on()
    sleep(1)
    pp_to_main()
    sleep(1)
    start_rapid()
    print("RAPID Initialized, and Mototrs On")

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
    cmd = 6
    fin_pos = 0.0
    set_sg_command(task=task_L, command=cmd, target_position=fin_pos)
    set_sg_command(task=task_R, command=cmd, target_position=fin_pos)
    run_sg_routine()
    sleep(1)
    cmd = 4
    fin_pos = 0.0
    set_sg_command(task=task_L, command=cmd, target_position=fin_pos)
    set_sg_command(task=task_R, command=cmd, target_position=fin_pos)
    run_sg_routine()
    sleep(2)
    # cmd = 7
    # fin_pos = 0.0
    # set_sg_command(task=task_L, command=cmd, target_position=fin_pos)
    # set_sg_command(task=task_R, command=cmd, target_position=fin_pos)
    # run_sg_routine()
    print('grippers calibrated')

if __name__ == '__main__':
    try:
        changeEGMSettings()
        init_rapid()
        calibrateGrippers()
    except:
        print("ERROR: is roscore and rosservices are running?!")