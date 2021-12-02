#!/usr/bin/env python3
import rospy



def sys_init():
    """
    Initialize the system, auto_mode, motors_on, rapid_running, egm_settings_adjusted
    """
    # TODO: define messge type
    system_info = rospy.wait_for_message("/yumi/rws/system_states", messageType)
    # TODO: check msg.auto_mode, msg.motors_on, msg.rapid_running

def motion_init():
    pass