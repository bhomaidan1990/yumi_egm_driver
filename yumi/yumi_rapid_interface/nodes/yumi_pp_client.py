#!/usr/bin/python3

import time
import getopt
import rospy
from abb_rapid_msgs.msg import *
from abb_robot_msgs.srv import *
from abb_robot_msgs.msg import *
from abb_rapid_sm_addin_msgs.srv import *

class PickPlaceRAPID():
    #
    def __init__(self):
        """
        
        """
        self.sys_ready = False

    def remove(self, string):
        return "".join(string.split())

    def sys_init(self):
        """
        Initialize the system, auto_mode, motors_on, rapid_running
        """
        # Reset Program Pointer to main and Start RAPID
        rospy.wait_for_service('/yumi/rws/pp_to_main')
        rospy.wait_for_service('/yumi/rws/stop_rapid')
        rospy.wait_for_service('/yumi/rws/start_rapid')
        rospy.wait_for_service('/yumi/rws/set_motors_on')
        #
        set_motors_on = rospy.ServiceProxy("/yumi/rws/set_motors_on", TriggerWithResultCode)
        stop_rapid    = rospy.ServiceProxy("/yumi/rws/stop_rapid"   , TriggerWithResultCode)
        pp_to_main    = rospy.ServiceProxy("/yumi/rws/pp_to_main"   , TriggerWithResultCode)
        start_rapid   = rospy.ServiceProxy("/yumi/rws/start_rapid"  , TriggerWithResultCode)

        # PP to main and start RAPID
        stop_rapid()
        pp_to_main()
        set_motors_on()
        # The delay is necesssary
        time.sleep(1)
        start_rapid()
        #
        self.sys_ready = True

    def set_RAPID_symbol(self, symbol_name, symbol_val, module='TRobRapid', left=True):
        """
        Function: set_RAPID_symbol, to set symbols inside RAPID using StateMachine Add-In.
        ---
        Parameters
        @param symbol_name: str, the name of the symbol in RAPID.
        @param symbol_val: str, the value of the symbol as plain text.
        @param module_: str, the RAPID script that includes the symbol.
        @param left: bool, select left arm, True for Left, False for Right arm selection.
        ---
        return None
        """
        rospy.wait_for_service('/yumi/rws/stop_rapid')
        rospy.wait_for_service('/yumi/rws/pp_to_main')
        rospy.wait_for_service('/yumi/rws/set_motors_on')
        rospy.wait_for_service('/yumi/rws/start_rapid')
        rospy.wait_for_service('/yumi/rws/get_rapid_symbol')
        rospy.wait_for_service('/yumi/rws/set_rapid_symbol')

        set_motors_on = rospy.ServiceProxy("/yumi/rws/set_motors_on", TriggerWithResultCode)
        stop_rapid    = rospy.ServiceProxy("/yumi/rws/stop_rapid"   , TriggerWithResultCode)
        pp_to_main    = rospy.ServiceProxy("/yumi/rws/pp_to_main"   , TriggerWithResultCode)
        start_rapid   = rospy.ServiceProxy("/yumi/rws/start_rapid"  , TriggerWithResultCode)
        # TODO: examine get_rapid_symbol
        get_symbol  = rospy.ServiceProxy('/yumi/rws/get_rapid_symbol', GetRAPIDSymbol)
        set_symbol  = rospy.ServiceProxy('/yumi/rws/set_rapid_symbol', SetRAPIDSymbol)

        # Task
        task_L = 'T_ROB_L'
        task_R = 'T_ROB_R'
        if left:
            task_ = task_L
        else:
            task_ = task_R

        symbol_val = self.remove(symbol_val)
        # RAPID Symbol Path message
        symbol_path_ = RAPIDSymbolPath()
        symbol_path_.task = task_
        symbol_path_.module = module
        symbol_path_.symbol = symbol_name

        stop_rapid()

        # # Response Reference:
        """
        https://github.com/ros-industrial/abb_robot_driver_interfaces/blob/master/abb_robot_msgs/msg/ServiceResponses.msg
        """
        # Set RAPID Symbol
        response = get_symbol(symbol_path_)
        # for thing in response:
        #     # str value, uint16 result_code, str message
        print('\n {} response\n'.format(symbol_name), response.value)
        # print('\n', symbol_name, symbol_val, '\n')
        exam = set_symbol(symbol_path_, symbol_val) # response.value
        # for thing in exam:
        print('\n {} exam \n'.format(symbol_name), exam)

        pp_to_main()
        set_motors_on()
        time.sleep(1)
        start_rapid()
        #
        self.sys_ready = True

    def set_run_RAPID_routine(self, task, routine):
        """
        Function: set_run_RAPID_routine, to set and run RAPID Routine.
        """
        rospy.wait_for_service('/yumi/rws/sm_addin/set_rapid_routine')
        rospy.wait_for_service('/yumi/rws/sm_addin/run_rapid_routine')
        #
        set_rapid_routine = rospy.ServiceProxy('/yumi/rws/sm_addin/set_rapid_routine', SetRAPIDRoutine)
        run_rapid_routine = rospy.ServiceProxy('/yumi/rws/sm_addin/run_rapid_routine', TriggerWithResultCode)
        #
        set_rapid_routine(task=task, routine=routine)
        time.sleep(0.5)
        #
        run_rapid_routine()

    def eax_completer(self, pattern):
        """
        Function: eax_completer, to complete unused patterns in the external joints.
        ---
        Parameters:
        @param pattern: str, pattern to be modified.
        ---
        return completed point details.
        """
        eax_all = ",9E9,9E9,9E9,9E9,9E9]]" # ",9E+09,9E+09,9E+09,9E+09,9E+09]]"
        # remove spaces at end (if any).
        pattern = pattern.rstrip()
        return pattern[:-2]+eax_all

    def pick_place_rapid(self, pick_pt, place_pt, speed, pick_ap_pt=None, place_ap_pt=None, left=True):
        """
        Function: pick_place_rapid
        ---
        Parameters:
        @param pick_pt:  str, "[[x, y, z], [q1, q2, q3, q4], [cfg1, cfg4, cfg6, cfgx], [eax_a]]"
        @param place_pt:    str, "[[x, y, z], [q1, q2, q3, q4], [cfg1, cfg4, cfg6, cfgx], [eax_a]]"
        @param speed: str, [V_TCP(mm/s), V_ReOrient (deg/sec), V_ext_lin_axis (mm/s), V_rot_ext_axis (mm/sec)].
        @param pick_ap_pt:  str, "[[x, y, z], [q1, q2, q3, q4], [cfg1, cfg4, cfg6, cfgx], [eax_a]]"
        @param place_ap_pt: str, "[[x, y, z], [q1, q2, q3, q4], [cfg1, cfg4, cfg6, cfgx], [eax_a]]"
        @param left: bool, True for left arm, False for the usage of the right arm.
        """
        # Task
        task_L = 'T_ROB_L'
        task_R = 'T_ROB_R'
        if left:
            task_ = task_L
        else:
            task_ = task_R

        if pick_ap_pt is not None:
            self.set_RAPID_symbol("pickap", self.eax_completer(pick_ap_pt), left=left)
        #
        if place_ap_pt is not None:
            self.set_RAPID_symbol("place_ap", self.eax_completer(place_ap_pt), left=left)
        
        """
        The speed data is defined with the following velocities:
        [v1, v2, v3, v4]
        - v1 mm/s for the TCP.
        - v2 degrees/s for reorientation of the tool.
        - v3 mm/s for linear external axes.
        - v4 degrees/s for rotating external axes.
        """
        self.set_RAPID_symbol("pp_speed", speed, left=left)
        self.set_RAPID_symbol("pickpoint", self.eax_completer(pick_pt), left=left)
        self.set_RAPID_symbol("placepoint", self.eax_completer(place_pt), left=left)
        self.set_run_RAPID_routine(task=task_, routine='pickplace')

    def usage_and_quit(self):
        """
        Function: usage_and_quit, to print help, in case of wrong parameters.
        ---
        Parameters:
        None
        @return: None.
        """
        print(" --pick_pt str, \"[[x, y, z], [q1, q2, q3, q4], [cfg1, cfg4, cfg6, cfgx], [eax_a]]\"")
        print(" --place_pt str, \"[[x, y, z], [q1, q2, q3, q4], [cfg1, cfg4, cfg6, cfgx], [eax_a]]\"")
        print(" --speed str, [V_TCP(mm/s), V_ReOrient (deg/sec), V_ext_lin_axis (mm/s), V_rot_ext_axis (mm/sec)].")
        print(" --pick_ap_pt str, \"[[x, y, z], [q1, q2, q3, q4], [cfg1, cfg4, cfg6, cfgx], [eax_a]]\"")
        print(" --place_ap_pt str, \"[[x, y, z], [q1, q2, q3, q4], [cfg1, cfg4, cfg6, cfgx], [eax_a]]\"")
        print(" --left bool, True for left arm, False to use right arm")
        sys.exit(2)


if __name__ == '__main__':
    # print(sys.argv)
    PP_RAPID_ = PickPlaceRAPID()
    try:
        opts, remainder = getopt.getopt(sys.argv[1:], "ha:d:s:b:c:l",
        ["help", "pick_pt=", "place_pt=", "speed=","pick_ap_pt=", "place_ap_pt=","left"])
    except getopt.GetoptError:
      PP_RAPID_.usage_and_quit()

    # Default Parameters
    pick_pt  = "[[285.266, 353.4495,106.9714],[0.005935209, 0.9999219,0.01035514, 0.003705649], [-1,1,2,4], [156.847]]"
    place_pt = "[[399.374,52.15265,110.8838],[0.005965178, 0.9999416,-0.006217764, -0.00652423], [-1,1,1,4], [-160.319]]"
    speed = "[100,500,5000,1000]"
    pick_ap_pt  = None
    place_ap_pt = None
    left = False
    # Get Options
    for opt, arg in opts:
        if opt in ("-h", "--help"):
            PP_RAPID_.usage_and_quit()       
        elif opt in ("-a", "--pick_pt"):
            pick_pt = arg
        elif opt in ("-d", "--place_pt"):
            place_pt = arg 
        elif opt in ("-s", "--speed"):
            speed = arg
        elif opt in ("-b", "--pick_ap_pt"):
            pick_ap_pt = arg
        elif opt in ("-c", "--place_ap_pt"):
            place_ap_pt = arg
        elif opt in ("-l", "--left"):
            left = True
        else:
            print("\nUnsupported Parameters!!!\n")
    #
    # PP_RAPID_.sys_init()
    #
    PP_RAPID_.pick_place_rapid(pick_pt, place_pt, speed, pick_ap_pt, place_ap_pt, left)