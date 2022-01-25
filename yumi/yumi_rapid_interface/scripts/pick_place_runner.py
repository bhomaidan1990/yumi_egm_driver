#!/usr/bin/python3

import subprocess
import rospy
import pandas as pd

def pp_handler(path, pt_name):
    """
    
    """
    # read DataFrame
    df = pd.read_csv(path)
    # Filter DataFrame by point name
    print(df.loc[df['point_name']==pt_name])
    pt = df.loc[(df['point_name']==pt_name)].iloc[0]
    # Point as String
    out_pt = str(list(pt[["x", "y", "z"]])) +','+ str(list(pt[["q1", "q2", "q3", "q4"]]))+','+ str(list(pt[["cfg1", "cfg4", "cfg6", "cfgx"]])) +','+ str(list(pt[[ "eax_a"]]))
    out_pt = '['+out_pt+']'
    # Arm
    left = True if (pt["arm"]=="left") else False

    return out_pt, left

# TODO: change the path to suit your needs.
pointsPath = "/home/belal/ros1_ws/EGM_Packages/EGM_new_yumi/src/yumi/yumi_rapid_interface/scripts/points.csv"

# selecting point by name
pick_pt_name = "l_xyz_0_23_0"
place_pt_name= "l_xyz_5_11_0"

# pick_ap_pt_name  = "l_air_1"
# place_ap_pt_name = "l_air_2"

pick_pt, left_k  = pp_handler(pointsPath, pick_pt_name)
place_pt, left_c = pp_handler(pointsPath, place_pt_name)
# pick_ap_pt, left_k_ap = pp_handler(pointsPath, pick_ap_pt_name)
# place_ap_pt, left_c_ap = pp_handler(pointsPath, place_ap_pt_name)

speed = "v200"

rospy.wait_for_service('/yumi/rws/set_rapid_symbol')

# # Left Arm
# if(left_k and left_c and left_k_ap and left_c_ap):
if(left_k and left_c):
    #
    subprocess.run(["rosrun", "yumi_rapid_interface", "yumi_pp_client.py", "-a "+pick_pt, "-d "+place_pt, "-s "+speed, "-l"])
    # subprocess.run(["rosrun", "yumi_rapid_interface", "yumi_pp_client.py", "-a "+pick_pt, "-d "+place_pt, "-s "+speed, "-b "+pick_ap_pt, "-c "+place_ap_pt, "-l"])
# # Right Arm
# elif(not left_k and not left_c and not left_k_ap and not left_c_ap):
elif(not left_k and not left_c):
    subprocess.run(["rosrun", "yumi_rapid_interface", "yumi_pp_client.py", "-a "+pick_pt, "-d "+place_pt, "-s "+speed])
    # subprocess.run(["rosrun", "yumi_rapid_interface", "yumi_pp_client.py", "-a "+pick_pt, "-d "+place_pt, "-s "+speed, "-b "+pick_ap_pt, "-c "+place_ap_pt])
# # Unknown Arm!!
else:
    print("\nWatch out for correct points selection!\n")




