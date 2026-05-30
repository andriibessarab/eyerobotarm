import dobotArm
import lib.DobotDllType as dType
import time

#Before running and commands, always run this
api = dType.load()
dobotArm.initialize_robot(api)

"""
    Sample Script that Moves the Dobot and the End Effector
"""
dobotArm.move_to_xyz(api, 200, 50, 50)    
robot_pose = dType.GetPose(api)
print(robot_pose)
dobotArm.rotate_end_effector(api, 90)
time.sleep(1)

dobotArm.move_to_home(api)
dobotArm.open_gripper(api)

dobotArm.move_to_xyz(api, 200, 100, 10)
dobotArm.close_gripper(api)
dobotArm.stop_pump(api)
robot_pose = dType.GetPose(api)
print(robot_pose)

print("PTP Motions done. Moving in Joint Space now")
#move by joint angles, in degrees    
dobotArm.move_joint_angles(api,0,45,45)

#Back to home
dobotArm.move_to_home(api)

#All done!