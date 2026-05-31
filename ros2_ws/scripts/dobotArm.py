import lib.DobotDllType as dType

#Useful global variables
# --- These are status strings that you might see, so we're defining them here ---
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}

#always begin with this line, or you can't connect to the robot at all. Just don't
#remove this line and keep it at the top of your code
api = dType.load()

"""
These coordinates are to the left of the robot's x axis and slight above the xy plane, viewed from
the top. This is a useful home position when dealing with the vision labs, since it moves
the robot out of the way. You can change the coordinates here if you really want.
"""
home_pos = [200,100,50]

def initialize_robot(api):
    #detect the robot's com port
    com_port = dType.SearchDobot(api)[0]
    
    #if we can't find it, then we can't continue, so exit
    if "COM" not in com_port:
        print("Error: The robot either isn't on or isn't responding. Exiting now")
        exit()
    
    #we've found it, so let's try to connect
    state = dType.ConnectDobot(api, "COM7", 115200)[0]
    
    #If the connection failed at this point, we also can't proceed, so we need to exit
    if state != dType.DobotConnect.DobotConnect_NoError:
        print("Failed to connect to Dobot!")
        exit()
    
    """
        stop any queued commands and clear the queue. You HAVE TO do this every time you initialize the robot
        If there are queued commands in the queue, then they will execute first. This can
        cause the robot to go well outside of its allowable range. The simplest way to do this
        is to stop anything that might be running or might try to run, then clear the queue.
        
        Other than at startup, during normal operation you shouldn't have to do this.
    """
    dType.SetQueuedCmdStopExec(api)
    dType.SetQueuedCmdClear(api)
    
    #Set the robot's max speed and acceleration. We're keeping these to 50% of max for safety
    dType.SetPTPCommonParams(api, 50, 50, isQueued=1)
    
    """
        Home the robot. 
    """
    #Set the home position
    dType.SetHOMEParams(api, home_pos[0], home_pos[1], home_pos[2], 0, isQueued=1)
    
    cmdIndx = -1
    """
        Enqueue the home command. This command always begins by moving the robot back to an initialization
        position so that the encoders are reset, then it will move the robot to its home position,
        and finally it will undergo a quick procedure to validate that its encoders are properly set. You definitely
        want to run this every time you initialize the robot
    """
    execCmd = dType.SetHOMECmd(api, temp=0, isQueued=1)[0]
    
    #Execute the three enqueued commands: set the speed/acceleration, set the home position, and move to home
    dType.SetQueuedCmdStartExec(api)
    
    #Allow the homing command to complete. The robot will beep and the LED will turn green
    #when it's ready to go
    while execCmd > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(25)
        
    #OK, the robot is ready to move!
    
"""
    CHANGE: Switched from using Cartesian Linear to a Joint Space PTP mode
"""
def move_to_xyz(api,x,y,z,rHead=0):
    cmdIndx = -1
    execCmd = dType.SetPTPCmd(api,dType.PTPMode.PTPMOVJXYZMode,x,y,z,rHead,isQueued=0)[0]
    #Allow the command to complete. The robot will stop moving when it's done
    while execCmd > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(25)

"""
    Move the robot to the given joint angles using PTP Linear ANGLE mode
    We will default J4 to zero, since it only matters if you have an end effector attached
"""
def move_joint_angles(api,J1,J2,J3,J4=0):
    cmdIndx = -1
    
    execCmd = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJANGLEMode, J1, J2, J3, J4, isQueued = 0)[0]
    #Allow the command to complete. The robot will stop moving when it's done
    while execCmd > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(25)
    
"""
    Move the robot to it's home position. Note: this will use basic PTP motion, rather than
    SetHOMECmd, since SetHOMECmd will re-run the sensor initialization stuff that we don't
    need during normal operation
"""
def move_to_home(api):
    move_to_xyz(api,home_pos[0],home_pos[1],home_pos[2])
    
    
def rotate_end_effector(api,angle):
    if angle <= 90 and angle >= -90:
        pose = dType.GetPose(api)
        cmdIndx = -1
        execCmd = dType.SetPTPCmd(api,dType.PTPMode.PTPMOVLXYZMode,pose[0],pose[1],pose[2],angle,isQueued=0)[0]
        #Allow the command to complete. The robot will stop moving when it's done
        while execCmd > dType.GetQueuedCmdCurrentIndex(api)[0]:
            dType.dSleep(25)
        
def open_gripper(api):
    #arguments are: api, enable control = 1, grip = 0 ("release"), isQueued = 0
    dType.SetEndEffectorGripper(api,1,0,0)[0]
    #This command just gets sent, there is no feedback, so we need to wait until the gripper
    #is done opening
    dType.dSleep(500)

def close_gripper(api):
    #arguments are: api, enable control = 1, grip = 1 ("close"), isQueued = 0
    dType.SetEndEffectorGripper(api,1,1,0)[0]
    #This command just gets sent, there is no feedback, so we need to wait until the gripper
    #is done opening
    dType.dSleep(500)
    
def stop_pump(api):
    #Yeah, I know it says suction cup. it's actually controlling the pneumatic pump
    dType.SetEndEffectorSuctionCup(api,1,0,0)[0]
    #This command just gets sent, there is no feedback, so we need to wait until the pump turns off
    #We don't need to wait as long as we do for the gripper
    dType.dSleep(50)

