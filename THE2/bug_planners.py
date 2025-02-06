import numpy as np

def planner(robot,sensor,target_position):
    if robot.planner == 0:
        bug0(robot,sensor,target_position)
    elif robot.planner == 1:
        bug1(robot,sensor,target_position)
    elif robot.planner == 2:
        bug2(robot,sensor,target_position)

####################################BUG0################################

def checkObstacleSide(position,target_position):
    v_r2t=[target_position[0]-position.x,target_position[1]-position.y]  # Vector from robot to target
    v_rh=[np.cos(np.radians(position.theta)),np.sin(np.radians(position.theta))] # Vector in the direction of robot heading
    direction = np.cross(v_rh,v_r2t) # Cross product sign will obey right hand rule
    if direction<=0: # negative sign obstacle on right 
        free_side=0  # since we have left turning robot, target will be on the right. So target are not on the free side
    else:
        free_side=1
    return free_side

def bug0(robot,bump_sensors,target_position):
    if robot.fsm == "m2g":
        if any(sensor.activated for sensor in bump_sensors):
            robot.fsm = "wf"
        else:
            robot.fsm = "m2g"
    elif robot.fsm == "wf":
        if checkObstacleSide(robot.position,target_position):
            robot.fsm = "m2g"
        else:
            robot.fsm = "wf"

######################################BUG1###############################
def writeMemory(robot, target_position):
    robot.memory.pos = np.append(robot.memory.pos, [robot.position.x, robot.position.y])
    d = np.linalg.norm(target_position - np.array([robot.position.x, robot.position.y]))
    robot.memory.d = np.append(robot.memory.d, d)

def findLeavePoint(robot):
    min_index = np.argmin(robot.memory.d) 
    robot.leavepoint = robot.memory.pos[min_index * 2:min_index * 2 + 2]


# !!!!!!! The code below is the same with bug 0. Make the necessary changes and additional functions for bug1
def bug1(robot, bump_sensors, target_position):
    if robot.fsm == "m2g":
        if np.linalg.norm(np.array([robot.position.x, robot.position.y]) - target_position) < 1:
            print("Target reached!")
            return  

        if any(sensor.activated for sensor in bump_sensors):
            robot.hitpoint = [robot.position.x, robot.position.y]
            robot.memory.d = []  # Belleği sıfırlıyoruz
            robot.fsm = "wf"
        else:
            robot.fsm = "m2g"

    elif robot.fsm == "wf":
        writeMemory(robot, target_position)

        findLeavePoint(robot)

        if checkObstacleSide(robot.position, target_position):
            robot.fsm = "m2lp"
        elif (
            np.allclose(robot.position.x, robot.hitpoint[0], atol=1e-1)
            and np.allclose(robot.position.y, robot.hitpoint[1], atol=1e-1)
            and len(robot.memory.d) > 0
            and np.min(robot.memory.d) >= np.linalg.norm(target_position - robot.hitpoint)
        ):
            print("Path does not exist")
            robot.fsm = "noSoln"  

    elif robot.fsm == "m2lp":
        # Leavepoint'e ulaşılırsa m2g moduna geçilir
        if np.linalg.norm(np.array(robot.leavepoint) - np.array([robot.position.x, robot.position.y])) < 1:
            robot.fsm = "m2g"
        
        elif any(sensor.activated for sensor in bump_sensors):
            robot.fsm = "wf"

    elif robot.fsm == "noSoln":
        print("Path does not exist")
        raise KeyError("Path does not exist")  
    
#################################BUG2##################################

def storeLineEquation(robot, bump_sensors):
    # Store line equation parameters in memory (slope, intercept)
    if any(sensor.activated for sensor in bump_sensors):
        # For simplicity, we assume a 2D linear equation y = mx + b
        # A more advanced line-following algorithm would calculate these based on the robot's movement
        m = (bump_sensors[1].world_y - bump_sensors[0].world_y) / (bump_sensors[1].world_x - bump_sensors[0].world_x)  # Slope
        b = bump_sensors[0].world_y - m * bump_sensors[0].world_x  # Intercept
        robot.memory.bug2params = np.array([m, b])  # Store the line equation parameters in memory


# !!!!!!! The code below is the same with bug 0. Make the necessary changes and additional functions for bug2
def bug2(robot, bump_sensors, target_position):
    if robot.fsm == "m2g":
        # Move directly toward the target
        if np.linalg.norm(np.array([robot.position.x, robot.position.y]) - target_position) < 1:
            print("Target reached!")
            return  # Target reached, terminate
        
        if any(sensor.activated for sensor in bump_sensors):
            # Store hitpoint and switch to wall-following mode
            robot.hitpoint = [robot.position.x, robot.position.y]
            robot.memory.d = []  # Clear memory
            storeLineEquation(robot, bump_sensors)
            robot.fsm = "wf"
        else:
            robot.fsm = "m2g"
    
    elif robot.fsm == "wf":
        writeMemory(robot, target_position)
        
        # Store the line equation as part of the memory
        storeLineEquation(robot, bump_sensors)
        
        # Follow the obstacle boundary
        if checkObstacleSide(robot.position, target_position):
            robot.fsm = "m2lp"  # Switch to move-to-leavepoint if we are closer to goal
        elif (
            np.allclose(robot.position.x, robot.hitpoint[0], atol=1e-1)
            and np.allclose(robot.position.y, robot.hitpoint[1], atol=1e-1)
        ):
            robot.fsm = "noSoln"  # If we are stuck, return "Path does not exist"
    
    elif robot.fsm == "m2lp":
        if np.linalg.norm(np.array(robot.leavepoint) - np.array([robot.position.x, robot.position.y])) < 1:
            robot.fsm = "m2g"
        elif any(sensor.activated for sensor in bump_sensors):
            robot.fsm = "wf"
    
    elif robot.fsm == "noSoln":
        print("Path does not exist")
        raise KeyError("Path does not exist")
