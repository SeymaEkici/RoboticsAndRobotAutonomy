import the2_simulation
import bug_planners
import matplotlib.pyplot as plt
from matplotlib.path import Path
from shapely.geometry import LineString, Polygon, Point
import numpy as np

# Initialize the parameters
prev_positions = [None,None,None,None]
plt.ion()

class Robot:
    class Position: #create states of robot
        def __init__(self, x, y, theta):
            self.x = x
            self.y = y
            self.theta = theta #in degree
    
    class PrevPosition:
        def __init__(self, prev_x, prev_y, prev_theta):
            self.prev_x = prev_x
            self.prev_y = prev_y
            self.prev_theta = prev_theta #in degree

    class Input: #define the input of robot as velocity and steering angle
        def __init__(self, vel, omega):
            self.vel = vel
            self.omega= omega

    class Memory:
        def __init__(self,pos,d,bug2param):
            self.pos=pos
            self.d=d
            self.bug2param=bug2param #[init,nintyflag,m,c,first_time,come_from_wf]

    def __init__(self, position, prev_position, input, memory, planner, fsm="m2g", hitpoint = None, leavepoint = None):
        self.position = position
        self.prev_position = prev_position
        self.input = input
        self.fsm = fsm
        self.hitpoint = hitpoint
        self.leavepoint = leavepoint
        self.memory = memory
        self.planner = planner

class BumpSensor:
    def __init__(self, alpha, radius=10):
        self.radius = radius
        self.alpha = alpha
        self.world_x = 0  # World coordinates of the sensor
        self.world_y = 0
        self.activated = False  # Boolean value indicating whether the sensor is activated

    def update_position(self, robot_position):
        # Update the world position of the sensor based on the robot's position and orientation
        self.world_x  = robot_position.x + self.radius * np.cos(np.radians(robot_position.theta + self.alpha))
        self.world_y  = robot_position.y + self.radius * np.sin(np.radians(robot_position.theta + self.alpha))

    def update_status(self, obstacles):
        # Check if the sensor intersects with any obstacles
        self.activated = any(obstacle.contains_point((self.world_x, self.world_y),radius=1) for obstacle in obstacles)
        

##############################CALCULATE INPUT SEQUENCE##############################

def angle_difference(angle1, angle2):
    # Ensure angles are in the range [0, 360)
    angle1 = angle1 % 360
    angle2 = angle2 % 360

    diff = angle2 - angle1
    # Normalize the difference to the range [-180, 180)
    normalized_diff = (diff + 180) % 360 - 180
    return normalized_diff

def turn_robot(angle):
        integer=int(abs(angle)//9)
        remainder= abs(angle) % 9
        vel_seq = np.array([0]*integer)
        omega_seq = np.array([90]*integer)
        omega_seq=np.append(omega_seq,remainder*10)
        vel_seq=np.append(vel_seq,0)
        if angle < 0:
            omega_seq=-1*np.array(omega_seq)
        return vel_seq,omega_seq

def calculate_input(robot,bump_sensors,target_position):
    if robot.fsm == "m2g":
        angle2goal = np.degrees(np.arctan2(target_position[1]-robot.position.y,target_position[0]-robot.position.x))
        angleDiff=angle_difference(robot.position.theta,angle2goal)
        if abs(angleDiff) > 2:
            vel_seq, omega_seq = turn_robot(angleDiff)
        else:
            vel_seq = np.array([20])
            omega_seq = np.array([0])
    elif robot.fsm == "wf" or robot.fsm == "m2lp":
        if robot.planner == 1 and robot.fsm == "wf":
            bug_planners.writeMemory(robot,target_position)
        if any(sensor.activated for sensor in bump_sensors):
            if bump_sensors[2].activated:
                if bump_sensors[1].activated:
                    vel_seq,omega_seq = turn_robot(30)
                elif bump_sensors[3].activated:
                    vel_seq,omega_seq = turn_robot(120)
                else:
                    vel_seq,omega_seq = turn_robot(90)
            else:
                if bump_sensors[1].activated:
                    if bump_sensors[0].activated:
                        vel_seq,omega_seq = turn_robot(15)
                    else:
                        vel_seq,omega_seq = turn_robot(45)
                elif bump_sensors[3].activated:
                    if bump_sensors[0].activated:
                        vel_seq,omega_seq = turn_robot(165)
                    else:
                        vel_seq,omega_seq = turn_robot(135)
                else:
                    vel_seq=np.array([])
                    omega_seq=np.array([])
            vel_seq=np.append(vel_seq,20)
            omega_seq=np.append(omega_seq,0)
        else:
            vel_seq,omega_seq = turn_robot(-30)
            vel_seq=np.append(vel_seq,20)
            omega_seq=np.append(omega_seq,0)
    elif robot.fsm == "noSoln":
        raise KeyError("Path does not exist")
    return vel_seq,omega_seq

###################################################### MAIN ##################################################

# Function to update the position of robot according to bicycle kinematic model
def diffDrive_kinematic(robot):
    robot.prev_position.prev_x = robot.position.x
    robot.prev_position.prev_y = robot.position.y
    robot.prev_position.prev_theta = robot.position.theta

    theta = np.radians(robot.position.theta)

    robot.position.x+=robot.input.vel*np.cos(theta)*0.1
    robot.position.y+=robot.input.vel*np.sin(theta)*0.1
    robot.position.theta+=robot.input.omega*0.1


if __name__ == "__main__":
    fig, ax = plt.subplots(figsize=(20, 20))
    simTime = 0.1

    #obstacle_corners =[ [(25, 25), (25, 150), (150, 150), (150, 140), (40, 140), (40,25)],
    #                    [(75, 35), (125, 35), (125, 50), (75, 50)],
    #                    [(110,110),(75,110),(110,80)]
    #]

    # obstacle_corners =[[(60,50),(140,50),(140,60),(60,60),(60,50)]]
    
    obstacle_corners = [
    [(100, 100), (150, 100), (150, 150), (100, 150)]
    ]


    
    obstacles=[]
    for obs_corner in obstacle_corners:
        obs = Path(obs_corner)
        obstacles.append(obs)
    
    position      = Robot.Position(x=10, y=10, theta=0) #initial position. x,y cm theta degree
    prev_position = Robot.PrevPosition(prev_x=position.x,prev_y=position.y,prev_theta=position.theta)
    input_        = Robot.Input(vel=20,omega=0)  #initial control input. vel cm/s, omega deg/s
    memory        = Robot.Memory(pos=np.array([]),d=np.array([]),bug2param=np.array([1]))
    #create a robot 
    # !!!!! You can change the algorithm by changing the planner
    robot         = Robot(position=position,prev_position=prev_position,input=input_,memory=memory,planner=0) 
    target_position = np.array([120, 120])
    
    sensor = [
        BumpSensor(alpha= -90),
        BumpSensor(alpha= -45),
        BumpSensor(alpha=   0),
        BumpSensor(alpha=  45),
        BumpSensor(alpha=  90)]

    fig, ax = the2_simulation.plot_environment(fig,ax,obstacles,target_position)
    prev_positions=the2_simulation.simulation(fig, ax, robot, prev_positions)
    plt.pause(0.00001)

    NoSoln=0
    error = np.linalg.norm(target_position-np.array([robot.position.x,robot.position.y]))

    while(error>1 or NoSoln == 1):
        if robot.planner == 3:
            sensor.update_position(robot.position)
            sensor.generate_readings(obstacles)
            #print(sensor.readings)
        else:
            for bump_sensor in sensor:
                bump_sensor.update_position(robot.position)
                bump_sensor.update_status(obstacles)

        bug_planners.planner(robot,sensor,target_position)
        vel_seq,omega_seq = calculate_input(robot,sensor,target_position)

        for i, (vel, omega) in enumerate(zip(vel_seq, omega_seq)):
            robot.input.vel = vel
            robot.input.omega = omega
            diffDrive_kinematic(robot)
            prev_positions=the2_simulation.simulation(fig, ax, robot, prev_positions)
            plt.pause(0.00001)
        error = np.linalg.norm(target_position-np.array([robot.position.x,robot.position.y]))
    plt.pause(50)