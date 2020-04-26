#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, PointStamped, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import time
from dwa.srv import GoalRequest, GoalRequestRequest, GoalCompletion, GoalCompletionRequest
from warehouse_manager.srv import Robot_Task_Request, Robot_Task_Complete
import cv2

ns = rospy.get_namespace()
BOT_NAME = ns[7:len(ns)-1]
ODOM_TOPIC = ns + "base_pose_ground_truth"
LASER_TOPIC = ns + "base_scan"
PATH_TOPIC = ns + "nav_path"
CMD_VEL = ns + "cmd_vel"
INI_POS = ns + "initialpose"
GOAL_POS = ns + "move_base_simple/goal"
'''
print(BOT_NAME)
print(ODOM_TOPIC)
print(LASER_TOPIC)
print(CMD_VEL)
'''
class Config():
    # simulation parameters

    def __init__(self):
        # robot parameter
        #NOTE good params:
        #NOTE 0.55,0.1,1.0,1.6,3.2,0.15,0.05,0.1,1.7,2.4,0.1,3.2,0.18
        self.max_speed = 0.8 # [m/s]
        self.min_speed = 0.0  # [m/s]
        self.max_yawrate = 1.0  # [rad/s]
        self.max_accel = 1.5  # [m/ss]
        self.max_dyawrate = 3.2  # [rad/ss]
        self.v_reso = 0.5  # [m/s]
        self.yawrate_reso = 1.5  # [rad/s]
        self.dt = 0.5  # [s]
        self.predict_time = 1.5  # [s]
        self.to_goal_cost_gain = 2.5 #lower = detour
        self.speed_cost_gain = 0.1 #lower = faster
        self.obs_cost_gain = 8.0 #lower z= fearless
        self.robot_radius = 0.60  # [m]
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_assigned = False
        self.goalX = self.x  
        self.goalY = self.y

        ## Final goal if path is not found
        self.goalFinalX = 0.0
        self.goalFinalY = 0.0
        self.r = rospy.Rate(20)
        self.busy = False
        self.canSendCompletionRequest = False #can send completion request after bot starts traversing 
        self.job_start = 0.0;
        self.job_end = 0.0;
        self.stall_count = 0;
        self.path = []
        self.got_path = False
        self.distance_x = 0.0
        self.distance_y = 0.0
        self.path_received = False
        #self._sub = rospy.Subscriber('/map', OccupancyGrid, self.getMap, queue_size=1)
        self.aux_stall_count = 0
        self.mapimg = None
        self.mapreceived = False
        self.obstacle_space = []
        self.final_assigned = False

    # Callback for Odometry
    def assignOdomCoords(self, msg):
        # X- and Y- coords and pose of robot fed back into the robot config
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if not self.start_assigned:
            self.start_x = self.x
            self.start_y = self.y
            self.start_assigned = True
        rot_q = msg.pose.pose.orientation
        (roll,pitch,theta) = \
            euler_from_quaternion ([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
        self.th = theta
        if self.busy == True and msg.twist.twist.linear.x == 0.0 and msg.twist.twist.linear.y == 0.0:
            self.stall_count += 1
        else:
            self.stall_count = 0

    def astarPath(self, msg):
        if not self.path_received: 
            print("Path received: ", len(msg.poses), " by ", BOT_NAME)
            del self.path[:]
            for p in msg.poses:
                self.path.append([p.pose.position.x, p.pose.position.y])
            #print("Path ", self.path)
            if len(self.path) > 2:
                del self.path[0:2]
                self.goalX = self.path[0][0]
                self.goalY = self.path[0][1]
                #print("Path", self.path)
            #print("Goal: ", self.goalX, " ", self.goalY)
            self.path_received = True
            self.mapreceived = True
    # Callback for attaining goal co-ordinates from Rviz Publish Point
    '''
    def getMap(self, msg):
        #sprint("Mappa ", len(msg.data))
        height = msg.info.height
        width = msg.info.width

        self.mapimg = np.ones((height, width,3), dtype=np.float)
        self.mapreceived = True
        for i in range(0, len(msg.data)):
            if msg.data[i] == 100 or msg.data[i] == -1:
                self.mapimg[i/width, i%width] = (0,0,0)
                self.obstacle_space.append([int(i/width), int(i%width)])
    '''

    def goalCB(self,msg):
        self.goalX = msg.point.x
        self.goalY = msg.point.y

    def goalServiceRequeset(self, name):
        rospy.wait_for_service('/request_available_task')
        print("Service available for ", name)
        try:
            goalCoord = rospy.ServiceProxy('/request_available_task',Robot_Task_Request)
            response = goalCoord(name)
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def goalCompleteRequest(self, name, time, dist):
        rospy.wait_for_service('/report_task_complete')
        try:
            goalComplete = rospy.ServiceProxy('/report_task_complete',Robot_Task_Complete)
            response = goalComplete(name, time, dist)
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

class Obstacles():
    def __init__(self):
        # Set of coordinates of obstacles in view
        self.obst = set()

    # Custom range implementation to loop over LaserScan degrees with
    # a step and include the final degree
    def myRange(self,start,end,step):
        i = start
        while i < end:
            yield i
            i += step
        yield end

    # Callback for LaserScan
    def assignObs(self, msg, config):

        deg = len(msg.ranges)   # Number of degrees - varies in Sim vs real world
        #print("Laser degree length {}".format(deg))
        self.obst = set()   # reset the obstacle set to only keep visible objects

        maxAngle = 60 #270
        scanSkip = 2
        anglePerSlot = (float(maxAngle) / deg) * scanSkip
        angleCount = 0
        angleValuePos = 0
        angleValueNeg = 0
        for angle in self.myRange(0,deg-1,scanSkip):
            distance = msg.ranges[angle]
            
            if(angleCount < (deg / (2*scanSkip))):
                # print("In negative angle zone")
                angleValueNeg += (anglePerSlot)  
                scanTheta = (angleValueNeg - 30) * math.pi/180.0
                #scanTheta = (angleValueNeg - 30) * math.pi/180.0    

            elif(angleCount>(deg / (2*scanSkip))):
                # print("In positive angle zone")
                angleValuePos += anglePerSlot
                scanTheta = angleValuePos * math.pi/180.0
            # only record obstacles that are within 4 metres away

            else:
                scanTheta = 0

            angleCount += 1

            if (distance < 6.5):
                # angle of obstacle wrt robot
                # angle/2.844 is to normalise the 512 degrees in real world
                # for simulation in Gazebo, use angle/4.0
                # laser from 0 to 180
                # scanTheta = (angle/2.844 + deg*(-180.0/deg)+90.0) *math.pi/180.0
                # # angle of obstacle wrt global frame
                # if config.th < 0:
                #     objTheta = config.th + scanTheta
                # else:
                #     objTheta = config.th - scanTheta
                # print("The scan theta is {}".format(scanTheta * 180 / math.pi))
                # print("The angel count is {}".format(angleCount))
                # print("Angle per slot is {}".format(anglePerSlot))
                objTheta =  scanTheta + config.th
                # # back quadrant negative X negative Y
                # if (objTheta < -math.pi):
                #     # e.g -405 degrees >> 135 degrees
                #     objTheta = objTheta + 1.5*math.pi
                # # back quadrant negative X positve Y
                # elif (objTheta > math.pi):
                #     objTheta = objTheta - 1.5*math.pi
                #print("The scan theta is {}".format(objTheta))
                #print("The angle is {}".format(objTheta * 180 / 3.14))
                # round coords to nearest 0.125m
                obsX = round((config.x + (distance * math.cos(abs(objTheta))))*8)/8
                # determine direction of Y coord
                # if (objTheta < 0): # uncomment and comment line below for Gazebo simulation
                if (objTheta < 0):
                    obsY = round((config.y - (distance * math.sin(abs(objTheta))))*8)/8
                else:
                    obsY = round((config.y + (distance * math.sin(abs(objTheta))))*8)/8

                # print("Robot's current location {} {}".format(config.x, config.y))
                # print("Obstacle's current location {} {}".format(obsX, obsY))
                # print("Current yaw of the robot {}".format(config.th))

                # add coords to set so as to only take unique obstacles
                self.obst.add((obsX,obsY))
                # print("The obstacle space is {}".format(self.obst))
                #print self.obst
        # print("The total angle count is {}".format(angleCount  ))

# Model to determine the expected position of the robot after moving along trajectory
def motion(x, u, dt):
    # motion model
    # x = [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    
    x[3] = u[0]
    x[4] = u[1]

    return x

# Determine the dynamic window from robot configurations
def calc_dynamic_window(x, config):

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [vmin, vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw

# Calculate a trajectory sampled across a prediction time
def calc_trajectory(xinit, v, y, config):

    x = np.array(xinit)
    traj = np.array(x)  # many motion models stored per trajectory
    time = 0
    while time <= config.predict_time:
        # store each motion model along a trajectory
        x = motion(x, [v, y], config.dt)
        traj = np.vstack((traj, x))
        time += config.dt # next sample

    return traj

# Calculate trajectory, costings, and return velocities to apply to robot
def calc_final_input(x, u, dw, config, ob):

    xinit = x[:]
    min_cost = 10000.0
    min_u = u
    min_u[0] = 0.0

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_reso):
        for w in np.arange(dw[2], dw[3], config.yawrate_reso):
            traj = calc_trajectory(xinit, v, w, config)

            # calc costs with weighted gains
            to_goal_cost = calc_to_goal_cost(traj, config) * config.to_goal_cost_gain
            speed_cost = config.speed_cost_gain * \
                (config.max_speed - traj[-1, 3])

            ob_cost = calc_obstacle_cost(traj, ob, config) * config.obs_cost_gain

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v, w]
    return min_u

# Calculate obstacle cost inf: collision, 0:free
def calc_obstacle_cost(traj, ob, config):
    skip_n = 2
    minr = float("inf")

    # Loop through every obstacle in set and calc Pythagorean distance
    # Use robot radius to determine if collision
    for ii in range(0, len(traj[:, 1]), skip_n):
        for i in ob.copy():
            ox = i[0]
            oy = i[1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx**2 + dy**2)

            if r <= config.robot_radius:
                return float("Inf")  # collision

            if minr >= r:
                minr = r

    return 1.0 / minr

# Calculate goal cost via Pythagorean distance to robot
def calc_to_goal_cost(traj, config):
    # If-Statements to determine negative vs positive goal/trajectory position
    # traj[-1,0] is the last predicted X coord position on the trajectory
    if (config.goalX >= 0 and traj[-1,0] < 0):
        dx = config.goalX - traj[-1,0]
    elif (config.goalX < 0 and traj[-1,0] >= 0):
        dx = traj[-1,0] - config.goalX
    else:
        dx = abs(config.goalX - traj[-1,0])
    # traj[-1,1] is the last predicted Y coord position on the trajectory
    if (config.goalY >= 0 and traj[-1,1] < 0):
        dy = config.goalY - traj[-1,1]
    elif (config.goalY < 0 and traj[-1,1] >= 0):
        dy = traj[-1,1] - config.goalY
    else:
        dy = abs(config.goalY - traj[-1,1])

    cost = math.sqrt(dx**2 + dy**2)
    return cost

# Begin DWA calculations
def dwa_control(x, u, config, ob):
    # Dynamic Window control

    dw = calc_dynamic_window(x, config)

    u = calc_final_input(x, u, dw, config, ob)

    return u

# Determine whether the robot has reached its goal
def atGoal(config, x):
    # check at goal
    if math.sqrt((x[0] - config.goalX)**2 + (x[1] - config.goalY)**2) \
        <= config.robot_radius * 1.25:
        return True
    return False


def main():
    #print(__file__ + " start!!")
    # robot specification
    config = Config()
    # position of obstacles
    obs = Obstacles()
    subOdom = rospy.Subscriber(ODOM_TOPIC, Odometry, config.assignOdomCoords)
    subLaser = rospy.Subscriber(LASER_TOPIC, LaserScan, obs.assignObs, config)
    subPath = rospy.Subscriber(PATH_TOPIC, Path, config.astarPath)
    #globPose = rospy.Subscriber("/robot_0/base_pose_ground_truth",)
    #subGoal = rospy.Subscriber("/clicked_point", PointStamped, config.goalCB)
    pub = rospy.Publisher(CMD_VEL, Twist, queue_size=1)
    pub_init = rospy.Publisher(INI_POS, PoseWithCovarianceStamped, queue_size=1)
    pub_goal = rospy.Publisher(GOAL_POS, PoseStamped, queue_size=1)
    speed = Twist()
    pose_stamped = PoseWithCovarianceStamped()
    goal_stamped = PoseStamped()
    # initial state [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    x = np.array([config.x, config.y, config.th, 0.0, 0.0])
    # initial linear and angular velocities
    u = np.array([0.0, 0.0])
    # runs until terminated externally
    while not rospy.is_shutdown():
        #print("Aux", config.aux_stall_count)
        if (atGoal(config,x) == False):
            #start_time = time.time()
            u = dwa_control(x, u, config, obs.obst)
            #print("Time to calculate a single pass through DWA {}".format(time.time() - start_time))
            x[0] = config.x
            x[1] = config.y
            x[2] = config.th
            x[3] = u[0]
            x[4] = u[1]
            speed.linear.x = x[3]
            speed.angular.z = x[4]
        
            pose_stamped.header.stamp = rospy.Time.now() 
            pose_stamped.pose.pose.position.x = config.x
            pose_stamped.pose.pose.position.y = config.y 
            pose_stamped.pose.pose.position.z = 0.0


            config.aux_stall_count += 1
            if config.aux_stall_count > 800:
                if len(config.path) > 2:
                    del config.path[0:2]
                    config.goalX = config.path[0][0]
                    config.goalY = config.path[0][1]
                else:
                    config.goalX = config.goalFinalX
                    config.goalY = config.goalFinalY
                    config.final_assigned = True

                #print("GoalAdj: ", config.goalX, " ", config.goalY)
                if not config.final_assigned:
                    config.aux_stall_count = 0

            if config.aux_stall_count > 10000:
                # situation is hopeless
                print("it's hopeless")
                config.aux_stall_count = 0
                config.start_assigned = False
                config.final_assigned = False
                config.busy = False
                config.job_end = time.time()
                if config.canSendCompletionRequest:
                    final_dist = np.sqrt((config.goalX - config.start_x)**2 + (config.goalY - config.start_y)**2)
                    goalComplete = config.goalCompleteRequest(BOT_NAME,config.job_end - config.job_start, final_dist)
                    config.canSendCompletionRequest = False


        else:
            # Waypoint reached
            config.aux_stall_count = 0
            if len(config.path) != 0:
                config.goalX = config.path[0][0]
                config.goalY = config.path[0][1]
                #print("Goalllll: ", config.goalX, " ", config.goalY)
                if len(config.path) < 2:
                    config.goalX = config.goalFinalX
                    config.goalY = config.goalFinalY
                del config.path[0:2]
                #print("GoalWaypoint: ", config.goalX, " ", config.goalY)

            else:
                # if at goal then stay there until new goal published
                config.job_end = time.time()
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                config.start_assigned = False
                config.final_assigned = False
                config.busy = False
                if config.canSendCompletionRequest:
                    final_dist = np.sqrt((config.goalX - config.start_x)**2 + (config.goalY - config.start_y)**2)
                    goalComplete = config.goalCompleteRequest(BOT_NAME,config.job_end - config.job_start, final_dist)
                    config.canSendCompletionRequest = False

        #print(config.x, " " , config.y)
        if not config.busy:
            pub.publish(speed)
            #time.sleep(5)
            #print(BOT_NAME, " Sent service request")
            goalCoord = config.goalServiceRequeset(BOT_NAME)
            if goalCoord.task_available:

                # send goal and current position to the Astar planner
                #print("publishing to ", INI_POS)
                #print("publishing goal to", GOAL_POS)


                config.goalX = int(goalCoord.x * 1) #- config.orig_x
                config.goalY = int(goalCoord.y * 1)#- config.orig_y
                config.busy = True

                config.goalFinalX = config.goalX
                config.goalFinalY = config.goalY
                
                goal_stamped.header.stamp = rospy.Time.now()
                goal_stamped.pose.position.x = config.goalFinalX #config.goalX 
                goal_stamped.pose.position.y = config.goalFinalY #config.goalY
                goal_stamped.pose.position.z = 0.0
                

                #config.goalReached = False
                config.canSendCompletionRequest = True
                config.job_start = time.time()
                config.path_received = False
                config.mapreceived = True
                print("Goal x:", config.goalX, "Goal y:", config.goalY)
            else:
                disp = BOT_NAME + " is Done"
                print(disp);
                while True:
                    continue;
        

        #if config.path_received:
        pub.publish(speed)
        #print("StartX: ", config.start_x, "StartY: ", config.start_y)
        
        if not config.path_received:
            pub_init.publish(pose_stamped)
            pub_goal.publish(goal_stamped)

        #print("X: ", config.x, " Y: ", config.y)
        config.r.sleep()


if __name__ == '__main__':
    rospy.init_node('dwa')
    main()
