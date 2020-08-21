#!/usr/bin/env python

import rospy
import tf
import heapq
import math



from geometry_msgs.msg import Twist,PoseStamped,Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import MapMetaData,OccupancyGrid,Path



turn = 0
speed = 0.8
turn_speed = 2
turn_rad = 1.5708
velocity_publisher = None
globalPathPublisher = None
localPathPublisher = None

vel_msg = Twist()

vel_msg.linear.x = 0
vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0
vel_msg.angular.z = 0


goal = PoseStamped()
poseReceived = False
listener = None
goal = None
map_holder = None


map_metadata = MapMetaData()

def mapMetaData(mapMetaData_input):
    global map_metadata
    map_metadata = mapMetaData_input
    print(map_metadata.resolution)
    print("[mapMetaData] - Received map metadata")
    

def createPose(x=0,y=0,z=0,roll=0,pitch=0,yaw=0,frame = "turtlebot/odom"):
    pose_stamped = PoseStamped()
    pose_stamped.pose.position.x = x
    pose_stamped.pose.position.y = y
    pose_stamped.pose.position.z = z
    pose_stamped.header.frame_id = frame
    pose_stamped.header.stamp = rospy.Time.now()
    return pose_stamped


def getIndex(row,col):
    return (row*4000+col)


def heuristicDistance(x1,y1,x2,y2):
    return (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1)

def inBounds(x,y):
    if(getIndex(x,y)>=0 and getIndex(x,y)<4000*4000):
        return True
    return False

def shortestPath(curr_x,curr_y,des_x,des_y,map,pathList,visited):
    # print("[shortestPath] - checking for " + str(curr_x) + " " + str(curr_y)+ " index given is " +str(getIndex(curr_x,curr_y)))
    # print("[shortestPath] - destination is " + str(des_x) + " " + str(des_y)+ " index given is " +str(getIndex(curr_x,curr_y)))
    # print("length of map is " + str(len(map.data)))
    if(getIndex(curr_x,curr_y)<0 or visited[getIndex(curr_x,curr_y)] == 1 or map.data[getIndex(curr_x,curr_y)]>60):
        return []

    if(curr_x==des_x and curr_y == des_y):
        return pathList

    visited[getIndex(curr_x,curr_y)] = 1
    pathList.append([curr_x,curr_y])

    tempList = []
    if(inBounds(curr_x+1,curr_y)):
        tempList.append([heuristicDistance(curr_x+1,curr_y,des_x,des_y),curr_x+1,curr_y])
    if(inBounds(curr_x-1,curr_y)):
        tempList.append([heuristicDistance(curr_x-1,curr_y,des_x,des_y),curr_x-1,curr_y])
    if(inBounds(curr_x,curr_y+1)):
        tempList.append([heuristicDistance(curr_x,curr_y+1,des_x,des_y),curr_x,curr_y+1])
    if(inBounds(curr_x,curr_y-1)):
        tempList.append([heuristicDistance(curr_x,curr_y-1,des_x,des_y),curr_x,curr_y-1])
    heapq.heapify(tempList)
    for val in tempList:
        temp_data = shortestPath(val[1],val[2],des_x,des_y,map,pathList,visited)
        if(temp_data!=[]):
            return temp_data

    pathList = pathList[:-1]
    return []


def seq(start,stop,step):
    temp =[]
    while(start<=stop):
        temp.append(start)
        start = start + step
    return temp

def runTurtle(path):
    global vel_msg

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    (trans,rot) = listener.lookupTransform('map', 'turtlebot/create::base', rospy.Time(0))
    linearVel = 0.3

    #local path planning approach (DWA)
    

    for angVel in seq(-1,1,0.2):
        localPath = Path()
        localPath.header.frame_id="map"
        xval = 0
        yval = 0
        for secs in seq(0,1,0.2):
            xval = xval + linearVel*secs + linearVel*math.cos(angVel*secs)
            yval = yval + linearVel*math.sin(angVel*secs)
            pose_stamped = PoseStamped()
            pose_stamped.pose = createPose(x=xval,y=yval,frame="turtlebot/create::base")
            pose_stamped.header.frame_id = "turtlebot/create::base"
            pose_stamped.header.stamp = rospy.Time.now()
            listener.waitForTransform("map","turtlebot/create::base",rospy.Time.now(), rospy.Duration(2.0))
            posestamp_temp = listener.transformPose("map",pose_stamped.pose,)
            localPath.poses.append(posestamp_temp)
        localPathPublisher.publish(localPath)
        rospy.sleep(0.1)




def getMap(map):
    print("[getMap] - Received map")
    global map_holder
    map_holder = map
    if(poseReceived!=True):
        return
    
    (trans,rot) = listener.lookupTransform('map', 'turtlebot/create::base', rospy.Time(0))

    # path = Path()
    # path.header.frame_id="map"
    # for cnt in range(0,20):
    #     path.poses.append(createPose(x=trans[0],y=trans[1]+cnt))

    # globalPathPublisher.publish(path)

    grid_curr_x = int((trans[0] - map_metadata.origin.position.x) / float(map_metadata.resolution))
    grid_curr_y = int((trans[1] - map_metadata.origin.position.y) / float(map_metadata.resolution))
    
    destination_x = goal.pose.position.x
    destination_y = goal.pose.position.y

    grid_des_x = int((destination_x- map_metadata.origin.position.x)/map_metadata.resolution)
    grid_des_y = int((destination_y- map_metadata.origin.position.y)/map_metadata.resolution)

    visited = [0]*(4000*4000)
    pathList = []
    pathList = shortestPath(grid_curr_x,grid_curr_y,grid_des_x,grid_des_y,map,pathList,visited)
    # print(pathList)

    path = Path()
    path.header.frame_id="map"
    for i,val in enumerate(pathList):
        path.poses.append(createPose(x=val[0]*map_metadata.resolution+map_metadata.origin.position.x,y=val[1]*map_metadata.resolution+map_metadata.origin.position.y))
    globalPathPublisher.publish(path)
    runTurtle(path)
    


def goalPose(goal_input):
    global poseReceived,goal
    print("[goalPose] - Received goal location .. Starting SLAM")
    goal = goal_input
    poseReceived = True
    getMap(map_holder)
    # print(goal.pose.position.x)


def laser_callback(laser):
    # print("reading laser scan data")
    global turn,turn_speed,velocity_publisher,vel_msg
    # print("in laser call back" + str(poseReceived))
    if(poseReceived==True):
        
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        #print(type(laser.ranges[-1]))
        # if(float("inf") in laser.ranges): 
        #     # print("count is " + str(laser.ranges[310:330].count(float("inf"))))
        #     if(laser.ranges[310:330].count(float("inf")) == 20):
        #         vel_msg.linear.x = 0.5
        #     else:
        #         vel_msg.linear.x = 0
        #         vel_msg.angular.z = 0.1
        # print("publishing message")
        velocity_publisher.publish(vel_msg)



def move():
    # Starts a new node
    global turn,turn_speed,velocity_publisher,vel_msg,listener,globalPathPublisher,localPathPublisher
    rospy.init_node('robot_runner')
    print("[move] - Initialized node 'robot_runner'")

    listener = tf.TransformListener()
    rospy.sleep(5)

    velocity_publisher = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=10)
    print("[move] - Initialized publisher for topic '/turtlebot/cmd_vel'")

    globalPathPublisher = rospy.Publisher('/globalPath',Path,queue_size=10)
    print("[move] - Initialized publisher for topic '/globalPath'")

    localPathPublisher = rospy.Publisher('/localPath',Path,queue_size=10)
    print("[move] - Initialized publisher for topic '/localPath'")

    rospy.Subscriber('/hokuyo_laser', LaserScan, laser_callback )
    print("[move] - Initialized subscriber for topic '/hokuyo_laser'")

    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goalPose )
    print("[move] - Initialized subscriber for topic '/move_base_simple/goal'")
    
    rospy.Subscriber('/map_metadata', MapMetaData, mapMetaData )
    print("[move] - Initialized subscriber for topic '/map_metadata'")

    rospy.Subscriber('/map', OccupancyGrid, getMap )
    print("[move] - Initialized subscriber for topic '/map'")




    #Receiveing the user's input
    #print("Let's move your robot")

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    rate = rospy.Rate(20)



    


    while not rospy.is_shutdown():

        #print("publishing velocity message")

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0


        rate.sleep()


if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass