#!/usr/bin/env python

import rospy
import tf
import heapq
import math



from geometry_msgs.msg import Twist,PoseStamped,Pose,Point,PointStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import MapMetaData,OccupancyGrid,Path
from visualization_msgs.msg import Marker



turn = 0
speed = 0.8
turn_speed = 2
turn_rad = 1.5708
velocity_publisher = None
globalPathPublisher = None
localPathPublisher = None
markerPublisher = None

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
    return (row+col*4000)


def heuristicDistance(x1,y1,x2,y2):
    return (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1)

def inBounds(x,y):
    if(getIndex(x,y)>=0 and getIndex(x,y)<4000*4000):
        return True
    return False

def visualizeTempPath(pathList):
    val = Marker()
    val.header.frame_id = "/map"
    # val.header.stamp = rospy.Time.now()
    val.type = val.POINTS
    val.lifetime = rospy.Duration(0)
    val.action = val.ADD
    val.scale.x = 0.3
    val.scale.y = 0.3
    val.color.a = 1.0
    val.color.r = 0.5
    val.color.g = 0.5 
    val.color.b = 0.5   
    val.pose.orientation.w = 1.0
    for temp in pathList:
        point = Point()
        point.x = temp[0]*map_metadata.resolution+map_metadata.origin.position.x
        point.y = temp[1]*map_metadata.resolution+map_metadata.origin.position.y
        val.points.append(point)

    markerPublisher.publish(val)
    






def shortestPath(curr_x,curr_y,des_x,des_y,map,pathList,visited):
    # print("[shortestPath] - checking for " + str(curr_x) + " " + str(curr_y)+ " index given is " +str(getIndex(curr_x,curr_y)))
    # print("[shortestPath] - destination is " + str(des_x) + " " + str(des_y)+ " index given is " +str(getIndex(curr_x,curr_y)))
    # print("length of map is " + str(len(map.data)))
    if(getIndex(curr_x,curr_y)<0 or visited[getIndex(curr_x,curr_y)] == 1 or map.data[getIndex(curr_x,curr_y)]==100):
        return []

    
    visited[getIndex(curr_x,curr_y)] = 1
    pathList.append([curr_x,curr_y])

    # visualizeTempPath(pathList)
    # rospy.sleep(100)
    if(curr_x==des_x and curr_y == des_y):
        return pathList

    
    # print(map.data[getIndex(curr_x+1,curr_y)],map.data[getIndex(curr_x-1,curr_y)],map.data[getIndex(curr_x,curr_y+1)],map.data[getIndex(curr_x,curr_y-1)])
    tempList = []
    if(inBounds(curr_x+1,curr_y) and map.data[getIndex(curr_x+1,curr_y)]!=100):
        tempList.append([heuristicDistance(curr_x+1,curr_y,des_x,des_y),curr_x+1,curr_y])
    if(inBounds(curr_x-1,curr_y) and map.data[getIndex(curr_x-1,curr_y)]!=100):
        tempList.append([heuristicDistance(curr_x-1,curr_y,des_x,des_y),curr_x-1,curr_y])
    if(inBounds(curr_x,curr_y+1) and map.data[getIndex(curr_x,curr_y+1)]!=100):
        tempList.append([heuristicDistance(curr_x,curr_y+1,des_x,des_y),curr_x,curr_y+1])
    if(inBounds(curr_x,curr_y-1) and map.data[getIndex(curr_x,curr_y-1)]!=100):
        tempList.append([heuristicDistance(curr_x,curr_y-1,des_x,des_y),curr_x,curr_y-1])
    heapq.heapify(tempList)

    for val in tempList:
        # print(val[0],val[1],val[2],map.data[getIndex(val[1],val[2])])
        temp_data = shortestPath(val[1],val[2],des_x,des_y,map,pathList,visited)

        if(temp_data!=[]):
            return temp_data

    #pathList = pathList[:-1]
    return []


def seq(start,stop,step):
    temp =[]
    while(start<=stop):
        temp.append(start)
        start = start + step
    return temp

def getScore(localpath,globalpath):
    
    score = 0
    for val in localpath.poses:
        dist = 10000
        for val2 in globalpath.poses:
            tempdist = heuristicDistance(val.pose.position.x,val.pose.position.y,val2.pose.position.x,val2.pose.position.y)
            if(tempdist<dist):
                dist =tempdist
        score = score + dist
    return score


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
    
    finalpath  = None
    score = 1000000000
    finalang = 0
    angularVelocityRange = rospy.get_param("angularVelocityRange")
    timeRange = rospy.get_param("timeRange")
    for angVel in seq(angularVelocityRange["start"],angularVelocityRange["end"],angularVelocityRange["step"]):
        localPath = Path()
        localPath.header.frame_id="map"
        xval = 0
        yval = 0
        theta = 0
        for secs in seq(timeRange["start"],timeRange["end"],timeRange["step"]):
            # xval = xval + linearVel*secs + linearVel*math.cos(angVel*secs)
            # yval = yval + linearVel*math.sin(angVel*secs)
            wheelRadius = rospy.get_param("wheelRadius")
            dr = 2*3.14*wheelRadius*(linearVel+angVel)*secs
            dl = 2*3.14*wheelRadius*(linearVel-angVel)*secs
            dc = (dr+dl)/2
            xval = xval + dc*math.cos(theta)
            yval = yval + dc*math.sin(theta)
            theta = theta + (dr-dl)/2
            pose_stamped = PoseStamped()
            pose_stamped.pose = createPose(x=xval,y=yval,frame="turtlebot/create::base")
            pose_stamped.header.frame_id = "turtlebot/create::base"
            pose_stamped.header.stamp = rospy.Time.now()
            listener.waitForTransform("map","turtlebot/create::base",rospy.Time.now(), rospy.Duration(2.0))
            posestamp_temp = listener.transformPose("map",pose_stamped.pose,)
            localPath.poses.append(posestamp_temp)

        localPathPublisher.publish(localPath)
        if(getScore(localPath,path)<score):
            score = getScore(localPath,path)
            finalpath = localPath
            finalang = angVel
    localPathPublisher.publish(finalpath)
    vel_msg.linear.x = linearVel
    vel_msg.angular.z = finalang
    velocity_publisher.publish(vel_msg)
    rospy.sleep(0.05)

# def getPoint(point):
#     pointx = point.point.x
#     pointy = point.point.y
#     grid_curr_x = int((pointx - map_metadata.origin.position.x) / 0.05)
#     grid_curr_y = int((pointy - map_metadata.origin.position.y) / 0.05)
#     curr_x = grid_curr_x
#     curr_y = grid_curr_y
#     map = map_holder
#     # print(map.data[getIndex(curr_x+1,curr_y)],map.data[getIndex(curr_x-1,curr_y)],map.data[getIndex(curr_x,curr_y+1)],map.data[getIndex(curr_x,curr_y-1)])

def getMap(map):
    print("[getMap] - Received map")
    # file1 = open("/home/sree/MyFile.txt","w+") 
    # if(map==None):
    #     return
    # cnt =0
    # for val in map.data:
    #     if (val > 0):
    #         cnt= cnt+1
    # print(cnt)
    #     # file1.write(str(val) + " ")
    # print("----------recorded map ----------------")
    # file1.close()
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

    grid_curr_x = int((trans[0] - map_metadata.origin.position.x) / 0.05)
    grid_curr_y = int((trans[1] - map_metadata.origin.position.y) / 0.05)

    # grid_curr_x = int((pointx - map_metadata.origin.position.x) / 0.05)
    # grid_curr_y = int((pointy - map_metadata.origin.position.y) / 0.05)

    destination_x = goal.pose.position.x
    destination_y = goal.pose.position.y

    grid_des_x = int((destination_x- map_metadata.origin.position.x)/map_metadata.resolution)
    grid_des_y = int((destination_y- map_metadata.origin.position.y)/map_metadata.resolution)

    visited = [0]*(4000*4000)
    pathList = []

    # curr_x = grid_curr_x
    # curr_y = grid_curr_y
    # print(map.data[getIndex(curr_x+1,curr_y)],map.data[getIndex(curr_x-1,curr_y)],map.data[getIndex(curr_x,curr_y+1)],map.data[getIndex(curr_x,curr_y-1)])

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
        
        
        velocity_publisher.publish(vel_msg)



def move():
    # Starts a new node
    global turn,turn_speed,velocity_publisher,vel_msg,listener,globalPathPublisher,localPathPublisher,markerPublisher
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

    markerPublisher = rospy.Publisher('/marker',Marker,queue_size=10)

    rospy.Subscriber('/hokuyo_laser', LaserScan, laser_callback )
    print("[move] - Initialized subscriber for topic '/hokuyo_laser'")

    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goalPose )
    print("[move] - Initialized subscriber for topic '/move_base_simple/goal'")
    
    rospy.Subscriber('/map_metadata', MapMetaData, mapMetaData )
    print("[move] - Initialized subscriber for topic '/map_metadata'")

    rospy.Subscriber('/map', OccupancyGrid, getMap )
    print("[move] - Initialized subscriber for topic '/map'")

    # rospy.Subscriber('/clicked_point', PointStamped, getPoint )


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


        rate.sleep()


if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass