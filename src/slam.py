#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist,PoseStamped,Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import MapMetaData,OccupancyGrid,Path



turn = 0
speed = 0.8
turn_speed = 2
turn_rad = 1.5708
velocity_publisher = None
globalPathPublisher = None

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


map_metadata = MapMetaData()

def mapMetaData(mapMetaData_input):
    global map_metadata
    map_metadata = mapMetaData_input
    print(map_metadata.resolution)
    print("[mapMetaData] - Received map metadata")
    

def createPose(x=0,y=0,z=0,roll=0,pitch=0,yaw=0):
    pose_stamped = PoseStamped()
    pose_stamped.pose.position.x = x
    pose_stamped.pose.position.y = y
    pose_stamped.pose.position.z = z
    pose_stamped.header.frame_id = "turtlebot/odom"
    pose_stamped.header.stamp = rospy.Time.now()
    return pose_stamped

def getMap(map):
    print("[getMap] - Received map")

    
    (trans,rot) = listener.lookupTransform('map', 'turtlebot/create::base', rospy.Time(0))
    print(trans)
    print(rot)

    path = Path()
    path.header.frame_id="map"
    for cnt in range(0,20):
        path.poses.append(createPose(x=trans[0],y=trans[1]+cnt))

    globalPathPublisher.publish(path)

    grid_x = int((trans[0] - map_metadata.origin.position.x) / float(map_metadata.resolution))
    grid_y = int((trans[1] - map_metadata.origin.position.y) / float(map_metadata.resolution))
    print([grid_x,grid_y])

    


def goalPose(goal_input):
    global poseReceived
    print("[goalPose] - Received goal location .. Starting SLAM")
    goal = goal_input
    poseReceived = True
    print(goal.pose.position.x)


def laser_callback(laser):
    #print("reading laser scan data")
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
        if(float("inf") in laser.ranges): 
            # print("count is " + str(laser.ranges[310:330].count(float("inf"))))
            if(laser.ranges[310:330].count(float("inf")) == 20):
                vel_msg.linear.x = 0.5
            else:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0.1
        # print("publishing message")
        velocity_publisher.publish(vel_msg)



def move():
    # Starts a new node
    global turn,turn_speed,velocity_publisher,vel_msg,listener,globalPathPublisher
    rospy.init_node('robot_runner')
    print("[move] - Initialized node 'robot_runner'")

    listener = tf.TransformListener()
    rospy.sleep(10)

    velocity_publisher = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=10)
    print("[move] - Initialized publisher for topic '/turtlebot/cmd_vel'")

    globalPathPublisher = rospy.Publisher('/globalPath',Path,queue_size=10)
    print("[move] - Initialized publisher for topic '/globalPath'")

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