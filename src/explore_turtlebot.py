#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

turn = 0
speed = 0.8
turn_speed = 2
turn_rad = 1.5708
velocity_publisher = None

vel_msg = Twist()

    #Receiveing the user's input
    #print("Let's move your robot")

vel_msg.linear.x = 0
vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0
vel_msg.angular.z = 0


def laser_callback(laser):
    #print("reading laser scan data")
    global turn,turn_speed,velocity_publisher,vel_msg
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    #print(type(laser.ranges[-1]))
    if(float("inf") in laser.ranges): 
        print("count is " + str(laser.ranges[310:330].count(float("inf"))))
        if(laser.ranges[310:330].count(float("inf")) == 20):
            vel_msg.linear.x = 0.5
        else:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0.1
    print("publishing message")
    velocity_publisher.publish(vel_msg)



def move():
    # Starts a new node
    global turn,turn_speed,velocity_publisher,vel_msg
    rospy.init_node('robot_runner')
    velocity_publisher = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/hokuyo_laser', LaserScan, laser_callback )

    

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


        velocity_publisher.publish(vel_msg)
       
        
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass