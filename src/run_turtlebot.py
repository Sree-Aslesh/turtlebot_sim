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
    #print(type(laser.ranges[-1]))
    if(float("inf") in laser.ranges):
        laser_len = len(laser.ranges)
        # if(float("inf") in laser.ranges[laser_len/2-20:(laser_len/2)+20]):
        #     turn = 0
        #     print("inf on center")
        # elif(float("inf") in laser.ranges[0:laser_len/2-20]):
        #     turn = -1
        #     print("inf on left")
        # elif(float("inf") in laser.ranges[laser_len/2+20:]):
        #     turn = 1
        #     print("inf on right")
        # print(laser.ranges[530:550])
        for val in laser.ranges[90:110]:
            if val<0.4:
				turn = -1
				turn_rad = 0.1708
				vel_msg.linear.x = 0
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0
				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = 0

				vel_msg.linear.x = 0.8
				vel_msg.angular.z = turn_speed-1.7

				time_taken = turn_rad/turn_speed
				curr_time = rospy.get_time();
				while rospy.get_time() - curr_time<time_taken:
				    # print(cnt)
				    velocity_publisher.publish(vel_msg)
				    continue
				print("too close to right wall")

        for val in laser.ranges[530:550]:
            if val<0.4:
				turn = 1
				turn_rad = 0.1708

				vel_msg.linear.x = 0
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0
				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = 0

				vel_msg.linear.x = 0.8
				vel_msg.angular.z = -1*(turn_speed-1.7)

				time_taken = turn_rad/turn_speed
				curr_time = rospy.get_time();
				while rospy.get_time() - curr_time<time_taken:
				    # print(cnt)
				    velocity_publisher.publish(vel_msg)
				    continue

				print("too close to left wall")

				
					
                



        if(float("inf") in laser.ranges[90:110]):
            turn = 1
            print("right")
            turn_rad = 1.5708
        else:
            turn = 0
            print("left")
            turn_rad = 1.5708
    else:
        turn = -2
        print("inf not found")


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

        #Setting the current time for distance calculus

        #After the loop, stops the robot
        if(turn == 0):
            vel_msg.linear.x = speed
        elif(turn == 1):
            vel_msg.linear.x = 0.3
            vel_msg.angular.z = -1*turn_speed
            
            time_taken = turn_rad/turn_speed
            curr_time = rospy.get_time();
            while rospy.get_time() - curr_time<time_taken:
                # print(cnt)
                velocity_publisher.publish(vel_msg)
                continue
        elif(turn == -1):
			vel_msg.linear.x = 0.3
			vel_msg.angular.z = turn_speed

			time_taken = turn_rad/turn_speed
			curr_time = rospy.get_time();
			while rospy.get_time() - curr_time<time_taken:
			    # print(cnt)
			    velocity_publisher.publish(vel_msg)
			    continue
        
        
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass