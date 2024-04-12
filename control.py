#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, String
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import math


def is_close_to_waypoint(x, y, z,oz, ls,ly,lz,loz):
   
    if ly==26:
        threshold=0.2
    else:
        threshold = 0.1 # Adjust as needed
    distance = math.sqrt((x - ls)**2 + (y - ly)**2 + (z - lz)**2+(oz-loz)**2)
    return distance <= threshold

def string_callback(msg, result_array):
    rospy.loginfo("Received string message: {}".format(msg.data))

    # Split the string message into words using spaces as separators
    words = msg.data.split()

    if len(words) > 0:
        # Extract the first word as 'fruit'
        fruit = words[0]
        rospy.loginfo("Extracted fruit: {}".format(fruit))

        # Read numbers after the first space
        numbers = words[1:]

        # Convert the string numbers to floats and append to the result_array
        result_array.extend([int(num) for num in numbers])

        rospy.loginfo("Current array: {}".format(result_array))

    else:
        rospy.logwarn("Received an empty string message")


def publish_waypoint(publisher, x, y, z, qx, qy, qz, qw):
    pose_msg = PoseStamped()
    pose_msg.header = Header()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pose_msg.pose.position.z = z
    pose_msg.pose.orientation.x = qx
    pose_msg.pose.orientation.y = qy
    pose_msg.pose.orientation.z = qz
    pose_msg.pose.orientation.w = qw
    
    publisher.publish(pose_msg)
    rospy.loginfo("Published waypoint: x={}, y={}, z={}".format(x, y, z))

def current_position(msg):
    global current_x, current_y, current_z,current_orientation_z
    current_x = round(msg.pose.pose.position.x,2)
    current_y = round(msg.pose.pose.position.y,2)
    current_z = round(msg.pose.pose.position.z,2)
    current_orientation_z= msg.pose.pose.orientation.z
def last_published(msg):
    global last_x,last_z,last_y,last_published_orientation_z

    last_x = msg.pose.position.x
    last_y= msg.pose.position.y
    last_z=msg.pose.position.z
    last_published_orientation_z = msg.pose.orientation.z

def main():
    rospy.init_node('waypoint_publisher', anonymous=True)

    result_array = []

    # Set up subscribers
    rospy.Subscriber('/red/plants_beds', String, string_callback, result_array)

    rospy.Subscriber('/red/odometry', Odometry, current_position)
    rospy.Subscriber('/red/carrot/pose', PoseStamped, last_published)
    reached_publisher=rospy.Publisher('/reached',Int32,queue_size=10)
    waypoint_publisher = rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size=10)
    rospy.loginfo("Starting navigation")

    rospy.sleep(2)  # Wait for a moment before starting to make sure everything is set up
    cx,cy,cz=current_x,current_y,current_z
    # Wait for a non-empty message to be received
    while not result_array:  
        rospy.loginfo("Waiting for non-empty message to be received...")
        rospy.sleep(1)

    waypoints = [
        (1.35, 5.9, 1.5, 0.0, 0.0, 0.0, 1.0),  # 1
        (1.35, 5.9, 4.2, 0.0, 0.0, 0.0, 1.0),  # 2
        (1.35, 5.9, 7, 0.0, 0.0, 0.0, 1.0),  # 3
        (1.35, 13.4, 1.5, 0.0, 0.0, 0.0, 1.0),  # 4
        (1.35, 13.4, 4.2, 0.0, 0.0, 0.0, 1.0),  # 5
        (1.35, 13.4, 7, 0.0, 0.0, 0.0, 1.0),  # 6
        (1.35, 20.9, 1.5, 0.0, 0.0, 0.0, 1.0),  # 7
        (1.35, 20.9, 4.2, 0.0, 0.0, 0.0, 1.0),  # 8
        (1.35, 20.9, 7, 0.0, 0.0, 0.0, 1.0), # 9

        (7.35, 5.9, 1.5, 0.0, 0.0, 0.0, 1.0),  # 10
        (7.35, 5.9, 4.2, 0.0, 0.0, 0.0, 1.0),  # 11
        (7.35, 5.9, 7.0, 0.0, 0.0, 0.0, 1.0),  # 12
        (7.35, 13.4, 1.5, 0.0, 0.0, 0.0, 1.0),  # 13
        (7.35, 13.4, 4.2, 0.0, 0.0, 0.0, 1.0),  # 14
        (7.35, 13.4, 7.0, 0.0, 0.0, 0.0, 1.0),  # 15
        (7.35, 20.9, 1.5, 0.0, 0.0, 0.0, 1.0),  # 16
        (7.35, 20.9, 4.2, 0.0, 0.0, 0.0, 1.0),  # 17
        (7.35, 20.9, 7.0, 0.0, 0.0, 0.0, 1.0),  # 18


        (13.3, 6.1, 1.5, 0.0, 0.0, 0.0, 1.0),  # 19
        (13.3, 6.1, 4.2, 0.0, 0.0, 0.0, 1.0),  # 20
        (13.3, 6.1, 7.0, 0.0, 0.0, 0.0, 1.0),  # 21
        (13.3, 13.4, 1.5, 0.0, 0.0, 0.0, 1.0),  # 22
        (13.3, 13.4, 4.2, 0.0, 0.0, 0.0, 1.0),  # 23
        (13.3, 13.4, 7, 0.0, 0.0, 0.0, 1.0),  # 24
        (13.3, 20.9, 1.5, 0.0, 0.0, 0.0, 1.0),  # 25
        (13.3, 20.9, 4.2, 0.0, 0.0, 0.0, 1.0),  # 26
        (13.3, 20.9, 7, 0.0, 0.0, 0.0, 1.0), # 27

    

    ]
    waypoints_back=[
        (6.6, 5.9, 1.5, 0.0, 0.0, 180.0, 1.0),  # 1back
        (6.6, 5.9, 4.2, 0.0, 0.0, 180.0, 1.0),  # 2
        (6.6, 5.9, 7, 0.0, 0.0, 180.0, 1.0),  # 3
        (6.6, 13.4, 1.5, 0.0, 0.0, 180.0, 1.0),  # 4
        (6.6, 13.4, 4.2, 0.0, 0.0, 180.0, 1.0),  # 5
        (6.6, 13.4, 7, 0.0, 0.0, 180.0, 1.0),  # 6
        (6.6, 20.9, 1.5, 0.0, 0.0, 180.0, 1.0),  # 7
        (6.6, 20.9, 4.2, 0.0, 0.0, 180.0, 1.0),  # 8
        (6.6, 20.9, 7, 0.0, 0.0, 180.0, 1.0), # 9
        (12.6, 6.05, 1.5, 0.0, 0.0, 180.0, 1.0),  # 10back
        (12.6, 6.05, 4.2, 0.0, 0.0, 180.0, 1.0),  # 11
        (12.6, 6.05, 7.0, 0.0, 0.0, 180.0, 1.0),  # 12
        (12.6, 13.4, 1.5, 0.0, 0.0, 180.0, 1.0),  # 13
        (12.6, 13.4, 4.2, 0.0, 0.0, 180.0, 1.0),  # 14
        (12.6, 13.4, 7.0, 0.0, 0.0, 180.0, 1.0),  # 15
        (12.6, 20.9, 1.5, 0.0, 0.0, 180.0, 1.0),  # 16
        (12.6, 20.9, 4.2, 0.0, 0.0, 180.0, 1.0),  # 17
        (12.6, 20.9, 7.0, 0.0, 0.0, 180.0, 1.0),  # 18
        (18.8, 6.14, 1.5, 0.0, 0.0,180.0, 1.0),  # 19back
        (18.8,6.14,4.2, 0.0, 0.0, 180.0, 1.0),  # 20back
        (18.8, 6.17, 7.0, 0.0, 0.0, 180.0, 1.0),  # 21
        (18.8, 13.35, 1.5, 0.0, 0.0, 180.0, 1.0),  # 22
        (18.8, 13.35, 4.2, 0.0, 0.0, 180.0, 1.0),  # 23
        (18.8, 13.35, 7, 0.0, 0.0, 180.0, 1.0),  # 24
        (18.8, 20.9, 1.5, 0.0, 0.0, 180.0, 1.0),  # 25
        (18.8, 20.9, 4.2, 0.0, 0.0, 180.0, 1.0),  # 26
        (18.8, 20.9, 7.0, 0.0, 0.0, 180.0, 1.0) # 27
    ]


    new_array = [waypoints[i-1] for i in result_array]
    new_array2 = [waypoints_back[i-1] for i in result_array]
   
    final_array=new_array+new_array2
    final_array= sorted(final_array, key=lambda waypoint: waypoint[0])
    print("sorted array")
    for s in range(len(final_array)):
        print(s, final_array[s])

    flag1=0
    flag2=0
    flag3=0
    if(result_array[0]>9 and result_array[0]<19):
        print("coming")
        final_array.insert(0, (7.4, 1.5, 1.5, 0.0, 0.0, 0.0, 1.0))
        flag1=1
    
    if(result_array[0]>18):
        final_array.insert(0, (13.5, 1.5, 1.5, 0.0, 0.0, 0.0, 1.0))

    for k in range(len(final_array)):
            
          
        if final_array[k + 1][0] == 6.6 and flag1==0:
            final_array.insert(k+1, (1.35, 26, final_array[k+1][2], 0.0, 0.0, 0.0, 1.0))
            final_array.insert(k +2, (7.4, 26, final_array[k+1][2], 0.0, 0.0, 0.0, 1.0))
            flag1=1
        if final_array[k + 1][0] == 7.4 and flag1==0:
            final_array.insert(k+1, (1.35, 26, final_array[k+1][2], 0.0, 0.0, 0.0, 1.0))
            final_array.insert(k +2, (7.4, 26, final_array[k+1][2], 0.0, 0.0, 0.0, 1.0))
            flag1=1
        if final_array[k + 1][0] == 12.6  and flag2==0:
            final_array.insert(k+1, (6.6, 26.0, final_array[k+1][2], 0.0, 0.0, 0.0, 1.0))
            final_array.insert(k + 2, (13.5, 26.0, final_array[k+1][2], 0.0, 0.0, 0.0, 1.0))
            flag2=1
        if final_array[k + 1][0] == 13.3  and flag2==0:
            final_array.insert(k+1, (6.6, 26.0, final_array[k+1][2], 0.0, 0.0, 0.0, 1.0))
            final_array.insert(k + 2, (13.5, 26.0, final_array[k+1][2], 0.0, 0.0, 0.0, 1.0))
            flag2=1
        
    
    for k in range(len(final_array)):
        if final_array[k +1][0] ==  18.8 and flag3 == 0:
            
            final_array.insert(k+1, (12.6, 26.0, final_array[k+1][2], 0.0, 0.0, 0.0, 1.0))
            final_array.insert(k+2, (18.75, 26.0, final_array[k+1][2], 0.0, 0.0, 0.0, 1.0))
            flag3 = 1
    if(final_array[len(final_array)-1][0]==18.8):
        final_array.insert(len(final_array),(18.75,1.5,2,0.0,0.0,0.0,1.0))
        final_array.insert(len(final_array)+1,(cx,cy,cz,0.0,0.0,0.0,1.0))
    else:
        final_array.insert(len(final_array),(12.6,1.5,2.0,0.0,0.0,0.0,1.0))
        final_array.insert(len(final_array)+1,(cx,cy,cz,0.0,0.0,0.0,1.0))       
    
    for s in range(len(final_array)):
        print(s,final_array[s])


    rate = rospy.Rate(1)  

    # Publish the first waypoint directly without checking with the last waypoint
    publish_waypoint(waypoint_publisher, *final_array[0])

    rospy.sleep(7)
    rate = rospy.Rate(1)  # Adjust the rate as needed
    idx = 0  # Start with the second waypoint

    while not rospy.is_shutdown():
        # Check if the current position is close to the last published waypoint
        if is_close_to_waypoint(current_x, current_y, current_z,current_orientation_z, last_x,last_y,last_z,last_published_orientation_z):
            reached_msg = Int32()
            reached_msg.data = 1
            reached_publisher.publish(reached_msg)
            # Increment index to move to the next waypoint
            idx += 1
            # If there are no more waypoints, break out of the loop
            if idx >= len(final_array):
                reached_msg = Int32()
                reached_msg.data = 2
                reached_publisher.publish(reached_msg)
                break
            # Publish the next waypoint
            
            publish_waypoint(waypoint_publisher, *final_array[idx])
            
        else:
            reached_msg = Int32()
            reached_msg.data = 0
            reached_publisher.publish(reached_msg)
            
        
        
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
