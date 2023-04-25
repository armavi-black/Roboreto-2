#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Point

# Setup Variables, parameters and messages to be used (if required)

path_point = Point

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  t = rospy.get_time() - t_be
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Input")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)
    t_be = rospy.get_time()
    #Setup Publishers
    pathPub = rospy.Publisher('pose', Point, queue_size=10)

    print("The Set Point Genertor is Running")
    np = 0
    #Run the node
    while not rospy.is_shutdown():
        t = rospy.get_time() - t_be
        
        pointx = float(input("Inserte la coordenada x({}): ".format(np)))
        pointy = float(input("Inserte la coordenada y({}): ".format(np)))
        pathPub.publish(pointx,pointy,0)
        np += 1

        rate.sleep()
        