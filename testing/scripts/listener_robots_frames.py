#!/usr/bin/env python3
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('node_listener_robots')

    listener = tf.TransformListener()
    rate = rospy.Rate(1.0)
    # esperar la transformacion de robot_3 respecto a world
    listener.waitForTransform('/world', '/robot_3', rospy.Time(), rospy.Duration(4.0))
    
    while not rospy.is_shutdown():
        try:
            # escuchar la transformacion
            (trans,rot) = listener.lookupTransform('/world', '/robot_3', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        quaternion = rot
        # transformar los angulos de cuaterniones a angulos normales
        rpy=tf.transformations.euler_from_quaternion(quaternion)
        print('------------------------------------------------')
        print('Transformaciòn entre robot_3 y world detectada!')
        print(f'Traslacion: (x={trans[0]}, y={trans[1]}, z={trans[2]})')
        print(f'Rotacion: (x={rpy[0]}, y={rpy[1]}, z={rpy[2]})')

        rate.sleep()