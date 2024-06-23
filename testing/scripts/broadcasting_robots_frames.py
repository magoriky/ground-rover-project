#!/usr/bin/env python3 
import roslib
import rospy
import tf
import time  
import math
import time

if __name__ == '__main__':
    rospy.init_node('node_broadcaster_robots')
    
    time.sleep(2)
    # crea un transform broadcaster
    transform_broadcaster = tf.TransformBroadcaster()

    while(not rospy.is_shutdown()):

        # transformar los angulos (en este caso son al azar) a cuaterniones
        rotation_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 1.57)

        # definir el vector de traslaciòn
        translation_vector = (3.0, 1.0, 0.0)

        # obtener el tiempo actual
        current_time = rospy.Time.now()
        
        # enviar la transformacion, recibe como parametros:
        # vector de traslacion, lista de cuaterniones, tiempo actual, frame hijo, frame padre
        transform_broadcaster.sendTransform(
            translation_vector, 
            rotation_quaternion,
            current_time, 
            "robot_3", "world") 
        
        time.sleep(0.5)

    rospy.spin()