#!/usr/bin/env python

import rospy
from rover_control.srv import RoverControl, RoverControlResponse
from geometry_msgs.msg import Twist

class RoverControlServer:
    def __init__(self):
        rospy.init_node('rover_control_server')
        self.speed = 0.0
        self.publisher = rospy.Publisher('/rover/cmd_vel', Twist, queue_size=1)
        rospy.Service('control_rover', RoverControl, self.handle_rover_control)

    def handle_rover_control(self, request):
        self.speed = request.speed
        if self.speed == 0:
            self.publisher.publish(Twist())  # Stop the rover
        else:
            twist = Twist()
            twist.linear.x = self.speed
            self.publisher.publish(twist)

        response = RoverControlResponse(success=True)
        return response

if __name__ == '__main__':
    RoverControlServer()
    rospy.spin()
