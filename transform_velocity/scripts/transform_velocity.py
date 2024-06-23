#!/usr/bin/env python2.7

import rospy
from nav_msgs.msg import Odometry
import numpy as np
from numpy import cos, sin, abs, sign,sqrt
from tf.transformations import euler_from_quaternion
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist

initial_time = None
initial_time_set = False
cmd_vel_pub = None


def clock_callback(clock_msg):
    global initial_time, initial_time_set
    if not initial_time_set:
        initial_time = clock_msg.clock
        initial_time_set = True

def controller(x,y,th,w,xd,yd,xr,yr,xdr,ydr,xddr,yddr):
    lnd1 = 750
    lnd2 = 750
    K1=5
    K2 =5
    e = 300
    s1 = lnd1*(x-xr)+ (xd-xdr)
    s2 = lnd2*(y-yr)+(yd-ydr)
    d =0.17 
    c1 = -(1/7.7094)
    c2 = -(1/7.6847)
    c3 = 12.0991*c1
    c4 = 61.5953*c1
    c5 = 206.0774*c2
    c6 = 175.5830*c2
    
    
    if abs(s1) <= e:
        Sat1 = s1/e
    else:
        Sat1 = sign(s1)
    
    if abs(s2) <= e:
        Sat2 = s2/e
    else:
        Sat2 = sign(s2)
        
        
    
    A = np.array([[lnd1*cos(th) - w*sin(th), -lnd1*d*sin(th) - w*d*cos(th)],
                  [lnd2*sin(th) + w*cos(th),  lnd2*d*sin(th) - d*w*cos(th)]])
    
    U = np.array([[lnd1*xdr + xddr],
                  [lnd2*ydr + yddr]]) - np.array([[ K1*Sat1],
                                                  [ K2*Sat2]])
                  
    U_final = np.linalg.inv(A).dot(U)
    vr = U_final[0,0]
    wr = U_final[1,0]
    

    #THWR = lnd1*(v*cos(th) - d*w*sin(th) - xdr) +  ((c3/c2)*(w**2) - (c4/c1)*v)*cos(th) - v*w*sin(th) + d*((-c5/c2)*v*w + (-c6/c2)*w)*sin(th) - d*(w**2)*cos(th) - xddr
    #THVR = lnd2*(v*sin(th) + d*w*cos(th) - ydr)  + ((c3/c1)*(w**2) - (c4/c1)*v)*sin(th) + v*w*cos(th) - d*((-c5/c2)*v*w + (-c6/c2)*w)*cos(th) - d*(w**2)*sin(th)-yddr
#
    #if abs(s1) <= e:
    #    S11 = s1/e
    #else:
    #    S11 = sign(s1)
#
    #if abs(s2) <= e:
    #    S22 = s2/e
    #else:
    #    S22=sign(s2)
    #    
    #u_hat1 = -THWR - K1*S11
    #u_hat2 = -THVR - K2*S22
    #A = np.array([[-d*sin(th)/c2, cos(th)/c1],[d*cos(th)/c2, sin(th)/c1]])
    #U = np.linalg.inv(A).dot(np.array([[u_hat1],[u_hat2]]))
    #wr = U[0,0]
    #vr = U[1,0]    

    return [vr,wr]

def callback(data):
    global intial_time, initial_time_set
    
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    qth = data.pose.pose.orientation
    r = [0,0,qth.z,qth.w]
    th = euler_from_quaternion(r)[2]
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    v = sqrt(vx*vx + vy*vy)
    w = data.twist.twist.angular.z
    #xr = 0.001
    #yr = 0.001
    #ydr = 0.001
    #xdr = 0.001
    #xddr =0
    #yddr =0
    xd = vx
    yd = vy
    
    if initial_time_set:
        elapsed_time = data.header.stamp - initial_time
        elapsed_seconds = elapsed_time.to_sec()
        rospy.loginfo("elapsed time since initial message: %f seconds",elapsed_seconds)
        xr = 4*cos(0.2*elapsed_seconds)
        yr = 4*sin(0.2*elapsed_seconds)
        xdr = -4*0.2*sin(0.2*elapsed_seconds)
        ydr = 4*0.2*cos(0.2*elapsed_seconds)
        xddr = -4*0.2*0.2*cos(0.2*elapsed_seconds)
        yddr = -4*0.2*0.2*sin(0.2*elapsed_seconds)
        [vr,wr]=controller(x,y,th,w,xd,yd,xr*elapsed_seconds,yr*elapsed_seconds,xdr,ydr,xddr,yddr)
        #if abs(vr)>1000:
        #    vr = vr/1000
        #elif abs(vr)>100:
        #    vr = vr/100
        #elif abs(vr) >10:
        #    vr = vr/10
        #if abs(wr)>1000:
        #    wr = wr/1000
        #elif abs(wr)>100:
        #    wr = wr/100
        #elif abs(wr) >10:
        #    wr = wr/10
        #
        #if abs(vr)>1:
        #    vr = vr*0.1
        #if abs(wr)>1:
        #    wr = wr*0.1
    
        twist_cmd = Twist()
        twist_cmd.linear.x  = vr
        twist_cmd.angular.z = wr
        cmd_vel_pub.publish(twist_cmd)
        
        
        rospy.loginfo("linear velocity: %f, angular velocity: %f",vr,wr)
    else:
        rospy.logwarn("Initial time not set yet")
        
    
    
if __name__ == '__main__':
    rospy.init_node('odometry_subscriber_node')
    cmd_vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    rospy.Subscriber('/clock',Clock, clock_callback)
    rospy.Subscriber('/odometry/filtered',Odometry,callback)
    rospy.spin()







