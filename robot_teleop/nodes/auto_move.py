#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Int32, Int8
from geometry_msgs.msg import Vector3, Twist

import math

class AUTO_MOVE():
    linear_vel = 0.05
    angular_vel = 0.3
    
    current_pos_x = 0
    current_pos_y = 0

    cmd_pos_x = 0
    cmd_pos_y = 0

    diff_x = 0
    diff_y = 0

    angular_cal = 0

    step_size = 0.1

    """Twist.linear.x = 0
    Twist.linear.y = 0
    Twist.linear.z = 0
    Twist.angular.x = 0
    Twist.angular.x = 0
    Twist.angular.x = 0"""

    def stateCb(self, msg):
        self.current_pos_x = msg.x
        self.current_pos_y = msg.y
        rospy.loginfo('Updating current position')
        
    def cmdCb(self, msg):
        self.cmd_pos_x = msg.x 
        self.cmd_pos_y = msg.y
        self.diff_x = self.cmd_pos_x - self.current_pos_x
        self.diff_y = self.cmd_pos_y - self.current_pos_y
        rospy.loginfo('self.cmd_pos_x %d, self.current_pos_x %d',self.cmd_pos_x, self.current_pos_x)
        rospy.loginfo('self.cmd_pos_y %d, self.current_pos_y %d',self.cmd_pos_y, self.current_pos_y)
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        
        if self.diff_x == -1 and self.diff_y == -1:
            twist.angular.z = self.angular_vel
            pub.publish(twist)
            # rospy.sleep(math.pi/4/self.angular_vel)
            twist.linear.x = -self.linear_vel # ; twist.linear.y = self.linear_vel
            twist.angular.z = 0
            pub.publish(twist)
            # rospy.sleep(math.sqrt(self.step_size * self.step_size * 2)/self.linear_vel)
            rospy.loginfo('I am moving')

        # elif self.diff_x
        else:
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            pub.publish(twist) 
            rospy.loginfo('Shut down')
        


autoMove = AUTO_MOVE()

"""LinearPub = rospy.Publisher("/command/linear", Twist, queue_size=5)
AngularPub = rospy.Publisher("/command/angular", Twist, queue_size=5)"""
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

if __name__ == '__main__':

    rospy.init_node('turtlebot3_teleop')

    turtlebot3_model = rospy.get_param("model", "burger")
    # pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # Set subscribers
    rospy.Subscriber("/state/current_pos", Vector3, autoMove.stateCb)
    rospy.Subscriber("/command/pos", Vector3, autoMove.cmdCb)
    
    # Server(AlignmentControllerConfig, dynamicReconfigureCb)

    rospy.spin()
