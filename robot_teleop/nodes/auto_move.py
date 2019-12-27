#!/usr/bin/env python
# encoding: utf-8
import rospy
import tf
from std_msgs.msg import Float64, Int32, Int8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from PID import PID
from math import sin, cos, pi, atan2, sqrt


class AUTO_MOVE():

    # all self properties
    linear_vel = 0.1
    angular_vel = 0.5

    current_pos_x = 0
    current_pos_y = 0

    cmd_pos_x = 0
    cmd_pos_y = 0

    diff_x = 0
    diff_y = 0

    angular_cal = 0

    step_size = 0.1

    pid_controller = PID(p=2, i=0.1, d=0, i_max=10, output_max=100)
    twist = Twist()

    def shutdown(self):
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        pub.publish(self.twist)
        rospy.loginfo('Shut down')

    def getState(self, msg):
        odom = msg

        position = odom.pose.pose.position
        self.position_x = position.x
        self.position_y = position.y
        self.position_z = position.z

        quaternion = odom.pose.pose.orientation
        q = [quaternion.x,
             quaternion.y,
             quaternion.z,
             quaternion.w]
        (self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion(q)

        
    def moveCommand(self, msg):
        self.cmd_pos_x = msg.x
        self.cmd_pos_y = msg.y
        # 当前姿态
        current_yaw = self.yaw
        # 当前位置
        current_x=self.position_x
        current_y=self.position_y
        

        if (self.cmd_pos_x == -1 and self.cmd_pos_y == -1):
            # 目标位置
            target_x=-0.5
            target_y=-0.0
            # linear error
            x_error=target_x-current_x
            y_error=target_y-current_y
            distance_error=sqrt(x_error**2+y_error**2)
            # # 改变角度
            # change_angle=-current_yaw
            # 目标绝对姿态
            target_yaw = atan2(y_error,x_error)
            # error
            yaw_error = target_yaw-current_yaw
            # error限幅，找到最小差角
            if yaw_error >= pi:
                yaw_error -= 2*pi
            elif yaw_error <= -pi:
                yaw_error += 2*pi
            # pid控制z轴旋转角速度
            # if abs(yaw_error) > 0.005:
            #     self.twist.angular.z = self.pid_controller.calculate_pid(yaw_error)
            #     pub.publish(self.twist)
            #     print('yaw_error: {:g}, current_yaw: {:g}'.format(yaw_error, current_yaw))
            # # else:
            # #     self.shutdown()
            # #     rospy.loginfo('reached')
            # else:
                # pid控制线速度
            if abs(x_error) > 0.005 :
                twist2=Twist()
                twist2.linear.x=self.pid_controller.calculate_pid(x_error)
                twist2.linear.y=0
                twist2.linear.z=0
                pub.publish(twist2)
                print('x_error: {:g}, current_x: {:g} '.format(x_error, current_x))
            else:
                self.shutdown()
                rospy.loginfo('reached')

        else:
            self.shutdown()


autoMove = AUTO_MOVE()

"""LinearPub = rospy.Publisher("/command/linear", self.twist, queue_size=5)
AngularPub = rospy.Publisher("/command/angular", self.twist, queue_size=5)"""
# pub = rospy.Publisher('cmd_vel', self.twist, queue_size=10)

if __name__ == '__main__':

    rospy.init_node('turtlebot3_teleop')

    turtlebot3_model = rospy.get_param("model", "burger")
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    #  Set subscribers
    rospy.Subscriber("/odom", Odometry, autoMove.getState)
    rospy.Subscriber("/command/pos", Vector3, autoMove.moveCommand)
    


    # Server(AlignmentControllerConfig, dynamicReconfigureCb)
    rospy.spin()
