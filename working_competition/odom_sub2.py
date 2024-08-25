#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
import numpy as np
# class ImuSubscriber(Node):
#     def __init__(self):
#         super().__init__('imu_subscriber')
#         self.latest_orientation = None
#         self.subscription = self.create_subscription(
#             Imu,
#             '/imu',
#             self.imu_callback,
#             10
#         )
#         self.subscription  # prevent unused variable warning

#     def imu_callback(self, msg):
#         orientation = msg.orientation
#         self.latest_orientation = orientation

#     def print_latest_orientation(self):
#         if self.latest_orientation is not None:
#             x = self.latest_orientation.x
#             y = self.latest_orientation.y
#             z = self.latest_orientation.z
#             w = self.latest_orientation.w
#             self.get_logger().info(
#                 f"Latest Orientation: x={x}, y={y}, z={z}, w={w}")
#         else:
#             self.get_logger().info("No orientation data available yet.")
class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.latest_position = None
        self.euler = None
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        fin_ang = np.degrees(euler[2])
        self.latest_position = position
        self.euler = fin_ang
    def print_latest_position(self):
        if self.latest_position is not None:
            x = self.latest_position.x
            y = self.latest_position.y
            ang = self.euler
            return x, y, ang
        else:
            self.get_logger().info("No position data available yet.")

    def move(self, goal):
        # enc_ticks = dist*4701*0.94
        # counter_FL = np.uint64(0)
        # counter_BR = np.uint64(0)
        goal_x, goal_y = goal
        lin_vel = 0.2
        ang_vel_left = 0.1
        ang_vel_right = -0.1
        val = 0
        # time.sleep(0.1)
    #     ser = serial.Serial('/dev/ttyUSB0', 9600)
        count = 0
        while True:
            odom_subscriber = OdomSubscriber()
            rclpy.spin_once(odom_subscriber, timeout_sec=1)
            # Print initial position if available
            x, y, ang = odom_subscriber.print_latest_position()
        #     imu_subscriber = ImuSubscriber()
        # # Spin once to allow subscription setup
        #     rclpy.spin_once(imu_subscriber, timeout_sec=1)
        #     imu_subscriber.print_latest_orientation()

            if count == 3:
                des_ang_x = ang
            if count > 3:
                cur_ang_x = ang
                error = (des_ang_x - cur_ang_x)
                # if error < 0:
                #     error = -1*error*0.05
                # else:
                #     error = 1*error*0.05
                error = 1*error*0.05
                # if (-1*error) > 300:
                #     error += 360
    #                 print(error)

                final_ang_pub = error
                final_lin_pub = 0.3
                # print(error)
                
                
                # pwm_L = 70 + error
                # pwm_R = 70 - error
                # pwm_R = max(min(90, pwm_R), 0)
                # pwm_L = max(min(90, pwm_L), 0)

                # 
                print(error)
                print(cur_ang_x)
                print(x,y)
                print(abs(x-goal_x), abs(y-goal_y)) 
                if abs(x-goal_x) <= 0.25 and abs(y-goal_y) <= 0.25:  # stop by distance
                    
                    final_ang_pub = error
                    final_lin_pub = 0.15
                    twist_msg = Twist()
                    twist_msg.linear.x = final_lin_pub  # Linear velocity
                    twist_msg.angular.z = final_ang_pub  # Angular velocity
                    self.vel_pub.publish(twist_msg)
                if abs(x-goal_x) <= 0.05 and abs(y-goal_y) <= 0.05:  # stop by distance
                    print("goal reached")
                    final_ang_pub = 0.0
                    final_lin_pub = 0.0
                    twist_msg = Twist()
                    twist_msg.linear.x = final_lin_pub  # Linear velocity
                    twist_msg.angular.z = final_ang_pub  # Angular velocity
                    self.vel_pub.publish(twist_msg)
                    break
                twist_msg = Twist()
                twist_msg.linear.x = final_lin_pub  # Linear velocity
                twist_msg.angular.z = final_ang_pub  # Angular velocity
                self.vel_pub.publish(twist_msg)
            count += 1
        return cur_ang_x
    def turn(self, g_ang, g_direc):
        if g_direc == "C":
            final_ang_pub = -0.3
        else:
            final_ang_pub = 0.3 
    #         g_ang = 360-g_ang
        
    #     ser = serial.Serial('/dev/ttyUSB0', 9600)
        count = 0
        while True:
            odom_subscriber = OdomSubscriber()
            rclpy.spin_once(odom_subscriber, timeout_sec=1)
            # Print initial position if available
            x, y, ang = odom_subscriber.print_latest_position()
            
            if count == 0:
                des_ang_x = ang
            if count > 0:
                cur_ang_x = ang
                # error = (des_ang_x - cur_ang_x)
                # print(counter_FL)
                # error = (counter_BR - counter_FL)*2
                twist_msg = Twist()
                # twist_msg.linear.x = final_lin_pub  # Linear velocity
                twist_msg.angular.z = final_ang_pub  # Angular velocity
                self.vel_pub.publish(twist_msg)
    #                 print(cur_ang_x,des_ang_x)
    #                 if cur_ang_x>= des_ang_x+(0.7*ang) or cur_ang_x<= des_ang_x-(0.7*ang):
    #                     error = (counter_BR - counter_FL)*2
    #                     pwm_L = 40 + error
    #                     pwm_R = 40 - error
    #                     pwm_R = max(min(90, pwm_R), 0)
    #                     pwm_L = max(min(90, pwm_L), 0)
    #                     pwm1.ChangeDutyCycle(pwm_L)
    #                     pwm2.ChangeDutyCycle(pwm_R)
                # ang, direc = good_theta(cur_ang_x, goal)
                if abs(des_ang_x - cur_ang_x) >= 0.8*g_ang:
                    if g_direc == "C":
                        final_ang_pub = -0.1
                    else:
                        final_ang_pub = 0.1
                    print("goal reached")
                    final_ang_pub = final_ang_pub
                    final_lin_pub = 0.0
                    twist_msg = Twist()
                    twist_msg.linear.x = final_lin_pub  # Linear velocity
                    twist_msg.angular.z = final_ang_pub  # Angular velocity
                    self.vel_pub.publish(twist_msg)
                if abs(des_ang_x - cur_ang_x) >= g_ang:
                    print("goal reached")
                    final_ang_pub = 0.0
                    final_lin_pub = 0.0
                    twist_msg = Twist()
                    twist_msg.linear.x = final_lin_pub  # Linear velocity
                    twist_msg.angular.z = final_ang_pub  # Angular velocity
                    self.vel_pub.publish(twist_msg)
                    break
            count += 1
        print(cur_ang_x)
        return cur_ang_x

    def angle_finder(self, goal_coord):
        odom_subscriber = OdomSubscriber()
        rclpy.spin_once(odom_subscriber, timeout_sec=1)
        # Print initial position if available
        x, y, ang = odom_subscriber.print_latest_position()
        to_ang = (360 - np.arctan2(goal_coord[1] - y,
               goal_coord[0] - x) * 360/(2*np.pi)) % 360
        print("to ang", to_ang)
        ang, direc = self.good_theta((ang), to_ang)
        print("ang", ang, direc)
        return ang, direc
    def good_theta(self, cur_ang, to_ang):
        if cur_ang>0:
            cur_ang = 360 - cur_ang
        if cur_ang <= 0:
            cur_ang = abs(cur_ang)
        if cur_ang == 180:
            cur_ang += 0.01
        if (cur_ang > 180 and to_ang > 180) or (cur_ang < 180 and to_ang < 180):
            dist = abs(cur_ang - to_ang)
            direc = "C" if to_ang > cur_ang else "CC"
        else:
            dist_pi = abs(180 - cur_ang) + abs(180 - to_ang)
            dist_zero = 360 - dist_pi
            if dist_zero < dist_pi:
                dist = dist_zero
                direc = "C" if cur_ang > 180 else "CC"
            else:
                dist = dist_pi
                direc = "C" if cur_ang < 180 else "CC"
        return dist, direc
    def offset_odom_pts(self):
        goal_list = [(1, 150), (130, 97), (174, 98), (278, 202),
                     (323, 202), (429, 97), (474, 98), (599, 100)]
        param_list=[]
        x = 1.0
        y = 150.0
        dist = (
            (x-goal_list[0][0])**2 + (y-goal_list[0][1])**2)**0.5
        to_ang = (360 - np.arctan2(goal_list[0][1] - y,
                                   goal_list[0][0] - x) * 360/(2*np.pi)) % 360
        param = (dist, to_ang)
        param_list.append(param)
        for i in range(len(goal_list)-2):
            dist = (
                (goal_list[i+1][0]-goal_list[i+2][0])**2 + (goal_list[i+1][1]-goal_list[i+2][1])**2)**0.5
            to_ang = (360 - np.arctan2(goal_list[i+1][1] - goal_list[i+2][1],
                                       goal_list[i+1][0] - goal_list[i+2][0]) * 360/(2*np.pi)) % 360
            param = (dist, to_ang)
            param_list.append(param)
        odom_subscriber = OdomSubscriber()
        rclpy.spin_once(odom_subscriber, timeout_sec=1)
        # Print initial position if available
        x, y, ang = odom_subscriber.print_latest_position()
        new_coord_list = []
        for j in range(len(param_list)):
            d, theta = param_list[j]
            x_new = x + d * np.cos(theta)
            y_new = y + d * np.sin(theta)
            x = x_new
            y = y_new
            new_coord = (x_new, y_new)
            new_coord_list.append(new_coord)
            

    def go_2_points(self):
        goal_list = [(1, 150), (130, 97), (174, 98), (278, 202),
                     (323, 202), (429, 97), (474, 98), (599, 100)]
        
        for i in goal_list:
            
            goal = (i[0]/100.0, i[1]/100.0)
            ang, direc = self.angle_finder(goal)
            self.turn(ang, direc)
            self.move(goal)
#     print(dist, direc)
# def turn(g_ang, g_direc):
#     if g_direc == "C":
#         pin_left = 31
#         pin_right = 35
#     else:
#         pin_left = 33
#         pin_right = 37
# #         g_ang = 360-g_ang
#     pwm1 = gpio.PWM(pin_left, 50)
#     pwm2 = gpio.PWM(pin_right, 50)
#     val = 0
#     time.sleep(0.1)
# #     ser = serial.Serial('/dev/ttyUSB0', 9600)
#     count = 0
#     while True:
        
#         count += 1
        
#         if count == 12:
#             des_ang_x = (float(line)) % 359
#             if g_direc == "C":
#                 goal = (360+(des_ang_x+g_ang)) % 360
#             else:
#                 goal = (360+(des_ang_x-g_ang)) % 360
#         if count > 20:
#             cur_ang_x = (float(line)) % 359
#                 # print(counter_FL)
#             error = (counter_BR - counter_FL)*2
#             pwm_L = 75 + error
#             pwm_R = 75 - error
#             pwm_R = max(min(90, pwm_R), 0)
#             pwm_L = max(min(90, pwm_L), 0)
#             pwm1.ChangeDutyCycle(pwm_L)
#             pwm2.ChangeDutyCycle(pwm_R)
# #                 print(cur_ang_x,des_ang_x)
# #                 if cur_ang_x>= des_ang_x+(0.7*ang) or cur_ang_x<= des_ang_x-(0.7*ang):
# #                     error = (counter_BR - counter_FL)*2
# #                     pwm_L = 40 + error
# #                     pwm_R = 40 - error
# #                     pwm_R = max(min(90, pwm_R), 0)
# #                     pwm_L = max(min(90, pwm_L), 0)
# #                     pwm1.ChangeDutyCycle(pwm_L)
# #                     pwm2.ChangeDutyCycle(pwm_R)
#             ang, direc = good_theta(cur_ang_x, goal)
#             if ang < 6.05:
#                 if direc != g_direc:
#                     #                 if cur_ang_x>= des_ang_x+(ang) or cur_ang_x<= des_ang_x-(ang):
#                     pwm1.stop()
#                     pwm2.stop()
#                     gameover()
#                     # print(cur_ang_x,des_ang_x)
#                     print("Goodbye")
#                     break
#     return cur_ang_x



def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()
    # Spin once to allow subscription setup
    # rclpy.spin_once(odom_subscriber, timeout_sec=1)
    # odom_subscriber.print_latest_position()  # Print initial position if available
    # imu_subscriber = ImuSubscriber()
    # # Spin once to allow subscription setup
    # rclpy.spin_once(imu_subscriber, timeout_sec=1)
    # imu_subscriber.print_latest_orientation()
    # rclpy.spin(odom_subscriber)
    odom_subscriber.go_2_points()
    odom_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
