# -*- coding: utf-8 -*-
"""
Created on Wed Oct 25 15:17:13 2023

@author: nehem
"""

# -*- coding: utf-8 -*-
"""
Created on Tue Oct 24 21:11:08 2023

@author: nehem
"""

import numpy as np
import matplotlib.pyplot as plt
import random

class treeNode():
    def __init__(self, locationX, locationY, agent_radius):
        self.locationX = locationX
        self.locationY = locationY
        self.agent_radius = agent_radius
        self.kids = []
        self.parent = None

# Define the waypoint list as a global variable
wp = []

class RRT_A():
    def __init__(self, start, goal, numIT, gs, stepSize, agent_radius, obstacle_radius):
        self.randomTree = treeNode(start[0], start[1], agent_radius)
        self.goal = treeNode(goal[0], goal[1], agent_radius)
        self.nearestNode = None
        self.itterations = min(numIT, 10000)
        self.grid_spacing = gs
        self.rho = stepSize
        self.agent_radius = agent_radius
        self.obstacle_radius = obstacle_radius
        self.nearestDist = 10000
        self.numWaypoints = 0

        self.obstacle_positions = [(2, 2), (2, 3), (2, 4), (2, 5), (0, 5), (1, 5), (2, 5), (3, 5), (4, 5), (5, 5), (5, 2), (5, 3), (5, 4), (5, 5), (8, 2), (9, 2), (10, 2), (11, 2), (12, 2), (13, 2), (8, 3), (8, 4), (8, 5), (8, 6), (8, 7), (8, 8), (8, 9), (2, 7), (3, 7), (4, 7), (5, 7), (6, 7), (7, 7), (9, 6), (10, 6), (11, 6), (12, 6), (13, 6), (14, 6), (15, 6), (2, 8), (2, 9), (2, 10), (2, 11), (2, 12), (2, 13), (5, 9), (5, 10), (5, 11), (5, 12), (5, 13), (5, 14), (5, 15), (6, 12), (7, 12), (8, 12), (9, 12), (10, 12), (11, 12), (12, 8), (12, 9), (12, 10), (12, 11), (12, 12)]
        self.grid = np.zeros((int(16 / gs), int(16 / gs)))

        for obstacle in self.obstacle_positions:
            x_idx = int(obstacle[0] / gs)
            y_idx = int(obstacle[1] / gs)
            self.grid[y_idx, x_idx] = 1

        self.obstacle_points = []
        for obstacle in self.obstacle_positions:
            x_idx = int(obstacle[0] / gs)
            y_idx = int(obstacle[1] / gs)
            x_center = x_idx * gs + gs / 2
            y_center = y_idx * gs + gs / 2
            self.obstacle_points.append((x_center, y_center))

    def KID(self, locationX, locationY):
        if locationX == self.goal.locationX:
            self.nearestNode.kids.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            tempnode = treeNode(locationX, locationY, self.agent_radius)
            self.nearestNode.kids.append(tempnode)
            tempnode.parent = self.nearestNode

    def CheckMove(self):
        while True:
            x = random.uniform(0, 15)
            y = random.uniform(0, 15)
            randiMove = np.array([x, y]) 

            agent_radius_with_obstacle = self.agent_radius + self.obstacle_radius
            for obstacle_point in self.obstacle_points:
                if np.linalg.norm(randiMove - obstacle_point) <= agent_radius_with_obstacle:
                    break
            else:
                return randiMove

    def Direction(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart[0], locationEnd[1] - locationStart[1]])
        mag = np.linalg.norm(v)
        if mag < self.rho:
            return locationEnd
        else:
            v = (v / mag) * self.rho
            return np.array([locationStart[0] + v[0], locationStart[1] + v[1]])

    def Obstacle(self, locationStart, locationEnd):
        num_samples = int(np.ceil(np.linalg.norm(np.array(locationEnd) - np.array(locationStart)) / self.rho))
        if num_samples == 0:
            return False

        delta = np.array(locationEnd) - np.array(locationStart)
        delta /= num_samples

        for i in range(num_samples + 1):
            test_point = np.array(locationStart) + i * delta

            for obstacle_point in self.obstacle_points:
                if np.linalg.norm(test_point - obstacle_point) <= (self.agent_radius + self.obstacle_radius):
                    return True

        return False

    def WhoClose(self, root, point):
        if not root:
            return
        dist = np.sqrt((root.locationX - point[0]) ** 2 + (root.locationY - point[1]) ** 2)
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        for kid in root.kids:
            self.WhoClose(kid, point)

    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0]) ** 2 + (node1.locationY - point[1]) ** 2)
        return dist

    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.rho:
            return True

    def val_reset(self):
        self.nearestNode = None
        self.nearestDist = 10000

    def resetpath(self, goal):
        if goal is None:
            return
        if goal.locationX == self.randomTree.locationX:
            return
        self.numWaypoints += 1

        currentPoint = np.array([goal.locationX, goal.locationY])
        wp.append(currentPoint)
        self.resetpath(goal.parent)

gs = 1
start = np.array([1.0, 1.0])
goal = np.array([7.0, 13.0])
numIT = 10000
stepSize = 0.5
agent_radius = 0.5
obstacle_radius = 0.25
we_did_it=0
rrt = RRT_A(start, goal, numIT, gs, stepSize, agent_radius, obstacle_radius)

fig, ax = plt.subplots()
ax.grid(True)

for y in range(rrt.grid.shape[0]):
    for x in range(rrt.grid.shape[1]):
        if rrt.grid[y, x] == 1:
            ax.add_patch(plt.Circle((x * gs + gs / 2, y * gs + gs / 2), rrt.obstacle_radius, color='red'))

for i in range(rrt.itterations):
    rrt.val_reset()
    randiMove = rrt.CheckMove()  
    rrt.WhoClose(rrt.randomTree, randiMove)
    new = rrt.Direction([rrt.nearestNode.locationX, rrt.nearestNode.locationY], randiMove)
    testin = rrt.Obstacle([rrt.nearestNode.locationX, rrt.nearestNode.locationY], new)
    if testin == False:
        rrt.KID(new[0], new[1])
        ax.plot([rrt.nearestNode.locationX, new[0]], [rrt.nearestNode.locationY, new[1]], 'go', linestyle="--")
        if rrt.goalFound(new):
            rrt.KID(goal[0], goal[1])
            break

rrt.resetpath(rrt.goal)
wp.insert(0, start)

for i in range(len(wp) - 1):
    ax.plot([wp[i][0], wp[i + 1][0]], [wp[i][1], wp[i + 1][1]], 'ro', linestyle="--")
    
wp.reverse()
wp.insert(0, wp.pop())


formatted_wp = [[point[0], point[1]] for point in wp]
formatted_bois = formatted_wp
print(formatted_bois)

plt.show()

we_did_it = we_did_it+1

if we_did_it == 1:
    print("we did it")

if we_did_it == 1:
    print("we did it")

if we_did_it == 1:


    from re import S
    import rclpy
    import math 
    import numpy as np

    from rclpy.node import Node
    from rclpy.duration import Duration

    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from unmanned_systems_ros2_pkg import some_python_module
    from unmanned_systems_ros2_pkg import PIDTemplate

    def get_time_in_secs(some_node:Node) -> float:
        return some_node.get_clock().now().nanoseconds /1E9
        
    def euler_from_quaternion(x:float, y:float, z:float, w:float) -> tuple:
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return roll_x, pitch_y, yaw_z # in radians

    class TurtleBotNode(Node):
        def __init__(self, ns=''):
            super().__init__('minimial_turtlebot')
            
            if ns != '':
                self.ns = ns
            else:
                self.ns = ns
                    
            #create vel and odom pub and subscribers
            self.vel_publisher = self.create_publisher(
                Twist, self.ns+ "/cmd_vel" ,  10) 
            
            self.odom_subscriber =self.create_subscription(
                Odometry, self.ns +"/odom", self.odom_callback, 10)
            
            self.current_position = [None,None]
            self.orientation_quat = [0,0,0,0] #x,y,z,w
            self.orientation_euler = [0,0,0] #roll, pitch, yaw

        def odom_callback(self,msg:Odometry) -> None:
            """subscribe to odometry"""
            self.current_position[0] = msg.pose.pose.position.x
            self.current_position[1] = msg.pose.pose.position.y
            
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            
            roll,pitch,yaw = euler_from_quaternion(qx, qy, qz, qw)
            
            self.orientation_euler[0] = roll
            self.orientation_euler[1] = pitch 
            self.orientation_euler[2] = yaw
            
            if self.orientation_euler[2] < 0:
                self.orientation_euler[2] += 2*np.pi
            else:
                self.orientation_euler[2]=self.orientation_euler[2]

            #print("yaw is", np.degrees(self.orientation_euler[2]))
            
        def move_turtle(self, linear_vel:float, angular_vel:float) -> None:
            """Moves turtlebot"""
            twist = Twist()
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
            self.vel_publisher.publish(twist)
        
    def main()->None:
        rclpy.init(args=None)
        print("starting")

        namespace = ''
        rate_val = 5
        turtlebot_node = TurtleBotNode(namespace)
        rate = turtlebot_node.create_rate(rate_val)
        
        kp_angular = 5.0
        ki_angular=0.5
        kd_angular= 0.2
        dt_angular= 1/20
        pid_angular = PIDTemplate
        pid_angular = PIDTemplate.PID(
            kp=kp_angular,
            ki = ki_angular,
            kd = kd_angular,
            dt = dt_angular,
        )

        time_now = get_time_in_secs(turtlebot_node)
        print("time now is", time_now)
        


        MAX_ANG_SPEED_RAD=2.84

        wp_list=[formatted_bois] #this is where rrt waypoints should be
        print("waypoints", wp_list[0])
        wp_list = wp_list[0]
        counter=0

        heading_error_tolerance_rad = np.deg2rad(2)
        distance_error_tolerance_meters=0.15

        try:
            rclpy.spin_once(turtlebot_node)
            while rclpy.ok():
                
                rclpy.spin_once(turtlebot_node)

                if turtlebot_node.current_position == [None, None]:
                    
                    rclpy.spin_once(turtlebot_node)
                    continue
            
                current_wp=wp_list[counter]
                print("current wp", current_wp)
                dy = current_wp[1] - turtlebot_node.current_position[1]
                dx = current_wp[0] - turtlebot_node.current_position[0]
                counter=counter+1

                desired_heading_rad=np.arctan2(dy,dx)

                if desired_heading_rad < 0:
                    desired_heading_rad += 2*np.pi
                else:
                    desired_heading_rad=desired_heading_rad
                

                current_heading_error_rad =pid_angular.compute_error(
                    desired_heading_rad, turtlebot_node.orientation_euler[2]
                )
                current_distance_error=np.sqrt(dx**2 + dy**2)

                #set headding
                while abs(current_heading_error_rad) >= heading_error_tolerance_rad:
                    current_heading_error_rad = pid_angular.compute_error(
                        desired_heading_rad, turtlebot_node.orientation_euler[2]
                    )


                    if (abs(current_heading_error_rad) <= heading_error_tolerance_rad):
                        print("im done rotating to",wp_list)
                        break

                    angular_gains=pid_angular.get_gains(
                        desired_heading_rad,
                        turtlebot_node.orientation_euler[2]
                    )
                    #print("my gains are", angular_gains)

                    if angular_gains >= MAX_ANG_SPEED_RAD:
                        angular_gains = MAX_ANG_SPEED_RAD
                    elif angular_gains <= -MAX_ANG_SPEED_RAD:
                        angular_gains = -MAX_ANG_SPEED_RAD

                    turtlebot_node.move_turtle(0.0, angular_gains)
                    
                    rclpy.spin_once(turtlebot_node)

                #send forward time bb#

                while abs(current_distance_error) >= distance_error_tolerance_meters:
                    dy = current_wp[1] - turtlebot_node.current_position[1]
                    dx = current_wp[0] - turtlebot_node.current_position[0]
                    current_distance_error= np.sqrt(dx**2 + dy**2)
                    #print("Im going bro")

                    print("current distance error", current_distance_error)

                    if (current_distance_error <= distance_error_tolerance_meters):
                        print("converged to wp")
                        turtlebot_node.move_turtle(0.0, 0.0)
                        break

                    turtlebot_node.move_turtle(0.2, 0.0)

                    rclpy.spin_once(turtlebot_node)

        except KeyboardInterrupt:
            turtlebot_node.move_turtle(0.0,0.0)

    if __name__ == '__main__':
        """apply imported function"""
        main()