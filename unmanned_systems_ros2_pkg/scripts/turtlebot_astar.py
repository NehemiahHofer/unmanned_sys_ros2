#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import math as m
class Node:
    def __init__(self,x,y,parent_cost,index, parent_position):
        # attributes
        self.x=x
        self.y=y 
        self.parent_cost=parent_cost
        self.index= int(index)
        self.parent_position = parent_position


class Obstacle():
    def __init__(self, x_pos:float, y_pos:float, radius:float)-> None:
        self.x_pos=x_pos
        self.y_pos=y_pos
        self.radius=radius
    
    def is_inside(self,curr_x:float, curr_y:float, robot_radius:float=0):
        dist_from = np.sqrt((curr_x-self.x_pos)**2 +(curr_y-self.y_pos)**2)
        if dist_from >= self.radius + robot_radius:
            return False
        return True
  
def is_not_valid(obstacle_list:list, x_min:int, y_min:int, y_max:int, x_max:int,
                 x_curr:float, y_curr:float, agent_radius:float=0.0,grid_radius:float=0.0):
    
    for obs in obstacle_list:
        if obs.is_inside(x_curr,y_curr,agent_radius):
            print("youre dead at", obs.x_pos, obs.y_pos, x_curr, y_curr)
            return True
    
    if x_min+grid_radius > x_curr:
        return True
    if x_max-grid_radius < x_curr:
        return True
    if y_min+grid_radius > y_curr:
        return True
    if y_max-grid_radius < y_curr:
        return True
    

def compute_index(x_min:int, x_max:int, y_min:int, y_max:int, gs:int, x_curr:int, y_curr:int ) ->int:
    index=((x_curr-x_min)/gs)+(((y_curr-y_min)/gs)*((x_max+gs)-x_min)/gs)
    return index

        
def get_all_moves(node_position:list) -> list:
    pass      


if __name__=='__main__':

    # INTIALIZE HERE
    min_x=0
    max_x=15
    min_y=0
    max_y=15
    gs=0.5
    x_curr=1
    y_curr=1
    x_start = 1
    
    y_start = 1
    agent_radius  = 0.5
    
    goal_position = [7.0,13.0]    
    obstacle_positions = []
    obstacle_positions = [
    (2, 2), (2, 3), (2, 4), (2, 5), (0, 5), (1, 5), (2, 5), (3, 5), (4, 5), (5, 5), (5, 2), (5, 3),
    (5, 4), (2, 5), (6, 2), (5, 2), (4, 2), (7, 2), (7, 2), (7, 2), (7, 2), (8, 2), (9, 3), (7, 4),
    (7, 5), (5, 6), (3, 7), (8, 8), (9, 9), (7, 7), (7, 8), (7, 9), (7, 7), (7, 7), (7, 6), (8, 6),
    (9, 6), (10, 6), (11, 8), (12, 9), (12, 10), (12, 11), (12, 12), (12, 13), (12, 9), (12, 10),
    (12, 11), (12, 12), (12, 13), (12, 14), (12, 15), (8, 12), (9, 12), (10, 12), (11, 12), (12, 12)]

    obstacle_list=[]
    obstacle_radius= 0.25
    
    for obs_pos in obstacle_positions:
        # print("obstacle_positions", obs_pos)
        obstacle= Obstacle(obs_pos[0], obs_pos[1], obstacle_radius)
        obstacle_list.append(obstacle)

    ### THIS IS DJIKSTRAS
    unvisited ={}        
    visited={}
    wp_list=[]
    # Node(x,y, parent_cost,index,parent_position)
    current_node=Node(x_start, y_start, 0, -1, [-1,-1])
    current_index = compute_index(min_x, max_x, min_y, max_y, gs, current_node.x,current_node.y)
    current_position = [current_node.x, current_node.y]
    unvisited[current_index] = current_node

    while [current_node.x, current_node.y] != goal_position:
        
        current_index = min(unvisited, key=lambda x:unvisited[x].parent_cost)
        
        current_node = unvisited[current_index]
        
        if [current_node.x, current_node.y] == goal_position:    
            #return 
            #wp_list = []
            wp_node = current_node
            
            wp_list.append([wp_node.x, wp_node.y])
            
            while (wp_node.index != -1):
                wp_node = visited[wp_node.index]
                wp_list.append([wp_node.x, wp_node.y])
                print("position is", wp_node.x, wp_node.y)
                
                if wp_node.index == -1:
                    wp_list= wp_list[::-1]    
                    break
                

        
        visited[current_index] = current_node
        del unvisited[current_index]

        gs_bounds = np.arange(-gs, gs+gs, gs)
        move_list = []
        
        # get all moves
        for y_step in gs_bounds:        
            for x_step in gs_bounds:

                move_x = x_step+current_node.x
                move_y = y_step+current_node.y
                    
                if move_x == current_node.x and move_y == current_node.y:
                    continue
                                
                move_list.append([move_x, move_y])
        
        # once you're here filter out all moves
        filtered_moves = []
        for move in move_list:
            if (is_not_valid(obstacle_list, min_x, min_y, max_x, max_y,
                             move[0], move[1],agent_radius, obstacle_radius) == True):
                continue
            
            filtered_moves.append(move)
                
        
        # loop through all filtered moves and put into unvisited
        for move in filtered_moves:
            # compute the cost to get to this filtered move FROM our current node
            current_position = [current_node.x, current_node.y]
            

            cost = current_node.parent_cost + m.dist(current_position, move)
            +m.dist(move,[goal_position[0],goal_position[1]])

            # calculate the index of this filtered move
            index = int(compute_index(min_x, max_x, min_y, max_y, gs, move[0],move[1]))
            
            # make sure its not in visited 
            if index in visited:
                continue
            
            # update cost
            if index in unvisited:
                if current_position == [0.5, 0.0]:
                    print("cost for ", move ,"is ", cost, "this index cost is", 
                          unvisited[index].parent_cost)
                    
                if current_position == [0.0, 0.0]:
                    print("cost from origin to  ", move ,"is ", cost)
                
                # compare the cost 
                if unvisited[index].parent_cost > cost:
                    # print("updating cost", unvisited[index].x, unvisited[index].y)
                    unvisited[index].parent_cost = cost
                    unvisited[index].index = current_node.index
                    unvisited[index].parent_position = current_position
                
                continue

                # if lower than update
            # make a temp node
            temp_node=Node(move[0], move[1], cost, current_index, 
                           [current_node.x, current_node.y])

            unvisited[index] = temp_node

    
    ### PLOT STUFF
    x_list = []
    y_list = []
    for wp in wp_list:
        x_list.append(wp[0])
        y_list.append(wp[1])
        
    for obs in obstacle_list:
        plt.scatter(obs.x_pos, obs.y_pos, c='r')

    plt.plot(x_list, y_list, '-o')
    plt.grid(which = "both")
    plt.minorticks_on()  
    plt.xlim([min_x,max_x+gs])
    plt.ylim([min_y,max_y+gs])        


if [current_node.x, current_node.y] == goal_position:


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
            
            self.odom_subscriber = self.create_subscription(
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

        #wp_list=[]
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