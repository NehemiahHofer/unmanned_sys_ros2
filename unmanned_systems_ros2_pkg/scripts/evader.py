#!/usr/bin/env python3
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
    max_x=10
    min_y=0
    max_y=10
    gs=1
    x_curr=2
    y_curr=1
    x_start = 2
    
    y_start = 1
    agent_radius  = 0.5
    
    goal_position = [7.0,2.0]    
    obstacle_positions = []
    obstacle_positions = [(5,0),(5,1),(5,2),(5,3),(5,4),(0,5),
                          (1,4),(2,3),(3,2),(3,3)]
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
            #+m.dist(move,[goal_position[0],goal_position[1]])

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


    import rclpy
    import math as m

    from random import randint
    from rclpy.node import Node
    from unmanned_systems_ros2_pkg import TurtleBotNode, quaternion_tools



    def generate_random_waypoints(n_random_waypoints:int, max_val:int)->list:
        """generate random waypoints from 1 to 1"""
        
        random_wp_list = []
        for i in range(0,n_random_waypoints+1):
            rand_x = randint(0, max_val)
            rand_y = randint(0, max_val)
            random_wp_list.append((rand_x, rand_y))
            
        return random_wp_list

    def compute_desired_heading(current_pos:list, des_pos:list) -> float:
        """compute desired heading based on positions"""
        return m.atan2(des_pos[1] - current_pos[1] , des_pos[0] - current_pos[0])

    def compute_dist_error(current_pos:list, des_pos:list)->float:
        """compute distance error"""
        return m.dist(des_pos,current_pos)

    def compute_heading_error(current_heading:float, des_heading:float) -> float:
        """compute heading error in radians"""
        return des_heading - current_heading

    def gimme_da_loot(turtlebot:TurtleBotNode, waypoint:list) -> list:
        """helper function"""
        desired_heading = compute_desired_heading(
            turtlebot.current_position, waypoint)
        
        heading_error = compute_heading_error(
            turtlebot.orientation_euler[2], desired_heading)

        dist_error = compute_dist_error(
            turtlebot.current_position, waypoint)
        
        return [desired_heading, heading_error, dist_error]

    def generate_zigzag_waypoints(start_point: tuple, goal_point: tuple, step_size: float) -> list:
         waypoints = [wp_list]
    #     waypoints.append(start_point)
    #     dx = goal_point[0] - start_point[0]
    #     dy = goal_point[1] - start_point[1]
        
    #     while abs(dx) > 0 or abs(dy) > 0:
            
    #         while dx > 0:
    #             waypoints.append((waypoints[-1][0] + step_size, waypoints[-1][1]))
    #             dx -= step_size
            
    #         while dy > 0:
    #             waypoints.append((waypoints[-1][0], waypoints[-1][1] + step_size))
    #             dy -= step_size

    #         while dx < 0:
    #             waypoints.append((waypoints[-1][0] - step_size, waypoints[-1][1]))
    #             dx += step_size

    #         while dy < 0:
    #             waypoints.append((waypoints[-1][0], waypoints[-1][1] - step_size))
    #             dy += step_size

    #     return waypoints




    def main() -> None:
        rclpy.init(args=None)
        
        turtlebot_evader = TurtleBotNode.TurtleBotNode('turtle', 'evader')    
        turtlebot_evader.move_turtle(0.0,0.0)

        
        set_random = False
        is_done = False
        n_random_waypoints =  5
        heading_tol = 0.1; #radians
        dist_tolerance = 0.25 #meters
        
        turn_speed = 1.1 #rad/speed
        line_speed = 0.15 #m/s
        stop_speed = 0.0 

        # if set_random == False:
        #     waypoints = [[9,9]]
        # else:
        #     start_point = (0, 0)
        #     goal_point = (9, 9)
        #     step_size = 1.0  
        #     waypoints = generate_zigzag_waypoints(start_point, goal_point, step_size)


            

        if set_random == False:
            waypoints = wp_list
        else:
            #waypoints = generate_random_waypoints(n_random_waypoints, 15);
            start_point = (2, 1)
            goal_point = (7, 2)
            step_size = 0.5  
            waypoints = generate_zigzag_waypoints(start_point, goal_point, step_size)
        
        while rclpy.ok():

            if is_done == True:
                print("I'm done")
                turtlebot_evader.move_turtle(stop_speed, stop_speed)
                rclpy.shutdown()

            for waypoint in waypoints:
                print("current waypoint is", waypoint)
                
                desired_heading, heading_error, dist_error = gimme_da_loot(turtlebot_evader, waypoint)

                while (abs(dist_error) >= dist_tolerance) or (abs(heading_error) >= heading_tol):

                    # print("current heading is", m.degrees(turtlebot_evader.orientation_euler[2]))
                    # print("desired heading is", m.degrees(desired_heading), heading_error)
            
                    if abs(dist_error) >= dist_tolerance and  abs(heading_error) <= heading_tol:
                        turtlebot_evader.move_turtle(line_speed, stop_speed)
                    elif abs(dist_error) < dist_tolerance and  abs(heading_error) >= heading_tol:
                        if heading_error > 0:
                            turtlebot_evader.move_turtle(stop_speed, turn_speed)
                        else: 
                            turtlebot_evader.move_turtle(stop_speed, -turn_speed)
                    else:
                        if heading_error > 0:
                            turtlebot_evader.move_turtle(line_speed, turn_speed)
                        else:
                            turtlebot_evader.move_turtle(line_speed, -turn_speed)
                    
                    desired_heading, heading_error, dist_error = gimme_da_loot(turtlebot_evader, waypoint)
                    
                    rclpy.spin_once(turtlebot_evader)
                                    
            #/we're done looping through our lists
            is_done = True
                            

    if __name__=="__main__":
        main()