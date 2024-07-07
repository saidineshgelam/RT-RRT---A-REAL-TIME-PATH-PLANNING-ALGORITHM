# Description: Method 2 for localized palnning using RRT and Bi-RRT* algorithm
import time
import sys, random, math, pygame
from math import sqrt,cos,sin,atan2
from pygame.locals import *

xlimit = 1250  # x-limit of the frame
ylimit = 800  # y-limit of the frame
window_size = [xlimit, ylimit] # size of the frame
stepsize = 20 # step size for the RRT* algorithm
radius = 70 # radius for the RRT* algorithm
goal_tolerence = 30 # goal tolerence for the RRT* algorithm
max_num_nodes = 100000 # maximum number of nodes for the RRT* algorithm

# Define colors
white = (255, 255, 255) 
black = (0, 0, 0)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)

global obstacles
obstacles = [[150,100,50,50]] # x_start, y_start, length, width
obstacles.append([350,200,50,50])
obstacles.append([550,300,50,50])
obstacles.append([750,400,50,50])
obstacles.append([950,500,50,50])
# obstacles.append([600,600,50,50])
# obstacles.append([700,700,50,50])
# obstacles.append([800,800,50,50])
# obstacles.append([900,500,50,50]) 

obstacle_control = [[0,1,1,0.5]] # move_x, move_y, x_dir, y_dir
obstacle_control.append([0,0.2,0.2,0.5])
obstacle_control.append([0,0.2,0.2,-0.5])
obstacle_control.append([0,0.2,0.2,0.5])
obstacle_control.append([0,1,0.2,-0.5]) 
# obstacle_control.append([0,1,0.2,0.5])
# obstacle_control.append([0,1,0.2,-0.5])
# obstacle_control.append([0,1,0.2,0.5])
# obstacle_control.append([0,1,0.2,-0.5])




def direction(obstacle, control, obstacles): #completed
    """
    Determine the direction of an obstacle.

    Args:
    - obstacle: obstacle object
    - control: control parameters for the obstacle
    - obstacles: list of obstacles

    Returns:
    - x_dir: x-direction of the obstacle
    - y_dir: y-direction of the obstacle
    """
    x_start = obstacle[0] #get the x-coordinate of the obstacle
    y_start = obstacle[1] #get the y-coordinate of the obstacle
    length = obstacle[2] #get the length of the obstacle
    width = obstacle[3] #get the width of the obstacle

    move_x = control[0] #get the x-coordinate of the movement
    move_y = control[1] #get the y-coordinate of the movement
 
    x_dir = control[2] 
    y_dir = control[3]

    if(x_start + move_x) < 0: #check if the obstacle is at the left edge of the frame
        x_dir = 1
    if(x_start + move_x + length) > xlimit: #check if the obstacle is at the right edge of the frame 
        x_dir = -1 #update the x-direction
    if(y_start + move_y) < 0: #check if the obstacle is at the top edge of the frame
        y_dir = 1
    if(y_start + move_y + width) > ylimit: #check if the obstacle is at the bottom edge of the frame
        y_dir = -1

    obstacle_clearence = 2
    for object in obstacles:
        if object != obstacle: #check if the object is not the obstacle
            if (x_start + move_x) > object[0] and (y_start + move_y) > object[1] and (x_start + move_x) < object[0] + object[2] and (y_start + move_y) < object[1] + object[3]: #check if the obstacle collides with any other obstacle 
                x_dir *= -1
                y_dir *= -1
  
            if (x_start + move_x + length) > object[0] and (y_start + move_y) > object[1] and (x_start + move_x + length) < object[0] + object[2] and (y_start + move_y) < object[1] + object[3]:
                x_dir *= -1
                y_dir *= -1
            if (x_start + move_x) > object[0] and (y_start + move_y + width) > object[1] and (x_start + move_x) < object[0] + object[2] and (y_start + move_y + width) < object[1] + object[3]:
                x_dir *= -1
                y_dir *= -1
            if (x_start + move_x + length) > object[0] and (y_start + move_y + width) > object[1] and (x_start + move_x + length) < object[0] + object[2] and (y_start + move_y + width) < object[1] + object[3]:
                x_dir *= -1
                y_dir *= -1

    return [x_dir, y_dir]


def move_obstacle(obstacle, control, obstacles): #completed
    """
    Move an obstacle based on its control parameters.

    Args:
    - obstacle: list representing the obstacle [x_start, y_start, length, width]
    - control: list representing the control parameters [move_x, move_y, x_dir, y_dir]
    - obstacles: list of all obstacles

    Returns:
    - updated obstacle
    - updated control parameters
    """
    x_start = obstacle[0] #get the x-coordinate of the obstacle
    y_start = obstacle[1]  #get the y-coordinate of the obstacle
    length = obstacle[2] 
    width = obstacle[3]

    move_x = control[0]
    move_y = control[1]

    [x_dir,y_dir] = direction(obstacle,control,obstacles) #get the direction of the obstacle
    x_start += move_x*x_dir
    y_start += move_y*y_dir

    obstacle[0]=x_start
    obstacle[1]=y_start
    control[2]=x_dir
    control[3]=y_dir
    return obstacle, control

def update_obstacles_pos(): #completed
    """
    Update the positions of all obstacles based on their control parameters.
    """
    number_of_obstacles = 0
    for obstacle in obstacles:
        number_of_obstacles += 1
    for j in range(0,number_of_obstacles):
        obstacles[j], obstacle_control[j] = move_obstacle(obstacles[j], obstacle_control[j], obstacles) #move the obstacle based on its control parameters



class node:
    def __init__(self, x_c=0, y_c=0, cost=0, parent_node=None):
        """
        Initialize a node object.

        Args:
        - x_c: x-coordinate of the node (default: 0)
        - y_c: y-coordinate of the node (default: 0)
        - cost: cost of reaching the node (default: 0)
        - parent_node: parent node of the current node (default: None)
        """
        self.x = x_c
        self.y = y_c
        self.cost = cost
        self.parent = parent_node  


def draw_obstacles(pygame, frame): #completed

    for obstacle in obstacles: 
        # pygame.draw.rect(frame, red, obstacle)
        pygame.draw.circle(frame, red, (obstacle[0] + obstacle[2]//2, obstacle[1] + obstacle[3]//2), obstacle[2]//2)
    pygame.display.update()



def distance(node1, node2): #completed
    """
    Calculate the Euclidean distance between two nodes.

    Args:
    - node1: first node
    - node2: second node

    Returns:
    - Euclidean distance between the two nodes
    """
    return sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2) #calculate the Euclidean distance between the two nodes

def interpolate(node1, node2): #completed
    """
    Interpolate between two nodes.

    Args:
    - node1: first node
    - node2: second node

    Returns:
    - interpolated node
    """
    if distance(node1,node2) < stepsize: #check if the distance between the two nodes is less than the step size
        return node2
    else:
        theta = atan2(node2[1] - node1[1], node2[0] - node1[0]) #calculate the angle between the two nodes
        return node1[0] + stepsize*cos(theta), node1[1] + stepsize*sin(theta) #interpolate between the two nodes 

def get_parent(nearest_node, new_node, nodes): #completed
    """
    Find the parent node for the new node.

    Args:
    - nearest_node: nearest node to the new node
    - new_node: new node
    - nodes: list of nodes

    Returns:
    - new_node with updated cost and parent node
    - nearest_node
    """
    for node in nodes:
        if path_object_check(node, new_node, obstacles) and distance([node.x, node.y], [new_node.x, new_node.y]) < radius and node.cost + distance([node.x, node.y], [new_node.x, new_node.y]) < nearest_node.cost + distance([nearest_node.x, nearest_node.y], [new_node.x, new_node.y]): #check if the path between the two nodes is collision-free
            nearest_node = node
        new_node.cost = nearest_node.cost + distance([nearest_node.x, nearest_node.y], [new_node.x, new_node.y]) #update the cost of the new node
        new_node.parent = nearest_node
    return new_node, nearest_node

def neighbor_nodes(nodes,rand_node): #completed
    """
    Find the nearest node to the given random node.

    Args:
    - nodes: list of nodes
    - rand_node: random node

    Returns:
    - nearest node to the random node
    """
    nearest_node = nodes[0]
    for node in nodes:
        if distance([node.x, node.y], [rand_node.x, rand_node.y]) < distance([nearest_node.x, nearest_node.y], [rand_node.x, rand_node.y]): #find the nearest node to the random node
            nearest_node = node #update the nearest node
    return nearest_node



def check_counter_clockwise(a, b, c): #completed
    """
    Check if three points are in counter-clockwise order.

    Args:
    - a, b, c: three points

    Returns:
    - True if the points are in counter-clockwise order, False otherwise
    """
    return (c[1] - a[1]) * (b[0] - a[0]) < (b[1] - a[1]) * (c[0] - a[0]) #check if the points are in counter-clockwise order

def path_object_check(node1, node2, obstacles): #completed
    """
    Check if the path between two nodes intersects with any obstacles.

    Args:
    - node1: first node
    - node2: second node
    - obstacles: list of obstacles

    Returns:
    - True if the path is collision-free, False otherwise
    """
    point_1 = [node1.x, node1.y] #get the x and y coordinates of the first node
    point_2 = [node2.x, node2.y]    #get the x and y coordinates of the second node
    for object in obstacles:
       object_corners = (object[0], object[1], object[0] + object[2], object[1]+object[3]) #get the corners of the obstacle
       x1 = (object_corners[0], object_corners[1]) #get the x-coordinates of the first corner
       y1 = (object_corners[0], object_corners[3])
       x2 = (object_corners[0], object_corners[1])
       y2 = (object_corners[2], object_corners[1])
       x3 = (object_corners[2], object_corners[3]) 
       y3 = (object_corners[2], object_corners[1])
       x4 = (object_corners[2], object_corners[3]) 
       y4 = (object_corners[0], object_corners[3])

       instant_of_collision_1 = check_counter_clockwise(point_1, x1, y1) != check_counter_clockwise(point_2, x1, y1) and check_counter_clockwise(point_1, point_2, x1) != check_counter_clockwise(point_1, point_2, y1) #check if the path intersects with the obstacle
       instant_of_collision_2 = check_counter_clockwise(point_1, x2, y2) != check_counter_clockwise(point_2, x2, y2) and check_counter_clockwise(point_1, point_2, x2) != check_counter_clockwise(point_1, point_2, y2)
       instant_of_collision_3 = check_counter_clockwise(point_1, x3, y3) != check_counter_clockwise(point_2, x3, y3) and check_counter_clockwise(point_1, point_2, x3) != check_counter_clockwise(point_1, point_2, y3)
       instant_of_collision_4 = check_counter_clockwise(point_1, x4, y4) != check_counter_clockwise(point_2, x4, y4) and check_counter_clockwise(point_1, point_2, x4) != check_counter_clockwise(point_1, point_2, y4)
       if instant_of_collision_1 == False and instant_of_collision_2 == False and instant_of_collision_3 == False and instant_of_collision_4 == False:
           continue #continue if there is no collision
       else:
            return False
    return True

def tree_extend(nodes, frame, blue): #completed
    """
    Extend the tree by adding a new node.

    Args:
    - nodes: list of nodes
    - frame: pygame surface to draw on
    - blue: color for drawing the tree

    Returns:
    - updated list of nodes
    """
    random_node = node(random.random()*xlimit, random.random()*ylimit) #generate random node
    # print("Random Node: ", random.random())
    nearest_node=  neighbor_nodes(nodes, random_node) #find the nearest node in the list to the random node
    actual_interpolated_node = interpolate([nearest_node.x, nearest_node.y], [random_node.x, random_node.y]) #interpolate between the nearest node and the random node
    new_node = node(actual_interpolated_node[0], actual_interpolated_node[1]) #create a new node at the interpolated position
    if path_object_check(nearest_node, new_node, obstacles): #check if the path between the nearest node and the new node is collision-free
       [new_node,nearest_node] = get_parent(nearest_node, new_node, nodes) #get the parent node for the new node
       nodes.append(new_node)
       pygame.draw.line(frame, blue, (nearest_node.x, nearest_node.y), (new_node.x, new_node.y))    #draw a line between the nearest node and the new node
       pygame.display.update()
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):  #check if the user wants to quit
            pygame.quit()
            sys.exit()
    return nodes

def draw_path(node_list, pygame, frame): #completed
    """
    Draw the path on the frame.

    Args:
    - node_list: list of nodes in the path
    - pygame: pygame module
    - frame: pygame surface to draw on
    """
    Last_node = node_list[-1] #get the last node in the list
    start_node = node_list[0] #get the first node in the list
    while Last_node != start_node:
        pygame.draw.line(frame, red, [Last_node.x, Last_node.y], [Last_node.parent.x, Last_node.parent.y],5) #draw a line between the last node and its parent
        Last_node = Last_node.parent    #update the last node
        # pygame.display.update() 

def draw_path2(node_list, pygame, frame): #completed
    """
    Draw the path on the frame.

    Args:
    - node_list: list of nodes in the path
    - pygame: pygame module
    - frame: pygame surface to draw on
    """
    Last_node = node_list[-1] #get the last node in the list
    start_node = node_list[0] #get the first node in the list
    while Last_node != start_node:
        pygame.draw.line(frame, green, [Last_node.x, Last_node.y], [Last_node.parent.x, Last_node.parent.y],5) #draw a line between the last node and its parent
        Last_node = Last_node.parent


def check_circle(x_,y_,xc,yc,r): #completed
    """
    Check if a point lies inside a circle."""
    if ((x_-xc)**2 + (y_-yc)**2 ) <= (r/2)**2:
        return True
    else:
        return False

def check_collision(robo, obstacles):#check if the robot collides with any obstacles
    
    start_x =robo[0]
    start_y = robo[1]
    radius = robo[2]+50
    for object in obstacles:
        object = (object[0], object[1], object[0] + object[2], object[1]+object[3]) #get the corners of the obstacle
        if check_circle(object[0], object[1], start_x, start_y, radius) == True: #check if the robot collides with the obstacle
            return True 
        if check_circle(object[0], object[3], start_x, start_y, radius) == True:
            return True
        if check_circle(object[2], object[1], start_x, start_y, radius) == True:
            return True
        if check_circle(object[2], object[3], start_x, start_y, radius) == True:
            return True
    return False
    
def check_goal(new_node, goal):
    """
    Check if the new node is the goal node.

    Args:
    - new_node: new node
    - goal: goal node

    Returns:
    - True if the new node is the goal node, False otherwise
    """
    if distance([new_node.x, new_node.y], [goal.x, goal.y]) < goal_tolerence and  path_object_check(new_node, goal, obstacles): #check if the distance between the two nodes is less than the goal tolerence
        return True

def bi_rrt_star_search(start_node, goal_node): #completed
    """
    Perform the Bi-RRT* search algorithm.

    Args:
    - start_node: start node
    - goal_node: goal node

    Returns:
    - starts: list of nodes in the start tree
    - goals: list of nodes in the goal tree
    - nodes_explored: number of nodes explored during the search
    """
    starts = []
    goals = []
    starts.append(start_node)
    goals.append(goal_node)

    status = False
    i = 0
    
    while i < max_num_nodes and status != True: #loop until the maximum number of nodes is reached or the goal is reached
        starts = tree_extend(starts, frame, blue) #extend the start tree
        goals = tree_extend(goals, frame, green) #extend the goal tree
        last_goal_node = goals[-1] #get the last node in the goal tree
        nearest_start_node = neighbor_nodes(starts, last_goal_node) #find the nearest node in the start tree to the last node in the goal tree
        if (distance([nearest_start_node.x, nearest_start_node.y], [last_goal_node.x, last_goal_node.y]) < goal_tolerence): #check if the distance between the two nodes is less than the goal tolerence
           if path_object_check(nearest_start_node, last_goal_node, obstacles): #check if the path between the two nodes is collision-free
                new_node = node(last_goal_node.x, last_goal_node.y) #create a new node at the last node in the goal tree
                [new_node, nearest_start_node] = get_parent(nearest_start_node, new_node, starts) #get the parent node for the new node
                starts.append(new_node) #add the new node to the start tree
                status = True #update the status
                break
        i += 1
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):  #check if the user wants to quit
                pygame.quit()
                sys.exit()

    if status == True:
        print("Goal Reached")
        pygame.draw.line(frame, green, (nearest_start_node.x, nearest_start_node.y), (new_node.x, new_node.y))   #draw a line between the nearest node and the new node
        # print("Time taken: ", time.time() - start_timer) #print the time taken
        # print("Number of nodes explored =: ", i) 
        draw_path(starts, pygame, frame) #draw the start tree
        draw_path(goals, pygame, frame) #draw the goal tree
        pygame.display.update()

    else:
        print("Goal Not Reached")
        print("increase the number of nodes or check the obstacles")
    nodes_explored = i
    return starts, goals, nodes_explored




  
            
def RRT_2(start2 ,goal2): #completed
    nodes_derived = []
    nodes_derived.append(start2)
    j = 0
    while j < max_num_nodes  and goal2 not in nodes_derived:
        

        
        rand_node = node(((random.random()))*xlimit, (random.random())*ylimit)#generate random node
        nearest_node = nodes_derived[0]
        for node_ in nodes_derived: #find the nearest node in the list to the random node
            if distance([node_.x, node_.y], [rand_node.x, rand_node.y]) < distance([nearest_node.x, nearest_node.y], [rand_node.x, rand_node.y]):
                nearest_node = node_
        actual_interpolated_node = interpolate([nearest_node.x, nearest_node.y], [rand_node.x, rand_node.y]) #interpolate between the nearest node and the random node
        new_node = node(actual_interpolated_node[0], actual_interpolated_node[1]) #create a new node at the interpolated position
        if path_object_check(nearest_node, new_node, obstacles): #check if the path between the nearest node and the new node is collision-free
            new_node.cost = nearest_node.cost + distance([nearest_node.x, nearest_node.y], [new_node.x, new_node.y]) #update the cost of the new node
            new_node.parent = nearest_node
            nodes_derived.append(new_node)
            pygame.draw.line(frame, white, (nearest_node.x, nearest_node.y), (new_node.x, new_node.y))   #draw a line between the nearest node and the new node

        
        if check_goal(new_node, goal2): #check if the new node is the goal node
            print("pathfound") #print path found: ", time.time() - start_timer)
            goal2.parent = new_node
            nodes_derived.append(goal2) #add the goal node to the list
            goal2.cost = new_node.cost + distance([new_node.x, new_node.y], [goal2.x, goal2.y]) #update the cost of the goal node
            pygame.draw.line(frame, red, (new_node.x, new_node.y), (goal2.x, goal2.y))
        pygame.display.update()
        j += 1
        for event in pygame.event.get(): #check if the user wants to quit
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE): 
                pygame.quit()
                sys.exit()
    
    if goal2 in nodes_derived: #check if the goal node is in the list
       
        # print("Number of nodes explored =: ", j)
        
        totalcost = nodes_derived[-1].cost
        # print("Total Cost: ", totalcost)
        draw_path2(nodes_derived, pygame, frame)
        pygame.display.update()
       
    else:
        print("Goal Not Reached")
        print("increase the number of nodes or check the obstacles")
    total_nodes = j
    return nodes_derived, total_nodes

  
def path_record(starts, goals):  # gets the path from start to goal
    """
    Record the path from start to goal.

    Args:
    - starts: list of nodes in the start tree
    - goals: list of nodes in the goal tree

    Returns:
    - Path: list of nodes in the path from start to goal
    """
    start_path= []
    goal_path = []
    Path = []
    Last = goals[-1]
    first = goals[0]
    while Last != first: #loop until the first node in the goal tree is reached
        Path.append(Last) #add the node to the path
        Last = Last.parent
    Last = starts[-1]
    first = starts[0]
    while Last != first: #loop until the first node in the start tree is reached
        Path.insert(0, Last) #add the node to the path
        Last = Last.parent
    return Path

def path_record2(nodes_derived): # gets the path from start to goal
    path = []
    Last = nodes_derived[-1]
    first = nodes_derived[0]
    while Last != first:
        path.append(Last)
        Last = Last.parent
    return path

def animate_move(a,b): # animate the movement of a point on the frame
    """
    Animate the movement of a point on the frame.

    Args:
    - a: x-coordinate of the point
    - b: y-coordinate of the point
    """
    pygame.draw.ellipse(frame, blue, [a -10, b-10,20,20]) #draw ellipse as robot on frame with 20x20 axis  #draw the point on the frame
    pygame.display.update()

def animate_circle(x,y,r): # animate the movement of a circle on the frame
    """
    Animate the movement of a box on the frame.

    Args:
    - x: x-coordinate of the box
    - y: y-coordinate of the box
"""

    # print("r: ", r)
    # pygame.draw.ellipse(frame, red, [x -r, y-r,2*r,2*r],2) #draw ellipse as robot on frame with 20x20 axis  #draw the point on the frame
    pygame.draw.circle(frame, blue, (x, y), r, 2)
    pygame.display.update()

def draw_start_goal(start_node, goal_node): # draw the start and goal nodes on the frame

    """
    Draw the start and goal nodes on the frame.

    Args:
    - start_node: start node object
    - goal_node: goal node object
    """ 
    pygame.draw.ellipse(frame, green, [start_node.x -10, start_node.y-10,20,20]) #draw ellipse as robot on frame with 20x20 axis
    pygame.draw.ellipse(frame, red, [goal_node.x -10, goal_node.y-10,20,20])
    pygame.display.update()
    
    

def optimal_path(Path, i, xp, yp,r): #get the optimal path
    count = i
    x = Path[count].x
    y = Path[count].y
    while check_circle(x,y,xp,yp,r) == True:
        count += 1
        x = Path[count].x
        y = Path[count].y
    return Path[i].x, Path[i].y, count




def main():  
    """
    Main function to run the Bi-RRT* search algorithm and animate the movement.
    """
    pygame.init() #initialize the pygame module
    global frame 
    frame = pygame.display.set_mode(window_size) #create a window
    pygame.display.set_caption('Real_time-RRT* Search') #set the title of the window
    frame.fill(black) #fill the window with black color
    draw_obstacles(pygame, frame) #draw the obstacles on the frame

    start_node = node(50.0, 50.0)  #start node 
    goal_node = node(1150.0, 750.0) #goal node
    start_timer = time.time()
    node_count = 0
    draw_start_goal(start_node, goal_node)
    starts1, goals1, nodes_explored = bi_rrt_star_search(start_node, goal_node) #run the Bi-RRT* search algorithm
    node_count = node_count+nodes_explored 
    Path = path_record(starts1, goals1) #record the path from start to goal
    goal_status = False #initialize the goal status
    count = 0 
    total_cost = 0
    derived_path_nodes = []
    validation_count = -1
    robot_in_region = False
    
    #update the number of nodes explored

    while goal_status == False: #loop until the goal is reached
        position = Path[count] #get the current position
        if count == validation_count:
        

            robot_in_region = False
        
        if position == Path[-1]: #check if the current position is the goal
            frame.fill(black) #fill the frame with black color
            update_obstacles_pos() #update the positions of the obstacles
            draw_obstacles(pygame, frame)
            draw_start_goal(start_node, goal_node)  #draw the obstacles on the frame
            # draw_tree(starts1, goals1) #draw the tree on the frame
             #draw the start and goal nodes on the frame
            animate_move(goal_node.x,goal_node.y) #animate the movement of the goal node
            goal_status = True
            # print("Goal Reached")
            # print("Total Cost: ", total_cost) #print the total cos
            print("Number of nodes explored =: ", node_count) #print the number of nodes explored
            print("goal reached please close the graphics window")
            print("Time taken: ", time.time() - start_timer)
            # print("Total Time: ", time.time() - start_timer)
            break
        
        robot_start = [position.x, position.y] #get the current position of the robot
        robot_goal = [Path[count+1].x, Path[count+1].y] #get the next position of the robot
        count += 1  #increment the count

        speed = 20 #speed of the robot
        for i in range(1, speed): #loop to animate the movement of the robot
            frame.fill(black)
            if robot_in_region == False:
             update_obstacles_pos() 
            draw_obstacles(pygame, frame)
            # draw_tree(starts1, goals1) 
            # draw_start_goal(start_node, goal_node) 
            draw_path(starts1, pygame, frame)
            draw_path(goals1, pygame, frame)
            if derived_path_nodes:
                for path in derived_path_nodes:
                    draw_path2(path, pygame, frame)
            draw_start_goal(start_node, goal_node) 
            x_ = robot_start[0] + i*(robot_goal[0] - robot_start[0])/speed #interpolate the x-coordinate of the robot
            y_ = robot_start[1] + i*(robot_goal[1] - robot_start[1])/speed #interpolate the y-coordinate of the robot
            animate_move(x_,y_) #animate the movement of the robot
            total_cost = total_cost+ (robot_goal[0] - robot_start[0])*(i/speed) #update the total cost

            # box_l =30 #length of the box
            # box_w = 30 #width of the box
            # box1 = [x_,y_,box_l,box_w] #create a box object
            rd = 60
            box_l = x_
            box_w = y_
            box1= [box_l, box_w, rd] #create a circle object
            # animate_box(x_,y_,box_l,box_w) #animate the movement of the box
            animate_circle(box_l, box_w, rd)

            if check_collision(box1, obstacles) == True and robot_in_region== False:#check if the robot collides with any obstacles
                print("Collision Detected.............................") #print collision detected
                robot_in_region = True
                newstart = node(x_,y_) #update the start node
                nodes_before = 3
                new_count = count + nodes_before
                nodes_box= [Path[new_count].x, Path[new_count].y,rd] #create a box object
                while check_collision(nodes_box, obstacles)== True:
                    nodes_before += 1
                    new_count = count + nodes_before
                    nodes_box= [Path[new_count].x, Path[new_count].y,rd]
                new_goal = Path[new_count] #update the goal node
                derived_nodes, nodes_explored = RRT_2(new_goal,newstart)
                node_count = node_count + nodes_explored #update the number of nodes explored
                Path2 = path_record2(derived_nodes) #record the path from the start to the goal
                invalid_count = count+ 1
                for id in range(1, nodes_before): #loop to remove the invalid nodes
                    Path.pop(invalid_count)
                validation_count= invalid_count #update the validation count
                for i_ in Path2:
                    Path.insert(validation_count, i_) #insert the nodes in the path
                    validation_count += 1
                derived_path_nodes.append(derived_nodes) #add the derived nodes to the list
                for p in derived_path_nodes: 
                    draw_path2(p, pygame, frame) #draw the derived path on the frame
                print("New Path Found ")
                break
            # time.sleep(0.001) #sleep for 0.1 seconds
                    
                

             

            
if __name__ == '__main__':
    main()
    z= True
    while z:
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                z = False
                pygame.quit()
                sys.exit()
