# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*
import random

import matplotlib.pyplot as plt
import numpy as np
import sys


# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost



# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        List = []
        temp = []
        for i in range(0,self.size_row):
            for j in range(0,self.size_col):
                temp.append(i)
                temp.append(j)
                List.append(temp)
                temp = []
        n_total = np.size(List)/2
        self.map = List
        self.n_total = n_total-1
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###

        return np.sqrt((node1.row - node2.row)**2 + (node1.col - node2.col)**2)

    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###
        x1 = node1.row
        x2 = node2.row
        y1 = node1.col
        y2 = node2.col

        steps = 50                          # number of points between the two given points to check collision
        X = np.linspace(x1, x2, steps)

        if x1 - x2 > 1:                                 # checking for finite slope of the line joining node1 and node2
            Y = y1 + ((y2 - y1) / (x2 - x1)) * (X - x1)
        else:
            Y = np.linspace(y1, y2, steps)

        for i in range(0, steps):                       # selecting a point p between node1 and node2
            x = int(np.around(X[i]))
            y = int(np.around(Y[i]))
            if self.map_array[x][y] == 0:               # checking collision for p
                return False

        return True


    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''

        if (random.random() - goal_bias) <= 0:            # if random num is less than goal bias, sample a new random
                                                          # at goal
            point = [self.goal.row,self.goal.col]
            return point
        else:
            point = random.randint(0, self.n_total)       # else sample a new point radomly
            return self.map[point]

    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        min_dist = sys.float_info.max
        ver_id = 0
        ver_id_min = 0

        for v in self.vertices:  # iterating all the vertices to find nearest node to new sampled point
            dist = np.sqrt((point[0] - v.row)**2 + (point[1] - v.col)**2)
            if dist < min_dist:
                ver_id_min = ver_id
                min_dist = dist
            ver_id +=1
        return self.vertices[ver_id_min]


    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''

        # finding the neighbours of a node and storing it in a list
        neighbors = []
        for v in self.vertices:
            if self.dis(new_node,v) <= neighbor_size:
                if self.check_collision(new_node,v):
                    neighbors.append(v)
        return neighbors


    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''

        # rewiring the neighbours of new node
        for n in neighbors:
            dist = self.dis(new_node,n)
            if n.cost > new_node.cost + dist:
                n.parent = new_node
                n.cost = new_node.cost + dist
    
    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col and cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()


    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()

        step_size = 10                 # size of one step from the tree towards random sample
        goal_neighborhood = 15         # goal vicinity

        #Implementing RRT with Extending method
        for i in range(0,n_pts):
            point = self.get_new_point(0.1)         # getting a new point
            nearest_node = self.get_nearest_node(point)   # finding nearest node to the new point
            x2 = point[0]
            x1 = nearest_node.row
            y2 = point[1]
            y1 = nearest_node.col

            if (x2!=x1) or (y2!=y1):    # checking for finite slope of the line between nearest node and sample point
                magnitude = np.sqrt((x2-x1)**2 + (y2-y1)**2)  # norm of the vector joining nearest node and point
                ux = ((x2-x1)/magnitude)               # magnitude of movement with direction
                uy = ((y2-y1)/magnitude)               # magnitude of movement with direction
                nodex = int(np.around(x1 + (step_size*ux)))
                nodey = int(np.around(y1 + (step_size*uy)))
                new_node = Node(nodex,nodey)            # making a new node after extending by the step size in the
                                                        # direction of the vector joing the nearest node to sample point

                # check if the node is in the map area
                if (nodex < self.size_row) and (nodey < self.size_col) and (nodex > 0) and (nodey > 0):
                    if self.map_array[nodex][nodey] == 1:   # check collision for the node
                        if self.check_collision(nearest_node,new_node):     # if no collsion make new node
                            new_node.parent = nearest_node                  # connect it to the tree
                            new_node.cost = self.dis(nearest_node,new_node) + new_node.parent.cost   # update new node cost
                            self.vertices.append(new_node)
                            if(self.dis(self.goal,new_node) < goal_neighborhood):   # check if new node is in vicinity of goal
                                if self.check_collision(self.goal,new_node):        # check if there is collision free path between
                                                                                    # the goal and the new node
                                    self.found = 1                                  # update path found variable
                                    self.goal.parent = new_node                     # connect goal to the tree
                                    self.goal.cost = self.dis(self.goal,new_node) + self.goal.parent.cost   # update cost
                                    self.vertices.append(self.goal)
                                    break
        # Output
        print("---------------------------------------------------------------------------")
        print("RRT")
        print("---------------------------------------------------------------------------")
        if self.found:
            steps = len(self.vertices)  - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()


    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###
        step_size = 10
        goal_neighborhood = 15
        neighbor_size = 20
        min_goal_cost = sys.float_info.max
        # self.goal.cost = sys.float_info.max

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.

        for i in range(0,n_pts):
            point = self.get_new_point(0.1)   # sample random point
            nearest_node = self.get_nearest_node(point)  # find nearest node to sampled point
            x2 = point[0]
            x1 = nearest_node.row
            y2 = point[1]
            y1 = nearest_node.col

            if (x2!=x1) or (y2!=y1):   # check for finite slope of the line joining nearest node and the sample point
                magnitude = np.sqrt((x2-x1)**2 + (y2-y1)**2)   # norm of the vector joining nearest node and point
                ux = ((x2-x1)/magnitude)            # magnitude of movement with direction
                uy = ((y2-y1)/magnitude)            # magnitude of movement with direction
                nodex = int(np.around(x1 + (step_size*ux)))
                nodey = int(np.around(y1 + (step_size*uy)))
                new_node = Node(nodex,nodey)            # making a new node after extending by the step size in the
                                                        # direction of the vector joing the nearest node to sample point

                # check if the node is in the map area
                if (nodex < self.size_row) and (nodey < self.size_col) and (nodex > 0) and (nodey > 0):
                    if self.map_array[nodex][nodey] == 1:      # check collision for the node
                        if self.check_collision(nearest_node,new_node):    # if no collision make new node
                            new_node.parent = nearest_node                 # connect it to the tree
                            new_node.cost = self.dis(nearest_node,new_node) + new_node.parent.cost  # update new node cost
                            self.vertices.append(new_node)
                            neighbors = self.get_neighbors(new_node,neighbor_size)  # find out the neighbours of the new node
                            self.rewire(new_node,neighbors)                         # rewire those neighbours
                            if(self.dis(self.goal,new_node) < goal_neighborhood):   # check if new node is in vicinity of goal
                                if self.check_collision(self.goal,new_node):        # check if there is collision free path between
                                                                                    # the goal and the new node
                                    if (self.goal.cost < min_goal_cost):            # update the cost only if it is minimum
                                        min_goal_cost = self.goal.cost
                                        self.found = 1                              # update the found varibale
                                        self.goal.parent = new_node                 # connect the foal to tree
                                        self.goal.cost = self.dis(self.goal,new_node) + self.goal.parent.cost  # update the goal cost
                                        if self.found == False:
                                            self.vertices.append(self.goal)

        # Output
        print("---------------------------------------------------------------------------")
        print("RRT Star")
        print("---------------------------------------------------------------------------")

        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
