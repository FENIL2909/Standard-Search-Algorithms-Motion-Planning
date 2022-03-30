# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
import random
from scipy import spatial


# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path

        # Generating a list of all possible locations on the map and storing it in self.map
        List = []
        temp = []
        for i in range(0, self.size_row):
            for j in range(0, self.size_col):
                temp.append(i)
                temp.append(j)
                List.append(temp)
                temp = []

        n_total = np.size(List) / 2
        self.map = List
        self.n_total = n_total-1   # number of all possible locations in the map

    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''

        x1 = p1[0]
        x2 = p2[0]
        y1 = p1[1]
        y2 = p2[1]

        steps = 50   # number of points between the two given points to check collision
        X = np.linspace(x1,x2,steps)


        if x1 - x2 >1:                         # checking for finite slope of the line joining p1 and p2
            Y = y1 + ((y2-y1)/(x2-x1))*(X-x1)
        else:
            Y = np.linspace(y1,y2,steps)

        for i in range(0,steps):
            x = int(np.around(X[i]))           # selecting a point p between p1 and p2
            y = int(np.around(Y[i]))
            if self.map_array[x][y]==0:        # checking collision for p
                return True

        return False


    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''

        dist = np.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)
        return dist


    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        samples = 0                       # to keep count of the number of samples tried
        nodes = 0                         # to keep count of how many nodes are actually formed
        n = int(np.sqrt(n_pts))           # sqrt of the desired samples
        rows = np.linspace(0,self.size_row-1,n,dtype=int)           # sampling the rows uniformly
        cols = np.linspace(0,self.size_col-1,n,dtype=int)           # sampling the columns uniformly
        for i in rows:
            for j in cols:
                samples +=1
                if self.map_array[i][j] == 1:
                    self.samples.append((i,j))
                    nodes +=1
        print("---------------------------------------------------------------------------")
        print("UNIFORM SAMPLING")
        print("---------------------------------------------------------------------------")
        print("nodes:", nodes)
        print("samples:", samples)


    
    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        samples = 0                                            # to keep count of the number of samples tried
        nodes = 0                                              # to keep count of how many nodes are actually formed
        for i in range(0, n_pts):
            samples += 1
            point = random.randint(0, self.n_total)      # pick a point randomly from all possible configurations in the map
            if self.map_array[self.map[point][0]][self.map[point][1]] == 1:   # check collision of that point
                self.samples.append((self.map[point][0], self.map[point][1])) # make it a node if no collision
                nodes += 1
        print("---------------------------------------------------------------------------")
        print("RANDOM SAMPLING")
        print("---------------------------------------------------------------------------")
        print("nodes:", nodes)
        print("samples:", samples)


    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        samples = 0                                 # to keep count of the number of samples tried
        nodes = 0                                   # to keep count of how many nodes are actually formed

        # Zero covariance is assumed before designing this approach
        sigmax = 30                                 # variance in x/row direction
        sigmay = 30                                 # varinace in y/column direction
        for i in range(0, n_pts):
            samples += 1
            q1 = random.randint(0, self.n_total)                         # sampling q1 point randomly from the map
            q2_row = random.gauss(self.map[q1][0],sigmax)                # making a gaussian distribution around q1
            q2_col = random.gauss(self.map[q1][1],sigmay)                # making a gaussian distribution around q1
            q2 = [int(np.around(q2_row)),int(np.around(q2_col))]         # sampling q2 from the gaussian

            # alternative method of generating a 2D gaussian around q1 is as mentioned here:
            # here we assume that there is some sort of correlation present in row(X) and columns(Y)
            # mean = [self.map[q1][0],self.map[q1][1]]
            # cov = [[20,10],[10,20]]
            # q2_a = np.random.multivariate_normal(mean,cov,1)
            # q2 = [int(q2_a[0][0]), int(q2_a[0][1])]

            # checking if one is in obstacle and another is in obstacle free region
            if (q2[0]< self.size_row and q2[0]> 0 and q2[1]<self.size_col and q2[1]>0):
                if (self.map_array[self.map[q1][0]][self.map[q1][1]] == 1) and (self.map_array[q2[0]][q2[1]]==0):
                    self.samples.append((self.map[q1][0], self.map[q1][1]))
                    nodes += 1
                if (self.map_array[self.map[q1][0]][self.map[q1][1]] == 0) and (self.map_array[q2[0]][q2[1]]==1):
                    self.samples.append((q2[0], q2[1]))
                    nodes += 1
        print("---------------------------------------------------------------------------")
        print("GAUSSIAN SAMPLING")
        print("---------------------------------------------------------------------------")
        print("nodes:", nodes)
        print("samples:", samples)


    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        samples = 0                                 # to keep count of the number of samples tried
        nodes = 0                                   # to keep count of how many nodes are actually formed

        # Zero covariance is assumed before designing this approach
        sigmax = 30                                 # variance in x/row direction
        sigmay = 30                                 # varinace in y/column direction

        for i in range(0, n_pts):
            samples += 1
            q1 = random.randint(0, self.n_total)        # sampling q1 randomly from the map
            if self.map_array[self.map[q1][0]][self.map[q1][1]] == 0:    # checking it if it is in obstacle
                # if q1 in obstacle sample another point q2 from gaussian
                q2_row = random.gauss(self.map[q1][0], sigmax)           # making a gaussian distribution around q1
                q2_col = random.gauss(self.map[q1][1], sigmay)           # making a gaussian distribution around q1
                q2 = [int(np.around(q2_row)), int(np.around(q2_col))]    # sampling q2 from the gaussian
                # mean = [self.map[q1][0],self.map[q1][1]]
                # cov = [[20,10],[10,20]]
                # q2_a = np.random.multivariate_normal(mean,cov,1)
                # q2 = [int(q2_a[0][0]), int(q2_a[0][1])]

                if (q2[0]< self.size_row and q2[0]>=0 and q2[1]<self.size_col and q2[1]>=0):
                    # check if q2 is also in obstacle
                    if self.map_array[q2[0]][q2[1]] == 0:
                        # find mid point between q1 and q2
                        qmid_row = (self.map[q1][0] + q2[0])/2
                        qmid_col = (self.map[q1][1] + q2[1])/2
                        # check if the mid point is not in obstacle
                        qmid = [int(np.around(qmid_row)), int(np.around(qmid_col))]

                        # if mid point not in obstacle make it a node
                        if self.map_array[qmid[0]][qmid[1]] == 1:
                            self.samples.append((qmid[0], qmid[1]))
                            nodes += 1
        print("---------------------------------------------------------------------------")
        print("BRIDGE SAMPLING")
        print("---------------------------------------------------------------------------")
        print("nodes:", nodes)
        print("samples:", samples)


    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict( zip( range( len(self.samples) ), node_pos) )
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])
        
        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y' ,ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12,  node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12,  node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()


    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "random":
            self.random_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        pairs = []
        r = 15
        kdtree = spatial.KDTree(self.samples)       # K-D tree
        pair_list = kdtree.query_pairs(r)           # pairs of all the nodes in the radius of r around each-other
        Nodes = set()

        for x in pair_list:
            p1 = self.samples[x[0]]
            p2 = self.samples[x[1]]
            # checking for collision-free path between the pairs of nodes
            if self.check_collision(p1,p2) == False:
                dist = self.dis(p1, p2)  # calculating the distance (weight) to travel between the nodes in the pairs
                pairs.append((x[0], x[1], dist))
                Nodes.add(x[0])
                Nodes.add(x[1])

        self.graph.add_nodes_from(list(Nodes))
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" %(n_nodes, n_edges))


    def search(self, start, goal):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.

        startx = start[0]
        starty = start[1]
        goalx = goal[0]
        goaly = goal[1]
        R_start = 35 #100 #35
        R_goal = 65 #100 #65
        start_id = 'start'
        goal_id = 'goal'
        start_pairs = []
        goal_pairs = []
        id_start = 0
        id_goal = 0

        # connecting the start and goal to the generated graph
        for s in self.samples:
            dist = (startx-s[0])**2 + (starty-s[1])**2
            if dist <= R_start**2 and dist!=0:
                if self.check_collision(start, s) == False:
                    cost = self.dis(start,s)
                    start_pairs.append((start_id,id_start,cost))
            id_start+=1

        for g in self.samples:
            dist = (goalx-g[0])**2 + (goaly-g[1])**2
            if dist <= R_goal**2 and dist!=0:
                if self.check_collision(goal, g) == False:
                    cost = self.dis(goal, g)
                    goal_pairs.append((goal_id,id_goal,cost))
            id_goal+=1

        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)
        
        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" %path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")
        
        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)
        