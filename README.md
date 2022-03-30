
## Overview

I have implemented **PRM**, **RRT** and **RRT*** algorithms. For **PRM**, I have implemented 4 different sampling methods - **uniform sampling**, **random sampling**, **gaussian sampling** and **bridge sampling**. These algorithms are the basic ones for sampling-based planning. 

Files included:

**PRM.py** is the file where a PRM class with four different sampling methods is implemented.

**RRT.py** is the file where a RRT class for RRT and RRT* is implemented.

**main.py** is the script that provides helper functions that load the map from an image and call the classes and functions from **PRM.py** and **RRT.py**. 

**WPI_map.jpg** is a binary WPI map image with school buildings. You could replace it with some other maps you prefer.


## Instruction

The **main.py** loads the map image **WPI_map.jpg** and calls classes and functions to run planning tasks. 

Please keep in mind that, the coordinate system used here is **[row, col]**, which is different from [x, y] in Cartesian coordinates. In README and the code comment, when the word '**point**' is used, it refers to a simple list [row, col]. When the word '**node**' or '**vertex**' is used, it refers to either the Node class in RRT ,or a node/vertex in a graph in PRM. 

## PRM

The two main phases of PRM are **Learning Phase** and **Query Phase**. 

#### Learning Phase

**Learning Phase** is coded in the function `sample`, where it samples points in the map according to different strategy, and connect these points to build a graph. In this template, the graph library [Networkx](https://networkx.org/documentation/stable/) is used to store the result graph. 

There are four different sampling methods implemented - `uniform_sample`, `random_sample`, `gaussian_sample` and `bridge_sample`. 

After sampling, I connected these sampling points to theirs k nearest neighbors. To find their neighbors, I have used K-D tree instead of simple brute force approach. 
Finally,  all the sampled points and their connection with neighbors are used as nodes and edges to build a Networkx graph.

#### Query Phase

**Query Phase** is coded in the function `search`, where it search for a path in the constructed graph given a start and goal point.

As start and goal points are not connected to the graph, I first added the start and goal node, find their nearest neighbors in the graph and connect them to these two nodes. Practically, as some of the graphs don't have a good connectivity, I will not only connect the start and goal node to their nearest node, but all the nodes within a certain distance, in order to increase the chance of finding a path.

Having connected start and goal node in the graph, I could use Dijkstra algorithm or any other algorithms to search for a valid path. This part done by using the Dijkstra function Networkx provided.

Finally, as PRM is a multi-query planning algorithms, one could call `search` with other start and goal point. So the previous start and goal nodes and their edges need to be removed in the end of each query phase. 


## RRT

For simplicity, this template uses a class 'Node' and a list 'vertices' in class 'RRT' as a tree structure. 

RRT is written in the function `RRT`. In each step, I get a new point, get its nearest node, extend the node and check collision to decide whether to add or drop this node. When I add a new node to the tree, I set the cost and parent of the new node, and add the new node to the list 'vertices'. I also need to check if it reaches the neighbor region of the goal. If so, connect to the goal directly and set the found flag to be true.

RRT* is written in the function `RRT_star`. The first few steps are pretty much the same as RRT. Besides, when a new node is added, we will need to rewire the new node AND all its neighbor nodes. Even a path is found, the algorithm should not stop as adding new nodes will possibly optimize the current found path.


