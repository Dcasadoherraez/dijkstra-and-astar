################################################################################
#                              Dijkstra_functions.py                           #
#                                                                              #
# Contains the functions for the Dijkstra's algorithm                          #
#                                                                              #
# Change History                                                               #
# 22/03/2021  Daniel Casado     Original code.                                 #
#                                                                              #
#                                                                              #
################################################################################
################################################################################
################################################################################
#                                                                              #
#  Copyright (C) 2021 Daniel Casado                                            #
#  dcasadoherraez@gmail.com                                                    #
#                                                                              #
#  This program is free software; you can redistribute it and/or modify        #
#  it under the terms of the GNU General Public License as published by        #
#  the Free Software Foundation; either version 2 of the License, or           #
#  (at your option) any later version.                                         #
#                                                                              #
#  This program is distributed in the hope that it will be useful,             #
#  but WITHOUT ANY WARRANTY; without even the implied warranty of              #
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               #
#  GNU General Public License for more details.                                #
#                                                                              #
#  You should have received a copy of the GNU General Public License           #
#  along with this program; if not, write to the Free Software                 #
#  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA   #
#                                                                              #
################################################################################
################################################################################
################################################################################

from Dijkstra_classes import Node, Edge
import numpy as np
import time


################################################################################
#  Function:  create_graph
# --------------------
# creates the graph with the nodes and edges G(V,E)
# inputs:
#  xlim, ylim: size of the graph
#  x_obs, y_obs: position of the obstacles
#
#  returns: list of edges and list of nodes
################################################################################
def create_graph(xlim, ylim, x_obs, y_obs):
    node_list = []
    # Creation of all the nodes in the grid
    for y in range(ylim):
        for x in range(xlim):
            # Set the "free" value of the nodes that are taken by an obstacle to False
            for i in range(len(x_obs)):  # Check if node coincides with any of all obstacles
                if ((x == x_obs[i]) and (y == y_obs[i])):
                    print("Obstacle detected")
                    # If it coincides set the free value to false
                    current_node = Node([x, y], 10000, False, [])
                    break
                else:
                    current_node = Node([x, y], float(np.inf), True, [])

            node_list.append(current_node)

    # Finding all links between nodes
    edge_list = []
    for node in [i for i in node_list if i.free]:
        for neighbor in neighbors(node, node_list):
            edge_list.append(
                Edge([node.name, neighbor.name], 1))
    return edge_list, node_list


################################################################################
#  Function:  neighbor
# --------------------
# checks all neighboring nodes to the given one
# inputs:
#  node: node to check neighbors for
#  V: set of all nodes in the graph
#
#  returns: list of all neighbor nodes to given 'node'
################################################################################
def neighbors(node, V):
    # 4 possible directions, up-down-left-right
    #dirs = [[1, 0], [0, 1], [-1, 0], [0, -1]]
    dirs = [[1, 0], [0, 1], [-1, 0], [0, -1],
            [1, 1], [1, -1], [-1, -1], [-1, 1]]
    result = []  # stores the neighbors
    for dir in dirs:
        neighbor_name = [node.name[0] + dir[0], node.name[1] + dir[1]]
        # If the neighbor belongs to the node list, it will be added to the neighbor list
        # List only containing the names of the nodes
        name_list = [x.name for x in V]
        if neighbor_name in name_list:
            # Get the index of the neighbor node in the list of names
            index = name_list.index(neighbor_name)
            result.append(V[index])  # Add the neighbor node to the result list
    return result


################################################################################
#  Function:  print_map
# --------------------
# prints map on the output line
# inputs:
#  node_list: all possible nodes
#  xlim,ylim: size of the graph
#
#  returns: node_map, which is matrix of all nodes
################################################################################
def print_map(node_list, xlim, ylim):
    # Create a matrix map to visualize the results
    node_map = np.zeros((ylim, xlim))

    for node in node_list:
        x = node.name[0]
        y = node.name[1]
        if node.value < float(np.inf):
            node_map[y][x] = round(node.value, 2)
        else:
            node_map[y][x] = 0
    return node_map


################################################################################
#  Function:  backpropagate
# --------------------
# find the path from goal to source node
# inputs:
#  V: set of all nodes in the graph
#  target_node: goal node
#
#  returns: list of all neighbor nodes to given 'node'
################################################################################
def backpropagate(V, target_node):
    # Get the path followed by backrpopagation
    path = []
    # V will be used instead of V_free for representation
    node_names_full = [x.name for x in V]
    current_index = node_names_full.index(target_node.name)
    current = V[current_index]
    path.append(current.name)

    while current.previous:
        current_index = node_names_full.index(current.previous)
        current = V[current_index]
        path.append(current.name)
        V[current_index].value = V[current_index].value + 100
    return path, V

# ===================================================================
#                           Dijkstra's algorithm
# ===================================================================
################################################################################
#  Function:  Dijkstra
# --------------------
# use Dijkstra's algorithm to go from the source node to the target node
# inputs:
#  xlim, ylim: size of the graph
#  x_obs, y_obs: position of the obstacles
#  start_node_name, target_node_name: name of the nodes in the format [x,y]
#  target_node: goal node
#
#  returns: list of all neighbor nodes to given 'node'
################################################################################


def Dijkstra(xlim, ylim, w, x_obs, y_obs, start_node_name, target_node_name):

    start_node = Node(start_node_name, 0, True, [])
    target_node = Node(target_node_name, 0, True, [])

    # Generate the graph and output edges and nodes G(E,V)
    E, V = create_graph(xlim, ylim, x_obs, y_obs)
    # Ignore all nodes with obstacles, keep V for representation
    V_free = [x for x in V if x.free]

    # Create a matrix map to visualize the results
    gridMap = print_map(V, xlim, ylim)
    print_map(V, xlim, ylim)

    node_names = [x.name for x in V_free]
    if (start_node.name not in node_names) and (target_node.name not in node_names):
        print("ERROR: Please select valid nodes")

    start_node_index = node_names.index(start_node.name)
    target_node_index = node_names.index(target_node.name)

    V_free[start_node_index].value = 0
    # V_free[target_node_index].value = 0

    # Empty set B is initialized
    B = []
    # Start node is set as current
    v_n = V_free[start_node_index]
    # Set A initially contains all free nodes
    # Error: A = V In Python, setting a variable equal to a list just points it to that list object
    A = V_free.copy()

    # Iterate through all free nodes
    start_time = time.time()
    while A:
        if (v_n.name == target_node_name) and (v_n.value < float(np.inf)):
            print("Target reached!")
            path, V = backpropagate(V, target_node)
            node_map = print_map(V, xlim, ylim)
            # print(node_map)
            break

        elif (len(A) == 0):
            print("No target found :(")
            break

        node_index = node_names.index(v_n.name)
        val_n = v_n.value
        # Function neighbors takes the current node and the node list
        v_n_neighbors = neighbors(v_n, V_free)
        for n in v_n_neighbors:
            neighbor_index = node_names.index(n.name)
            if (n.value > val_n+w):
                n.value = val_n+w
                n.previous = v_n.name
            V_free[neighbor_index].value = n.value

        B.append(v_n)
        A.remove(v_n)
        A.sort(key=lambda x: x.value)

        v_n = A[0]

        node_map = print_map(V, xlim, ylim)
        # print(node_map)
    t_value = time.time() - start_time
    print("The path followed: ")
    print(path)
    print("--- %s seconds ---" % (t_value))

    return node_map, t_value
