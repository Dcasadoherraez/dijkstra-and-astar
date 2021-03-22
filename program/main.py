################################################################################
#                                   main.py                                    #
#                                                                              #
# Comparison of the Dijkstra's and A* algorithm for robot navigation in a grid #
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

import numpy as np
from Astar_classes import *
from Astar_functions import *

from Dijkstra_classes import *
from Dijkstra_functions import *

from matplotlib import pyplot as plt
from matplotlib import colors

import time
# ================== Replace with values in test_maps.py =====================
# Limits of the size of the grid map
xlim, ylim = 30, 30

# Weight for the edges
w = 1

# Set up the positions of the obstacle (copy & paste from test_maps.py)
x_obs = [*[3]*11, 4, 5, 6, 7, *[8]*10]
y_obs = [*range(0, 10+1), 10, 10, 10, 10, *range(1, 10+1)]

# Given in map coordinates
start_node_name = [0, 0]
target_node_name = [5, 4]
# ===================== end of replecement part =====================

# Comment the unused method
# node_map, t_value = Dijkstra(xlim, ylim, w, x_obs, y_obs,
#                             start_node_name, target_node_name)

node_map, t_value = A_star(xlim, ylim, w, x_obs, y_obs,
                           start_node_name, target_node_name)

#  ======================================
# Plotting
xi = np.arange(0, xlim)
yi = np.arange(0, ylim)
X, Y = np.meshgrid(xi, yi)

"""for i in range(ylim):
    for j in range(xlim):
        plt.text(j, i, node_map[i, j], color="k")
"""
cmap = colors.ListedColormap(
    ['white', '#FFCCFFFF', '#FF99CCFF', '#FF6699CC', 'blue', 'red'])
boundaries = [0, 1, 100, 200, 400, 900, np.inf]
fig = plt.figure()
fig.suptitle('A* Algorithm: ' + str(round(t_value, 5)) + ' seconds',
             fontsize=14, fontweight='bold')

norm = colors.BoundaryNorm(boundaries, len(boundaries)-1)
map_plot = plt.pcolor(node_map[::], cmap=cmap,
                      norm=norm, edgecolors='k', linewidths=2)
plt.show()
#  ======================================
