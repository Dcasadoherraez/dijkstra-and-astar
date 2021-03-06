################################################################################
#                              test_maps.py                                    #
#                                                                              #
# Contains the maps for testing the algorithms in different cases              #
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

# __________________________________________________________________
#                           Small map testing
# __________________________________________________________________
# Limits of the size of the grid map
xlim, ylim = 4, 2

# Weight for the edges
w = 1

# Set up the positions of the obstacle
x_obs = [2]
y_obs = [1]

# Given in map coordinates
start_node_name = [0, 1]
target_node_name = [3, 1]

# __________________________________________________________________
#                           Medium sized map testing
# __________________________________________________________________
# Limits of the size of the grid map
xlim, ylim = 8, 8

# Weight for the edges
w = 1

# Set up the positions of the obstacle
x_obs = [*range(2, 5+1)]
y_obs = [*range(0, 3+1)]

# Given in map coordinates
start_node_name = [0, 0]
target_node_name = [4, 0]
# __________________________________________________________________
#                           Big map testing
# __________________________________________________________________
# Limits of the size of the grid map
xlim, ylim = 30, 30

# Weight for the edges
w = 1

# Set up the positions of the obstacle
x_obs = [*[3]*11, 4, 5, 6, 7, *[8]*10]
y_obs = [*range(0, 10+1), 10, 10, 10, 10, *range(1, 10+1)]

# Given in map coordinates
start_node_name = [0, 0]
target_node_name = [5, 4]
