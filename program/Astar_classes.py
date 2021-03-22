################################################################################
#                              Astar_classes.py                                #
#                                                                              #
# Classes for the A* algorithm                                                 #
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

# A* classes
class Node:
    def __init__(self, name, value, free, previous, gvalue, hvalue):
        self.name = name
        self.free = free
        self.previous = previous
        self.value = value
        self.gvalue = gvalue
        self.hvalue = hvalue


class Edge:
    def __init__(self, name, value):
        self.name = name
        self.value = value