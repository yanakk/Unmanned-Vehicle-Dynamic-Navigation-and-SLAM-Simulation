#!/usr/bin/env python

import math

import matplotlib.pyplot as plt

show_animation = False


class AStarPlanner:

    def __init__(self, ox, oy, reso, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        reso: grid resolution [m]
        rr: robot radius[m]
        """

        self.reso = reso
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()
 
    class Node:
        def __init__(self, x, y, cost, pind):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.pind = pind    # previous index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.pind)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        nstart = self.Node(self.calc_xyindex(sx, self.minx),
                           self.calc_xyindex(sy, self.miny), 0.0, -1)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx),
                          self.calc_xyindex(gy, self.miny), 0.0, -1)
        open_set, closed_set = dict(), dict()                           # creat open set and closed set; determine start and goal point
        open_set[self.calc_grid_index(nstart)] = nstart                 # calc_grid_index returns an index for a point

        while 1:
            # TODO here----------------------------------------------------------
            # Remove the item from the open set
            # Add it to the closed set
            # expand_grid search grid based on motion model
            
            # final goal: put points/nodes in the closed set!
            
            # F = G+H
            current_index = min(open_set, key=lambda x: open_set[x].cost + self.calc_heuristic(ngoal,open_set[x]))
            current_node = open_set[current_index]

            # if find final route, then quit loop
            if current_node.x == ngoal.x and current_node.y == ngoal.y:
                ngoal.pind = current_node.pind
                ngoal.cost = current_node.cost
                print("successfully find")
                break

            # After finding the current node index corresponding to the minimum F value, delete it from the open set and add it to the closed set
            del open_set[current_index]
            closed_set[current_index] = current_node

            # expand new node surround the current node, based on move motion
            # node:[x, y, cost, pind]
            for i in range(len(self.motion)):
                newnode = self.Node(current_node.x + self.motion[i][0], current_node.y + self.motion[i][1], current_node.cost + self.motion[i][2], current_index)
                newnode_index = self.calc_grid_index(newnode)

                # verify if the node is in ob
                if not self.verify_node(newnode):
                    continue

                if newnode_index in closed_set:
                    continue
                # two situations, already in the openset or not
                if newnode_index not in open_set:
                    open_set[newnode_index] = newnode  # add new node to open set
                elif open_set[newnode_index].cost > newnode.cost:
                    open_set[newnode_index] = newnode   # record better PATH

        # ---------------------------------end-----------------------------------------

        rx, ry = self.calc_final_path(ngoal, closed_set)    # input goal point and closed set

        return rx, ry

    def calc_final_path(self, ngoal, closedset):
        # generate final PATH from close set
        rx, ry = [self.calc_grid_position(ngoal.x, self.minx)], [
            self.calc_grid_position(ngoal.y, self.miny)]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            rx.append(self.calc_grid_position(n.x, self.minx))
            ry.append(self.calc_grid_position(n.y, self.miny))
            pind = n.pind

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)    # cal length of the hypotenuse
        return d

    def calc_grid_position(self, index, minp):
        """
        calc grid position

        :param index:
        :param minp:
        :return:
        """
        pos = index * self.reso + minp
        return pos

    def calc_xyindex(self, position, min_pos):
        return int(round((position - min_pos) / self.reso))
    # creat grid based on rules
    def calc_grid_index(self, node):
        return int((node.y - self.miny) * self.xwidth + (node.x - self.minx))

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.minx)
        py = self.calc_grid_position(node.y, self.miny)

        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # collision check
        if self.obmap[int(node.x)][int(node.y)]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.minx = int(round(min(ox)))
        self.miny = int(round(min(oy)))
        self.maxx = int(round(max(ox)))
        self.maxy = int(round(max(oy)))
        # print("minx:", self.minx)
        # print("miny:", self.miny)
        # print("maxx:", self.maxx)
        # print("maxy:", self.maxy)

        self.xwidth = int(round((self.maxx - self.minx) / self.reso))
        self.ywidth = int(round((self.maxy - self.miny) / self.reso))
        # print("xwidth:", self.xwidth)
        # print("ywidth:", self.ywidth)

        # obstacle map generation
        self.obmap = [[False for i in range(self.ywidth)]
                      for i in range(self.xwidth)]
        for ix in range(self.xwidth):
            x = self.calc_grid_position(ix, self.minx)
            for iy in range(self.ywidth):
                y = self.calc_grid_position(iy, self.miny)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obmap[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
