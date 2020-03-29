#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import matplotlib.pyplot as plt

show_animation = False


class AStarPlanner:

    def __init__(self, ox, oy, reso, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        reso: grid resolution [m] 网格分辨率
        rr: robot radius[m]
        """

        self.reso = reso
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()
 
    #节点 x y 花费 父节点编号
    class Node:
        def __init__(self, x, y, cost, pind):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.pind = pind

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.pind)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            sx: start x position [m]  起点
            sy: start y position [m]
            gx: goal x position [m]  目标点
            gy: goal y position [m]

        output:
            rx: x position list of the final path  终点到起点
            ry: y position list of the final path
        """
    
        nstart = self.Node(self.calc_xyindex(sx, self.minx),
                           self.calc_xyindex(sy, self.miny), 0.0, -1)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx),
                          self.calc_xyindex(gy, self.miny), 0.0, -1)
        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart)] = nstart

        fn = dict()
        fn[self.calc_grid_index(nstart)] = self.calc_heuristic(nstart, ngoal)

        while 1:
            # TODO here
            # Remove the item from the open set
            # Add it to the closed set
            # expand_grid search grid based on motion model
            num = min(fn, key=lambda k:fn[k])
            nnow = open_set[num]
            if nnow.x == ngoal.x and nnow.y == ngoal.y:
                closed_set[num] = nnow
                print("HAHAHA Find it!!!")
                break
            else:
                closed_set[num] = nnow
                open_set.pop(num)
                fn.pop(num)
                for i in self.motion:
                    nnext = self.Node(nnow.x+i[0], nnow.y+i[1], nnow.cost+i[2], self.calc_grid_index(nnow))
                    if self.verify_node(nnext) == True:
                        if self.calc_grid_index(nnext) not in open_set.keys() and self.calc_grid_index(nnext) not in closed_set.keys():
                            open_set[self.calc_grid_index(nnext)] = nnext
                            fn[self.calc_grid_index(nnext)] = nnext.cost + self.calc_heuristic(nnext, ngoal)
                        else:
                            if self.calc_grid_index(nnext) in closed_set.keys() and nnext.cost<closed_set[self.calc_grid_index(nnext)].cost:
                                open_set[self.calc_grid_index(nnext)] = nnext
                                fn[self.calc_grid_index(nnext)] = nnext.cost + self.calc_heuristic(nnext, ngoal)
                            elif self.calc_grid_index(nnext) in open_set.keys() and nnext.cost<open_set[self.calc_grid_index(nnext)].cost:
                                open_set[self.calc_grid_index(nnext)] = nnext
                                fn[self.calc_grid_index(nnext)] = nnext.cost + self.calc_heuristic(nnext, ngoal)

        rx, ry = self.calc_final_path(ngoal, closed_set)
        return rx, ry

    def calc_final_path(self, ngoal, closedset):
        # generate final course
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
    #估计代价
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    #计算位置（实际位置）
    def calc_grid_position(self, index, minp):
        """
        calc grid position

        :param index:
        :param minp:
        :return:
        """
        pos = index * self.reso + minp
        return pos

    #计算节点xy值（网格数）
    def calc_xyindex(self, position, min_pos):
        return int(round((position - min_pos) / self.reso))

    #计算网格索引 1号格2号格...
    def calc_grid_index(self, node):
        return int((node.y - self.miny) * self.xwidth + (node.x - self.minx))

    #验证是否可行节点 1可行 0不可行
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
        print("minx:", self.minx)
        print("miny:", self.miny)
        print("maxx:", self.maxx)
        print("maxy:", self.maxy)

        #地图宽度（网格数）
        self.xwidth = int(round((self.maxx - self.minx) / self.reso))
        self.ywidth = int(round((self.maxy - self.miny) / self.reso))
        print("xwidth:", self.xwidth)
        print("ywidth:", self.ywidth)

        # obstacle map generation障碍物地图
        #全0地图 障碍物为1
        self.obmap = [[False for i in range(self.ywidth)]
                      for j in range(self.xwidth)]
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
    #单步花费
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
