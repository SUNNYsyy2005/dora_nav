from sys import path
from scipy.spatial import KDTree
import numpy as np
import math
import time
from PIL import Image, ImageDraw
import csv
import os
class Node(object):
    def __init__(self, x, y, parent_node):
        self.x = x
        self.y = y #position
        self.parent = parent_node 
        self.G = 0 
        self.H = 0 


class Astar_DWA(object):

    def __init__(self, MAX_EDGE_LEN=5000, LIMIT_TRIAL=500000):

        self.MAX_EDGE_LEN = MAX_EDGE_LEN
        self.LIMIT_TRIAL = LIMIT_TRIAL #trial limit 
        self.minx = 0
        self.maxx = 800
        self.miny = 0
        self.maxy = 800
        #the shape of the robot
        self.robot_size = 4
        self.avoid_dist = 4
        self.r = 10
        self.obstree = ()
        self.step_lenth = 5 #the steplenth of robot
        self.openlist = []
        self.closelist = []
        #self.pathDebugger = Debugger()
        self.pathPointInterval = 20 # A_star中取的路径点数量


    def plan(self, start_x, start_y, start_angle, goal_x, goal_y, goal_angle):
        #init
        # Obstacles
        obstacle_x = [-999999]
        obstacle_y = [-999999]
        # 打开PGM文件
        # 获取当前工作目录
        current_directory = os.getcwd()

        # 输出当前工作目录
        print("当前工作目录:", current_directory)
        image_path = 'build/nav/1.pgm'
        img = Image.open(image_path)

        # 将图像转换为灰度模式
        img = img.convert('L')

        # 初始化障碍物坐标数组
        obstacle_x = []
        obstacle_y = []

        # 遍历图像中的每个像素
        width, height = img.size
        print(width, height)
        draw2 = ImageDraw.Draw(img)
        for y in range(height):
            for x in range(width):
                # 获取当前像素的灰度值
                pixel_value = img.getpixel((x, y))
                # 假设灰度值小于某个阈值（例如，100）的像素表示障碍物
                if pixel_value < 254:
                    obstacle_x.append(x)
                    obstacle_y.append(y)  
                    #draw2.point((x, y), fill=1)  
        flag = 0
        print(start_x, start_y, goal_x, goal_y)
        #draw2.point((883, 960), fill=1)
        #draw2.point((1040, 965), fill=1)
        draw2.point((start_x, start_y), fill=1)
        draw2.point((goal_x, goal_y), fill=1)
        img.save('build/nav/modified.pgm')
        #exit(0)
        # Obstacle KD Tree
        # print(np.vstack((obstacle_x, obstacle_y)).T)
        
        self.obstree = KDTree(np.vstack((obstacle_x, obstacle_y)).T)
        # 检查路径文件是否存在
        if not os.path.exists('build/nav/path.csv'):
            path = self.A_star(start_x, start_y, goal_x, goal_y)
            if not path:
                print("No path found!")
                return [], [], flag
            path = path[::3]
            # 存储路径为CSV文件
            with open('build/nav/path.csv', 'w', newline='') as csvfile:
                pathwriter = csv.writer(csvfile, delimiter=',')
                for point in path:
                    pathwriter.writerow(point)
        else:
            # 读取路径CSV文件
            path = []
            with open('build/nav/path.csv', 'r') as csvfile:
                pathreader = csv.reader(csvfile, delimiter=',')
                for row in pathreader:
                    # 假设每行是一个点的x和y坐标
                    point = [float(row[0]), float(row[1])]
                    path.append(point)
            path = np.array(path)  # 转换为NumPy数组，如果需要
            print(path.shape)
        #x = [110-96,110-92,110-82,110-64,110-55,110-50,110-48,110-46,110-44,110-40]
        #path = [path[i] for i in x]
        #path = path[::3]
        cur = [start_x, start_y]
        path = np.append(path,[cur],axis=0)
        #path = np.insert(path, 0, cur, axis=0)
        
        print(path)
        path_x = []
        path_y = []
        # 获取绘图对象
        draw = ImageDraw.Draw(img)
        for path_point in path:
            print(path_point)
            draw.point((path_point[0], path_point[1]), fill=1)
        img.save('build/nav/modified_1.pgm')
        flag =1
        path = np.array(path)       
        path_x = path[:,0]
        path_y = path[:,1]
        return path_x, path_y, flag

    def search_path(self, node, goal_x, goal_y):
        
        # if it has a collision, ignore
        if self.check_obs(node.x, node.y, self.obstree):
            return
        #print(node.x,node.y)
        # if it's in closelist, ignore
        if self.isCloseList(node.x, node.y):
            return
        
        #calculate G，H
        node.G = node.parent.G + self.step_lenth
        node.H = np.abs(node.x-goal_x)+np.abs(node.y-goal_y)
        #print(node.G,node.H)
        #relpace if its f < (minf of openlist)
        point = self.isOpenList(node.x, node.y)
        if point:
            if (node.G + node.H) <= (point.G + point.H):
                point = node
        else:
            self.openlist.append(node)
            #print(node.x,node.y)

    #check if collision
    def check_obs(self, node_x, node_y, obstree):
        x = node_x
        y = node_y
        tree = obstree
        dis, index = tree.query(np.array([x, y]))
        # 检查是否查询到的是虚拟障碍物
        if index == 0:  
            print("Warning: The robot is out of the map!")
            return False  # 或者采取其他适当的处理措施
        if dis > self.MAX_EDGE_LEN:
            return True
        step_size = self.robot_size + self.avoid_dist
        steps = round(dis/step_size)
        for i in range(steps):
            if dis <= self.robot_size + self.avoid_dist:
                #print("Warning: The robot is too close to the obstacle!")
                return True
        if dis <= step_size:
            return True
        return False

    def A_star(self, start_x, start_y, goal_x, goal_y):
        # 当前节点
        # debugger = Debugger()
        start_position = Node(start_x, start_y,-1)
        #print(start_position.x,start_position.y,start_position.G)
        self.openlist = [start_position]
        path_list = []

        # 调试
        x1 = [];  y1 = [];  x2 = [];  y2 = []
        for i in range(self.LIMIT_TRIAL):
            #if goal is in closelist
            #for a in self.openlist:
                #print(a.x,a.y,a.G,a.H)
            #print(self.openlist)
            point = self.is_final(goal_x, goal_y)
            if point:
                path_list = [[point.x ,point.y]]
                while point.parent!= -1 :
                    point = point.parent
                    path_list.append([point.x, point.y])
                #reverse the list
                #path_list.reverse()
                print("The path is found!")
                return path_list
            if not self.openlist:
                return False
                
            #select the point with minF from the openlist
            point_minF = self.ifFmin()
             # 调试（查看实时树）
            parent = point_minF.parent
            if parent != -1 :
                x1.append(point_minF.x)
                y1.append(point_minF.y)
                x2.append(parent.x)
                y2.append(parent.y)

            #将minF移出poenlist和移入closelist
            self.closelist.append(point_minF)
            self.openlist.remove(point_minF)
            # Search Path,the order:←, ↓, →, ↑,且将该节点作为父节点
            node1 = Node(point_minF.x - self.step_lenth, point_minF.y, point_minF)
            node2 = Node(point_minF.x - self.step_lenth, point_minF.y - self.step_lenth, point_minF)
            node3 = Node(point_minF.x, point_minF.y - self.step_lenth, point_minF)
            node4 = Node(point_minF.x + self.step_lenth, point_minF.y - self.step_lenth, point_minF)            
            node5 = Node(point_minF.x + self.step_lenth, point_minF.y, point_minF)
            node6 = Node(point_minF.x + self.step_lenth, point_minF.y + self.step_lenth, point_minF)
            node7 = Node(point_minF.x, point_minF.y + self.step_lenth, point_minF)
            node8 = Node(point_minF.x - self.step_lenth, point_minF.y + self.step_lenth, point_minF)

            self.search_path(node1, goal_x, goal_y)
            self.search_path(node2, goal_x, goal_y)
            self.search_path(node3, goal_x, goal_y)
            self.search_path(node4, goal_x, goal_y) 
            self.search_path(node5, goal_x, goal_y)  
            self.search_path(node6, goal_x, goal_y)  
            self.search_path(node7, goal_x, goal_y)  
            self.search_path(node8, goal_x, goal_y)  
        if i == self.LIMIT_TRIAL-1:
            print("cannot find path!")

    #select the node with min f(n)
    def ifFmin(self):
        Fmin = self.openlist[0]
        for node in self.openlist:
            if (node.G + node.H) <= (Fmin.G+Fmin.H) :
                Fmin = node
        return Fmin
        
    def isOpenList(self,x,y):
        for node in self.openlist:
            if node.x == x and node.y == y:
                return node
        return False
       
    def is_final(self, goal_x, goal_y):
        for node in self.closelist:
            dx = node.x-goal_x
            dy = node.y-goal_y
            if math.hypot(dx, dy) <= self.r :
                return node   
        return False
    
    
    def isCloseList(self,goal_x,goal_y):
        for node in self.closelist:
            if node.x == goal_x and node.y == goal_y:
                return node   
        return False