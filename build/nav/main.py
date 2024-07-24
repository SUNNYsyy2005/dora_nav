from A_star_dwa import Astar_DWA

import time
import math
import sys

def getAngle(x1, y1, x2, y2):
    angle = math.atan2(y1-y2, x1-x2)
    # transform from -pi-pi to 0-2pi
    if angle<0:
        angle += 2*math.pi
    return angle


if __name__ == '__main__':
    #debugger = Debugger()
    time.sleep(0.1) # 否则是-999999
    start_x, start_y, start_angle= 400,400,math.pi/2
    #print("start_x: "+str(start_x)+" start_y: "+str(start_y))
    #sys.exit(0)
    goal_x, goal_y = 330, 300
    
    # draw the start and end points
    #debugger.draw_specialPoints([start_x,goal_x],[start_y,goal_y])
    goal_angle = getAngle(start_x, start_y, goal_x, goal_y)
    
    # try A_star+dwa
    planner = Astar_DWA()
    for i in range(1,2):
        path_x, path_y, flag= planner.plan(start_x=start_x, start_y=start_y, start_angle=start_angle,
                               goal_x=goal_x, goal_y=goal_y, goal_angle=goal_angle)
        # turn back
        #MaxW = 5