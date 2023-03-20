import math
import random
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import sys, os
import cubic_spline_planner
import numpy as np
import time

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))

from elastic_band import elastic_band

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0
        self.parent = None

class LineNode:
    def __init__(self, x, y) :
        self.x = x
        self.y = y

class Line:
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2


class RRTStar:
    def __init__(self, start, goal, obstacles, lines ,epsilon, delta, max_iter):
        self.start = Node(start[0], start[1]) #출발점 좌표(x,y)
        self.goal = Node(goal[0], goal[1]) #목적지 좌표(x,y)
        self.obstacles = obstacles #장애물의 좌표와 반지름 리스트르 [x,y,r]
        self.lines = lines
        self.epsilon = epsilon #트리에서 노드 간의 거리
        self.delta = delta #목적지까지의 확률적 편향 정도
        self.max_iter = max_iter#최대 반복 횟수
        self.nodes = [self.start]

    def run(self):
        
        for i in range(self.max_iter):
            q = self.sample()
            nearest_node = self.find_nearest(q)
            new_node = self.steer(q, nearest_node)
            if self.check_collision(new_node, nearest_node) and not self.isOverlappedLine(new_node, nearest_node):
                near_nodes = self.find_near(new_node)
                self.rewire(new_node, near_nodes)
                self.nodes.append(new_node)
        ax = plt.subplot()
        for node in self.nodes :
            if node.parent :
                  ax.plot([node.x, node.parent.x], [node.y, node.parent.y], color='green')




    def sample(self):
        if random.random() > self.delta:
            theta = math.atan2(self.goal.y - self.start.y, self.goal.x - self.start.x)
            dist = math.sqrt((self.goal.y - self.start.y)**2 + (self.goal.x - self.start.x)**2)

            # Generate a random point within a fan-shaped area around the line connecting start and goal nodes
            radius = random.uniform(0, dist)
            angle = random.uniform(-math.pi/4, math.pi/4)
            x = self.start.x + radius * math.cos(theta + angle)
            y = self.start.y + radius * math.sin(theta + angle)

            return Node(x, y)
        else:
            return self.goal

    def find_nearest(self, q):
        distances = [(n.x - q.x) ** 2 + (n.y - q.y) ** 2 for n in self.nodes]
        return self.nodes[distances.index(min(distances))]

    def steer(self, q, nearest_node):
        theta = math.atan2(q.y - nearest_node.y, q.x - nearest_node.x)
        x = nearest_node.x + self.epsilon * math.cos(theta)
        y = nearest_node.y + self.epsilon * math.sin(theta)
        new_node = Node(x, y)
        new_node.cost = nearest_node.cost + self.epsilon
        new_node.parent = nearest_node

        return new_node

    def check_collision(self, node, nearest_node):
        for o in self.obstacles:
            if(self.circle_line_intersection(o, node, nearest_node)) :
                return False
        return True

    def circle_line_intersection(self, obstacle, node, nearest_node) :
        distance = self.distance_point_line(obstacle[0], obstacle[1], node, nearest_node)
        if(distance <= obstacle[2]) :
            return True
        else :
            return False
    
    def distance_point_line(self, obs_x, obs_y, node, nearest_node) :
        dx = nearest_node.x-node.x
        dy = nearest_node.y-node.y

        if(dx == 0 and dy == 0) :
            return self.distance_between_points(obs_x, obs_y, node.x, node.y)

        t = ((obs_x - node.x) * dx + (obs_y - node.y) * dy) / (dx * dx + dy * dy)

        if(t < 0) :
            return self.distance_between_points(obs_x, obs_y, node.x, node.y)
        elif(t > 1) :
            return self.distance_between_points(obs_x, obs_y, nearest_node.x, nearest_node.y)
        
        projection_x = node.x + t*dx
        projection_y = node.y + t*dy

        return self.distance_between_points(obs_x, obs_y, projection_x, projection_y)

    def distance_between_points(self, x1, y1, x2, y2) :
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)
    

    def ccw(self, p1, p2, p3) :
        a = (p1.x * p2.y) + (p2.x * p3.y) + (p3.x * p1.y)
        b = (p1.x * p3.y) + (p2.x * p1.y) + (p3.x * p2.y)

        if(a-b > 0) :
            return 1
        elif(a-b == 0) :
            return 0
        else :
            return -1


    def isOverlappedLine(self, node, nearest_node) :
        for line in self.lines :
            ans1 = self.ccw(node, nearest_node, line.p1) * self.ccw(node, nearest_node, line.p2)
            ans2 = self.ccw(line.p1, line.p2, node) * self.ccw(line.p1, line.p2, nearest_node)
            if((ans1 <= 0) and (ans2 <= 0)) :
                return 1
        return 0

    def find_near(self, node):
        #radius = min(50, )
        radius = max(0.5, math.sqrt(100 * math.log(len(self.nodes)) / (len(self.nodes)))/2)
        distances = [(n.x - node.x) ** 2 + (n.y - node.y) ** 2 for n in self.nodes]
        return [n for i, n in enumerate(self.nodes) if distances[i] < radius ** 2]

    def rewire(self, new_node, near_nodes):
        for n in near_nodes:
            cost = new_node.cost + math.sqrt((n.x - new_node.x) ** 2 + (n.y - new_node.y) ** 2)
            if cost < n.cost and self.check_collision(new_node,n):
                n.parent = new_node
                n.cost = cost
                self.propagate_cost_to_leaves(n)

    def propagate_cost_to_leaves(self, node):
        for n in self.nodes:
            if n.parent == node:
                n.cost = node.cost + math.sqrt((n.x - node.x) ** 2 + (n.y - node.y) ** 2)
                self.propagate_cost_to_leaves(n)

    def find_goal_point_nearest(self) :
        distances = [(n.x - self.goal.x) ** 2 + (n.y - self.goal.y) ** 2 for n in self.nodes]
        return self.nodes[distances.index(min(distances))]

    def get_path(self):
        path = []

        goalpoint=self.find_goal_point_nearest()

        node = self.nodes[self.nodes.index(goalpoint)]
        while node.parent:
            path.append([node.x, node.y])
            node= node.parent
        path.append([self.start.x, self.start.y])
        path.reverse()

        ax = plt.subplot()
        for i in range(len(path) - 1):
            ax.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], color='red',linewidth = 3)

        return path

def check_rrtstar_success(rrt_star_last_point, goal) :
    dis = ((rrt_star_last_point[0]-goal[0])**2 + (rrt_star_last_point[1]-goal[1])**2)**0.5
    if(dis >= 2) : #goal과 rrt_star_last_point의 거리가 2m 이상이면 다시 rrt* 생성
        return True
    return False


if __name__ == "__main__" :
    start_time = time.time()
    start, goal = (19.5,25), (20, 35)
    epsilon = 0.2
    delta_goal_bias = 0.2
    iter = 300
    obs_r = 0.4
    step_size = 5

    rx = []
    ry = []
    ryaw = []
    rk = []
    rdk = []
    s = []


    obs = []
    lines = []

    lines = [Line(LineNode(19,i/2), LineNode(19,(i+1)/2)) for i in range(80)]
    lines.extend([Line(LineNode(21,i/2), LineNode(21,(i+1)/2)) for i in range(80)])

    obs = [[16,10,obs_r],[17,10,obs_r],[18,10,obs_r],[19,10,obs_r],[19,30,obs_r],[19.5,30,obs_r],[20,30,obs_r],[24,20,obs_r],[23,20,obs_r],[22,20,obs_r],[21,20,obs_r],[20,20,obs_r]]


    fig = plt.figure(figsize=(7, 7))
    subplot = fig.add_subplot(111)
    subplot.set_xlabel('X-distance: m')
    subplot.set_ylabel('Y-distance: m')
    subplot.axis([15, 25, 25, 35])
    subplot.plot(start[0], start[1], '*r')
    subplot.plot(goal[0], goal[1], '*r')

    for OB in obs:
        circle = Circle(xy=(OB[0],OB[1]), radius=obs_r,alpha = 0.2)
        subplot.add_patch(circle)
        subplot.plot(OB[0],OB[1], 'xk')

    ax = plt.subplot()
    for LINE in lines:
        ax.plot([LINE.p1.x, LINE.p2.x], [LINE.p1.y, LINE.p2.y], color='black')

    rrts = RRTStar(start=start, goal=goal, obstacles=obs,lines=lines , epsilon=epsilon, delta=delta_goal_bias,max_iter=iter)
    
    rrts.run()  
    path = rrts.get_path()

    while check_rrtstar_success(path[-1], goal) :
        print("regeneration")
        rrts.run()
        path = rrts.get_path()
    

    obs = []
    obs = [[19,i] for i in range(40)]
    obs.extend([21,i] for i in range(40))
    obs.extend([[16,10],[17,10],[18,10],[19,10],[20,30],[16,30],[17,30],[18,30],[19,30],[20,30],[24,20],[23,20],[22,20],[21,20],[20,20]])

    

    eb = elastic_band.Elastic_Band(path=path,obs=obs)

    
    path = eb.elastic_band_path_planning()

    path = np.transpose(path)

    rx, ry, ryaw, rk , rdk, s = cubic_spline_planner.calc_spline_course(path[0],path[1],ds=0.5)

    path = np.transpose([rx,ry])
    
    ##################################
    #step_size 영향을 받는 부분, 지우면 모든 경로를 보여줌
    """
    path_ = []
    i = step_size
    while(i < len(path)) :
        path_.append(path[i])
        i+=step_size
    path = path_
    """
    ##################################
    px, py = [K[0] for K in path], [K[1] for K in path]  # x 좌표 목록, y 좌표 목록
    subplot.plot(px, py, '.b')

    print(time.time()-start_time)
    plt.show()
    

