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

class RRTStar:
    def __init__(self, start, goal, obstacles, epsilon, delta, max_iter):
        self.start = Node(start[0], start[1]) #출발점 좌표(x,y)
        self.goal = Node(goal[0], goal[1]) #목적지 좌표(x,y)
        self.obstacles = obstacles #장애물의 좌표와 반지름 리스트르 [x,y,r]
        self.epsilon = epsilon #트리에서 노드 간의 거리
        self.delta = delta #목적지까지의 확률적 편향 정도
        self.max_iter = max_iter#최대 반복 횟수
        self.nodes = [self.start]

    def run(self):
        
        for i in range(self.max_iter):
            q = self.sample()
            nearest_node = self.find_nearest(q)
            new_node = self.steer(q, nearest_node)
            if self.check_collision(new_node):
                near_nodes = self.find_near(new_node)
                self.rewire(new_node, near_nodes)
                self.nodes.append(new_node)
        ax = plt.subplot()
        for node in self.nodes :
            if node.parent :
                  None
                  #ax.plot([node.x, node.parent.x], [node.y, node.parent.y], color='green')




    def sample(self):
        if random.random() > self.delta:
            theta = math.atan2(self.goal.y - self.start.y, self.goal.x - self.start.x)
            dist = math.sqrt((self.goal.y - self.start.y)**2 + (self.goal.x - self.start.x)**2)

            # Generate a random point within a fan-shaped area around the line connecting start and goal nodes
            radius = random.uniform(0, dist)
            angle = random.uniform(-math.pi/3, math.pi/3)
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

    def check_collision(self, node):
        for o in self.obstacles:
            if (node.x - o[0]) ** 2 + (node.y - o[1]) ** 2 <= o[2] ** 2:
                return False
        return True

    def find_near(self, node):
        #radius = min(50, math.sqrt(100 * math.log(len(self.nodes)) / len(self.nodes)))
        radius = 1
        distances = [(n.x - node.x) ** 2 + (n.y - node.y) ** 2 for n in self.nodes]
        return [n for i, n in enumerate(self.nodes) if distances[i] < radius ** 2]

    def rewire(self, new_node, near_nodes):
        for n in near_nodes:
            cost = new_node.cost + math.sqrt((n.x - new_node.x) ** 2 + (n.y - new_node.y) ** 2)
            if cost < n.cost and self.check_collision(n):
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


if __name__ == "__main__" :
    start_time = time.time()
    start, goal = (20,25), (20, 35)
    epsilon = 0.2
    delta_goal_bias = 0.2
    iter = 500
    obs_r = 0.7
    step_size = 5

    rx = []
    ry = []
    ryaw = []
    rk = []
    rdk = []
    s = []


    obs = []
    obs = [[18,i,obs_r] for i in range(40)]
    obs.extend([22,i,obs_r] for i in range(40))
    obs.extend([[16,10,obs_r],[17,10,obs_r],[18,10,obs_r],[19,10,obs_r],[20,30,obs_r],[19,30,obs_r],[20,30,obs_r],[21,20,obs_r],[20,20,obs_r]])

    fig = plt.figure(figsize=(7, 7))
    subplot = fig.add_subplot(111)
    subplot.set_xlabel('X-distance: m')
    subplot.set_ylabel('Y-distance: m')
    subplot.axis([10, 30, 20, 40])
    subplot.plot(start[0], start[1], '*r')
    subplot.plot(goal[0], goal[1], '*r')

    for OB in obs:
        circle = Circle(xy=(OB[0],OB[1]), radius=obs_r,alpha = 0.2)
        subplot.add_patch(circle)
        subplot.plot(OB[0],OB[1], 'xk')


    rrts = RRTStar(start=start, goal=goal, obstacles=obs, epsilon=epsilon, delta=delta_goal_bias,max_iter=iter)
    rrts.run()

    path = rrts.get_path()

    

    obs = []
    obs = [[18,i] for i in range(40)]
    obs.extend([22,i] for i in range(40))
    obs.extend([[16,10],[17,10],[18,10],[19,10],[20,30],[16,30],[17,30],[18,30],[19,30],[20,30],[24,20],[23,20],[22,20],[21,20],[20,20]])

    

    eb = elastic_band.Elastic_Band(path=path,obs=obs)

    path = eb.elastic_band_path_planning()

    path = np.transpose(path)

    rx, ry, ryaw, rk , rdk, s = cubic_spline_planner.calc_spline_course(path[0],path[1],ds=0.1)

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
    subplot.plot(px, py, '.g')
    print(time.time()-start_time)
    plt.show()

    

