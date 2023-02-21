"""
https://github.com/ShuiXinYun/Path_Plan/tree/master/Artificial_Potential_Field
"개선된 인공 유력장 방법을 구현한 인공 유력장 알고리즘. 불가능 문제를 해결하였으나, 여전히 지역 최소점 문제가 존재한다."
"""
from Original_APF import APF, Vector2d
import matplotlib.pyplot as plt
import math
from matplotlib.patches import Circle
import random
import sys,os

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from elastic_band import elastic_band


def check_vec_angle(v1: Vector2d, v2: Vector2d):
    v1_v2 = v1.deltaX * v2.deltaX + v1.deltaY * v2.deltaY
    angle = math.acos(v1_v2 / (v1.length * v2.length)) * 180 / math.pi
    return angle


class APF_Improved(APF):
    def __init__(self, start, goal, obstacles, k_att: float, k_rep: float, rr: float,
        step_size: float, max_iters: int, goal_threshold: float, is_plot=False):
        self.start = Vector2d(start[0], start[1])
        self.current_pos = Vector2d(start[0], start[1])
        self.goal = Vector2d(goal[0], goal[1])
        self.obstacles = [Vector2d(OB[0], OB[1]) for OB in obstacles]
        self.k_att = k_att
        self.k_rep = k_rep
        self.rr = rr  # 반동 범위
        self.step_size = step_size
        self.max_iters = max_iters
        self.iters = 0
        self.goal_threashold = goal_threshold
        self.path = list()
        self.is_path_plan_success = False
        self.is_plot = is_plot
        self.delta_t = 0.01

    def repulsion(self):
        """
        저항 계산, 개선된 저항 함수, 달성 불가능성 문제 해결
        :return: 저항 계산
        """
        rep = Vector2d(0, 0)  # 모든 장애물의 전체 반력
        for obstacle in self.obstacles:
            # obstacle = Vector2d(0, 0)
            obs_to_rob = self.current_pos - obstacle
            rob_to_goal = self.goal - self.current_pos
            if (obs_to_rob.length > self.rr):  # 장애물 마찰력 영향 범위를 벗어난
                pass
            else:
                rep_1 = Vector2d(obs_to_rob.direction[0], obs_to_rob.direction[1]) * self.k_rep * (
                        1.0 / obs_to_rob.length - 1.0 / self.rr) / (obs_to_rob.length ** 2) * (rob_to_goal.length ** 2)
                rep_2 = Vector2d(rob_to_goal.direction[0], rob_to_goal.direction[1]) * self.k_rep * ((1.0 / obs_to_rob.length - 1.0 / self.rr) ** 2) * rob_to_goal.length
                rep +=(rep_1+rep_2)
        return rep


if __name__ == '__main__':
    # 관련 매개 변수 설정
    k_att, k_rep = 10.0, 2
    rr = 5
    step_size, max_iters, goal_threashold = .2, 500, .2  # "보폭 0.5로 1000회 경로탐색 시 소요시간은 4.37초이고, 보폭 0.1로 1000회 경로탐색 시 소요시간은 21초입니다."
    step_size_ = 1

    # 시작점과 도착점 설정 및 그리기
    start, goal = (20,0), (20, 40)
    #start, goal = (0,0), (20,20)
    is_plot = True
    if is_plot:
        fig = plt.figure(figsize=(7, 7))
        subplot = fig.add_subplot(111)
        subplot.set_xlabel('X-distance: m')
        subplot.set_ylabel('Y-distance: m')
        subplot.axis([0, 40, 0, 40])
        #subplot.axis([-1, 21, -1, 21])
        subplot.plot(start[0], start[1], '*r')
        subplot.plot(goal[0], goal[1], '*r')
    # '장애물 설정 및 그리기'
    obs = []
    obs = [[15,i] for i in range(40)]
    obs.extend([25,i] for i in range(40))
    obs.extend([[16,10],[17,10],[18,10],[19,10],[20,30],[16,30],[17,30],[18,30],[19,30],[20,30],[24,20],[23,20],[22,20],[21,20],[20,20]])
    for i in range(0):
        obs.append([random.uniform(2, goal[1] - 1), random.uniform(2, goal[1] - 1)])

    if is_plot:
        for OB in obs:
            circle = Circle(xy=(OB[0], OB[1]), radius=rr, alpha=0.3)
            subplot.add_patch(circle)
            subplot.plot(OB[0], OB[1], 'xk')
    # t1 = time.time()
    # for i in range(1000):

    # path plan
    if is_plot:
        apf = APF_Improved(start, goal, obs, k_att, k_rep, rr, step_size, max_iters, goal_threashold, is_plot)
    else:
        apf = APF_Improved(start, goal, obs, k_att, k_rep, rr, step_size, max_iters, goal_threashold, is_plot)
    apf.path_plan()


    if apf.is_path_plan_success:
        path_ = []
        eb = elastic_band.Elastic_Band(path=apf.path,obs=obs)
        path = eb.elastic_band_path_planning()

        i = int(step_size_ / step_size)
        while (i < len(path)):
            path_.append(path[i])
            i += int(step_size_ / step_size)



        if path_[-1] != path[-1]:  # '최종 포인트 추가'
            path_.append(path[-1])
        #print('planed path points:{}'.format(path))
        #print('planed path points:{}'.format(path_))
        print('path plan success')
        if is_plot:
            px, py = [K[0] for K in path], [K[1] for K in path]  # x 좌표 목록, y 좌표 목록
            subplot.plot(px, py, '.k')
            plt.show()
    else:
        print('path plan failed')
    # t2 = time.time()
    #print('1000번의 경로 탐색에 소요된 시간: {}, 한 번의 경로 탐색에 소요된 시간: {}'.format(t2-t1, (t2-t1)/1000)


