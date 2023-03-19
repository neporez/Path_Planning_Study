import rospy
import time
import os, sys
import numpy as np

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float64
from math import pi


from global_path import GlobalPath
import frenet_path as frenet_path



# Cost Weight
W_OFFSET = 1 #safety cost 가중치
W_CONSISTENCY = 0.5 #smoothness cost 가중치
# MACARON_TREAD = 3 # 충돌 지름
ROAD_WIDTH = 3.0 # 예선 : 3.0 본선 : 4.0

#parameter
sl_d = 0.5      # sl 경로 사이의 거리 (m)

# mode 1은 곡률, mode 2 는 yaw값 비교
mode = 2


class TrajectoryPlanner: # path planner

    def __init__(self, glob_path):
        self.last_selected_path = frenet_path.Frenet_path() # for consistency cost
        self.glob_path = glob_path

        #중앙차선
        PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/" #/home/gigi/catkin_ws/src/macaron_3/
        self.center = []
        #self.center.extend(np.load(file=PATH_ROOT+"obstacle/" + "curb_PJ1.npy")) # 팔정도
        # self.center.extend(np.load(file=PATH_ROOT+"obstacle/" + "kcity_tryout_solidline.npy")) #kcity
        # self.center.extend(np.load(file=PATH_ROOT+"obstacle/" + "kcity_final_busline.npy")) #kcity
        # self.center.extend(np.load(file=PATH_ROOT+"obstacle/" + "kcity_final_busline.npy")) #kcity
        # self.center.extend(np.load(file=PATH_ROOT+"obstacle/" + "kcity_static_center.npy")) #kcity
        # self.center.extend(np.load(file=PATH_ROOT+"path/" + "center_12_29.npy")) #kcity
        # self.center.extend(np.load(file=PATH_ROOT+"obstacle/" + "snu_small_obstacle_yellow.npy"))
        # self.center.extend(np.load(file=PATH_ROOT+"obstacle/" + "mh_stop_green.npy"))
        self.center.extend(np.load(file=PATH_ROOT+"path/" + "wonline10203.npy"))
        self.center.extend(np.load(file=PATH_ROOT+"path/" + "wonline20203.npy"))

        # self.center.extend(np.load(file=PATH_ROOT+"obstacle/" + "snu_bus_line.npy"))
        # self.center.extend(np.load(file=PATH_ROOT+"obstacle/" + "PG_solidline.npy")) #대운동장

        self.selected_pub = rospy.Publisher('/SLpath', PointCloud, queue_size = 3)
        self.curvature_pub = rospy.Publisher("curvature", Float64, queue_size=1)

        self.obstacle_time = 0
        self.visual = True

        self.current_s = 0
        self.current_q = 0

        self.S_MARGIN = 3 #5    # 생성한 경로 끝 추가로 경로 따라서 생성할 길이
        self.S_MARGINadd = 5
        self.collision_count = False


    def visual_selected(self, selected_path) :
        self.sl_path = PointCloud()

        # sl_x = selected_path[0].x
        # sl_y = selected_path[0].y

        for i in range(len(selected_path.x)):
            p = Point32()
            p.x = selected_path.x[i]
            p.y = selected_path.y[i]
            p.z = 0
            self.sl_path.points.append(p)

        self.selected_pub.publish(self.sl_path)


    def max_curvature_pub(self, selected_path, collision_count, path_len, heading):

        if mode == 1:
            max_max = max(selected_path.k[((path_len*2) + 1) :])
            min_min = abs(min(selected_path.k[((path_len*2) + 1) :]))
            # print((selected_path.yaw[-1]))
            # print(selected_path.k[((path_len*2) + 1) :])

            if max_max <= min_min:
                max_max = min_min

            # 경로를 펼치면 무조건 최대 감속
            if collision_count == True:
                max_max = 1.4

        ##############헤딩과 마지막 yaw 값 비교방식##############
        elif mode == 2:
            path_yaw = selected_path.yaw[-1]
            max_max = abs(heading - path_yaw)

            # yaw 값이 360도를 넘어갈때
            if max_max >= pi:
                max_max = 2*pi - max_max

            # 경로를 펼치면 무조건 최대 감속
            if collision_count == True:
                self.obstacle_time = time.time() + 3
                max_max = 90 * pi/180
            if self.obstacle_time > time.time():
                max_max = 90 * pi/180

        max_cur = Float64()
        max_cur.data = max_max

        self.curvature_pub.publish(max_cur)


    def set_global_path(self, glob_path):
        self.glob_path = glob_path


    def generate_path()

