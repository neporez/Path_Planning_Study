import numpy as np
import os

class tf_txt_to_npy() :
    def __init__(self) :
        self.SAVE_ROOT = (os.path.abspath(os.path.dirname(__file__)))+"/npy_file/"
        self.PATH_ROOT = (os.path.abspath(os.path.dirname(__file__)))+"/txt_file/"
        self.np_path_array = []
        self.np_obstacle_array = []

    def load_file_path(self, file_name) :
        with open(self.PATH_ROOT + file_name) as f:
            self.np_path_array = []
            for line in f:
                point = []
                x,y = line.split(' ')
                point = [float(x),float(y)]
                self.np_path_array.append(point)

    def load_file_obstacle(self, file_name) :
        with open(self.PATH_ROOT + file_name) as f:
            self.np_obstacle_array = []
            for line in f:
                point = []
                x,y = line.split(' ')
                point = [float(x),float(y)]
                self.np_obstacle_array.append(point)\

    def save_numpy_obstacle(self, file_name):
        np.save(self.SAVE_ROOT + file_name, arr = self.np_obstacle_array)

    def save_numpy_path(self, file_name):
        np.save(self.SAVE_ROOT + file_name, arr = self.np_path_array)

def file_conversion():
    tf = tf_txt_to_npy()
    try :
        tf.load_file_path("path.txt")
        tf.save_numpy_path("path.npy")
    except :
        print("path failed")

    try :
        tf.load_file_obstacle("obstacle.txt")
        tf.save_numpy_obstacle("obstacle.npy")
    except :
        print("obstacle failed")

if __name__ == '__main__' :
    file_conversion()

