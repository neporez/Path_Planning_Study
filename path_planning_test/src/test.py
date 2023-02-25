import os
import numpy as np
import matplotlib.pyplot as plt

dir_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "..", "path" , "npy_file")


data_path = os.path.join(dir_path, "path.npy")
data_obstacle = os.path.join(dir_path, "obstacle.npy")

pathArray = np.load(data_path)
obstacleArray = np.load(data_obstacle)

px = pathArray[0:pathArray.shape[0]-1, 0]
py = pathArray[0:pathArray.shape[0]-1, 1]

ox = obstacleArray[0:obstacleArray.shape[0]-1, 0]
oy = obstacleArray[0:obstacleArray.shape[0]-1, 1]

minX = min(min(px),min(ox))-10
maxX = max(max(px),max(ox))+10
minY = min(min(py),min(oy))-10
maxY = max(max(py),max(oy))+10

fig = plt.figure(figsize=(7, 7))
subplot = fig.add_subplot(111)
subplot.set_xlabel('X-distance: m')
subplot.set_ylabel('Y-distance: m')
subplot.axis([minX, maxX, minY, maxY])

for i in range(len(px)) :
    subplot.plot(px[i],py[i], '.k')

for i in range(len(ox)) :
    subplot.plot(ox[i],oy[i], 'xr')

plt.show()
