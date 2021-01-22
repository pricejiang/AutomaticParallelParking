# from parallel_parking.src.decision.parking import calcParking
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
from parking import calcParking
from std_msgs.msg import Float32

class VehicleDecision():
    def __init__(self):
        pass

    def parkingDecision(self, grid):
        gridMap = np.array(grid.data).reshape((42, 42))
        info = grid.info
        origin = info.origin

        obstacle = np.where(gridMap == 1)
        # print(origin)

        obstacle = zip(obstacle[0], obstacle[1])
        kmeans = KMeans(n_clusters=2, random_state=0).fit(obstacle)
        # print(obstacle)
        # print(kmeans.labels_)
        # print(kmeans.cluster_centers_)

        # dy = obstacle[np.argmin(((20,20) - np.array(obstacle))**2)]
        dy = 0
        mindist = float('inf')
        nearest_obs = None
        for obs in obstacle:
            dist = np.sqrt((obs[0]-20)**2 + (obs[1]-20)**2)
            if dist < mindist:
                nearest_obs = obs
                mindist = dist
        # print(mindist - 0.7)
        dy = mindist - 0.7
        # print(nearest_obs)
        k1, k2 = kmeans.cluster_centers_

        ye = k1[1]
        if k2[0] < k1[0]:
            xe = k2[0]
        else:
            xe = k1[0]

        parkSide = 1
        if ye < 20:
            parkSide = -1

        centerTheta = calcParking(xe, 20, dy)

        return centerTheta, parkSide

if __name__ == "__main__":
    rospy.init_node("VechileDecision")

    VehicleDecision()

    rospy.spin()
