import rospy
import numpy as np
import argparse

from gazebo_msgs.msg import  ModelState
from controller.controller import VehicleController
from perception.perception import VehiclePerception
from decision.decision import VehicleDecision
from util.util import euler_to_quaternion, quaternion_to_euler
import time
import pickle

theta = [0.0, 0.029583925305344615, 0.05921408009757669, 0.08891685844998544, 0.11871923782174466, 0.14864900822938693, 0.17873502021788523, 0.20900745811205132, 0.2394981462471894, 0.2702408975430531, 0.30127191604150266, 0.3326302680845194, 0.36435844096767855, 0.3965030136083065, 0.42911547167611425, 0.462253210741976, 0.49598078683709085, 0.5303714967799077, 0.565509404573289, 0.6014919814386465, 0.6384336063695326, 0.6764703002107149, 0.7157662731815019, 0.7565232176174645, 0.7989939015237478, 0.8435027802322914, 0.8316451742336197, 0.7887586880595199, 0.7476499956014122, 0.7080527038869882, 0.6697552837559617, 0.6325862272957427, 0.596403975305336, 0.5610898642583626, 0.5265430538914333, 0.49267679392202773, 0.459415620083236, 0.42669320983989334, 0.39445071573451745, 0.36263545059753965, 0.33119983593872526, 0.3001005498118582, 0.26929782759867926, 0.23875488114737822, 0.20843741021055845, 0.17831318624891773, 0.14835169312001675, 0.11852381243930675, 0.08880154381169787, 0.05915775191234876, 0.029565933701624508, 0.0]
speed = [0.05296577757114048, 0.05296577757114048, 0.05296577757114048, 0.05296577757114048, 0.14397591070183482, 0.14397591070183482, 0.14397591070183482, 0.14397591070183482, 0.14397591070183482, 0.14397591070183482, 0.14397591070183482, 0.14397591070183482, 0.14397591070183482, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.39136710179663975, 0.14397591070183482, 0.14397591070183482, 0.14397591070183482, 0.14397591070183482, 0.14397591070183482, 0.14397591070183482, 0.14397591070183482, 0.14397591070183482, 0.14397591070183482, 0.05296577757114048, 0.05296577757114048, 0.05296577757114048]
def gaussian(x, mu=len(theta)/2, sigma=15):
    return 1/(sigma * np.sqrt(2 * np.pi)) * np.exp( - (x - mu)**2 / (2 * sigma**2))

def run_model(model_name):
    rospy.init_node("gem_dynamics")
    rate = rospy.Rate(10)  # 100 Hz   

    assert(len(theta) == len(speed)) 

    perceptionModule = VehiclePerception(model_name)
    # decisionModule = VehicleDecision('./waypoints')
    controlModule = VehicleController(model_name)
    centerTheta = max(theta)
    flag = 0
    idx = 0
    currState =  perceptionModule.gpsReading()

    refState = [flag, -speed[idx]-1]
    controlModule.execute(currState, refState)
    while not rospy.is_shutdown():
        # res = sensors.lidarReading()
        # print(res)
        rate.sleep()  # Wait a while before trying to get a new state

        # Get the current position and orientation of the vehicle
        currState =  perceptionModule.gpsReading()

        currentEuler = quaternion_to_euler(currState.pose.orientation.x,
                                           currState.pose.orientation.y,
                                           currState.pose.orientation.z,
                                           currState.pose.orientation.w)
        
        print("Current heading: ", currentEuler[2])
        print("flag is ", flag)
        if currentEuler[2] > centerTheta and not flag:
            flag = 1

        refState = [flag, -speed[idx]-1]
        controlModule.execute(currState, refState)

        if abs(currentEuler[2] - theta[idx]) < 0.05:
            print("idx: ", idx)
            idx += 1
        if idx == len(speed):
            controlModule.stop()
            break
        print()

if __name__ == "__main__":
    run_model('gem')
    