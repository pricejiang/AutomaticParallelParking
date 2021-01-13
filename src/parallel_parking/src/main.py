import rospy
import numpy as np
import argparse

from gazebo_msgs.msg import  ModelState
from controller.controller import VehicleController
from perception.perception import VehiclePerception
from decision.decision import VehicleDecision
from util.util import euler_to_quaternion, quaternion_to_euler
from std_msgs.msg import Float32
import time
import pickle


def run_model(model_name):
    global centerTheta

    rate = rospy.Rate(10)  # 100 Hz
    constructedMap = None
    perceptionModule = VehiclePerception(model_name)
    decisionModule = VehicleDecision()
    while constructedMap == None:
        constructedMap = perceptionModule.lidarReading()
    controlModule = VehicleController(model_name)

    centerTheta = decisionModule.parkingDecision(constructedMap)

    flag = 0
    idx = 0
    currState =  perceptionModule.gpsReading()

    currentEuler = quaternion_to_euler(currState.pose.orientation.x,
                                       currState.pose.orientation.y,
                                       currState.pose.orientation.z,
                                       currState.pose.orientation.w)

    init_euler = currentEuler[2]

    refState = [flag, -1.39136710179663975]
    controlModule.execute(currState, refState)
    while not rospy.is_shutdown():
        # res = sensors.lidarReading()
        if centerTheta == 0:
            continue
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

        refState = [flag, -1.39136710179663975]
        controlModule.execute(currState, refState)

        # if abs(currentEuler[2] - theta[idx]) < 0.05:
        print("idx: ", idx)

        if flag and abs(init_euler-currentEuler[2]) < 0.1:
            controlModule.forward()
            break
        print()



if __name__ == "__main__":
    rospy.init_node("gem_dynamics")

    run_model('gem')
