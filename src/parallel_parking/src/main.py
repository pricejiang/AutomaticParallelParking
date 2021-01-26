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

verbose = False

def main(model_name):
    perceptionModule = VehiclePerception(model_name)
    decisionModule = VehicleDecision()
    controlModule = VehicleController(model_name)
    parkCar(perceptionModule, decisionModule, controlModule)

def parkCar(perceptionModule, decisionModule, controlModule):
    rate = rospy.Rate(10)  # 10 Hz
    constructedMap = None
    # perceptionModule = VehiclePerception(model_name)
    # decisionModule = VehicleDecision()
    while constructedMap == None:
        constructedMap = perceptionModule.lidarReading()
    # controlModule = VehicleController(model_name)

    centerTheta, parkSide = decisionModule.parkingDecision(constructedMap)

    flag = -parkSide
    currState =  perceptionModule.gpsReading()

    currentEuler = quaternion_to_euler(currState.pose.orientation.x,
                                       currState.pose.orientation.y,
                                       currState.pose.orientation.z,
                                       currState.pose.orientation.w)

    init_euler = currentEuler[2]
    centerTheta += init_euler
    refState = [flag, -1.39136710179663975]
    controlModule.execute(currState, refState)

    print("parkSide:", parkSide)
    print("centerTheta: ", centerTheta)

    while not rospy.is_shutdown():

        rate.sleep()  # Wait a while before trying to get a new state

        # Get the current position and orientation of the vehicle
        currState =  perceptionModule.gpsReading()

        currentEuler = quaternion_to_euler(currState.pose.orientation.x,
                                           currState.pose.orientation.y,
                                           currState.pose.orientation.z,
                                           currState.pose.orientation.w)
        if verbose:
            print("Current heading: ", currentEuler[2])
            print("flag is ", flag)

        if flag == -parkSide:
            if (parkSide == 1 and currentEuler[2] > centerTheta) or (parkSide == -1 and currentEuler[2] < -centerTheta):
                flag *= -1

        refState = [flag, -1.39136710179663975]
        controlModule.execute(currState, refState)

        if flag == parkSide and abs(init_euler-currentEuler[2]) < 0.1:
            controlModule.forward()
            break
        # print(" ")

if __name__ == "__main__":
    rospy.init_node("gem_dynamics")

    main('gem')
