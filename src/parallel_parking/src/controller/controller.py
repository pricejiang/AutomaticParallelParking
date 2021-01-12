import rospy
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from util.util import euler_to_quaternion, quaternion_to_euler

class VehicleController():

    def __init__(self, model_name='gem'):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.model_name = model_name

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        newAckermannCmd.steering_angle = 0
        # print(v, delta)
        print(newAckermannCmd)
        self.controlPub.publish(newAckermannCmd)

    def forward(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 1
        newAckermannCmd.steering_angle = 0

        self.controlPub.publish(newAckermannCmd)

    def execute(self, currentPose, targetPose):
        """
            This function takes the current state of the vehicle and
            the target state to compute low-level control input to the vehicle
            Inputs:
                currentPose: ModelState, the current state of vehicle
                targetPose: The desired state of the vehicle
        """

        currentEuler = quaternion_to_euler(currentPose.pose.orientation.x,
                                           currentPose.pose.orientation.y,
                                           currentPose.pose.orientation.z,
                                           currentPose.pose.orientation.w)

        print("target ", targetPose )
        target_v = targetPose[1]
        target_orientation = targetPose[0]

        k_s = 0.1
        k_ds = 1
        k_n = 0.1
        k_theta = 1

        #compute errors
        thetaError = target_orientation - currentEuler[2]
        curr_v = -np.sqrt(currentPose.twist.linear.x**2 + currentPose.twist.linear.y**2)
        vError = target_v - curr_v

        # Checking if the vehicle need to stop
        v = vError*k_ds
        delta = k_theta*thetaError

        if target_orientation == 0:
            delta = -0.57
        else:
            delta = 0.57

        #Send computed control input to vehicle
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = v
        newAckermannCmd.steering_angle = delta
        print(newAckermannCmd)
        self.controlPub.publish(newAckermannCmd)
