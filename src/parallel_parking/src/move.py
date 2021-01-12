import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import time
import numpy as np
from util.util import euler_to_quaternion

def move(pose, quat=[0, 0, 0, 0]):
    '''
        This function will directly set car's state (including poses and orientations)
        :param state_msg: current set model's state message
        :param pose: the pose to be set
        :return: Nothing
    '''
    state_msg = ModelState()
    state_msg.model_name = "gem"
    rospy.loginfo("Currently on (%f, %f)", pose[0], pose[1])
    state_msg.pose.position.x = pose[0]
    state_msg.pose.position.y = pose[1]
    state_msg.pose.position.z = 0.3
    state_msg.pose.orientation.x = quat[0]
    state_msg.pose.orientation.y = quat[1]
    state_msg.pose.orientation.z = quat[2]
    state_msg.pose.orientation.w = quat[3]

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed with %s", e)

if __name__ == '__main__':
    #rospy.init_node("move_node")
    path = [(-11.609859558236511, -104.40494430424025), (-11.509859558236512, -104.40346589985867), (-11.409859558236512, -104.39902680318825), (-11.309859558236512, -104.39161531246546), (-11.209859558236513, -104.38121175254763), (-11.109859558236513, -104.36778821098005), (-11.009859558236514, -104.35130815690759), (-10.909859558236514, -104.33172593045505), (-10.809859558236514, -104.30898608545019), (-10.709859558236515, -104.28302256254915), (-10.609859558236515, -104.25375766251318), (-10.509859558236515, -104.22110077998643), (-10.409859558236516, -104.18494684580969), (-10.309859558236516, -104.1451744094901), (-10.209859558236516, -104.10164327121315), (-10.109859558236517, -104.05419154218225), (-10.009859558236517, -104.00263196925708), (-9.909859558236517, -103.94674729891835), (-9.809859558236518, -103.88628436720435), (-9.709859558236518, -103.82094647145934), (-9.609859558236518, -103.75038338173948), (-9.509859558236519, -103.67417804240819), (-9.40985955823652, -103.59182852376276), (-9.30985955823652, -103.50272297438052), (-9.20985955823652, -103.40610394014719), (-9.10985955823652, -103.30101594159567), (-9.00985955823652, -103.18622554837356), (-8.909859558236521, -103.0600938870353), (-8.809859558236521, -102.92036146474098), (-8.709859558236522, -102.76375765217426), (-8.609859558236522, -102.58521952041369), (-8.509859558236522, -102.3760974036911), (-8.409859558236523, -102.11903554306586), (-8.309859558236523, -101.76588137412072), (-8.231769709290528, -101.19994430424025), (-8.131769709290529, -100.54271234130668), (-8.03176970929053, -100.21890014479264), (-7.9317697092905295, -99.97469212023695), (-7.83176970929053, -99.77331668791197), (-7.73176970929053, -99.60015647831787), (-7.631769709290531, -99.4476024725978), (-7.531769709290531, -99.31108629168611), (-7.431769709290531, -99.18760680904084), (-7.331769709290532, -99.07506578495115), (-7.231769709290532, -98.97192764498548), (-7.131769709290532, -98.8770286793915), (-7.031769709290533, -98.7894624525003), (-6.931769709290533, -98.7085071897748), (-6.831769709290533, -98.63357773380478), (-6.731769709290534, -98.56419260610811), (-6.631769709290534, -98.49995074361638), (-6.5317697092905345, -98.44051464872456), (-6.431769709290535, -98.38559791848144), (-6.331769709290535, -98.33495584149985), (-6.2317697092905355, -98.28837819288565), (-6.131769709290536, -98.24568363593657), (-6.031769709290536, -98.20671531979616), (-5.931769709290537, -98.17133738206573), (-5.831769709290537, -98.13943214671058), (-5.731769709290537, -98.1108978639164), (-5.631769709290538, -98.08564687827163), (-5.531769709290538, -98.06360414014425), (-5.431769709290538, -98.0447059958977), (-5.331769709290539, -98.02889920798145), (-5.231769709290539, -98.01614016752167), (-5.131769709290539, -98.00639427091768), (-5.03176970929054, -97.9996354388888), (-4.93176970929054, -97.99584576196469)]

    path.reverse()

    xs, ys = path[0]
    xe, ye = path[-1]

    eulers = [0,0,0]
    R_min_ = 3.38276381085053

    arr = []

    for i, p in enumerate(path):
        if i < len(path)/2:
            x, y = p
            l2 = (xs-x)*(xs-x) + (ys-y)*(ys-y)
            # print(l2)
            # print((2*R_min_*R_min_ - l2)/(2*R_min_*R_min_))
            eulers[2] = np.arccos((2*R_min_*R_min_ - l2)/(2*R_min_*R_min_))
        else:
            x, y = p
            l2 = (xe-x)*(xe-x) + (ye-y)*(ye-y)
            # print(l2)
            # print((2*R_min_*R_min_ - l2)/(2*R_min_*R_min_))
            eulers[2] = np.arccos((2*R_min_*R_min_ - l2)/(2*R_min_*R_min_))
        quat = euler_to_quaternion(eulers)
        arr.append(eulers[2])
        # move(p, quat)
        # time.sleep(0.1)
    print(arr)
    #rospy.spin()