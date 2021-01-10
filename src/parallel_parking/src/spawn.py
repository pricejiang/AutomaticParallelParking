
import rospy, time
from gazebo_msgs.srv    import SpawnModel, SpawnModelRequest, SpawnModelResponse, DeleteModel
from copy               import deepcopy

from sklearn import cluster
from util.util import euler_to_quaternion
from geometry_msgs.msg  import Point, PointStamped, Twist, Pose, PoseStamped
from std_msgs.msg       import Float64, String

# TODO: Use this class to create waypoint object and obstacle in Gazebo
class Spawn:

    def __init__(self):
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        self.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.count = 0
        self.paths = {}

        # NOTE: This is the object description which is a cube; may be change to other object
        self.sdf_cube = """<?xml version="1.0" ?>
        <sdf version="1.4">
          <model name="MODELNAME">
            <static>0</static>
            <link name="link">
              <inertial>
                <mass>5000.0</mass>
                <inertia>
                  <ixx>0.01</ixx>
                  <ixy>0.0</ixy>
                  <ixz>0.0</ixz>
                  <iyy>0.01</iyy>
                  <iyz>0.0</iyz>
                  <izz>0.01</izz>
                </inertia>
              </inertial>
              <collision name="stairs_collision0">
                <pose>0 0 0 0 0 0</pose>
                <geometry>
                  <box>
                    <size>SIZEXYZ</size>
                  </box>
                </geometry>
                <surface>
                  <bounce />
                  <friction>
                    <ode>
                      <mu>1.0</mu>
                      <mu2>1.0</mu2>
                    </ode>
                  </friction>
                  <contact>
                    <ode>
                      <kp>10000000.0</kp>
                      <kd>1.0</kd>
                      <min_depth>0.0</min_depth>
                      <max_vel>0.0</max_vel>
                    </ode>
                  </contact>
                </surface>
              </collision>
              <visual name="stairs_visual0">
                <pose>0 0 0 0 0 0</pose>
                <geometry>
                  <box>
                    <size>SIZEXYZ</size>
                  </box>
                </geometry>
                <material>
                  <script>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    <name>Gazebo/Wood</name>
                  </script>
                </material>
              </visual>
              <velocity_decay>
                <linear>0.000000</linear>
                <angular>0.000000</angular>
              </velocity_decay>
              <self_collide>0</self_collide>
              <kinematic>0</kinematic>
              <gravity>0</gravity>
            </link>
          </model>
        </sdf>
        """

    def create_point(self, x, y, z, idx):
        req = self.create_cube_request("waypoint"+str(idx),
                                   x, y, z,  # position
                                   0.0, 0.0, 0.0,  # rotation
                                   0.5, 0.5, 0.2)  # size
        self.spawn_model(req)
        self.count += 1
        rospy.sleep(0.01)
        return idx

    def delete_point(self, idx):
        self.delete_model("waypoint"+str(idx))
        rospy.sleep(0.01)

    def create_cube_request(self, modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
        """Create a SpawnModelRequest with the parameters of the cube given.
        modelname: name of the model for gazebo
        px py pz: position of the cube (and it's collision cube)
        rr rp ry: rotation (roll, pitch, yaw) of the model
        sx sy sz: size of the cube"""
        cube = deepcopy(self.sdf_cube)
        # Replace size of model
        size_str = str(round(sx, 3)) + " " + \
            str(round(sy, 3)) + " " + str(round(sz, 3))
        cube = cube.replace('SIZEXYZ', size_str)
        # Replace modelname
        cube = cube.replace('MODELNAME', str(modelname))

        req = SpawnModelRequest()
        req.model_name = modelname
        req.model_xml = cube
        req.initial_pose.position.x = px
        req.initial_pose.position.y = py
        req.initial_pose.position.z = pz

        q = euler_to_quaternion([rr, rp, ry])
        req.initial_pose.orientation.x = q[0]
        req.initial_pose.orientation.y = q[1]
        req.initial_pose.orientation.z = q[2]
        req.initial_pose.orientation.w = q[3]

        return req


# ----------------------------------------------------
def remove(msg, args):
    if('TRUE' in str(msg) ):
        print("_____________remove___________________")
        track = args
        for point in range(track.count):
            track.delete_point(point)
            time.sleep(0.5)
    

def add(msg, args):
    print("_____________add___________________")
    track = args
    pos_x = msg.pose.position.x
    pos_y = msg.pose.position.y
    pos_z = msg.pose.position.z
    track.create_point(pos_x, pos_y, pos_z, track.count)



def main():
    rospy.init_node("Spawn_model", anonymous=True)

    track = Spawn()

    # path = [(4.342337681591619, -102.00005280713374), (3.3423376815916193, -102.00005280713374), (3.3423376815916193, -103.00005280713374), (2.3423376815916193, -102.00005280713374), (1.3423376815916193, -102.00005280713374), (0.34233768159161926, -102.00005280713374), (-0.6576623184083807, -102.00005280713374), (-1.6576623184083807, -102.00005280713374), (-2.6576623184083807, -102.00005280713374), (-9.65766231840838, -102.00005280713374), (-9.65766231840838, -103.00005280713374), (-9.65766231840838, -104.00005280713374), (-10.65766231840838, -102.00005280713374), (-10.65766231840838, -103.00005280713374), (-10.65766231840838, -104.00005280713374), (-11.65766231840838, -102.00005280713374), (-11.65766231840838, -103.00005280713374), (-11.65766231840838, -104.00005280713374), (-12.657662318408384, -102.00005280713374), (-12.657662318408384, -103.00005280713374), (-12.657662318408384, -104.00005280713374), (-13.657662318408384, -102.00005280713374), (-14.657662318408384, -102.00005280713374), (-15.657662318408384, -102.00005280713374), (-16.657662318408384, -102.00005280713374)]

    path = [(-11.609859558236511, -104.40494430424025), (-11.509859558236512, -104.40346589985867), (-11.409859558236512, -104.39902680318825), (-11.309859558236512, -104.39161531246546), (-11.209859558236513, -104.38121175254763), (-11.109859558236513, -104.36778821098005), (-11.009859558236514, -104.35130815690759), (-10.909859558236514, -104.33172593045505), (-10.809859558236514, -104.30898608545019), (-10.709859558236515, -104.28302256254915), (-10.609859558236515, -104.25375766251318), (-10.509859558236515, -104.22110077998643), (-10.409859558236516, -104.18494684580969), (-10.309859558236516, -104.1451744094901), (-10.209859558236516, -104.10164327121315), (-10.109859558236517, -104.05419154218225), (-10.009859558236517, -104.00263196925708), (-9.909859558236517, -103.94674729891835), (-9.809859558236518, -103.88628436720435), (-9.709859558236518, -103.82094647145934), (-9.609859558236518, -103.75038338173948), (-9.509859558236519, -103.67417804240819), (-9.40985955823652, -103.59182852376276), (-9.30985955823652, -103.50272297438052), (-9.20985955823652, -103.40610394014719), (-9.10985955823652, -103.30101594159567), (-9.00985955823652, -103.18622554837356), (-8.909859558236521, -103.0600938870353), (-8.809859558236521, -102.92036146474098), (-8.709859558236522, -102.76375765217426), (-8.609859558236522, -102.58521952041369), (-8.509859558236522, -102.3760974036911), (-8.409859558236523, -102.11903554306586), (-8.309859558236523, -101.76588137412072), (-8.231769709290528, -101.19994430424025), (-8.131769709290529, -100.54271234130668), (-8.03176970929053, -100.21890014479264), (-7.9317697092905295, -99.97469212023695), (-7.83176970929053, -99.77331668791197), (-7.73176970929053, -99.60015647831787), (-7.631769709290531, -99.4476024725978), (-7.531769709290531, -99.31108629168611), (-7.431769709290531, -99.18760680904084), (-7.331769709290532, -99.07506578495115), (-7.231769709290532, -98.97192764498548), (-7.131769709290532, -98.8770286793915), (-7.031769709290533, -98.7894624525003), (-6.931769709290533, -98.7085071897748), (-6.831769709290533, -98.63357773380478), (-6.731769709290534, -98.56419260610811), (-6.631769709290534, -98.49995074361638), (-6.5317697092905345, -98.44051464872456), (-6.431769709290535, -98.38559791848144), (-6.331769709290535, -98.33495584149985), (-6.2317697092905355, -98.28837819288565), (-6.131769709290536, -98.24568363593657), (-6.031769709290536, -98.20671531979616), (-5.931769709290537, -98.17133738206573), (-5.831769709290537, -98.13943214671058), (-5.731769709290537, -98.1108978639164), (-5.631769709290538, -98.08564687827163), (-5.531769709290538, -98.06360414014425), (-5.431769709290538, -98.0447059958977), (-5.331769709290539, -98.02889920798145), (-5.231769709290539, -98.01614016752167), (-5.131769709290539, -98.00639427091768), (-5.03176970929054, -97.9996354388888), (-4.93176970929054, -97.99584576196469)]

    cluster = [[-11.27876376, -102.74016973], [1.99901401, -102.10128084]]


    c = 0
    for p in path:
        print("create")
        track.create_point(p[0], p[1], 2, c)
        c+=1
    for i in range(len(path)):
        track.delete_point(i)

    rospy.spin()




if __name__ == '__main__':
    main()
    

    
   