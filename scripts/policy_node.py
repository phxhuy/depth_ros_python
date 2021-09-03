import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from stable_baselines import PPO2
import sys
import matplotlib.pyplot as plt
np.set_printoptions(threshold=sys.maxsize)


def euler_to_quaternion(yaw, pitch=0, roll=0):
    quat = PoseStamped().pose.orientation
    quat.x = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    quat.y = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    quat.z = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    quat.w = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return quat

def yaw_from_quaternion(ori):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    #t0 = +2.0 * (w * x + y * z)
    #t1 = +1.0 - 2.0 * (x * x + y * y)
    #roll_x = np.atan2(t0, t1)

    #t2 = +2.0 * (w * y - z * x)
    #t2 = +1.0 if t2 > +1.0 else t2
    #t2 = -1.0 if t2 < -1.0 else t2
    #pitch_y = np.asin(t2)

    t3 = +2.0 * (ori.w * ori.z + ori.x * ori.y)
    t4 = +1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
    yaw_z = np.arctan2(t3, t4)

    return yaw_z #/np.pi*180  # in degrees

class PolicyNode():

    def __init__(self):
        self.current_pose = PoseStamped().pose
        self.depth_image = np.zeros((64, 64))
        self.observation = np.zeros((64, 64, 2))
        self.vector_observation = np.array([5, 0, 0])

        self.global_traj_x = 2
        self.global_traj_z = 1
        self.step_length = 1

        self.action_dictionary = {0: (np.cos(22.5 / 180 * np.pi), np.sin(22.5 / 180 * np.pi), 1),
                                  1: (np.cos(22.5 / 180 * np.pi), np.sin(22.5 / 180 * np.pi), 0),
                                  2: (1, 0, 1),
                                  3: (1, 0, 0),
                                  4: (1, 0, -1),
                                  5: (np.cos(22.5 / 180 * np.pi), -np.sin(22.5 / 180 * np.pi), 0),
                                  6: (np.cos(22.5 / 180 * np.pi), -np.sin(22.5 / 180 * np.pi), -1)}

        rospy.init_node('policy_node')

        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/processed_depth_images_array", Float32MultiArray, self.range_image_callback)
        self.pub_action = rospy.Publisher("/action", PoseStamped)

        self.model = PPO2.load("../models/best_model55325with_mean_rew53.17254135.pkl")

        self.r = rospy.Rate(10)
        self.r.sleep()

        while not rospy.is_shutdown():
            self.calculate_target_vector()
            self.observation[:, :, 0] = np.copy(self.depth_image)
            self.observation[0, 0:3, 1] = np.copy(self.vector_observation)
            discrete_action = self.model.predict(self.observation, deterministic=True)[0]
            action = self.action_dictionary[discrete_action]
            current_yaw = yaw_from_quaternion(self.current_pose.orientation)
            current_yaw += 0.5 * np.pi
            next_pose = PoseStamped()
            next_pose.header.frame_id = "map"
            next_pose.pose.position.y = self.current_pose.position.y + self.step_length * (- action[0]*np.cos(current_yaw) + action[1]*np.sin(current_yaw))
            next_pose.pose.position.x = self.current_pose.position.x + self.step_length * (action[0]*np.sin(current_yaw) + action[1]*np.cos(current_yaw))
            next_pose.pose.position.z = self.global_traj_z
            next_pose.pose.orientation = euler_to_quaternion(current_yaw + action[2] * 22.5 / 180 * np.pi - np.pi/2)
            self.pub_action.publish(next_pose)
            self.r.sleep()

    def pose_callback(self, data):
        self.current_pose = data.pose

    def range_image_callback(self, data):
        self.depth_image = np.array(data.data).reshape((64, 64))
        # print(self.depth_image)
        # plt.imshow(self.depth_image, cmap="gray")
        # plt.hist(self.depth_image.flatten(), bins=20)
        # plt.show()

    def calculate_target_vector(self):
        yaw = yaw_from_quaternion(self.current_pose.orientation)
        yaw += 0.5*np.pi
        self.vector_observation = self.vector_observation = np.array([5, 0, 0])
        vo = [5, 1*(self.global_traj_x - self.current_pose.position.x), 0]
        # print(vo, self.current_pose.position.x)
        self.vector_observation = np.array([vo[0] * np.cos(yaw) + vo[1] * np.sin(yaw),
                                            -vo[0] * np.sin(yaw) + vo[1] * np.cos(yaw),
                                            vo[2]])


policy_node = PolicyNode()