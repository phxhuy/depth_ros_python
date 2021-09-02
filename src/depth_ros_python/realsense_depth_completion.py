import rospy
import cv2
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
   import pyrealsense2.pyrealsense2 as rs2
np.set_printoptions(threshold=sys.maxsize)

from ip_basic import depth_map_utils
from ip_basic import vis_utils

class ImageListener:
    def __init__(self, depth_image_topic, depth_info_topic):

        self.pub_array_image = rospy.Publisher('/processed_depth_images_array', Float32MultiArray, queue_size=10)
        self.pub_image = rospy.Publisher('/processed_depth_images', msg_Image, queue_size=10)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(depth_image_topic, msg_Image, self.imageDepthCallback)

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

            if cv_image.shape[1] is not 640:
                mid_point = cv_image.shape[1]/2
                cv_image = cv_image[:, mid_point-320:mid_point+320]

            cv_image_resized = np.ones([640, 640]).astype(np.uint16)
            cv_image_resized = cv_image_resized * 15000  #cv_image[cv_image > 0].max()

            cv_image_resized[80:560, :] = cv_image

            # cv_image_resized = cv2.resize(cv_image, (640, 640))

            # cv2.imshow('depth', cv_image_resized)
            # cv2.waitKey(0)

            # IP_basic process
            # projected_depths = np.float32(cv_image_resized / 256.0)
            # Fill in
            extrapolate = True
            blur_type = 'gaussian'
            final_depths = depth_map_utils.fill_in_fast(cv_image_resized, extrapolate=extrapolate, blur_type=blur_type)
            final_depths = np.clip(final_depths, 0, 15000)
            mask = final_depths > 5000
            np.put(final_depths, np.where(mask.flatten()), 15000)
            # plt.ion()
            # plt.imshow(final_depths, cmap="gray")
            # plt.show(block=False)

            float_image = cv2.resize(final_depths[80:360, :], (64, 64))


            cv_image_msg = msg_Image()
            cv_image_msg = self.bridge.cv2_to_imgmsg(float_image, encoding="16UC1")

            image_array = Float32MultiArray()
            image_array.data = list((float_image.reshape(4096)/15000.))
        
            self.pub_array_image.publish(image_array)
            self.pub_image.publish(cv_image_msg)


        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return


def main():
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)

    depth_image_topic = '/camera/depth/image_rect_raw'
    depth_info_topic = '/camera/depth/camera_info'

    print ('')
    print ('process raw depth image')
    print ('--------------------')
    print ('Please match raw depth topics correctly. (default: /camera/depth/image_rect_raw)')
    print ('')
    print ('Application will process raw depth image ')
    
    listener = ImageListener(depth_image_topic, depth_info_topic)
    rospy.spin()

# if __name__ == '__main__':
#     main()
