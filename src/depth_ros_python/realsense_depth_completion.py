import rospy
import cv2
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

from ip_basic import depth_map_utils
from ip_basic import vis_utils

class ImageListener:
    def __init__(self, depth_image_topic, depth_info_topic):

        self.pub =  rospy.Publisher('/processed_depth_images', msg_Image, queue_size=10)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(depth_image_topic, msg_Image, self.imageDepthCallback)

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            
            cv_image_resized = np.ones([640, 640]).astype(np.uint16)
            cv_image_resized = cv_image_resized * 10000;  #cv_image[cv_image > 0].max()

            cv_image_resized[80:560,:] = cv_image

            # cv2.imshow('depth', cv_image_resized)
            # cv2.waitKey(0)

            # IP_basic process
            # projected_depths = np.float32(cv_image_resized / 256.0)
            # Fill in
            extrapolate = True
            blur_type = 'gaussian'
            final_depths = depth_map_utils.fill_in_fast(cv_image_resized, extrapolate=extrapolate, blur_type=blur_type)

            cv_image_msg = msg_Image()
            cv_image_msg = self.bridge.cv2_to_imgmsg(final_depths, encoding="16UC1")
        
            self.pub.publish(cv_image_msg)


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
