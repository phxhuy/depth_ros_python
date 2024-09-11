import rospy
import cv2
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo

import ros_numpy

import sys
import os
import numpy as np
import copy
import time

from ip_basic import depth_map_utils
from ip_basic import vis_utils

class ImageListener:
    def __init__(self, depth_image_topic, depth_info_topic):

        self.pub_image = rospy.Publisher('/processed_depth_images', msg_Image, queue_size=10)

        self.sub = rospy.Subscriber(depth_image_topic, msg_Image, self.imageDepthCallback, queue_size=1)
        self.skip_counter = 1

    def imageDepthCallback(self, data):
        if self.skip_counter >= 1:
            # self.skip_counter = 0
            try:
                print "image delay: ", (rospy.Time.now() - data.header.stamp)/1000000, " ms"
                start_t = time.time()

                cv_image = ros_numpy.numpify(data)
                # cv_image_resized = cv2.resize(cv_image, (212, 120))
                resize_factor = 2.0 #6.625 #1.0
                cv_image_resized = cv2.resize(cv_image, (int(848/resize_factor), int(480/resize_factor)))
                print "shape changed from:",cv_image.shape, "to:" , cv_image_resized.shape

                # if cv_image.shape[1] is not 640:
                #     mid_point = cv_image.shape[1]/2
                #     cv_image = cv_image[:, mid_point-320:mid_point+320]
                # cv_image_resized = np.ones([640, 640]).astype(np.uint16)
                # cv_image_resized = cv_image_resized * 15000  #cv_image[cv_image > 0].max()
                # cv_image_resized[80:560, :] = cv_image
                # for image size 424x240
                # cv_image_resized = np.ones([424, 424]).astype(np.uint16)
                # cv_image_resized = cv_image_resized * 15000  #cv_image[cv_image > 0].max()


                # IP_basic process
                # Fill in

                """Fast, in-place depth completion.
                def fill_in_fast(depth_map, max_depth=100.0, custom_kernel=DIAMOND_KERNEL_5,
                            extrapolate=False, blur_type='bilateral'):
                Args:
                    depth_map: projected depths
                    max_depth: max depth value for inversion
                    custom_kernel: kernel to apply initial dilation
                    extrapolate: whether to extrapolate by extending depths to top of
                        the frame, and applying a 31x31 full kernel dilation
                    blur_type:
                        'bilateral' - preserves local structure (recommended)
                        'gaussian' - provides lower RMSE

                Returns:
                    depth_map: dense depth map


                # 3x3 cross kernel
                CROSS_KERNEL_3 = np.asarray(
                    [
                        [0, 1, 0],
                        [1, 1, 1],
                        [0, 1, 0],
                    ], dtype=np.uint8)


                # 5x5 diamond kernel
                DIAMOND_KERNEL_5 = np.array(
                    [
                        [0, 0, 1, 0, 0],
                        [0, 1, 1, 1, 0],
                        [1, 1, 1, 1, 1],
                        [0, 1, 1, 1, 0],
                        [0, 0, 1, 0, 0],
                    ], dtype=np.uint8)
                """


                # projected_depths = np.float32(cv_image_resized / 256.0)
                extrapolate = True
                blur_type = 'gaussian'
                # final_depths = depth_map_utils.fill_in_fast(cv_image, custom_kernel=DIAMOND_KERNEL_5, extrapolate=extrapolate, blur_type=blur_type)
                final_depths = depth_map_utils.fill_in_fast(cv_image_resized, extrapolate=extrapolate, blur_type=blur_type)



                #process the depth image with max depth
                final_depths = np.clip(final_depths, 0, 15000)
                mask = final_depths > 5000
                np.put(final_depths, np.where(mask.flatten()), 15000)


                # float_image = cv2.resize(final_depths[80:360, :], (64, 64))
                # float_image = cv2.resize(final_depths, (640, 480))

                cv_image_msg = msg_Image()
                
                # cv_image_msg = ros_numpy.msgify(msg_Image, float_image, encoding="16UC1")
                float_image = cv2.resize(final_depths, (128, 96))

                cv_image_msg = ros_numpy.msgify(msg_Image, float_image, encoding="16UC1")

                cv_image_msg.header.stamp.secs = data.header.stamp.secs
                cv_image_msg.header.stamp.nsecs = data.header.stamp.nsecs

                self.pub_image.publish(cv_image_msg)
                end_t = time.time()
                print("time elapsed: ", end_t - start_t)


            except ValueError as e:
                print(e)
                return

        else:
            self.skip_counter += 1
            print("skipped one image")

    def pose_callback(self, data):
        self.current_pose = data


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
