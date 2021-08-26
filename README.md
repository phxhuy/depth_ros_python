# depth_ros_python
a ros package to handle basic depth sensor operation such as
 
- [IP_basic](https://github.com/kujason/ip_basic)

![Alt Text](https://media.giphy.com/media/y30CmdlpwmXLxyXlEz/giphy.gif)
## dependencies
- cv2
- numpy
- pyrealsense2
- librealsense

install the dependencies by:

```
pip install pyrealsense2

```

## Setup

1. Clone the package
2. Install all dependencies
3. Build the ros package using catkin build / catkin_make
4. Run the package by:

```
rosrun depth_ros_python node_depth_py.py

```

The ros node will publish a topic called "processed_depth_images"