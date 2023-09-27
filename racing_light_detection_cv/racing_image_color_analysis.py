# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy, cv2, cv_bridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

image = None

def get_hsv_value(event, x, y, flags, param):
    global image 

    if event == cv2.EVENT_LBUTTONDOWN:
        if image is not None:
            pixel_bgr = image[y, x]
            pixel_hsv = cv2.cvtColor(np.uint8([[pixel_bgr]]), cv2.COLOR_BGR2HSV)
            print(f"HSV Value at ({x}, {y}): {pixel_hsv[0][0]}")

def image_callback(msg):
    global image
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

def main():
    global image
    rclpy.init()
    node = rclpy.create_node('image_viewer_node')
    node.declare_parameter("sub_image_topic","/image_raw")
    cv2.namedWindow('Image Viewer')
    cv2.setMouseCallback('Image Viewer', get_hsv_value)

    image_subscription = node.create_subscription(
        Image, node.get_parameter('sub_image_topic').get_parameter_value().string_value, image_callback, 10)

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            if image is not None:
                cv2.imshow('Image Viewer', image)
                key = cv2.waitKey(1) & 0xFF
                if key == 27: 
                    break
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
