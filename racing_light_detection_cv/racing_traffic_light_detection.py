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

import rclpy, cv2, cv_bridge, numpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ai_msgs.msg import Point
from geometry_msgs.msg import Point32

class TrafficLightDetectionNode(Node):
    def __init__(self):
        super().__init__('TrafficLightDetectionNode')

        self.declare_parameter("sub_img_topic","/image_raw")
        self.declare_parameter("pub_mask",False)
        self.declare_parameter("pub_result",False)
        self.declare_parameter("size_threshold",2000)

        self.lower_red_array = self.declare_parameter('lower_red_array', [150, 70, 250]).value
        self.upper_red_array = self.declare_parameter('upper_red_array', [200, 160, 255]).value
        self.lower_green_array = self.declare_parameter('lower_green_array', [35, 200, 200]).value
        self.upper_green_array = self.declare_parameter('upper_green_array', [100, 255, 255]).value
        self.get_logger().info("Start traffic light detect.")

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = self.create_subscription(Image, self.get_parameter('sub_img_topic').get_parameter_value().string_value, self.image_callback, 10)

        if (self.get_parameter('pub_mask').get_parameter_value().bool_value == True):
            self.pub_red = self.create_publisher(Image, '/process_image/mask_red', 10)
            self.pub_green = self.create_publisher(Image, '/process_image/mask_green', 10)
        if (self.get_parameter('pub_result').get_parameter_value().bool_value == True):
            self.pub_result = self.create_publisher(Image, '/process_image', 10)
        self.pub_point = self.create_publisher(Point, '/traffic_light_detection', 10)


    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = numpy.array(self.lower_red_array)
        upper_red = numpy.array(self.upper_red_array)
        mask_red = cv2.inRange(hsv, lower_red, upper_red)

        lower_green = numpy.array(self.lower_green_array)
        upper_green = numpy.array(self.upper_green_array)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        h, w, d = image.shape

        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours_red) > 0:
            point = Point()
            point32 = Point32()
            largest_contour = max(contours_red, key=cv2.contourArea)
            moments = cv2.moments(largest_contour)
            if moments['m00'] > self.get_parameter('size_threshold').get_parameter_value().integer_value:
                cx_red = int(moments['m10']/moments['m00'])
                cy_red = int(moments['m01']/moments['m00'])
                cv2.circle(image, (cx_red, cy_red), 5, (0,0,255), -1)
                point.type = "red"
                point32.x = float(cx_red)
                point32.y = float(cy_red)
                point.point.append(point32)
                self.pub_point.publish(point)

        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours_green) > 0:
            point = Point()
            point32 = Point32()
            largest_contour = max(contours_green, key=cv2.contourArea)
            moments = cv2.moments(largest_contour)
            if moments['m00'] > self.get_parameter('size_threshold').get_parameter_value().integer_value:
                cx_green = int(moments['m10']/moments['m00'])
                cy_green = int(moments['m01']/moments['m00'])
                cv2.circle(image, (cx_green, cy_green), 5, (0,255,0), -1)
                point.type = "green"
                point32.x = float(cx_green)
                point32.y = float(cy_green)
                point.point.append(point32)
                self.pub_point.publish(point)

        if (self.get_parameter('pub_mask').get_parameter_value().bool_value == True):
            self.pub_red.publish(self.bridge.cv2_to_imgmsg(mask_red, 'mono8'))
            self.pub_green.publish(self.bridge.cv2_to_imgmsg(mask_green, 'mono8'))
        if (self.get_parameter('pub_result').get_parameter_value().bool_value == True):
            self.pub_result.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))
        

def main(args=None):
    rclpy.init(args=args)    
    traffic_light_detection_node = TrafficLightDetectionNode()
    rclpy.spin(traffic_light_detection_node)
    traffic_light_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()