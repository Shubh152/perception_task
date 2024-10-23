#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class DepthImageProcessor(Node):
    def __init__(self):
        super().__init__('depth_image_processor')
        self.x_coord = 320
        self.y_coord = 240
        # Initialize the CvBridge
        self.bridge = CvBridge()

        self.depth_subscriber = self.create_subscription(
            Image,
            '/depth_camera/depth_image',  # Adjust the topic name based on your setup
            self.depth_callback,
            10
        )

        self.depth_subscriber = self.create_subscription(
            Image,
            '/depth_camera/image',  # Adjust the topic name based on your setup
            self.rgb_callback,
            10
        )

    def rgb_callback(self,msg):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting rgb image: {e}')
            return
        def set_coord(event,x,y,flag,params):
            if event == cv2.EVENT_LBUTTONDOWN:
                self.y_coord = y
                self.x_coord = x
            return 
        cv2.imshow('RGB Image', rgb_image) 
        cv2.setMouseCallback('RGB Image',set_coord)
        cv2.waitKey(0)
        cv2.destroyAllWindows() 

    def depth_callback(self, msg):
        try:
            # Convert the ROS depth image message to an OpenCV image
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting depth image: {e}')
            return        
        
        x = self.x_coord
        y = self.y_coord

        if x >= 0 and x < depth_image.shape[1] and y >= 0 and y < depth_image.shape[0]:
            
            depth_value = depth_image[y, x]
            # Handle invalid depth (in case of no return, NaN, or 0 depth)
            if np.isnan(depth_value) or depth_value <= 0:
                self.get_logger().warn(f'Invalid depth at ({x}, {y}): {depth_value}')
            else:
                self.get_logger().info(f'Depth at ({x}, {y}): {depth_value:.3f} meters')
        else:
            self.get_logger().warn(f'Coordinates ({x}, {y}) are out of bounds')

def main(args=None):
    rclpy.init(args=args)
    depth_image_processor = DepthImageProcessor()

    try:
        rclpy.spin(depth_image_processor)
    except KeyboardInterrupt:
        depth_image_processor.get_logger().info('Shutting down depth image processor node')
    finally:
        depth_image_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
