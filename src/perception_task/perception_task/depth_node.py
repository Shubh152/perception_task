#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import PointCloud2 as pc2
from geometry_msgs.msg import Point
import cv2
import numpy as np

class DepthImageProcessor(Node):
    def __init__(self):
        super().__init__('depth_image_processor')
        self.x_coord = 320
        self.y_coord = 240
        
        self.bridge = CvBridge()

        # self.depth_subscriber = self.create_subscription(
        #     Image,
        #     '/depth_camera/depth_image',  # Adjust the topic name based on your setup
        #     self.depth_callback,
        #     10
        # )


        self.point_subscribe = self.create_subscription(
            pc2,
            '/depth_camera/points',
            self.pixel_to_3d_point,
            10
        )

        self.point_publisher = self.create_publisher(
            Point,
            '/depth_camera/coord',
            10
        )
        
        self.rgb_subscriber = self.create_subscription(
            Image,
            '/depth_camera/image', 
            self.rgb_callback,
            10
        )

    def pixel_to_3d_point(self,pCloud):
        v = self.y_coord
        u =  self.x_coord
        # Convert from u (column / width), v (row/height) to position in array
        # where X,Y,Z data starts
        array_position = v * pCloud.row_step + u * pCloud.point_step
        
        # Compute position in array where x,y,z data start
        array_pos_x = array_position + pCloud.fields[0].offset  # X has an offset of 0
        array_pos_y = array_position + pCloud.fields[1].offset  # Y has an offset of 4
        array_pos_z = array_position + pCloud.fields[2].offset  # Z has an offset of 8
        
        # Extract X, Y, Z coordinates from the point cloud data
        X = np.frombuffer(pCloud.data[array_pos_x:array_pos_x + 4], dtype=np.float32)[0]
        Y = np.frombuffer(pCloud.data[array_pos_y:array_pos_y + 4], dtype=np.float32)[0]
        Z = np.frombuffer(pCloud.data[array_pos_z:array_pos_z + 4], dtype=np.float32)[0]
        
        # Create a geometry_msgs Point object and assign X, Y, Z to it
        point = Point()
        point.x = float(X)
        point.y = float(Y)
        point.z = float(Z)
        
        self.get_logger().info("Publishing point")
        self.point_publisher.publish(point)

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
