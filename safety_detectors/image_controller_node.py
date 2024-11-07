#!/usr/bin/env python3

import os

import numpy as np
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


class ImageControllerNode(Node):
    
    def __init__(self):
        super().__init__("image_controller_node")
        
        # Initialize the publisher and subscriber
        self.publisher = self.create_publisher(
            CompressedImage, "/image/pub", 10)
        self.subscriber = self.create_subscription(
            CompressedImage, "/image/sub", self.image_callback, 10)
        
        self.bridge = CvBridge()
        
        # Publish the image
        self.create_image()

    def image_callback(self, msg):
        self.get_logger().info('Received an image.')
        
        # Decode the compressed image
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
        if image is not None:
            self.get_logger().info("Image decoded successfully, displaying...")
            cv2.imshow("Received Image", image)
            cv2.waitKey(2)  # Display the image window briefly to allow it to refresh
        else:
            self.get_logger().error("Failed to decode received image.")

    def create_image(self):
        # Create CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"

        # Load the image
        image = cv2.imread("/home/giselly/ros2_ws/src/safety_detectors/safety_detectors/lake-3.jpeg")
        
        if image is None:
            self.get_logger().error("Image failed to load. Please check the file path.")
            return

        # Encode the image to JPEG
        success, compressed_image = cv2.imencode('.jpg', image)
        
        if not success:
            self.get_logger().error("Failed to encode the image.")
            return
        
        msg.data = compressed_image.tobytes()
        self.publisher.publish(msg)
        self.get_logger().info("Published an image.")



def main(args=None):
    rclpy.init(args=args)

    node = ImageControllerNode()
    rclpy.spin(node)

    cv2.destroyAllWindows()
    node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
