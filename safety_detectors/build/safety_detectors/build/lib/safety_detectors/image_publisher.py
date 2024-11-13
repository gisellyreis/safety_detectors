#!/usr/bin/env python3

import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class ImagePublisher(Node):
    
    def __init__(self, image_path):
        super().__init__("image_publisher")
        self.publisher = self.create_publisher(CompressedImage, "/image", 10)
        self.bridge = CvBridge()
        self.image_path = image_path
        self.publish_image()

    def publish_image(self):
        # Load and compress the image
        image = self.load_image(self.image_path)
        if image is not None:
            compressed_msg = self.compress_image(image)
            self.publisher.publish(compressed_msg)
            self.get_logger().info("Published image.")
        else:
            self.get_logger().error("Failed to load image.")

    def load_image(self, path):
        # Loads an image from the provided path.
        image = cv2.imread(path)
        if image is None:
            self.get_logger().error(f"Image at path '{path}' could not be loaded.")
        return image

    def compress_image(self, image):
        # Compresses an OpenCV image and returns a CompressedImage message.
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        
        success, compressed_image = cv2.imencode('.jpg', image)
        if not success:
            self.get_logger().error("Failed to compress the image.")
            return None
        
        msg.data = compressed_image.tobytes()
        return msg

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run safety_detectors image_publisher <image_path>")
        return

    image_path = sys.argv[1]
    node = ImagePublisher(image_path)
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
