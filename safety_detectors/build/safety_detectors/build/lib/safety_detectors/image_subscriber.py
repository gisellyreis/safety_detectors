#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    
    def __init__(self):
        super().__init__("image_subscriber")
        self.subscriber = self.create_subscription(
            CompressedImage, "/image", self.image_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info("Image Subscriber Node initialized, waiting for images...")

    def image_callback(self, msg):
        self.get_logger().info('Received an image.')
        
        # Decompress the image and display
        image = self.decompress_image(msg)
        if image is not None:
            self.display_image(image)
        else:
            self.get_logger().error("Failed to decompress the image.")

    def decompress_image(self, msg):
        # Decompresses a CompressedImage message into an OpenCV image.
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return image

    def display_image(self, image):
        # Displays an OpenCV image in a window.
        if image is not None:
            cv2.imshow("Received Image", image)
            cv2.waitKey(2)
        else:
            self.get_logger().error("Cannot display image.")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
