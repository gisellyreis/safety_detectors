import os
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from safety_detectors_env.msg import WaterClassificationResult
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class WaterDetectionNode(Node):

    def __init__(self):
        super().__init__('water_detection_node')

        package_share_directory = get_package_share_directory('safety_detectors')

        model_path = os.path.join(package_share_directory, 'models', 'best.pt')
        
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        
        self.subscriber = self.create_subscription(
            WaterClassificationResult,
            '/water_classification',
            self.classification_callback,
            10
        )
        self.detection_publisher = self.create_publisher(
            CompressedImage, 
            '/detection', 
            10
        )

    def classification_callback(self, msg):
        if msg.result == 1:
            self.get_logger().info("Classified as water, running detection.")
            image = self.decompress_image(msg.image)
            if image is not None:
                self.run_detection(image)
            else:
                self.get_logger().error("Failed to decompress image.")
        else:
            self.get_logger().info("No water detected.")

    def decompress_image(self, msg):
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return image

    def run_detection(self, image):
        results = self.model(image, save=False)
        annotated_image = results[0].plot()

        cv2.imshow("Detection Results", annotated_image)
        cv2.waitKey(1)

        self.publish_detection(annotated_image)

    def publish_detection(self, image):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        success, compressed_image = cv2.imencode('.jpg', image)
        if success:
            msg.data = compressed_image.tobytes()
            self.detection_publisher.publish(msg)
            self.get_logger().info("Published detection result.")
        else:
            self.get_logger().error("Failed to compress image.")

def main(args=None):
    rclpy.init(args=args)

    water_detection_node = WaterDetectionNode()
    rclpy.spin(water_detection_node)

    water_detection_node.destroy_node()
    cv2.destroyAllWindows()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
