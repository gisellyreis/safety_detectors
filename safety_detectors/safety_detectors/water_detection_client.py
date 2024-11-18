import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from safety_detectors_env.srv import ClassifyWaterImage
import cv2

class WaterDetectionClient(Node):
    def __init__(self):
        super().__init__('water_detection_client')
        self.cli = self.create_client(ClassifyWaterImage, 'classify_water_image')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
    
    def send_request(self, image_path):
        cv_image = cv2.imread(image_path)
        compressed_image = CompressedImage()
        compressed_image.header.stamp = self.get_clock().now().to_msg()
        compressed_image.format = "jpeg"
        
        success, encoded_image = cv2.imencode('.jpg', cv_image)
        if success:
            compressed_image.data = encoded_image.tobytes()
        else:
            self.get_logger().error("Failed to encode image.")
            return
        
        request = ClassifyWaterImage.Request()
        request.image = compressed_image
        self.future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
    
    def decompress_image(self, compressed_image):
        np_arr = np.frombuffer(compressed_image, dtype=np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return image
    
    def process_response(self, response):
        if response.result == 1:
            self.get_logger().info("Water detected. Displaying annotated image.")
            detected_image = self.decompress_image(response.image.data)
            cv2.imshow("Detection Results", detected_image)

            while True:
                key = cv2.waitKey(1)
                if key == 27:  # Escape key to close the window
                    break
                
            cv2.destroyAllWindows()
        else:
            self.get_logger().info("No water detected in the image.")


def main(args=None):
    rclpy.init(args=args)

    image_path = sys.argv[1]

    client = WaterDetectionClient()
    client.send_request(image_path)

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
                client.process_response(response)
            except Exception as e:
                client.get_logger().info(
                    'Service call failed: %r' % (e,))
            break

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()