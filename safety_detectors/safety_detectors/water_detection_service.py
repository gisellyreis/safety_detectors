import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from safety_detectors_env.srv import ClassifyWaterImage
import cv2
import numpy as np
from ultralytics import YOLO
from keras.models import load_model  # type: ignore
from keras.preprocessing.image import img_to_array  # type: ignore
from keras.applications.vgg16 import preprocess_input  # type: ignore
from ament_index_python.packages import get_package_share_directory

class WaterDetectionService(Node):
    def __init__(self):
        super().__init__('water_detection_service')
        self.srv = self.create_service(ClassifyWaterImage, 'classify_water_image', self.classify_image_callback)

        package_share_directory = get_package_share_directory('safety_detectors')

        classification_model_path = os.path.join(package_share_directory, 'models', 'water_classifier.keras')
        detection_model_path = os.path.join(package_share_directory, 'models', 'best.pt')

        self.classification_model = load_model(classification_model_path)
        self.detection_model = YOLO(detection_model_path)
    
    def classify_image_callback(self, request, response):
        image = self.decompress_image(request.image)

        if image is not None:
            preprocessed_image = self.preprocess_image(image)
            result = self.run_classification(preprocessed_image)
            response.result = int(result)
            
            if result == 1:
                self.get_logger().info("Water detected, running detection model.")
                detection_image = self.run_detection(image)
                if detection_image is not None:
                    response.image = self.compress_image(detection_image)
            else:
                response.image = request.image
                self.get_logger().info("No water detected.")

        return response
    
    def decompress_image(self, compressed_image):
        np_arr = np.frombuffer(compressed_image.data, dtype=np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return image
    
    def preprocess_image(self, image, target_size=(224, 224)):
        image = cv2.resize(image, target_size)
        image = img_to_array(image)
        image = preprocess_input(image)
        return np.expand_dims(image, axis=0)
    
    def run_classification(self, image):
        predictions = self.classification_model.predict(image)
        predicted_class = np.argmax(predictions, axis=-1)
        return int(predicted_class[0])
    
    def run_detection(self, image):
        results = self.detection_model(image, save=False)
        if results and len(results) > 0:
            annotated_image = results[0].plot()
            return annotated_image
        else:
            self.get_logger().warning("Detection model returned no results.")
            return None
    
    def compress_image(self, image):
        compressed_image = CompressedImage()
        compressed_image.header.stamp = self.get_clock().now().to_msg()
        compressed_image.format = "jpeg"
        success, encoded_image = cv2.imencode('.jpg', image)
        if success:
            compressed_image.data = encoded_image.tobytes()
            return compressed_image
        else:
            self.get_logger().error("Failed to compress image.")
            return None


def main(args=None):
    rclpy.init(args=args)

    service = WaterDetectionService()
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
