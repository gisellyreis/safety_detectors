import os
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from safety_detectors_env.msg import WaterClassificationResult
from keras.preprocessing.image import img_to_array # type: ignore
from keras.applications.vgg16 import preprocess_input # type: ignore
from keras.models import load_model # type: ignore
from ament_index_python.packages import get_package_share_directory


class WaterClassifierNode(Node):
    
    def __init__(self):
        super().__init__('water_classifier_node')

        package_share_directory = get_package_share_directory('safety_detectors')

        model_path = os.path.join(package_share_directory, 'models', 'water_classifier.keras')

        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file not found at {model_path}")
            return

        self.model = load_model(model_path)
        
        # Initialize the subscriber and publisher
        self.subscriber = self.create_subscription(
            CompressedImage, 
            '/image', 
            self.image_callback, 
            10
        )

        self.publisher = self.create_publisher(
            WaterClassificationResult, 
            '/water_classification', 
            10
        )

        
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Decode the compressed image to an OpenCV image
        self.get_logger().info('Received an image.')
        image = self.decompress_image(msg)
        
        if image is not None:
            # Preprocess the image for the model
            preprocessed_image = self.load_and_preprocess_image(image)
            
            # Run inference
            result = self.run_inference(preprocessed_image)
            
            # Publish the result (0 or 1)
            self.publish_result(result, msg)
        else:
            self.get_logger().error("Failed to decode received image.")

    def decompress_image(self, msg):
        # Decompresses the CompressedImage message into an OpenCV image.
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return image

    def load_and_preprocess_image(self, image, target_size=(224, 224)):
        # Preprocesses the image to the format required by the model.
        image = cv2.resize(image, target_size)
        image = img_to_array(image)  
        image = preprocess_input(image) 
        return np.expand_dims(image, axis=0) 

    def run_inference(self, image):
        # Runs inference on the image and returns the prediction.
        predictions = self.model.predict(image)
        predicted_class = np.argmax(predictions, axis=-1) 
        return predicted_class[0]  

    def publish_result(self, result, msg):
        water_classification_msg = WaterClassificationResult()
        water_classification_msg.result = int(result) 
        water_classification_msg.image = msg 
        self.publisher.publish(water_classification_msg)
        self.get_logger().info(f"Published result: {result}")


def main(args=None):
    rclpy.init(args=args)

    water_classifier_node = WaterClassifierNode()
    
    rclpy.spin(water_classifier_node)

    water_classifier_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
