import os
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32  
from keras.preprocessing.image import img_to_array
from keras.applications.vgg16 import preprocess_input
from keras.models import load_model
from cv_bridge import CvBridge

class WaterDetectionNode(Node):
    
    def __init__(self):
        super().__init__('water_detection_node')

        model_path = os.path.join(os.getcwd(), 'src/safety_detectors/safety_detectors/water_classifier.keras')

        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file not found at {model_path}")
            return

        
        # Load the model
        self.model = load_model('water_classifier.keras')
        
        # Initialize the subscriber and publisher
        self.subscriber = self.create_subscription(
            CompressedImage, 
            '/image', 
            self.image_callback, 
            10
        )
        self.publisher = self.create_publisher(
            Int32,  
            '/water_detection', 
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
            self.publish_result(result)
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

    def publish_result(self, result):
        # Publishes the inference results to the /water_detection topic.
        msg = Int32()
        msg.data = result
        self.publisher.publish(msg)
        self.get_logger().info(f"Published result: {result}")

def main(args=None):
    rclpy.init(args=args)

    water_detection_node = WaterDetectionNode()
    
    rclpy.spin(water_detection_node)

    water_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
