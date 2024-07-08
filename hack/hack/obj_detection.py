import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from shapes import detectShapes, localizeShapes, drawShapes

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')

        # Subscriber to camera image
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        

    def image_callback(self, msg):
        # Do object detection
        result = ObjectDetectionResult()
        result.objects = detected_objects
        self.publisher_.publish(result)