import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from .shapes import detectShapes, localizeShapes, drawShapes
from cv_bridge import CvBridge

import cv2

# Colors in Hue (HSV)
colors = {
        "yellow": 60,
        "navy": 240,
        "purple": 300,
        "green": 120,
        "blue": 210,
        "red": 0
    }

cameraInfo = {
        'k': [907.4107666015625, 0.0, 648.4189453125,
              0.0, 907.2595825195312, 357.08447265625,
              0.0, 0.0, 1.0]
    }

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')

        # Subscriber to camera image
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS image to OpenCV image using cv_bridge
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Do object detection
        shapes = detectShapes(cv_image, colors)
        for shp in shapes:
            print(shp['color'])
            
        # TODO: fix color vocabulary

        #shapes = localizeShapes(shapes, 0.1, cameraInfo)
        #img = drawShapes(cv_image, shapes, colors)
        # Display image
        #cv2.imshow('Object Detection', img)
        #cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    obj_detection = ObjectDetection()

    rclpy.spin(obj_detection)

    obj_detection.destroy_node()
    rclpy.shutdown()