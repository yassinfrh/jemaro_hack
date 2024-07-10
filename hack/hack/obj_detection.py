import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from .shapes import detectShapes, localizeShapes, drawShapes, contourProps, countCorners
from cv_bridge import CvBridge
from custom_msgs_pkg.msg import Shape, ListShape

import cv2
import numpy as np

# Colors in BGR
colors = {
        'green': (60, 100, 8),
        'blue': (85, 46, 18),
        'red': (7, 22, 136),
    }

# Shapes dictionary
shapes_dic = {
    3: 'triangle',
    4: 'square',
    0: 'circle'
}

# yellow...RGB: 255, 245, 0, HSV: 58, 100, 100 
# navy...RGB: 0, 40, 166, HSV: 226, 100, 65
# purple...RGB: 203, 86, 171, HSV: 316, 58, 80
# green...RGB: 0, 162, 87, HSV: 152, 100, 64
# blue...RGB: 0, 149, 250, HSV: 204, 100, 98
# red...RGB: 194, 38, 8, HSV: 10, 96, 76

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')

        # Subscriber to camera image
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        
        # Subscriber to camera info
        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10)
        
        # Publisher for detected shapes
        self.shape_pub = self.create_publisher(ListShape, '/shapes', 10)
        
        self.bridge = CvBridge()
        self.camera_info = {'k': [0, 0, 0, 0, 0, 0, 0, 0, 0]}
        self.shapes = []

    def image_callback(self, msg):
        # Convert ROS image to OpenCV image using cv_bridge
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert to grey scale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Blur the image
        blur = cv2.GaussianBlur(gray, (3, 3), 0)

        # Edge detection
        edge = cv2.Canny(blur, 20, 60)

        # Dilate and close the edges
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        edge = cv2.dilate(edge, kernel)
        edge = cv2.morphologyEx(edge, cv2.MORPH_CLOSE, kernel)

        # Show the edge image
        cv2.imshow('Edge Detection', edge)
        cv2.waitKey(1)
        
        # using a findContours() function 
        contours, _ = cv2.findContours( 
            edge, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        
        i = 0

        self.shapes = []
        
        # list for storing names of shapes 
        for contour in contours: 
        
            # here we are ignoring first counter because  
            # findcontour function detects whole image as shape 
            if i == 0: 
                i = 1
                continue

            # If the contour has too few points, ignore it
            if len(contour) < 5:
                continue


            cp = contourProps(contour)

            # If the area of contour is less than 1000 or greater than 20000, ignore it
            if cp['ca'] < 1000 or cp['ca'] > 16000:
                continue

            # reject non-regular (elongated) shapes
            [rx, ry] = cp['el'][1]
            if min(rx, ry) / max(rx, ry) < 0.65:
                continue

            [corners, ori, rad] = countCorners(contour, cp)

            # Offset the orientation by pi/4 and normalize it
            ori = (ori + np.pi/4) % (2*np.pi)


            shape = {
                'color': self.findColor(img, contour),
                'center': cp['cc'],
                'corners': corners,
                'ori': ori,
                'rad': rad,
                'cnt': contour
            }

            # Append the shape to the list of shapes if another one is not too close
            if len(self.shapes) == 0 or np.linalg.norm(np.array(shape['center']) - np.array(self.shapes[-1]['center'])) > 50:
                self.shapes.append(shape)
        
            # cv2.approxPloyDP() function to approximate the shape 
            '''approx = cv2.approxPolyDP( 
                contour, 0.02 * cv2.arcLength(contour, True), True) 
            
            # using drawContours() function 
            cv2.drawContours(img, [contour], 0, (0, 0, 255), 2) 
        
            # finding center point of shape 
            M = cv2.moments(contour) 
            if M['m00'] != 0.0: 
                x = int(M['m10']/M['m00']) 
                y = int(M['m01']/M['m00']) 
        
                # putting shape name at center of each shape 
                if len(approx) == 3: 
                    cv2.putText(img, 'Triangle', (x, y), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) 
            
                elif len(approx) == 4: 
                    cv2.putText(img, 'Quadrilateral', (x, y), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) 
            
                elif len(approx) > 6: 
                    cv2.putText(img, 'circle', (x, y), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) '''
            
        # Localize shapes
        self.shapes = localizeShapes(self.shapes, 0.05, self.camera_info)

        # Draw the shapes
        img = drawShapes(img, self.shapes, colors)

                
        # Display image
        cv2.imshow('Object Detection', img)
        cv2.waitKey(1)

        # Publish the shapes
        msg = ListShape()
        msg.shapes = []
        for shape in self.shapes:
            s = Shape()

            if shape['corners'] not in shapes_dic:
                continue

            s.shape = shapes_dic[shape['corners']]
            s.color = shape['color']
            s.x = shape['pos'][0]
            s.y = shape['pos'][1]
            s.z = shape['pos'][2]
            s.o = shape['ori']
            msg.shapes.append(s)
        self.shape_pub.publish(msg)

    def camera_info_callback(self, msg):
        self.camera_info['k'] = msg.k

    # Function to find the color of the shape
    def findColor(self, img, cnt):
        # Cut the shape from the image
        mask = np.zeros(img.shape[:2], np.uint8)
        cv2.drawContours(mask, [cnt], -1, 255, -1)

        # Calculate the mean color of the shape in RGB
        mean = cv2.mean(img, mask=mask)[:3]

        # Find the color with the a distance to the mean color less than 50
        for color, ref in colors.items():
            if np.linalg.norm(np.array(ref) - np.array(mean)) < 50:
                return color
        
        return 'None'

    

def main(args=None):
    rclpy.init(args=args)

    obj_detection = ObjectDetection()

    rclpy.spin(obj_detection)

    obj_detection.destroy_node()
    rclpy.shutdown()