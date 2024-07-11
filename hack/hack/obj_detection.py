import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from .shapes import detectShapes, localizeShapes, drawShapes, contourProps, countCorners
from cv_bridge import CvBridge
from custom_msgs_pkg.msg import Shape, ListShape
import tf2_ros
from scipy.spatial.transform import Rotation

import cv2
import numpy as np
import PIL

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
        self.camera_info = {'k': [1, 1, 1, 1, 1, 1, 1, 1, 1]}
        self.shapes = []

        # TransformListener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.from_frame = 'magician_base_link'
        self.to_frame =  'camera_lens'

        # Timer for lookup transform
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.T = np.eye(4)

    def timer_callback(self):
        try:
            # Trans is in the form of geometry_msgs.msg.TransformStamped
            trans = self.tfBuffer.lookup_transform(self.from_frame, self.to_frame, rclpy.time.Time())

            # Convert quaternion to rotation matrix
            rot = Rotation.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])

            # Get the translation
            [x, y, z] = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]

            # Build the transformation matrix
            self.T = np.eye(4)
            self.T[:3, :3] = rot.as_matrix()
            self.T[:3, 3] = [x, y, z]

            #print(self.T)
               
        except tf2_ros.TransformException as ex:
                    self.get_logger().warn(
                        f'Could not transform {self.from_frame} to {self.to_frame}: {ex}')
                    return
        

    def image_callback(self, msg):
        # Convert ROS image to OpenCV image using cv_bridge
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Threshold the image to get only the colors
        mask = cv2.inRange(hsv, (0, 90, 0), (180, 255, 255))

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(img, img, mask=mask)

        # Show the image
        cv2.imshow('Image', res)
        cv2.waitKey(1)

        # Convert to grey scale
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

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
            if cp['ca'] < 1000 or cp['ca'] > 20000:
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

            '''# Append the shape to the list of shapes if another one is not too close
            if len(self.shapes) == 0 or np.linalg.norm(np.array(shape['center']) - np.array(self.shapes[-1]['center'])) > 50:
                self.shapes.append(shape)
            # Otherwise update the position of the shape
            else:
                self.shapes[-1] = shape'''
            
            # Check if the shape is already in the list (close to another shape)
            close = False
            idx = -1
            for i, sh in enumerate(self.shapes):
                if np.linalg.norm(np.array(shape['center']) - np.array(sh['center'])) < 50:
                    close = True
                    idx = i
                    break

            # If the shape is close to another one, update the position
            if close:
                self.shapes[idx] = shape
            else:
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
            s = Shape()        #print(self.detected_shapes)


            if shape['corners'] not in shapes_dic:
                continue

            s.shape = shapes_dic[shape['corners']]
            s.color = shape['color']
            x = shape['pos'][0]
            y = shape['pos'][1]
            z = shape['pos'][2]

            #print(f'x{x} y{y} z{z}')

            # Convert angle to rad
            angle = np.deg2rad(shape['ori']) 


            # Build rotation matrix from angle around z-axis
            rot = np.array([[np.cos(angle), -np.sin(angle), 0],
                            [np.sin(angle), np.cos(angle), 0],
                            [0, 0, 1]])
            
            # Build the transformation matrix
            T_obj = np.eye(4)
            T_obj[:3, :3] = rot
            T_obj[:3, 3] = [x, y, z]

            # Compute transformation from from base to object
            T_base_obj = self.T @ T_obj

            # Extract position and orientation as angle in degrees
            s.x = T_base_obj[0, 3]
            s.y = T_base_obj[1, 3]
            s.z = T_base_obj[2, 3]
            s.o = np.rad2deg(Rotation.from_matrix(T_base_obj[:3, :3]).as_euler('zyx')[0])

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