import rclpy
from rclpy.node import Node
from custom_msgs_pkg.msg import ListShape, Shape
from rclpy.action import ActionClient
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import SuctionCupControl
from scipy.spatial.transform import Rotation
import numpy as np

from .shapes import cart2pol

RES = "null"

# Vector from the end effector to the suction cup
EE_to_suction = np.array([60.0, 0.0, -75.0]) # [mm]

# Class for the logic of the robot
class Logic(Node):
    def __init__(self):
        super().__init__('logic')
        self.get_logger().info('Logic node started')
        
        # Subscriber to the shapes topic
        self.create_subscription(
            ListShape,
            '/shapes',
            self.shapes_callback,
            10)
        
        # Service to control the suction cup
        self.suction_client = self.create_client(SuctionCupControl, 'dobot_suction_cup_service')
        while not self.suction_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        # Action Client to PTP_action Server
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action')
        self._action_client.wait_for_server()

        self.going_to_start = True

        self.shapes = []

        self.detected_shapes = []


    def shapes_callback(self, msg):
        global RES

        if self.going_to_start:
            return
        
        # print(len(msg.shapes))

        # For each shape
        for s in msg.shapes:
            # Extract info from the message
            shape = {
                'shape': s.shape,
                'color': s.color,
                'position': [s.x * 1000 , s.y * 1000, s.z * 1000],
                'orientation': s.o
            }

            self.detected_shapes.append(shape)


    def send_goal(self, pose, motion_type, velocity_ratio=1.0):
        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = pose
        goal_msg.motion_type = motion_type
        goal_msg.velocity_ratio = velocity_ratio

        self._future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._future.add_done_callback(self.goal_response_callback)


    def feedback_callback(self, feedback):
        pass

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self._goal_handle = goal_handle

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        global RES
        result = future.result().result
        RES = result

    def starting_position(self):
        # Initial position for exploring the workspace 
        x = 143.0
        y = -43.587
        z = 172.0
        orientation = -120.0

        # Send goal to PTP_action server
        pose = [x, y, z, orientation]
        motion_type = 1
        self.send_goal(pose, motion_type)


    def explore(self):
        # Final position for exploring the workspace (in joint space)
        x = 17.68
        y = 3.057
        z = -14.648
        orientation = -120.0
        velocity_ratio = 0.1

        # Send goal to PTP_action server
        pose = [x, y, z, orientation]
        motion_type = 4
        self.send_goal(pose, motion_type, velocity_ratio)
                
    def look_for_shapes(self):
        global RES

        # Spin once
        rclpy.spin_once(self)

        close = False


        # Append the shape if the same type of shape is not close
        for shape in self.detected_shapes:
            for sh in self.shapes:
                if sh['shape'] == shape['shape']:
                    # print(f"sh{sh['position'][0]} shape{shape['position'][0]}")
                    # print(f"sh{sh['position'][1]} shape{shape['position'][1]}")
                    if abs(sh['position'][0] - shape['position'][0]) < 15 and abs(sh['position'][1] - shape['position'][1]) < 15:
                        close = True
                        break
            if not close:
                print('IM HERE')
                self.shapes.append(shape)
        

        # If two same shapes are in the detected shapes, start the pick and place
        for i in range(len(self.shapes)):
            for j in range(i+1, len(self.shapes)):
                if self.shapes[i]['shape'] == self.shapes[j]['shape']:
                    # Print the name of the shapes that are the same
                    self.get_logger().info('Found two same shapes')
                    self.get_logger().info(self.shapes[i]['shape'])    

                    return self.shapes[i], self.shapes[j] 
                
        return None

    def pick_and_place(self, shape1, shape2):
        global RES

        # If the triangle is blue, the circle is red or the square is green, it's the object to pick
        if (shape1['shape'] == 'triangle' and shape1['color'] == 'blue') or (shape1['shape'] == 'circle' and shape1['color'] == 'red') or (shape1['shape'] == 'square' and shape1['color'] == 'green'):
            pick = shape1
            place = shape2
        else:
            pick = shape2
            place = shape1

        # # Drive the suction cup above the object to pick
        # x = pick['position'][0]
        # y = pick['position'][1]
        z = pick['position'][2] + 55.0
        # orientation = -120.0

        # Transform goal to  polar coordinates
        [r, theta, phi] = cart2pol(pick['position'][0],pick['position'][1]) 
        # represent offset in polar
        [r_off,theta_off, phi_off] = cart2pol(60, 0)

        # Add it up
        r = r + r_off
        theta = theta + theta_off
        phi = phi + phi_off

        # Go back to cartesian
        [x, y] = [r*np.cos(theta), r*np.sin(theta)]

        
        # [x, y, z] = [x, y, z] #+ EE_to_suction
        
        print(f'x{x} y{y} z{z}')

        # [x, y, z] = [208.213, 1.25, -62.955 + 45.0]

        pose = [x, y, z, orientation]
        motion_type = 1

        print('sending goal to pick the object')
        
        self.send_goal(pose, motion_type)

        print('Sent goal to pick the object')

        # Wait for the robot to reach the starting position
        while rclpy.ok() and RES == "null":
            rclpy.spin_once(self)
        RES = "null"

        print('going down')

        # Drive the robot down to the object
        pose = [x, y, z, orientation]
        self.send_goal(pose, motion_type)

        # Wait for the robot to reach the starting position
        while rclpy.ok() and RES == "null":
            rclpy.spin_once(self)
        RES = "null"

        # Turn on the suction cup
        request = SuctionCupControl.Request()
        request.enable_suction = True
        self.suction_client.call_async(request)

        # Drive the robot up
        z = z + 150.0

        print('going up')

        pose = [x, y, z, orientation]
        motion_type = 1
        self.send_goal(pose, motion_type)

        # Wait for the robot to reach the starting position
        while rclpy.ok() and RES == "null":
            rclpy.spin_once(self)
        RES = "null"

        # Drive the robot to the place position
        x = place['position'][0]
        y = place['position'][1]
        z = place['position'][2] + 100.0
        orientation = -120.0

        pose = [x, y, z, orientation]
        motion_type = 1

        print('going to place the object')
        self.send_goal(pose, motion_type)

          # Wait for the robot to reach the starting position
        while rclpy.ok() and RES == "null":
            rclpy.spin_once(self)
        RES = "null"


        # Turn off the suction cup
        request = SuctionCupControl.Request()
        request.enable_suction = False
        self.suction_client.call_async(request)



def main(args=None):
    global RES

    rclpy.init(args=args)

    logic = Logic()

    logic.starting_position()

    # Wait for the robot to reach the starting position
    while rclpy.ok() and RES == "null":
        rclpy.spin_once(logic)
    RES = "null"

    logic.going_to_start = False

    logic.explore()

    objects = logic.look_for_shapes()
    while objects == None:
        objects = logic.look_for_shapes()

    # Wait for the robot to reach the final position
    while rclpy.ok() and RES == "null":
        rclpy.spin_once(logic)
    RES = "null"          

    print('going to pick and place')      
    
    logic.pick_and_place(objects[0], objects[1])

    # logic.pick_and_place({'shape': 'triangle', 'color': 'blue', 'position': [0.0, 0.0, 0.0], 'orientation': 0.0}, {'shape': 'circle', 'color': 'red', 'position': [0.0, 0.0, 0.0], 'orientation': 0.0})

    rclpy.spin(logic)

    logic.destroy_node()
    rclpy.shutdown()

### Exploring the workspace
# Initial position for exploring the workspace 
# x = 0.12935, y = -0.02844, z = 0.30732
# degree of saction = -120 degrees

# rotate right

# rotate left



### Find target object
# 
