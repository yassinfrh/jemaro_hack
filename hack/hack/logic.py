import rclpy
from rclpy.node import Node
from custom_msgs_pkg.msg import ListShape
from rclpy.action import ActionClient
from dobot_msgs.action import PointToPoint

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
        
        # Action Client to PTP_action Server
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action')

        self.shapes = []

    def shapes_callback(self, msg):
        self.shapes = msg.shapes

        # TODO: implement when to start picking objects

    
    def starting_position(self):
        # Initial position for exploring the workspace 
        x = 0.12935
        y = -0.02844
        z = 0.30732
        orientation = -120.0

        # Build goal
        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = [x, y, z, orientation]
        goal_msg.motion_type = 4

        # Send goal
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


    def explore(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    logic = Logic()

    logic.starting_position()

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
