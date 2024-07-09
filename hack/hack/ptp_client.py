from dobot_msgs.action import PointToPoint
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from custom_msgs_pkg.msg import Target



class PTPClient(Node):

    def __init__(self):
        super().__init__('PTP_client')
        
        # Action Client to PTP_action Server
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action')

        self.new_target_sub = self.create_subscription(
            Target,
            'new_target',
            self.new_target_callback,
            10)
        self.get_logger().info('Subsriber corerctly set')




    def cancel_done(self, future): 
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')
        rclpy.shutdown()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self._goal_handle = goal_handle

        self.get_logger().info('Goal accepted :)')

        # Start a 0.5 second timer
        # self._timer = self.create_timer(0.5, self.timer_callback)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        # rclpy.shutdown()

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.current_pose))

    def timer_callback(self):
        self.get_logger().info('Canceling goal')
        # Cancel the goal
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

        # Cancel the timer
        self._timer.cancel()

    def send_goal(self, target, mode):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = target
        goal_msg.motion_type = mode
        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def new_target_callback(self, msg):
        #Right now sending only one type of message
        print(f'x{msg.x} y{msg.y} z{msg.z} o{msg.o}')
        self.send_goal(target = [msg.x, msg.y, msg.z, msg.o], mode = 1)
        # self.send_goal(target = [200.0, 0.0, 100.0, 0.0], mode = 1)



def main(args=None):
    rclpy.init(args=args)

    action_client = PTPClient()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()