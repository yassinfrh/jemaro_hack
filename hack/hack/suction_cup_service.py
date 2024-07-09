from dobot_msgs.srv import SuctionCupControl
import rclpy
from rclpy.node import Node

from std_srvs import Empty


class SuctionCupService(Node):

    def __init__(self):
        super().__init__('suction_cup_service')
        self.cli = self.create_client(SuctionCupControl, 'suction_cup_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SuctionCupControl.Request()

    def send_request(self, enable_suction):
        self.req.enable_suction = enable_suction
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = SuctionCupService()
    response = minimal_client.send_request(True)
    minimal_client.get_logger().info(
        'Result of calling service: %s' %(response))
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

