#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class MapSaveNode(Node):
    def __init__(self):
        super().__init__('map_save_node')
        self.timer = self.create_timer(10.0, self.call_map_save_service)  # Adjust frequency as needed

    def call_map_save_service(self):
        self.get_logger().info('Calling map_save service...')
        cli = self.create_client(Trigger, '/map_save')
        req = Trigger.Request()
        future = cli.call_async(req)
        future.add_done_callback(self.callback_map_save)

    def callback_map_save(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Map save service response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = MapSaveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
