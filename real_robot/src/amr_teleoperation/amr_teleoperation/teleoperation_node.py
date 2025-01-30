import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from amr_msgs.msg import KeyboardMsg


class KeyboardDirModification(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.keyboard_subscription = self.create_subscription(
            KeyboardMsg,
            'keyboard_input',
            self.listener_callback,
            10)
        
        self.keyboard_subscription
        
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.lidar_subscription = self.create_subscription(
            KeyboardMsg,
            'scan',
            self.listener_callback_lidar,
            10)
            
        self.lidar_subscription

    def listener_callback_keyboard(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
        if msg.data == 'w':
            msg = Twist()
            msg.linear.x = 0.1
            self.cmd_vel_publisher.publish(msg)
        
    def listener_callback_lidar(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
    def publisher_cmd_vel_callback(self, msg):
        self.get_logger().info('I publish: "%s"' % msg.data)
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
