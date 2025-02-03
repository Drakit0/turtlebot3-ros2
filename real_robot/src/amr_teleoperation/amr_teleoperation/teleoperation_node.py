import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from amr_msgs.msg import KeyboardMsg

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)



class KeyboardDirModification(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.keyboard_subscription = self.create_subscription(
            KeyboardMsg,
            'keyboard_input',
            self.listener_callback_keyboard,
            10)
        
        self.keyboard_subscription
        
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', qos_profile=10)
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE, # Make sure that the robot receives the command to stop
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        
        self.lidar_subscription = self.create_subscription(
            KeyboardMsg,
            'scan',
            self.listener_callback_lidar,
            qos_profile=qos_profile)
            
        self.lidar_subscription
        
        self.stop_distance = 10

    def listener_callback_keyboard(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.key)
        
        if msg.key == "w": # Forward
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.1
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            
        elif msg.key == "s": # Backward
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = -0.1
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            
        elif msg.key == "a": # Rotate Left
            cmd_vel_msg = Twist()
            cmd_vel_msg.angular.z = 0.1
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            
        elif msg.key == "d": # Rotate Right
            cmd_vel_msg = Twist()
            cmd_vel_msg.angular.z = -0.1
            self.cmd_vel_publisher.publish(cmd_vel_msg)
        
        elif msg.key == "space": # Stop
            cmd_vel_msg = Twist()
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            
    def create_stop_msg(): # It is enough to send an empty Twist message to stop the robot
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        return msg
        
    def listener_callback_lidar(self, msg):
        ranges = msg.ranges
        start_idx = 3
        end_idx = 7
        
        if any(map(lambda x: x < self.stop_distance, ranges[start_idx: end_idx])):
            stop_msg = self.create_stop_msg()
            self.cmd_vel_publisher.publish(stop_msg)
            self.get_logger().info('STOP!!!')
        
    def publisher_cmd_vel_callback(self, msg):
        self.get_logger().info('I publish: "%s"' % msg.data)
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = KeyboardDirModification()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
