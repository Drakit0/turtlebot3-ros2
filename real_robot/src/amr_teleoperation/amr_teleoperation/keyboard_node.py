import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from amr_msgs.msg import KeyboardMsg

from sshkeyboard import listen_keyboard

class KeyboardPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(KeyboardMsg, 'keyboard_input', 10)
        listen_keyboard(on_press=self.press, on_release=self.release)
        
    def press(self, key):
        msg = KeyboardMsg()
        msg.key = key
        self.publisher_.publish(msg)
            
    def release(self, key):
        # msg = KeyboardMsg()
        # msg.key = "space"
        # self.publisher_.publish(msg)
        pass


def main(args=None):
    rclpy.init(args=args)

    keyboard_publisher = KeyboardPublisher()

    rclpy.spin(keyboard_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keyboard_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()