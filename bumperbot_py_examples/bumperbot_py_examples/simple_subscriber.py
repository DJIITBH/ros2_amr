import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class simpleSubscriber(Node):
    def __init__(self):
        super().__init__("anand_sub")
        self.subscriber_ = self.create_subscription(String, "anandtopic", self.data_callback,10)
    
    def data_callback(self,msg):
        data = msg.data
        self.get_logger().info(data)
    
def main():
    rclpy.init()
    node = simpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()