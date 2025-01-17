import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class simplePublisher(Node):
    def __init__(self):
        super().__init__("simple_publisher") #name of node

        self.pub_ = self.create_publisher(String, "anandtopic",10)
        self.counter_ = 0
        self.frequency_ = 1.0

        self.get_logger().info("Hi Anand here %d" % self.frequency_)
        self.timer_ = self.create_timer(self.frequency_,self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "Hello Anand %d" % self.counter_

        self.pub_.publish(msg)
        self.counter_ +=1

def main():
    rclpy.init()
    anand = simplePublisher()
    rclpy.spin(anand)
    anand.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()