import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class simpleParameter(Node):
    def __init__(self):
        super().__init__("anand_parameter")

        self.declare_parameter("simple_int_param",28) #name of param with the default value
        self.declare_parameter("simple_string_param","Anand") 

        self.add_on_set_parameters_callback(self.param_change_callback) #when parameter value is changed
        # parameter value can be changed while code is running!

    def param_change_callback(self,params):
        # receives new value of parameter
        result = SetParametersResult() #standard ros2 interface 

        for param in params: # loop through all parameters
            # verfify name and type of parameter!
            if param.name == "simple_int_param" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info("Param simple_int_param changed! new value is %d: "% param.value)
                result.successful = True #change has been correctly applied to param
            
            if param.name == "simple_string_param" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info("Param simple_string_param changed! new value is %s: "% param.value)
                result.successful = True #change has been correctly applied to param
            
        return result
    

def main():
    rclpy.init()
    simple_parameter = simpleParameter()
    rclpy.spin(simple_parameter)
    simple_parameter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()