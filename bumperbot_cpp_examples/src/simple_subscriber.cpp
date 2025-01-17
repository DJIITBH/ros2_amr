#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class simpleSubscriber : public rclcpp::Node
{
public:
    simpleSubscriber() : Node("anand_sub")
    {
        sub_ = create_subscription<std_msgs::msg::String>("anandcpp", 10, std::bind(&simpleSubscriber::data_callback, this, _1)); //1 input 
    }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    void data_callback(const std_msgs::msg::String &msg) const{
        RCLCPP_INFO_STREAM(get_logger(), "anand ne suna: "<<msg.data.c_str());

    }
};

int main(int argc, char* argv[]){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<simpleSubscriber>();
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}