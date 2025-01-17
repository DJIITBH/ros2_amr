#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class simplePublisher : public rclcpp::Node
{
    public:
        simplePublisher() : Node("anand_pub"), counter_(0) //Constructor
        {
           pub_ =  create_publisher<std_msgs::msg::String>("anandcpp",10); //create publisher and assign to shared pointer
           timer_ = create_wall_timer(1s, std::bind(&simplePublisher::timer_callback,this)); //create timer callback 

           RCLCPP_INFO(get_logger(), "anand here doing c++");
        }

    private:
        unsigned int counter_ ;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_; //create a shared pointer of the given data type for publisher
        rclcpp::TimerBase::SharedPtr timer_;  //create a shared pointer for the timer

        void timer_callback(){
            auto message = std_msgs::msg::String();
            message.data = "Anand hu mai!" + std::to_string(counter_++);
            pub_->publish(message);    // publishing the data through shared pointer of publisher
        }
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<simplePublisher>();  //create a shared pointer of class obj
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}