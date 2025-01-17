#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <string>
#include <vector>
#include <memory>

using std::placeholders::_1;

class simpleParamter: public rclcpp::Node{
public:
    simpleParamter() : Node("anand_parameter")
    {
        declare_parameter<int>("simple_int_param",28);
        declare_parameter<std::string>("simple_string_param","Anand");

      param_callback_handle_ =  add_on_set_parameters_callback(std::bind(&simpleParamter::param_change_callback,this,_1)); //when param changes
    }    
private:
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    rcl_interfaces::msg::SetParametersResult  param_change_callback(const std::vector<rclcpp::Parameter> & parameters){
            rcl_interfaces::msg::SetParametersResult result;
            for (const auto &param : parameters){
                if (param.get_name()=="simple_int_param" && param.get_type()== rclcpp::ParameterType::PARAMETER_INTEGER){
                    RCLCPP_INFO_STREAM(get_logger(), "Param simple_int_param changed! "<<param.as_int());
                    result.successful = true;

                }
                if (param.get_name()=="simple_string_param" && param.get_type()== rclcpp::ParameterType::PARAMETER_STRING){
                    RCLCPP_INFO_STREAM(get_logger(), "Param simple_string_param changed! "<<param.as_string());
                    result.successful = true;

                }
            }
            return result;
    }
};

int main(int argc, char* argv []){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<simpleParamter>();
    rclcpp::spin(node);
    rclcpp::shutdown();

}