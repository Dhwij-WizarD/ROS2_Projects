#include<rclcpp/rclcpp.hpp>
#include<example_interfaces/msg/string.hpp>

class TestCppSubNode : public rclcpp::Node
{
    public:
        TestCppSubNode():Node("test_cpp_sub")
        {
            subscriber_=this->create_subscription<example_interfaces::msg::String>("data_transfer",10,std::bind(&TestCppSubNode::call_back_fun,this,std::placeholders::_1));
            std::cout<<"Subscriber created";
            std::cout.flush();
        }
    private:
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
    void call_back_fun(const example_interfaces::msg::String::SharedPtr msg)
    {
        // std::cout<<"\n%s",msg->data.c_str();
        std::cout << msg->data << '\n';
        // RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
        std::cout.flush();
    }
};

int main(int argc,char ** argv)
{
    rclcpp::init(argc,argv);
    auto node= std::make_shared<TestCppSubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}