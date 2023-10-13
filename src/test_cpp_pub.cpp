#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>


class TestCppNode : public rclcpp::Node
{
    public:
        TestCppNode():Node("test_cpp_pub") , name("BumbleBee")
        {
            publisher_ = this->create_publisher<example_interfaces::msg::String>("cpp_topic",10);
            timer_ = this->create_wall_timer(std::chrono::seconds(2),std::bind(&TestCppNode::publish, this));
            std::cout<<"Node created";
            std::cout.flush();
            
        }

    private:
    void publish()
    {
        auto msg= example_interfaces::msg::String();
        msg.data = std::string("This is ")+ name ;
        publisher_->publish(msg);
    }
    std::string name;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TestCppNode>();
    rclcpp::spin(node);
    rclcpp::shutdown(); 
    return 0;
}