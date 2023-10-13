#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>

class Pub2Node : public rclcpp::Node
{
    public:
        Pub2Node():Node("pub_2") 
        {
            publisher_= this->create_publisher<example_interfaces::msg::String>("discuss",10);
            timer_= this->create_wall_timer(std::chrono::seconds(1),std::bind(&Pub2Node::callback_fun,this));
        }
    private:
    std::int16_t count=0;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        example_interfaces::msg::String msg = example_interfaces::msg::String();
        void callback_fun()
        {
            
            msg.data = std::string("Timer ") + std::to_string(count);
            publisher_->publish(msg);
            this->count++;
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Pub2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
