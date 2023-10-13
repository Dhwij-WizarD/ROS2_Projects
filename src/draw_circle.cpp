#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

class DrawCircleNode : public rclcpp::Node
{

public:
    DrawCircleNode() : Node("draw_circle")
    {

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&DrawCircleNode::set_value, this, std::placeholders::_1));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist val = geometry_msgs::msg::Twist();
    void set_value(const turtlesim::msg::Pose pose)
    {
        if (pose.x < 2 || pose.x > 9 || pose.y < 2 || pose.y > 9)
        {
            val.linear.x = 1.0;
            val.angular.z = 1.0;
        }
        else
        {
            val.linear.x = 6.0;
            val.angular.z = 0.0;
        }
        publisher_->publish(val);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DrawCircleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
