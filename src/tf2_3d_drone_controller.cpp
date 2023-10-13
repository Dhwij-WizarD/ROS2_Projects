#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>


class DroneControllerNode : public rclcpp::Node
{
public:
    DroneControllerNode() : Node("tf2_3d_drone_controller")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("move_drone", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&DroneControllerNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Throttel up = (w)  Throttel down = (s)  Yaw aniclockwise = (a)  Yaw clockwise = (d)    Pitch forward = (i)  Pitch backward = (k)    Roll left = (j)  Roll right = (l)");
        while (rclcpp::ok())
        {
            char ch;
            ch = std::getchar();
            switch ((ch))
            {
            case 'w':
                msg.linear.x = 0;
                msg.linear.y = 0;
                msg.linear.z = 1;
                msg.angular.x = 0;
                msg.angular.y = 0;
                msg.angular.z = 0;
                break;

            case 's':
                msg.linear.x = 0;
                msg.linear.y = 0;
                msg.linear.z = -1;
                msg.angular.x = 0;
                msg.angular.y = 0;
                msg.angular.z = 0;
                break;

            case 'a':
                msg.linear.x = 0;
                msg.linear.y = 0;
                msg.linear.z = 0;
                msg.angular.x = 0;
                msg.angular.y = 0;
                msg.angular.z = 1;
                break;

            case 'd':
                msg.linear.x = 0;
                msg.linear.y = 0;
                msg.linear.z = 0;
                msg.angular.x = 0;
                msg.angular.y = 0;
                msg.angular.z = -1;
                break;

            case 'i':
                msg.linear.x = 1;
                msg.linear.y = 0;
                msg.linear.z = 0;
                msg.angular.x = 0;
                msg.angular.y = 0;
                msg.angular.z = 0;
                break;

            case 'k':
                msg.linear.x = -1;
                msg.linear.y = 0;
                msg.linear.z = 0;
                msg.angular.x = 0;
                msg.angular.y = 0;
                msg.angular.z = 0;
                break;

            case 'j':
                msg.linear.x = 0;
                msg.linear.y = -1;
                msg.linear.z = 0;
                msg.angular.x = 0;
                msg.angular.y = 0;
                msg.angular.z = 0;
                break;

            case 'l':
                msg.linear.x = 0;
                msg.linear.y = 1;
                msg.linear.z = 0;
                msg.angular.x = 0;
                msg.angular.y = 0;
                msg.angular.z = 0;
                break;

            default:
                RCLCPP_WARN(this->get_logger(), "Enter valid command");
                break;
            }
            publisher_->publish(msg);
        }
    }

private:
    void timer_callback()
    {
        publisher_->publish(msg);
    }
    geometry_msgs::msg::Twist msg;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneControllerNode>());
    rclcpp::shutdown();
    return 0;
}