#include <rclcpp/rclcpp.hpp>
#include <my_custom/msg/drone_controller.hpp>

class DroneControllerNode : public rclcpp::Node
{
public:
    DroneControllerNode() : Node("tf2_3d_drone_controller_2")
    {
        publisher_ = this->create_publisher<my_custom::msg::DroneController>("move_drone_2", 10);
        RCLCPP_INFO(this->get_logger(), "Throttel up = (w)  Throttel down = (s)  Yaw aniclockwise = (a)  Yaw clockwise = (d)    Pitch forward = (i)  Pitch backward = (k)    Roll left = (j)  Roll right = (l)");
        while (rclcpp::ok())
        {
            std::system("stty raw");
            ch = std::getchar();
            std::system("stty cooked");
            // std::cin >> ch;
            if (std::isalpha(ch))
            {
                switch (ch)
                {
                case 'w':
                    dc.throttle = 1;
                    dc.roll = 0;
                    dc.pitch = 0;
                    dc.yaw = 0;
                    break;
                case 's':
                    dc.throttle = -1;
                    dc.roll = 0;
                    dc.pitch = 0;
                    dc.yaw = 0;
                    break;
                case 'a':
                    dc.throttle = 0;
                    dc.roll = 0;
                    dc.pitch = 0;
                    dc.yaw = 1;
                    break;
                case 'd':
                    dc.throttle = 0;
                    dc.roll = 0;
                    dc.pitch = 0;
                    dc.yaw = -1;
                    break;
                case 'i':
                    dc.throttle = 0;
                    dc.roll = 0;
                    dc.pitch = 1;
                    dc.yaw = 0;
                    break;
                case 'k':
                    dc.throttle = 0;
                    dc.roll = 0;
                    dc.pitch = -1;
                    dc.yaw = 0;
                    break;
                case 'j':
                    dc.throttle = 0;
                    dc.roll = -1;
                    dc.pitch = 0;
                    dc.yaw = 0;
                    break;
                case 'l':
                    dc.throttle = 0;
                    dc.roll = 1;
                    dc.pitch = 0;
                    dc.yaw = 0;
                    break;
                case 'c':
                    rclcpp::shutdown();
                    break;
                default:
                    dc.throttle = 0;
                    dc.pitch = 0;
                    dc.roll = 0;
                    dc.yaw = 0;
                    break;
                }
                publisher_->publish(dc);
            }
        }
    }

private:
    char ch;
    my_custom::msg::DroneController dc;
    rclcpp::Publisher<my_custom::msg::DroneController>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneControllerNode>());
    rclcpp::shutdown();
    return 0;
}