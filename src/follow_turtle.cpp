#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/spawn.hpp>

#define PI 3.14159265359
#define THRESH 5 * PI / 180 / 2

class FollowTurtleNode : public rclcpp::Node
{
public:
    FollowTurtleNode() : Node("follow_turtle")
    {
        RCLCPP_INFO(get_logger(), "Enter kpa, kps");
        // std::cin >> kpa >> kps;
        create_turtle(4.0, 4.0, 0.0, "turtle2");

        subscriber_1 = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,
                                                                       std::bind(&FollowTurtleNode::get_location, this, std::placeholders::_1));

        subscriber_2 = this->create_subscription<turtlesim::msg::Pose>("turtle2/pose", 10,
                                                                       std::bind(&FollowTurtleNode::get_location_self, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);
    }

private:
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_1;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_2;
    geometry_msgs::msg::Twist msg;
    float dx{};
    float dy{};
    float ds{};
    float dtheta{};
    float turtle2_theta = 0.0;
    float turtle1_x = 0.0;
    float turtle1_y = 0.0;
    float kps{1.6}, kpa{1.3};

    void create_turtle(float x, float y, float theta, std::string name)
    {
        RCLCPP_INFO(get_logger(), "Client Started...");

        client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");

        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for server...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->name = name;
        request->x = x;
        request->y = y;
        request->theta = theta;
        auto future = client_->async_send_request(request);
    }

    void get_location(const turtlesim::msg::Pose &pose)
    {
        turtle1_x = pose.x;
        turtle1_y = pose.y;
        RCLCPP_INFO(get_logger(), "location of turtle1 x = %f, y = %f", turtle1_x, turtle1_y);
    }

    void get_location_self(const turtlesim::msg::Pose &pose)
    {
        dx = turtle1_x - pose.x;
        dy = turtle1_y - pose.y;
        ds = std::sqrt(dx * dx + dy * dy);
        turtle2_theta = pose.theta;
        dtheta = std::atan2(dy, dx) - turtle2_theta;
        normalize_dtheta();
        if (correctTheta())
        {
        }
        else
            correctPose();
        publisher_->publish(msg);
        // RCLCPP_INFO(get_logger(), "location of turtle2 x = %f, y = %f, theta = %f", pose.x, pose.y, pose.theta);
    }

    inline void normalize_dtheta()
    {
        while (dtheta > PI)
        {
            dtheta -= 2 * PI;
        }
        while (dtheta < -PI)
        {
            dtheta += 2 * PI;
        }
    }

    inline bool correctTheta()
    {
        msg.linear.x = 0.0;
        if (dtheta > THRESH || dtheta < -THRESH)
        {
            msg.angular.z = kpa * dtheta;
            return true;
        }
        return false;
    }

    inline void correctPose()
    {
        msg.linear.x = kps * ds;
        msg.angular.z = kpa * dtheta;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowTurtleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}