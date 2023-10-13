#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <rclcpp/rclcpp.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#define PI 3.14159265359

class TFSpawnNode : public rclcpp::Node
{
public:
    TFSpawnNode() : Node("tf2_turtle1")
    {
        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        spawn_turtle();
        subscriber_1 = this->create_subscription<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10, std::bind(&TFSpawnNode::cmd_vel_cb, this, std::placeholders::_1));
        timer = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&TFSpawnNode::update_transform, this));
    }

private:
    inline void spawn_turtle()
    {

        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "turtle1";

        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }

    void cmd_vel_cb(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
    {
        v = msg->linear.x;
        w = msg->angular.z;
        time_post = std::chrono::high_resolution_clock::now();
    }

    void update_transform()
    {
        time_span = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::high_resolution_clock::now() - time_post);

        if (time_span.count() < 0.5)
        {
            double ds = v / 30; // msg is Twist msg
            double dw = w / 30;
            theta += dw;
            t.transform.translation.x += ds * cos(theta);
            t.transform.translation.y += ds * sin(theta);

            tf2::Quaternion q;
            q.setRPY(0, 0, theta);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
        }
        tf_broadcaster_->sendTransform(t);
    }
    
    std::chrono::duration<double> time_span;
    std::chrono::high_resolution_clock::time_point time_post;
    geometry_msgs::msg::TransformStamped t;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_1;
    rclcpp::TimerBase::SharedPtr timer;
    double v{}, w{};
    double theta{};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFSpawnNode>());
    rclcpp::shutdown();
    return 0;
}