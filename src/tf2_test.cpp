#include <rclcpp/rclcpp.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class Tf2Test : public rclcpp::Node
{
public:
    Tf2Test() : Node("tf2_test")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Tf2Test::broadcast_timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Broadcaster Started");
    }

private:
    void broadcast_timer_callback()
    {
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "drone";

        t.transform.translation.x = 1.0;
        t.transform.translation.y = 1.0;
        t.transform.translation.z = 1.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;

        tf_broadcaster_->sendTransform(t);

        t2.header.stamp = this->get_clock()->now();
        t2.header.frame_id = "drone";
        t2.child_frame_id = "drone2";

        t2.transform.translation.x = 1.0;
        t2.transform.translation.y = 1.0;
        t2.transform.translation.z = 1.0;
        t2.transform.rotation.x = 0.0;
        t2.transform.rotation.y = 0.0;
        t2.transform.rotation.z = 0.0;

        tf_broadcaster_->sendTransform(t2);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::TransformStamped t, t2;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tf2Test>());
    rclcpp::shutdown();
    return 0;
}