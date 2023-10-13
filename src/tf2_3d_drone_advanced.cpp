#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <rclcpp/rclcpp.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <my_custom/msg/drone_controller.hpp>


class DroneAddvancedNode : public rclcpp::Node
{
public:
    DroneAddvancedNode() : Node("tf2_3d_drone_advanced")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        subscriber_1 = this->create_subscription<my_custom::msg::DroneController>(
            "/move_drone_2", 10, std::bind(&DroneAddvancedNode::cmd_vel_cb, this, std::placeholders::_1));
        timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DroneAddvancedNode::update_transform, this));
    }

private:
    void cmd_vel_cb(const std::shared_ptr<my_custom::msg::DroneController> msg)
    {

        throttle = msg->throttle;
        yaw = msg->yaw;
        roll = msg->roll;
        pitch = msg->pitch;

        time_post = std::chrono::high_resolution_clock::now();
    }

    void update_transform()
    {
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "drone";

        tf_broadcaster_->sendTransform(t);
        t2.header.stamp = this->get_clock()->now(); // create new d2 transform
        t2.header.frame_id = "drone";
        t2.child_frame_id = "d2";

        time_span = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::high_resolution_clock::now() - time_post);

        if (time_span.count() < 0.2)
        {
            double dyaw = (yaw / 30);
            double droll = (roll / 30);
            double dpitch = (pitch / 30);
            double dthrottle = (throttle / 30);

            ddp += dpitch; // dd - second derivitive, use pitch
            ddy += dyaw;
            ddr += droll;

            tf2::Quaternion q;
            q.setRPY(ddr, ddp, ddy);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(t);
            tf_broadcaster_->sendTransform(t2);

            t2.transform.translation.x = 0.0;
            t2.transform.translation.y = 0.0;
            t2.transform.translation.z = dthrottle;

            q.setRPY(ddr, ddp, ddy);
            t2.transform.rotation.x = q.x();
            t2.transform.rotation.y = q.y();
            t2.transform.rotation.z = q.z();
            t2.transform.rotation.w = q.w();

            try
            {
                t3 = tf_buffer_->lookupTransform("map", "d2", tf2::TimePointZero);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_INFO(
                    this->get_logger(), "Could not transform ");
            }
            t.transform.translation.x = t3.transform.translation.x;
            t.transform.translation.y = t3.transform.translation.y;
            t.transform.translation.z = t3.transform.translation.z;

            t.transform.rotation.x = t3.transform.rotation.x;
            t.transform.rotation.y = t3.transform.rotation.y;
            t.transform.rotation.z = t3.transform.rotation.z;
            t.transform.rotation.w = t3.transform.rotation.w;

            ddr = 0;
            ddp = 0;
            ddy = 0;
            dthrottle = 0;
        }
        tf_broadcaster_->sendTransform(t);
        tf_broadcaster_->sendTransform(t2);
    }

    std::chrono::duration<double> time_span;
    std::chrono::high_resolution_clock::time_point time_post;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::TransformStamped t2;
    geometry_msgs::msg::TransformStamped t3;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_{};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    rclcpp::Subscription<my_custom::msg::DroneController>::SharedPtr subscriber_1;
    rclcpp::TimerBase::SharedPtr timer;
    double x{}, y{}, z{}, roll{}, pitch{}, yaw{}, throttle{};
    double ddr{}, ddp{}, ddy{};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node{std::make_shared<DroneAddvancedNode>()};
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}