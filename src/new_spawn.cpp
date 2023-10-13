#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>

class SpawnTurtleNode : public rclcpp::Node
{
public:
    SpawnTurtleNode() : Node("new_spawn")
    {
        thread_1 = std::thread(std::bind(&SpawnTurtleNode::call, this));
    }
    inline void call()
    {
        timer_ = this->create_wall_timer(std::chrono::seconds(3),
                                         std::bind(&SpawnTurtleNode::create_turtle, this));
    }

    void create_turtle()
    {
        RCLCPP_INFO(get_logger(), "Client Started...");

        client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");

        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for server...");
        }
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();

        request->name = t_name + std::to_string(counter++);
        request->x = std::rand() % 10;
        request->y = std::rand() % 10;
        request->theta = 0.0;
        using ServiceResponseFuture =
            rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future)
        {
            auto result = future.get();
            RCLCPP_INFO(this->get_logger(), "%s is spawned", result->name.c_str());
        };
        auto future = client_->async_send_request(request, response_received_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
    std::thread thread_1;
    std::string t_name{"turtle"};
    int counter{};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpawnTurtleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}