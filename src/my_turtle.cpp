#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/spawn.hpp>

#define PI 3.14159265359
#define THRESH 0.1

std::mutex m;
bool flag{false};

class TurtleNode
{
public:
    TurtleNode(std::string name, std::string target_name, float x, float y, float theta, float mass, float inertia, float kpa, float kps, float kda, float kds, rclcpp::Node *node)
        : node{node}, name{name}, target_name{target_name}, mass{mass}, inertia{inertia}, kpa{kpa}, kps{kps}, kda{kda}, kds{kds}
    {
        if (!node)
        {
            RCLCPP_ERROR(node->get_logger(), "Invalid node pointer!");
            return; // Return or handle the error appropriately
        }
        flag = true;
        while (rclcpp::ok())
        {
            if (m.try_lock())
                break;
        }
        client_spawn = node->create_client<turtlesim::srv::Spawn>("/spawn");
        create_turtle(x, y, theta, name);
        subscriber_target = node->create_subscription<turtlesim::msg::Pose>("/" + target_name + "/pose", 10, std::bind(&TurtleNode::get_target_location, this, std::placeholders::_1));
        subscriber_self = node->create_subscription<turtlesim::msg::Pose>("/" + name + "/pose", 10, std::bind(&TurtleNode::get_self_location, this, std::placeholders::_1));
        publisher_self = node->create_publisher<geometry_msgs::msg::Twist>("/" + name + "/cmd_vel", 10);
        m.unlock();
        flag = false;
    }

private:
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_spawn;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_self;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_target;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_self;
    geometry_msgs::msg::Twist msg{};

    rclcpp::Node *node;
    std::string name{}, target_name{};
    float mass{}, inertia{}, kpa{}, kps{}, kda{}, kds{};
    float turtle_target_x{}, turtle_target_y{};
    float dx{}, dy{}, ds{}, dtheta{}, self_theta{};
    float pds{}, pdtheta{};
    float dds{}, ddtheta{};
    float force{}, torque{}, dv{}, acceleration{}, alpha{}, dw{};
    std::chrono::nanoseconds dt{};

    void create_turtle(float x, float y, float theta, std::string name)
    {
        RCLCPP_INFO(node->get_logger(), "Client Started...");
        while (!client_spawn->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(node->get_logger(), "Waiting for server...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->name = name;
        request->x = x;
        request->y = y;
        request->theta = theta;
        auto future = client_spawn->async_send_request(request);

        try
        {
            std::chrono::seconds t{2};
            auto status{future.wait_for(t)};
            if (status == std::future_status::ready)
            {
                auto res = future.get();
                RCLCPP_INFO(node->get_logger(), "spawn successful");
            }
            else
            {
                RCLCPP_INFO(node->get_logger(), "didn't get a response!");
            }
        }
        catch (std::exception &e)
        {
            RCLCPP_INFO(node->get_logger(), "%s", e.what());
        }
    }

    void get_target_location(const turtlesim::msg::Pose &pose)
    {
        turtle_target_x = pose.x;
        turtle_target_y = pose.y;
        // RCLCPP_INFO(node->get_logger(), "location of turtle1 x = %f, y = %f", turtle_target_x, turtle_target_y);
    }

    void get_self_location(const turtlesim::msg::Pose &pose)
    {
        // RCLCPP_INFO(node->get_logger(), "self Location");
        dx = turtle_target_x - pose.x;
        dy = turtle_target_y - pose.y;
        self_theta = pose.theta;
        dtheta = std::atan2(dy, dx) - self_theta;
        normalise();
        ds = std::sqrt(dx * dx + dy * dy) * std::cos(dtheta);
        auto temp{std::chrono::high_resolution_clock::now()};
        static auto prev{temp};
        dt = temp - prev;
        prev = temp;
        if (dt.count() == 0)
        {
            dds = 0;
            ddtheta = 0;
            pds = ds;
            pdtheta = dtheta;
        }
        else
        {
            dds = 1e9 * (ds - pds) / dt.count();
            ddtheta = 1e9 * (dtheta - pdtheta) / dt.count();
        }
        // std::cout << "dds =" << dds;
        // std::cout << "\ndt =" << std::to_string(dt.count());
        // std::cout << "\nddtheta =" << ddtheta;
        // std::cout.flush();
        pds = ds;
        pdtheta = dtheta;
        move();
    }

    inline void normalise()
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

    inline void move()
    {
        // Proportional
        force = kps * ds;
        // Derivative
        force += (kds * dds);

        acceleration = force / mass;
        dv = dt.count() * acceleration / 1e9;
        msg.linear.x += dv;

        // Proportional
        torque = kpa * dtheta;

        // Derivative
        torque += (kda * ddtheta);

        alpha = torque / inertia;
        dw = dt.count() * alpha / 1e9;

        ds < THRESH &&msg.linear.x < 2 ? msg.angular.z = 0 : msg.angular.z += dw;

        publisher_self->publish(msg);
    }
};

class New_Nodes : public rclcpp::Node
{
public:
    New_Nodes() : Node("my_turtle")
    {
        th = std::thread{std::bind(&New_Nodes::addTurtles, this)};
    }

private:
    std::thread th;
    static std::vector<TurtleNode> TurtleVector;

    void addTurtles()
    {
        while (rclcpp::ok())
        {
            std::string name, target_name;
            float x, y, theta, mass, inertia, kpa, kps, kda, kds;
            RCLCPP_INFO(get_logger(), "Enter name of turtle =");
            std::cin >> name;
            RCLCPP_INFO(get_logger(), "Enter target name of turtle =");
            std::cin >> target_name;
            RCLCPP_INFO(get_logger(), "Enter position(x,y,theta) of turtle =");
            std::cin >> x >> y >> theta;
            RCLCPP_INFO(get_logger(), "Enter mass & inertia =");
            std::cin >> mass >> inertia;
            RCLCPP_INFO(get_logger(), "Enter kpa & kps =");
            std::cin >> kpa >> kps;
            RCLCPP_INFO(get_logger(), "Enter kda & kds =");
            std::cin >> kda >> kds;
            TurtleVector.emplace_back(name, target_name, x, y, theta, mass, inertia, kpa, kps, kda, kds, this);
        }
    }
};
std::vector<TurtleNode> New_Nodes::TurtleVector{};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<New_Nodes>();
    while (rclcpp::ok())
    {
        while (flag)
            ;
        while (rclcpp::ok())
        {
            if (m.try_lock())
                break;
        }
        rclcpp::spin_some(node);
        m.unlock();
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}