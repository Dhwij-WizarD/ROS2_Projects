class DroneNode : public rclcpp::Node
{
    public :
    DroneNode() : Node("tf2_3d_drone_basic")
    {

    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<DroneNode>());
    rclcpp::shutdown();
    return 0;
}