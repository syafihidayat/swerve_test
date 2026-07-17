#include <rclcpp/rclcpp.hpp>

class SwerveControllerNode : public rclcpp::Node
{
public:
    SwerveControllerNode()
    : Node("swerve_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Swerve Controller Started");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SwerveControllerNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}