#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "gordon_hardware/motor.hpp"

using namespace std::chrono_literals;

class MotorNode : public rclcpp::Node {
public:
    MotorNode()
        : Node("motor_node")
    {
        auto direction_pin = this->declare_parameter<int>("direction_pin", 0);
        auto speed_pin = this->declare_parameter<int>("speed_pin", 0);
    }

private:
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorNode>());
    rclcpp::shutdown();
    return 0;
}
