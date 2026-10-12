#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/multi_dof_command.hpp>
#include "swerve_control/kinematic.h"

#include <map>
#include <string>
#include <cmath>

using std::placeholders::_1;

class KinematicNode : public rclcpp::Node
{
public:
    KinematicNode()
    : Node("kinematic_node"),
      kinematic_(Kinematic::SWERVE_DRIVE,
                 300, 1.0, 12.0, 12.0,
                 0.02, 0.12, 0.12)
    {
        steering_names_ = {"front_right_steering_joint", "front_left_steering_joint",
                            "rear_right_steering_joint", "rear_left_steering_joint"};
        wheel_names_    = {"front_right_wheel_joint", "front_left_wheel_joint",
                            "rear_right_wheel_joint", "rear_left_wheel_joint"};

        for (const auto &name : steering_names_)
        {
            current_steering_angle_[name] = 0.0f;
            // ---- Fix #2: simpan last commanded angle terpisah dari encoder feedback ----
            last_commanded_angle_[name] = 0.0f;
        }

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&KinematicNode::cmdVelCallback, this, _1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&KinematicNode::jointStateCallback, this, _1));

        steering_pub_ = this->create_publisher<control_msgs::msg::MultiDOFCommand>(
            "/steering_pid_controller/reference", 10);

        wheel_pub_ = this->create_publisher<control_msgs::msg::MultiDOFCommand>(
            "/wheel_pid_controller/reference", 10);

        RCLCPP_INFO(this->get_logger(), "Kinematic node started. Max RPS: %.3f", kinematic_.getMaxRPS());
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // encoder feedback ini sekarang HANYA dipakai untuk monitoring/odometry,
        // bukan lagi sebagai basis keputusan optimizeModule (lihat fix #2)
        for (size_t i = 0; i < msg->name.size(); i++)
        {
            auto it = current_steering_angle_.find(msg->name[i]);
            if (it != current_steering_angle_.end())
                it->second = static_cast<float>(msg->position[i]);
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float vx = static_cast<float>(msg->linear.x);
        float vy = static_cast<float>(msg->linear.y);
        float wz = static_cast<float>(msg->angular.z);

        Kinematic::module_state state = kinematic_.getModuleStates(vx, vy, wz);

        struct ModuleRaw { float speed; float angle; std::string steer_name; std::string wheel_name; };

        std::vector<ModuleRaw> modules = {
            {state.speed1, state.angle1, "front_left_steering_joint",  "front_left_wheel_joint"},
            {state.speed2, state.angle2, "front_right_steering_joint", "front_right_wheel_joint"},
            {state.speed3, state.angle3, "rear_left_steering_joint",   "rear_left_wheel_joint"},
            {state.speed4, state.angle4, "rear_right_steering_joint",  "rear_right_wheel_joint"},
        };

        std::map<std::string, float> out_speed_rps;
        std::map<std::string, float> out_angle_rad;

        int module_index = 0;
        for (auto &m : modules)
        {
            // ---- Fix #2: basis optimize pakai last commanded angle, ----
            // ---- bukan current_steering_angle_ (encoder feedback) ----
            // Alasan: encoder feedback tiap motor punya lag berbeda-beda,
            // sehingga keputusan flip antar module bisa tidak sinkron dan
            // menyebabkan robot terasa "muter sendiri" saat transisi gerakan.
            float base_angle = last_commanded_angle_[m.steer_name];

            float out_speed = 0.0f, out_angle = 0.0f;
            kinematic_.optimizeModule(m.speed, m.angle, base_angle, out_speed, out_angle, module_index);

            out_speed_rps[m.wheel_name] = out_speed;
            out_angle_rad[m.steer_name] = out_angle;

            last_commanded_angle_[m.steer_name] = out_angle;

            module_index++;
        }

        control_msgs::msg::MultiDOFCommand steering_msg;
        steering_msg.dof_names = steering_names_;
        for (const auto &name : steering_names_)
            steering_msg.values.push_back(out_angle_rad[name]);
        steering_pub_->publish(steering_msg);

        control_msgs::msg::MultiDOFCommand wheel_msg;
        wheel_msg.dof_names = wheel_names_;
        for (const auto &name : wheel_names_)
        {
            float rad_per_s = out_speed_rps[name] * 2.0f * static_cast<float>(M_PI);
            wheel_msg.values.push_back(rad_per_s);
        }
        wheel_pub_->publish(wheel_msg);
    }

    Kinematic kinematic_;
    std::vector<std::string> steering_names_;
    std::vector<std::string> wheel_names_;
    std::map<std::string, float> current_steering_angle_;   // encoder feedback (monitoring saja)
    std::map<std::string, float> last_commanded_angle_;     // basis untuk optimizeModule (fix #2)

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<control_msgs::msg::MultiDOFCommand>::SharedPtr steering_pub_;
    rclcpp::Publisher<control_msgs::msg::MultiDOFCommand>::SharedPtr wheel_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicNode>());
    rclcpp::shutdown();
    return 0;
}