#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/multi_dof_command.hpp>
#include "swerve_control/kinematic.h"

#include <map>
#include <string>
#include <cmath>
#include <chrono>
#include <algorithm>

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

        // ---- Parameter yang bisa di-tune tanpa recompile ----
        // CATATAN: teleop_twist_joy di ROS Humble TIDAK punya parameter
        // deadband, jadi filter ini WAJIB ada di sisi node ini.
        linear_deadband_  = this->declare_parameter<double>("linear_deadband", 0.05);  // m/s
        angular_deadband_ = this->declare_parameter<double>("angular_deadband", 0.05); // rad/s
        lpf_alpha_        = this->declare_parameter<double>("lpf_alpha", 0.35);        // 0..1
        cmd_vel_timeout_  = this->declare_parameter<double>("cmd_vel_timeout", 0.25);  // detik

        // ---- Watchdog: kalau /cmd_vel berhenti, publish perintah nol ----
        // Tanpa ini perintah terakhir dari teleop terus dikirim dan robot
        // tetap berjalan walaupun joystick sudah dilepas / teleop mati.
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&KinematicNode::watchdogCallback, this));

        RCLCPP_INFO(this->get_logger(),
                    "Kinematic node started. Max RPS: %.3f | linear_deadband: %.3f m/s | "
                    "angular_deadband: %.3f rad/s | lpf_alpha: %.2f | timeout: %.2fs",
                    kinematic_.getMaxRPS(), linear_deadband_, angular_deadband_,
                    lpf_alpha_, cmd_vel_timeout_);
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

    // ---- Filter 1: deadband ----
    // Memotong nilai kecil (noise joystick / trigger yang idle di bukan 0).
    // Tanpa ini, atan2() di dalam inverse kinematics menerima vektor
    // yang sangat kecil sehingga sudut hasil atan2() liar dan tidak stabil.
    static float applyDeadband(float v, float deadband)
    {
        if (std::fabs(v) < deadband) return 0.0f;
        // Geser sedikit supaya output tidak "melompat" dari 0 ke deadband.
        return v - std::copysign(deadband, v);
    }

    // ---- Normalisasi sudut ke [-pi, pi] ----
    // Sudut di-wrap supaya tidak keluar dari batas joint di URDF.
    static float wrapToPi(float angle)
    {
        while (angle >  M_PI) angle -= 2.0f * static_cast<float>(M_PI);
        while (angle < -M_PI) angle += 2.0f * static_cast<float>(M_PI);
        return angle;
    }

    // ---- Filter 2: low-pass filter eksponensial ----
    // Meredam lonjakan jerks dan membuat transisi halus.
    void lpf(float &state, float input)
    {
        state = lpf_alpha_ * input + (1.0f - lpf_alpha_) * state;
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Terakhir kali ada perintah masuk (dipakai watchdog).
        last_cmd_time_ = this->now();
        timed_out_ = false;

        float vx = static_cast<float>(msg->linear.x);
        float vy = static_cast<float>(msg->linear.y);
        float wz = static_cast<float>(msg->angular.z);

        // ---- PENTING: urut deadband dulu, baru LPF ----
        // Kalau LPF dulu, noise kecil akan terakumulasi jadi command
        // yang tidak pernah benar-benar nol.
        vx = applyDeadband(vx, linear_deadband_);
        vy = applyDeadband(vy, linear_deadband_);
        wz = applyDeadband(wz, angular_deadband_);

        // Lewati LPF pada frame pertama supaya tidak mulai dari kondisi 0
        // (yang akan membuat robot mulai bergerak dari bawah).
        if (!lpf_initialized_)
        {
            filt_vx_ = vx;
            filt_vy_ = vy;
            filt_wz_ = wz;
            lpf_initialized_ = true;
        }
        else
        {
            lpf(filt_vx_, vx);
            lpf(filt_vy_, vy);
            lpf(filt_wz_, wz);
        }

        vx = filt_vx_;
        vy = filt_vy_;
        wz = filt_wz_;

        // Kalau semua benar-benar nol (setelah filter), unset LPF supaya
        // state filter ikut kembali ke 0 dan tidak ada "memory" gerak.
        if (vx == 0.0f && vy == 0.0f && wz == 0.0f)
        {
            filt_vx_ = filt_vy_ = filt_wz_ = 0.0f;
        }

        // Deteksi kondisi idle total untuk trigger reset di bawah.
        bool fully_idle = (vx == 0.0f && vy == 0.0f && wz == 0.0f);

        // ---- Reset state saat idle ----
        // Kalau tidak, last_commanded_angle_ dan last_angle_ di Kinematic
        // akan "menempel" pada sudut terakhir sebelum berhenti.
        // Akibatnya, perintah idle berikutnya bisa memakai sudut usang
        // dan roda bergerak sia-sia saat robot seharusnya diam.
        if (fully_idle && !was_idle_)
        {
            resetModuleState();
            was_idle_ = true;
        }
        else if (!fully_idle)
        {
            was_idle_ = false;
        }

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

            // ---- Normalisasi sudut ke rentang [-pi, pi] ----
            // Penting: joint steering di URDF dibatasi [-pi, pi]
            // (swerve_drive.urdf.xacro). Kalau reference keluar dari
            // rentang itu, PID akan terus mendorong motor melawan limit,
            // sehingga module ikut bergulat dan robot bergerak "ngawur".
            out_angle = wrapToPi(out_angle);
            base_angle = wrapToPi(base_angle);

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

    // ---- Watchdog ----
    // Kalau /cmd_vel tidak menerima pesan dalam cmd_vel_timeout_ detik,
    // kita publish perintah nol sendiri. Ini mencegah robot terus jalan
    // dengan perintah basi ketika teleop mati / joystick disconnect.
    void watchdogCallback()
    {
        double elapsed = (this->now() - last_cmd_time_).seconds();

        if (elapsed <= cmd_vel_timeout_)
        {
            // Masih dapat perintah baru dari teleop, biarkan apa adanya.
            return;
        }

        // Sudah timeout: publish nol, dan reset state modul supaya
        // sudut steering tidak "menempel" di posisi terakhir.
        publishZeroCommand();

        if (!timed_out_)
        {
            timed_out_ = true;
            resetModuleState();
        }
    }

    void publishZeroCommand()
    {
        control_msgs::msg::MultiDOFCommand steering_msg;
        steering_msg.dof_names = steering_names_;
        steering_msg.values.assign(steering_names_.size(), 0.0f);
        steering_pub_->publish(steering_msg);

        control_msgs::msg::MultiDOFCommand wheel_msg;
        wheel_msg.dof_names = wheel_names_;
        wheel_msg.values.assign(wheel_names_.size(), 0.0f);
        wheel_pub_->publish(wheel_msg);
    }

    // Kembalikan semua sudut internal ke netral (0 rad = roda lurus maju).
    // Dipanggil saat robot terdeteksi idle supaya tidak ada sudut
    // "tersisa" yang membuat roda bergerak saat diperintah berhenti.
    void resetModuleState()
    {
        for (auto &kv : last_commanded_angle_)
            kv.second = 0.0f;
        kinematic_.resetModuleAngles();
    }

    Kinematic kinematic_;
    std::vector<std::string> steering_names_;
    std::vector<std::string> wheel_names_;
    std::map<std::string, float> current_steering_angle_;   // encoder feedback (monitoring saja)
    std::map<std::string, float> last_commanded_angle_;     // basis untuk optimizeModule (fix #2)

    // Filter
    double linear_deadband_;
    double angular_deadband_;
    double lpf_alpha_;
    double cmd_vel_timeout_;

    // State filter
    float filt_vx_ = 0.0f, filt_vy_ = 0.0f, filt_wz_ = 0.0f;
    bool lpf_initialized_ = false;
    bool was_idle_ = false;

    // Watchdog
    rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
    bool timed_out_ = false;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<control_msgs::msg::MultiDOFCommand>::SharedPtr steering_pub_;
    rclcpp::Publisher<control_msgs::msg::MultiDOFCommand>::SharedPtr wheel_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicNode>());
    rclcpp::shutdown();
    return 0;
}