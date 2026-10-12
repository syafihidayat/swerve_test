#include "swerve_control/kinematic.h"

// ============================================================
// Constructor
// ============================================================
Kinematic::Kinematic(base robot_base, int motor_max_rpm, float max_rpm_ratio,
                      float motor_operating_voltage, float motor_power_max_voltage,
                      float wheel_diameter, float wheelbase_length, float wheelbase_width)
    : base_platform_(robot_base),
      wheel_radius_(wheel_diameter / 2.0),
      wheel_circumference_(M_PI * wheel_diameter),
      wheelbase_length_(wheelbase_length),
      wheelbase_width_(wheelbase_width),
      half_length_(wheelbase_length / 2.0),
      half_width_(wheelbase_width / 2.0)
{
    motor_power_max_voltage = std::clamp(motor_power_max_voltage, 0.0f, motor_operating_voltage);
    max_rps_ = ((motor_power_max_voltage / motor_operating_voltage) * motor_max_rpm) * max_rpm_ratio;
}

// ============================================================
// INVERSE KINEMATICS
// chassis speed (vx, vy, omega) -> module speed (rps) + angle (rad)
//
// Layout wheel:
//   module 1 = Front Left  (rx=+half_length_, ry=+half_width_)
//   module 2 = Front Right (rx=+half_length_, ry=-half_width_)
//   module 3 = Back Left   (rx=-half_length_, ry=+half_width_)
//   module 4 = Back Right  (rx=-half_length_, ry=-half_width_)
//
// Rumus per module (persamaan 11.14):
//   v_ix = vx - omega * ry
//   v_iy = vy + omega * rx
//   speed_i = sqrt(v_ix^2 + v_iy^2)
//   angle_i = atan2(v_iy, v_ix)
// ============================================================
Kinematic::module_state Kinematic::calculateModuleStates(float linear_x, float linear_y, float angular_z)
{
    float rx[4] = { +half_length_, +half_length_, -half_length_, -half_length_ };
    float ry[4] = { +half_width_, -half_width_, +half_width_, -half_width_ };

    float vix[4], viy[4], speed_mps[4], angle[4];

    for (int i = 0; i < 4; i++)
    {
        vix[i] = linear_x - angular_z * ry[i];
        viy[i] = linear_y + angular_z * rx[i];

        speed_mps[i] = sqrt(vix[i] * vix[i] + viy[i] * viy[i]);

        // ---- Fix #1: deadband untuk angle saat speed sangat kecil ----
        // Kalau speed module ini nyaris nol, atan2() sangat sensitif
        // terhadap noise kecil dan bisa lompat drastis. Daripada ikut
        // hasil atan2() yang liar, pertahankan angle terakhir yang valid.
        if (speed_mps[i] > angle_deadband_speed_)
        {
            angle[i] = atan2(viy[i], vix[i]);
            last_angle_[i] = angle[i];
        }
        else
        {
            angle[i] = last_angle_[i];
            speed_mps[i] = 0.0f; // pastikan wheel juga benar-benar berhenti
        }
    }

    Kinematic::module_state state;

    state.speed1 = std::clamp(speed_mps[0] / wheel_circumference_, -max_rps_, max_rps_);
    state.angle1 = angle[0];

    state.speed2 = std::clamp(speed_mps[1] / wheel_circumference_, -max_rps_, max_rps_);
    state.angle2 = angle[1];

    state.speed3 = std::clamp(speed_mps[2] / wheel_circumference_, -max_rps_, max_rps_);
    state.angle3 = angle[2];

    state.speed4 = std::clamp(speed_mps[3] / wheel_circumference_, -max_rps_, max_rps_);
    state.angle4 = angle[3];

    return state;
}

Kinematic::module_state Kinematic::getModuleStates(float linear_x, float linear_y, float angular_z)
{
    return calculateModuleStates(linear_x, linear_y, angular_z);
}

// ============================================================
// FORWARD KINEMATICS
// module speed (rps) + angle (rad) -> chassis speed (vx, vy, omega)
// ============================================================
Kinematic::velocities Kinematic::getVelocities(float speed1, float angle1,
                                                float speed2, float angle2,
                                                float speed3, float angle3,
                                                float speed4, float angle4)
{
    float v1x = (speed1 * wheel_circumference_) * cos(angle1);
    float v1y = (speed1 * wheel_circumference_) * sin(angle1);

    float v2x = (speed2 * wheel_circumference_) * cos(angle2);
    float v2y = (speed2 * wheel_circumference_) * sin(angle2);

    float v3x = (speed3 * wheel_circumference_) * cos(angle3);
    float v3y = (speed3 * wheel_circumference_) * sin(angle3);

    float v4x = (speed4 * wheel_circumference_) * cos(angle4);
    float v4y = (speed4 * wheel_circumference_) * sin(angle4);

    Kinematic::velocities vel;

    vel.linear_x = (v1x + v2x + v3x + v4x) / 4.0;
    vel.linear_y = (v1y + v2y + v3y + v4y) / 4.0;

    float omega_from_x = ((v3x + v4x) - (v1x + v2x)) / (4.0 * half_length_);
    float omega_from_y = ((v1y + v3y) - (v2y + v4y)) / (4.0 * half_width_);

    vel.angular_z = (omega_from_x + omega_from_y) / 2.0;

    return vel;
}

// ============================================================
// Optimisasi steering (dipanggil per module sebelum kirim ke motor)
// Kalau selisih sudut > 90 derajat (dengan hysteresis), lebih cepat
// balik arah speed daripada muter steering jauh-jauh.
//
// module_index (0..3) dipakai untuk menyimpan status flip terakhir
// per module secara terpisah -> mencegah chattering (fix #3).
// ============================================================
void Kinematic::optimizeModule(float desiredSpeed, float desiredAngle, float currentAngle,
                                float &outSpeed, float &outAngle, int module_index)
{
    float delta = desiredAngle - currentAngle;

    while (delta > M_PI)  delta -= 2 * M_PI;
    while (delta < -M_PI) delta += 2 * M_PI;

    const float flip_threshold = M_PI / 2.0f;
    const float hysteresis = toRad(5.0f); // margin 5 derajat, sesuaikan kalau perlu

    bool should_flip;
    if (last_flip_state_[module_index])
    {
        // sebelumnya sudah flip -> butuh delta lebih kecil dulu buat balik lagi
        should_flip = fabs(delta) > (flip_threshold - hysteresis);
    }
    else
    {
        should_flip = fabs(delta) > (flip_threshold + hysteresis);
    }
    last_flip_state_[module_index] = should_flip;

    if (should_flip)
    {
        outSpeed = -desiredSpeed;
        outAngle = currentAngle + delta - (delta > 0 ? M_PI : -M_PI);
    }
    else
    {
        outSpeed = desiredSpeed;
        outAngle = currentAngle + delta;
    }
}

float Kinematic::getMaxRPS()
{
    return max_rps_;
}

float Kinematic::toRad(float deg)
{
    return deg * M_PI / 180.0;
}

float Kinematic::toDeg(float rad)
{
    return rad * 180.0 / M_PI;
}