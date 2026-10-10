// Copyright 2026 Kaan Ural
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Read-only 1 kHz recorder of the FR3 robot state, for diagnosing what the
// 50 Hz panel and tracking logs cannot resolve (the 2026-09-23 ~43 Hz buzz
// showed up in them only as 5-7 Hz aliases).
//
//     ros2 run fr3_tracking state_recorder            # until Ctrl-C
//     ros2 run fr3_tracking state_recorder --ros-args -p duration_s:=60.0
//
// Subscribes to /franka_robot_state_broadcaster/robot_state and writes one
// CSV row per message to $FR3_LOG_DIR/YYYY-MM-DD/state_<local time>.csv (or
// -p out:=<path>). It publishes nothing and calls nothing. C++ on purpose:
// decoding the 1 kHz state in Python cost 86% of a core.
//
// Columns (appended ones last, so older readers by name still work):
//   time            robot time, s;  mode, success: robot_mode, FCI success rate
//   q dq            measured joint position / velocity
//   tau dtau        measured link-side torque tau_J (includes gravity) and its rate
//   tau_ext         estimated external torque (filtered)
//   fx..mz          O_F_ext_hat_K: estimated external wrench, base frame
//   x..qw           O_T_EE
//   stamp           the message's header stamp, epoch s (joins with bags and logs)
//   tau_d           tau_J_d: the commanded torque, without gravity - the
//                   controller's output after its caps and rate limit
//   theta dtheta    motor-side position / velocity (before the joint elasticity)
//   contact_*       Cartesian contact flags (Desk's contact thresholds), 0/1
// Beside it, state_<...>.json holds what does not change within a recording:
// F_T_EE, EE_T_K and the end-effector, load and total inertia (mass, CoM, I).
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <memory>
#include <string>

#include <franka_msgs/msg/franka_robot_state.hpp>
#include <rclcpp/rclcpp.hpp>

using franka_msgs::msg::FrankaRobotState;

namespace {

std::string local_stamp(const char *format)
{
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    char out[32] = {0};
    std::strftime(out, sizeof(out), format, &local);
    return out;
}

std::string default_path()
{
    const char *root = std::getenv("FR3_LOG_DIR");
    const std::filesystem::path dir =
        std::filesystem::path(root ? root : ".") / local_stamp("%Y-%m-%d");
    std::filesystem::create_directories(dir);
    return (dir / ("state_" + local_stamp("%Y%m%d_%H%M%S") + ".csv")).string();
}

}  // namespace

class StateRecorder : public rclcpp::Node
{
public:
    StateRecorder() : Node("state_recorder")
    {
        const std::string topic =
            declare_parameter<std::string>("topic", "/franka_robot_state_broadcaster/robot_state");
        std::string path = declare_parameter<std::string>("out", "");
        const double duration_s = declare_parameter<double>("duration_s", 0.0);
        if (path.empty()) {
            path = default_path();
        }
        file_ = std::fopen(path.c_str(), "w");
        if (!file_) {
            throw std::runtime_error("cannot write " + path);
        }
        std::setvbuf(file_, nullptr, _IOFBF, 1 << 20);  // write in 1 MB blocks
        std::fprintf(file_, "time,mode,success");
        for (const char *group : {"q", "dq", "tau", "dtau", "tau_ext"}) {
            for (int j = 1; j <= 7; ++j) {
                std::fprintf(file_, ",%s%d", group, j);
            }
        }
        std::fprintf(file_, ",fx,fy,fz,mx,my,mz,x,y,z,qx,qy,qz,qw,stamp");
        for (const char *group : {"tau_d", "theta", "dtheta"}) {
            for (int j = 1; j <= 7; ++j) {
                std::fprintf(file_, ",%s%d", group, j);
            }
        }
        std::fprintf(file_, ",contact_x,contact_y,contact_z,contact_rx,contact_ry,contact_rz\n");
        sidecar_ = std::filesystem::path(path).replace_extension(".json").string();

        sub_ = create_subscription<FrankaRobotState>(
            topic, rclcpp::QoS(1000), [this](const FrankaRobotState::SharedPtr s) { write(*s); });
        if (duration_s > 0.0) {
            stop_ = create_wall_timer(std::chrono::duration<double>(duration_s),
                                      [] { rclcpp::shutdown(); });
        }
        RCLCPP_INFO(get_logger(), "Recording %s -> %s", topic.c_str(), path.c_str());
    }

    ~StateRecorder() override
    {
        std::fclose(file_);
        RCLCPP_INFO(get_logger(), "Wrote %zu rows.", rows_);
    }

private:
    void write(const FrankaRobotState &s)
    {
        std::fprintf(file_, "%.6f,%u,%.3f", s.time, s.robot_mode, s.control_command_success_rate);
        const auto seven = [this](const auto &v) {
            for (size_t j = 0; j < 7; ++j) {
                std::fprintf(file_, ",%.6g", j < v.size() ? v[j] : 0.0);
            }
        };
        seven(s.measured_joint_state.position);
        seven(s.measured_joint_state.velocity);
        seven(s.measured_joint_state.effort);
        seven(s.dtau_j);
        seven(s.tau_ext_hat_filtered.effort);
        const auto &w = s.o_f_ext_hat_k.wrench;
        const auto &p = s.o_t_ee.pose;
        std::fprintf(file_, ",%.4g,%.4g,%.4g,%.4g,%.4g,%.4g,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                     w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z,
                     p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y,
                     p.orientation.z, p.orientation.w);
        std::fprintf(file_, ",%.6f", s.header.stamp.sec + 1e-9 * s.header.stamp.nanosec);
        seven(s.desired_joint_state.effort);
        seven(s.measured_joint_motor_state.position);
        seven(s.measured_joint_motor_state.velocity);
        const auto &cl = s.collision_indicators.is_cartesian_linear_contact;
        const auto &ca = s.collision_indicators.is_cartesian_angular_contact;
        std::fprintf(file_, ",%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\n", cl.x, cl.y, cl.z, ca.x, ca.y, ca.z);
        if (rows_ == 0) {
            write_sidecar(s);
        }
        ++rows_;
    }

    void write_sidecar(const FrankaRobotState &s) const
    {
        std::FILE *f = std::fopen(sidecar_.c_str(), "w");
        if (!f) {
            RCLCPP_WARN(get_logger(), "cannot write %s", sidecar_.c_str());
            return;
        }
        const auto pose = [f](const char *name, const geometry_msgs::msg::Pose &p) {
            std::fprintf(f,
                         "  \"%s\": {\"xyz\": [%.6f, %.6f, %.6f], "
                         "\"quat_xyzw\": [%.6f, %.6f, %.6f, %.6f]},\n",
                         name, p.position.x, p.position.y, p.position.z, p.orientation.x,
                         p.orientation.y, p.orientation.z, p.orientation.w);
        };
        const auto inertia = [f](const char *name, const geometry_msgs::msg::Inertia &i,
                                 bool last) {
            std::fprintf(f,
                         "  \"%s\": {\"m\": %.6f, \"com\": [%.6f, %.6f, %.6f], "
                         "\"ixx_ixy_ixz_iyy_iyz_izz\": [%.6g, %.6g, %.6g, %.6g, %.6g, %.6g]}%s\n",
                         name, i.m, i.com.x, i.com.y, i.com.z, i.ixx, i.ixy, i.ixz, i.iyy, i.iyz,
                         i.izz, last ? "" : ",");
        };
        std::fprintf(f, "{\n  \"first_row_time\": %.6f,\n  \"first_row_stamp\": %.6f,\n", s.time,
                     s.header.stamp.sec + 1e-9 * s.header.stamp.nanosec);
        pose("F_T_EE", s.f_t_ee.pose);
        pose("EE_T_K", s.ee_t_k.pose);
        inertia("inertia_ee", s.inertia_ee.inertia, false);
        inertia("inertia_load", s.inertia_load.inertia, false);
        inertia("inertia_total", s.inertia_total.inertia, true);
        std::fprintf(f, "}\n");
        std::fclose(f);
    }

    std::FILE *file_{nullptr};
    std::string sidecar_;
    size_t rows_{0};
    rclcpp::Subscription<FrankaRobotState>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr stop_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateRecorder>());
    rclcpp::shutdown();
    return 0;
}
