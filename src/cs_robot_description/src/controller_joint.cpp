// ===========================================================================
// 底盘 + 云台折叠控制节点
//   服务 chassis_cmd (cs_interface/srv/ChassisCmd): vx, vy, w, fold
//   - vx/vy/w  -> geometry_msgs/Twist 发布到 chassis_cmd_vel_topic，
//                 经 ros_gz_bridge 送给 MecanumDrive2 插件（车体系速度）
//   - fold     -> 云台 4 个关节在 unfold_positions / fold_positions 之间
//                 按 fold_speed 线性插值，发布到 gimbal_position_controller
// 指令保持到下一次服务调用为止（和原来的行为一致）。
// ===========================================================================
#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "cs_interface/srv/chassis_cmd.hpp"

using namespace std::chrono_literals;

namespace cs_controller {

class ControllerJoint : public rclcpp::Node {
public:
    explicit ControllerJoint(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("controller_joint", options)
    {
        // 关节顺序必须和 controller.yaml 里 gimbal_position_controller.joints 一致:
        // [0_joint, 1_joint, 2_joint, 3_joint]
        unfold_pos_ = declare_parameter<std::vector<double>>("unfold_positions", {0.0, 0.0, 0.0, 0.0});
        fold_pos_   = declare_parameter<std::vector<double>>("fold_positions",   {0.0, 0.0, 0.0, 0.0});
        fold_speed_ = declare_parameter<double>("fold_speed", 1.5);   // rad/s，每个关节的最大速度
        const auto twist_topic = declare_parameter<std::string>("chassis_cmd_vel_topic", "/rm92/cmd_vel");

        if (unfold_pos_.size() != 4 || fold_pos_.size() != 4) {
            throw std::invalid_argument("unfold_positions / fold_positions 必须是 4 个值");
        }
        current_pos_ = unfold_pos_;

        twist_pub_  = create_publisher<geometry_msgs::msg::Twist>(twist_topic, 10);
        gimbal_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/gimbal_position_controller/commands", 10);

        chassis_srv_ = create_service<cs_interface::srv::ChassisCmd>(
            "chassis_cmd",
            [this](const cs_interface::srv::ChassisCmd::Request::SharedPtr req,
                   cs_interface::srv::ChassisCmd::Response::SharedPtr res) {
                twist_.linear.x  = req->vx;
                twist_.linear.y  = req->vy;
                twist_.angular.z = req->w;
                if (req->fold != fold_) {
                    RCLCPP_INFO(get_logger(), "云台%s", req->fold ? "折叠" : "展开");
                }
                fold_ = req->fold;
                res->success = true;
            });

        last_tick_ = now_s();
        timer_ = create_wall_timer(20ms, [this]() { tick(); });
        RCLCPP_INFO(get_logger(), "ControllerJoint 启动，等待 chassis_cmd (vx, vy, w, fold)...");
    }

private:
    static double now_s()
    {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    void tick()
    {
        const double t = now_s();
        const double step = fold_speed_ * (t - last_tick_);
        last_tick_ = t;

        const auto & target = fold_ ? fold_pos_ : unfold_pos_;
        for (size_t i = 0; i < current_pos_.size(); ++i) {
            const double err = target[i] - current_pos_[i];
            current_pos_[i] += std::clamp(err, -step, step);
        }

        std_msgs::msg::Float64MultiArray g;
        g.data = current_pos_;
        gimbal_pub_->publish(g);
        twist_pub_->publish(twist_);
    }

    std::vector<double> unfold_pos_, fold_pos_, current_pos_;
    double fold_speed_{1.5};
    bool fold_{false};
    double last_tick_{0.0};
    geometry_msgs::msg::Twist twist_;  // 默认全零，上电静止

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_pub_;
    rclcpp::Service<cs_interface::srv::ChassisCmd>::SharedPtr chassis_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cs_controller

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cs_controller::ControllerJoint>());
    rclcpp::shutdown();
    return 0;
}
