#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp> 
#include "my_robot_msgs/action/move_to_pose.hpp"
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <Eigen/Dense>

class MoveToPoseActionServer : public rclcpp::Node
{
public:
    using MoveToPose = my_robot_msgs::action::MoveToPose;
    using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

    MoveToPoseActionServer() : Node("move_to_pose_action_server")
    {
        // 创建 action 服务端
        action_server_ = rclcpp_action::create_server<MoveToPose>(
            this,
            "move_to_pose",
            std::bind(&MoveToPoseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveToPoseActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&MoveToPoseActionServer::handle_accepted, this, std::placeholders::_1));

        // 创建 ROS2 发布者用于显示逆运动学结果
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("calculated_pose", 10);

        // 初始化 Pinocchio 模型
        std::string urdf_filename = "/path/to/your/robot.urdf"; // 替换为你的 URDF 文件路径
        pinocchio::urdf::buildModel(urdf_filename, model_);
        data_ = pinocchio::Data(model_);
    }

private:
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

    pinocchio::Model model_;
    pinocchio::Data data_;

    // 处理接收到的新目标
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveToPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "收到目标位姿请求.");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // 处理取消请求
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "收到取消目标请求.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 处理被接受的目标
    void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        std::thread{
            std::bind(&MoveToPoseActionServer::execute, this, std::placeholders::_1),
            goal_handle}.detach();
    }

    // 执行目标
    void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveToPose::Feedback>();
        auto result = std::make_shared<MoveToPose::Result>();

        RCLCPP_INFO(this->get_logger(), "开始执行逆运动学计算.");

        // 获取目标位姿
        Eigen::Vector3d target_position(goal->pose.position.x, goal->pose.position.y, goal->pose.position.z);
        Eigen::Quaterniond target_orientation(
            goal->pose.orientation.w, 
            goal->pose.orientation.x, 
            goal->pose.orientation.y, 
            goal->pose.orientation.z);

        pinocchio::SE3 target_SE3(target_orientation.toRotationMatrix(), target_position);

        // 初始化关节配置为中立位置
        Eigen::VectorXd q = pinocchio::neutral(model_);
        
        // 目标末端执行器
        const pinocchio::FrameIndex frame_id = model_.getFrameId("tool0");  // 替换为你的末端执行器
        
        // 迭代求解逆运动学
        const double tolerance = 1e-4;  // 设定误差容忍度
        const int max_iter = 1000;      // 最大迭代次数
        double alpha = 0.1;             // 设定步长

        for (int i = 0; i < max_iter; ++i)
        {
            // 计算雅可比矩阵
            pinocchio::forwardKinematics(model_, data_, q);
            pinocchio::updateFramePlacement(model_, data_, frame_id);
            pinocchio::computeJointJacobians(model_, data_, q);
            Eigen::MatrixXd J = pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED);

            // 计算末端当前位姿
            const pinocchio::SE3 &current_SE3 = data_.oMf[frame_id];
            Eigen::VectorXd error = pinocchio::log6(current_SE3.actInv(target_SE3)).toVector();

            // 如果误差小于设定容忍度则认为已收敛
            if (error.norm() < tolerance)
            {
                RCLCPP_INFO(this->get_logger(), "逆运动学计算成功，误差: %f", error.norm());

                // 发布逆运动学计算的位姿到 ROS2
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp = this->now();
                pose_msg.header.frame_id = "world";
                pose_msg.pose.position.x = current_SE3.translation().x();
                pose_msg.pose.position.y = current_SE3.translation().y();
                pose_msg.pose.position.z = current_SE3.translation().z();
                Eigen::Quaterniond current_orientation(current_SE3.rotation());
                pose_msg.pose.orientation.w = current_orientation.w();
                pose_msg.pose.orientation.x = current_orientation.x();
                pose_msg.pose.orientation.y = current_orientation.y();
                pose_msg.pose.orientation.z = current_orientation.z();
                pose_publisher_->publish(pose_msg);

                // 反馈进度
                feedback->progress = 100.0;  // 假设直接到达
                goal_handle->publish_feedback(feedback);

                // 任务完成
                result->success = true;
                goal_handle->succeed(result);
                return;
            }

            // 使用伪逆更新关节配置
            Eigen::MatrixXd J_pseudo_inverse = J.completeOrthogonalDecomposition().pseudoInverse();
            q = pinocchio::integrate(model_, q, -alpha * J_pseudo_inverse * error);
        }

        // 超出迭代次数，未收敛
        result->success = false;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "逆运动学未收敛，计算失败.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToPoseActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
