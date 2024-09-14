#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "my_robot_msgs/action/move_to_pose.hpp"
#include "AMRA_Star/amra_star.hpp"

class AMRAStarActionClient : public rclcpp::Node
{
public:
    using MoveToPose = my_robot_msgs::action::MoveToPose;

    AMRAStarActionClient() : Node("amra_star_action_client")
    {
        // 初始化 Action 客户端，用于发送目标位姿
        this->action_client_ = rclcpp_action::create_client<MoveToPose>(this, "move_to_pose");

        // 假设初始 start 和 goal 的位姿
        std::array<int, 3> start_position = {0, 0, 0};
        std::array<double, 4> start_orientation = {1.0, 0.0, 0.0, 0.0};
        std::array<int, 3> goal_position = {10, 10, 0};
        std::array<double, 4> goal_orientation = {0.0, 0.0, 0.707, 0.707};

        // 创建 AMRAstar 对象并计算路径
        amra_star_ = std::make_shared<AMRA_Star::AMRAstar>(10, 10, "maps/Testmap.map", start_position, start_orientation, goal_position, goal_orientation, 1, 100);

        // 启动首次路径计算并发送第一个目标
        this->calculate_and_send_next_goal();
    }

private:
    rclcpp_action::Client<MoveToPose>::SharedPtr action_client_;
    std::shared_ptr<AMRA_Star::AMRAstar> amra_star_;
    size_t current_pose_index_ = 0;  // 当前路径中的索引

    // 计算路径并发送第一个目标位姿
    void calculate_and_send_next_goal()
    {
        // 先等待 Action Server 准备好
        if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action Server 未准备好.");
            return;
        }

        // 获取路径中的第一个目标位姿
        if (current_pose_index_ < amra_star_->solutionPath.size())
        {
            const auto& current_node = amra_star_->solutionPath[current_pose_index_];

            auto goal_msg = MoveToPose::Goal();
            goal_msg.pose.position.x = current_node.position[0];
            goal_msg.pose.position.y = current_node.position[1];
            goal_msg.pose.position.z = current_node.position[2];

            goal_msg.pose.orientation.w = current_node.orientation.getW();
            goal_msg.pose.orientation.x = current_node.orientation.getX();
            goal_msg.pose.orientation.y = current_node.orientation.getY();
            goal_msg.pose.orientation.z = current_node.orientation.getZ();

            // 发送目标位姿并设置回调
            auto send_goal_options = rclcpp_action::Client<MoveToPose>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&AMRAStarActionClient::goal_response_callback, this, std::placeholders::_1);
            send_goal_options.result_callback = std::bind(&AMRAStarActionClient::result_callback, this, std::placeholders::_1);
            send_goal_options.feedback_callback = std::bind(&AMRAStarActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

            this->action_client_->async_send_goal(goal_msg, send_goal_options);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "路径已经完成，所有目标已发送.");
        }
    }

    // 处理目标响应
    void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<MoveToPose>::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "目标被拒绝.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "目标已被接受.");
        }
    }

    // 处理反馈
    void feedback_callback(rclcpp_action::ClientGoalHandle<MoveToPose>::SharedPtr, const std::shared_ptr<const MoveToPose::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "当前进度: %f%%", feedback->progress);
    }

    // 处理任务结果
    void result_callback(const rclcpp_action::ClientGoalHandle<MoveToPose>::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "成功到达目标.");
            
            // 更新起始位置并重新计算路径
            if (++current_pose_index_ < amra_star_->solutionPath.size())
            {
                this->calculate_and_send_next_goal();
            }
            else
            {
                // 假设需要重新计算路径，更新起始位置
                RCLCPP_INFO(this->get_logger(), "路径完成，重新计算新路径.");
                this->recalculate_path();
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "任务未完成.");
        }
    }

    // 重新计算路径，假设当前路径的最后一个位置是新的起始位置
    void recalculate_path()
    {
        // 假设当前路径的最后一个点为新的起始点
        const auto& last_node = amra_star_->solutionPath.back();

        std::array<int, 3> new_start_position = last_node.position;
        std::array<double, 4> new_start_orientation = {
            last_node.orientation.getW(), 
            last_node.orientation.getX(), 
            last_node.orientation.getY(), 
            last_node.orientation.getZ()
        };

        // 设置一个新的目标位置，或者保留原始目标
        std::array<int, 3> new_goal_position = {10, 10, 0};  // 可以根据需要动态改变目标
        std::array<double, 4> new_goal_orientation = {0.0, 0.0, 0.707, 0.707};

        // 重新初始化 AMRAstar 对象并重新计算路径
        amra_star_ = std::make_shared<AMRA_Star::AMRAstar>(10, 10, "maps/Testmap.map", new_start_position, new_start_orientation, new_goal_position, new_goal_orientation, 1, 100);

        // 重新计算路径并发送第一个目标位姿
        this->calculate_and_send_next_goal();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AMRAStarActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
