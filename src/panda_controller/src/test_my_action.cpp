#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "my_robot_msgs/action/move_to_pose.hpp"

class TestActionClient : public rclcpp::Node
{
public:
    using MoveToPose = my_robot_msgs::action::MoveToPose;
    using GoalHandleMoveToPose = rclcpp_action::ClientGoalHandle<MoveToPose>;

    TestActionClient()
        : Node("test_action_client")
    {
        client_ = rclcpp_action::create_client<MoveToPose>(this, "move_to_pose");

        //wait
        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "动作服务器不可用！");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = MoveToPose::Goal();

        // goal
        goal_msg.pose.position.x = 0.2;
        goal_msg.pose.position.y = 0.0;
        goal_msg.pose.position.z = 1.0;

        goal_msg.pose.orientation.w = 1.0;
        goal_msg.pose.orientation.x = 0.0;
        goal_msg.pose.orientation.y = 0.0;
        goal_msg.pose.orientation.z = 0.0;

        auto send_goal_options = rclcpp_action::Client<MoveToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&TestActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&TestActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&TestActionClient::result_callback, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<MoveToPose>::SharedPtr client_;

    void goal_response_callback(rclcpp_action::ClientGoalHandle<MoveToPose>::SharedPtr goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "目标被拒绝。");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "目标已接受。");
        }
    }

    void feedback_callback(
        GoalHandleMoveToPose::SharedPtr,
        const std::shared_ptr<const MoveToPose::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "收到反馈：进度 = %f%%", feedback->progress * 100);
    }

    void result_callback(const GoalHandleMoveToPose::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "目标成功！");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "目标被中止！");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "目标被取消！");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "未知的结果代码！");
                break;
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
