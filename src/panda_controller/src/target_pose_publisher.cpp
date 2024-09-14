#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

class TargetPosePublisher : public rclcpp::Node {
public:
    TargetPosePublisher() : Node("target_pose_publisher") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("target_pose", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TargetPosePublisher::publish_pose, this));
    }

private:
    void publish_pose() {
        auto message = geometry_msgs::msg::Pose();
        // 设置目标位姿
        message.position.x = 0.2;
        message.position.y = 0.0;
        message.position.z = 1.0;
        message.orientation.w = 1.0;
        publisher_->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetPosePublisher>());
    rclcpp::shutdown();
    return 0;
}