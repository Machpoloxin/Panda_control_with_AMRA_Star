#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <vector>

namespace rvt = rviz_visual_tools;

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher() : Node("trajectory_publisher")
    {
        // Initialize rviz_visual_tools with a valid frame ID
        visual_tools_ = std::make_shared<rvt::RvizVisualTools>("panda.panda_link0_0", "/trajectory_visualization", this);

        // Create publisher
        visual_tools_->loadMarkerPub();
        bool has_sub = visual_tools_->waitForMarkerSub(10.0);
        if (!has_sub)
        {
            RCLCPP_WARN(get_logger(), "/trajectory_visualization topic has no subscribers after 10 seconds, visualization may not be visible");
        }

        // Clear previous markers
        visual_tools_->deleteAllMarkers();
        visual_tools_->enableBatchPublishing();

        // Create subscriber to the existing pose topic
        ee_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "calculated_pose", 10,
            std::bind(&TrajectoryPublisher::eePoseCallback, this, std::placeholders::_1));
    }

private:
    rvt::RvizVisualToolsPtr visual_tools_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_subscriber_;

    // Trajectory points
    std::vector<geometry_msgs::msg::Point> trajectory_points_;

    void eePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Extract position from the pose message
        geometry_msgs::msg::Point point;
        point.x = msg->pose.position.x;
        point.y = msg->pose.position.y;
        point.z = msg->pose.position.z;

        // Add position to trajectory
        trajectory_points_.push_back(point);

        // Publish the trajectory as a path
        if (trajectory_points_.size() > 1)
        {
            visual_tools_->publishPath(trajectory_points_, rvt::LIME_GREEN, rvt::SMALL);
        }

        // Optionally, publish the current EE position as a sphere
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = Eigen::Vector3d(point.x, point.y, point.z);
        visual_tools_->publishSphere(pose, rvt::BLUE, rvt::SMALL);

        // Trigger the publishing of markers
        RCLCPP_INFO(this->get_logger(), "Published trajctory");
        visual_tools_->trigger();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
