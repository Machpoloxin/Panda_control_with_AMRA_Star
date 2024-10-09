#include <rclcpp/rclcpp.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

// Use namespace to simplify code
namespace rvt = rviz_visual_tools;

class ObstaclePublisher : public rclcpp::Node
{
public:
  ObstaclePublisher() : Node("obstacle_publisher")
  {
    // Initialize rviz_visual_tools with a valid frame ID
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("panda.panda_link0_0", "/my_rviz_visual_tools", this));

    // Create publisher
    visual_tools_->loadMarkerPub();
    bool has_sub = visual_tools_->waitForMarkerSub(10.0);
    if (!has_sub)
    {
      RCLCPP_WARN(get_logger(), "/rviz_visual_tools topic has no subscribers after 10 seconds, visualization may not be visible");
    }

    // Clear previous markers
    visual_tools_->deleteAllMarkers();
    visual_tools_->enableBatchPublishing();

    // **Declare parameters for start and goal positions**
    this->declare_parameter<std::vector<double>>("start_position", {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("goal_position", {1.0, 1.0, 1.0});

    // **Declare parameters for file path and scale**
    this->declare_parameter<std::string>("file_path", "");
    this->declare_parameter<double>("scale_sensor_robot", 1.0);
    file_path_ = this->get_parameter("file_path").as_string();
    scale_sensor_robot_ = this->get_parameter("scale_sensor_robot").as_double();

    loadObstacleData();


    // Create a timer to periodically publish the obstacle
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ObstaclePublisher::publishObstacle, this));
  }

private:

  void loadObstacleData()
  {
    std::ifstream infile(file_path_);
    if (!infile)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open obstacle data file: %s", file_path_.c_str());
      return;
    }

    std::string line;
    while (std::getline(infile, line))
    {
      // Remove any leading/trailing whitespace
      line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end());

      // Skip empty lines
      if (line.empty())
        continue;

      // Check if line starts with '('
      if (line.front() != '(')
        continue;

      // Parse the line
      // Expected format: (x, y, z, label)
      double x, y, z;
      std::string label;

      size_t pos1 = line.find('(');
      size_t pos2 = line.find(',', pos1);
      size_t pos3 = line.find(',', pos2 + 1);
      size_t pos4 = line.find(',', pos3 + 1);
      size_t pos5 = line.find(')', pos4 + 1);

      if (pos1 == std::string::npos || pos2 == std::string::npos || pos3 == std::string::npos ||
          pos4 == std::string::npos || pos5 == std::string::npos)
      {
        RCLCPP_WARN(this->get_logger(), "Skipping malformed line: %s", line.c_str());
        continue;
      }

      try
      {
        x = std::stod(line.substr(pos1 + 1, pos2 - pos1 - 1));
        y = std::stod(line.substr(pos2 + 1, pos3 - pos2 - 1));
        z = std::stod(line.substr(pos3 + 1, pos4 - pos3 - 1));
        label = line.substr(pos4 + 1, pos5 - pos4 - 1);
      }
      catch (const std::exception &e)
      {
        RCLCPP_WARN(this->get_logger(), "Exception parsing line: %s", e.what());
        continue;
      }

      // Remove any whitespace from label
      label.erase(std::remove_if(label.begin(), label.end(), ::isspace), label.end());

      // Only process obstacles labeled 'ob'
      if (label != "ob")
        continue;

      // Scale the position
      Eigen::Vector3d pos(x, y, z);
      pos /= scale_sensor_robot_;

      // Store the position
      obstacle_positions_.push_back(pos);
    }
    infile.close();
    RCLCPP_INFO(this->get_logger(), "Loaded %zu obstacle positions", obstacle_positions_.size());
  }

  // **Function to publish sensor data (obstacle positions)**
  void publishSensorData()
  {
    for (const auto &pos : obstacle_positions_)
    {
      Eigen::Isometry3d obstacle_pose = Eigen::Isometry3d::Identity();
      obstacle_pose.translation() = pos;

      // Publish sphere at the obstacle position
      visual_tools_->publishSphere(obstacle_pose, rviz_visual_tools::BLUE, rviz_visual_tools::SMALL);
    }
  }


  void publishObstacle()
  {
     publishSensorData();
    // Plot start and goal position
    std::vector<double> start_pos = this->get_parameter("start_position").as_double_array();
    std::vector<double> goal_pos = this->get_parameter("goal_position").as_double_array();

    // Ensure they have three elements
    if (start_pos.size() != 3 || goal_pos.size() != 3)
    {
      RCLCPP_ERROR(this->get_logger(), "start_position and goal_position parameters must be of size 3");
      return;
    }

    // Create poses for start and goal positions
    Eigen::Isometry3d start_pose = Eigen::Isometry3d::Identity();
    start_pose.translation() = Eigen::Vector3d(start_pos[0], start_pos[1], start_pos[2]);

    Eigen::Isometry3d goal_pose = Eigen::Isometry3d::Identity();
    goal_pose.translation() = Eigen::Vector3d(goal_pos[0], goal_pos[1], goal_pos[2]);

    // Publish spheres at start and goal positions
    visual_tools_->publishSphere(start_pose, rviz_visual_tools::YELLOW, rviz_visual_tools::LARGE);
    visual_tools_->publishSphere(goal_pose, rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);


    // // Plot first obstacle
    // // Define obstacle position and size
    // Eigen::Isometry3d obstacle_pose = Eigen::Isometry3d::Identity();
    // // Set obstacle position
    // obstacle_pose.translation().x() = 0.5;
    // obstacle_pose.translation().y() = 0.5;
    // obstacle_pose.translation().z() = 0.5;
    // // Set obstacle size
    // double depth = 0.5;   // Obstacle depth (along x-axis)
    // double width = 0.5;   // Obstacle width (along y-axis)
    // double height = 0.5;  // Obstacle height (along z-axis)
    // // Publish the cuboid as a red box
    // visual_tools_->publishCuboid(obstacle_pose, depth, width, height, rvt::RED);

    // Ensure markers are actually published to RViz
    RCLCPP_INFO(this->get_logger(), "Published start and goal points");
    visual_tools_->trigger();
  }

  // Member variables
  rvt::RvizVisualToolsPtr visual_tools_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Obstacle positions
  std::vector<Eigen::Vector3d> obstacle_positions_;

  // Parameters
  std::string file_path_;
  double scale_sensor_robot_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ObstaclePublisher>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
