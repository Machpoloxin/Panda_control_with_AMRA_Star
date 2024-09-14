#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "my_robot_msgs/action/move_to_pose.hpp"
#include "AMRA_Star/amra_star.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <functional>
#include <future>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


class MoveToPoseActionClient : public rclcpp::Node
{
public:
    using MoveToPose = my_robot_msgs::action::MoveToPose;

    MoveToPoseActionClient() : Node("move_to_pose_action_client")
    {
        RCLCPP_INFO(this->get_logger(),"Node started: move_to_pose_action_client");
        // Initialization

        std::string urdf_file = ament_index_cpp::get_package_share_directory("panda_description") + "/urdf/panda.urdf";
        std::string map_file = ament_index_cpp::get_package_share_directory("amra_star") + "/maps/Testmap.map";

        pinocchio::Model model_;
        pinocchio::Data data_;
        pinocchio::urdf::buildModel(urdf_file, model_);
        data_ = pinocchio::Data(model_);
        Eigen::VectorXd neutral_configuration = pinocchio::neutral(model_);
        pinocchio::forwardKinematics(model_, data_, neutral_configuration);
        pinocchio::SE3 neutral_pose = data_.oMi[model_.nq - 1];  
        Eigen::Quaterniond neutral_quaternion(neutral_pose.rotation());
        Eigen::Vector3d neutral_position = neutral_pose.translation();

        // Declare Parameters
        // Scale between sensor maps and robot
        this->declare_parameter("scale_sensor_robot",1.0);
        this->get_parameter("scale_sensor_robot", scaleSensorMapRobot); //map/robot = sensor / real

        // Default position (real world)
        this->declare_parameter("start_position", std::vector<double>{neutral_position.x(), 
                                                                      neutral_position.y(), 
                                                                      neutral_position.z()});

        this->declare_parameter("start_orientation", std::vector<double>{neutral_quaternion.w(), 
                                                                         neutral_quaternion.x(), 
                                                                         neutral_quaternion.y(), 
                                                                         neutral_quaternion.z()});

        this->declare_parameter("goal_position", std::vector<double>{neutral_position.x(), 
                                                                     neutral_position.y(), 
                                                                     neutral_position.z()});

        this->declare_parameter("goal_orientation", std::vector<double>{neutral_quaternion.w(), 
                                                                        neutral_quaternion.x(), 
                                                                        neutral_quaternion.y(), 
                                                                        neutral_quaternion.z()});

        std::vector<double> start_position_vector; // sensor space(for amra_star need to be discreted)
        std::vector<double> start_orientation_vector;
        std::vector<double> goal_position_vector; //sensor space
        std::vector<double> goal_orientation_vector;

        this->get_parameter("start_position", start_position_vector);
        this->get_parameter("start_orientation", start_orientation_vector);
        this->get_parameter("goal_position", goal_position_vector);
        this->get_parameter("goal_orientation", goal_orientation_vector);
        
        // Add safty check here and scaling !!!
        /*
        if (!safty_check(start_position_vector,...)){
            RCLCPP_ERROR(this->get_logger(), "Invalid start or goal configuration!!!");
            return;
        }
        */

        std::array<int, 3> start_position = { // discretization for amra_star alg.
            static_cast<int>(scaleSensorMapRobot * start_position_vector[0]),
            static_cast<int>(scaleSensorMapRobot * start_position_vector[1]),
            static_cast<int>(scaleSensorMapRobot * start_position_vector[2])
        };

        std::array<double, 4> start_orientation = {start_orientation_vector[0], 
                                                   start_orientation_vector[1], 
                                                   start_orientation_vector[2], 
                                                   start_orientation_vector[3]};

        std::array<int, 3> goal_position = {
            static_cast<int>(scaleSensorMapRobot * goal_position_vector[0]),
            static_cast<int>(scaleSensorMapRobot * goal_position_vector[1]),
            static_cast<int>(scaleSensorMapRobot * goal_position_vector[2])
        };

        std::array<double, 4> goal_orientation = {goal_orientation_vector[0], 
                                                  goal_orientation_vector[1], 
                                                  goal_orientation_vector[2], 
                                                  goal_orientation_vector[3]};

        // Initialize AMRAstar object and calculate the path
        final_goal_position_ = {goal_position[0], goal_position[1], goal_position[2]};
        final_goal_orientation_ = {goal_orientation[0], goal_orientation[1], 
                               goal_orientation[2], goal_orientation[3]};

        amra_star_ = std::make_shared<AMRA_Star::AMRAstar>(10, 10, map_file,
                                                           start_position, start_orientation,
                                                           final_goal_position_, final_goal_orientation_,
                                                           1, 100);
        //amra_star_->search();

        // Initialize Action client
        this->action_client_ = rclcpp_action::create_client<MoveToPose>(this, "move_to_pose");
        // Wait for the action server to be available
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        // Start the first path calculation and send the first goal
        this->calculate_and_send_next_goal();
    }

private:
    double scaleSensorMapRobot;
    rclcpp_action::Client<MoveToPose>::SharedPtr action_client_;
    std::shared_ptr<AMRA_Star::AMRAstar> amra_star_;
    size_t current_pose_index_ = 0;  //current index of solution of amra_star
    std::array<int, 3> final_goal_position_;  
    std::array<double, 4> final_goal_orientation_; 
    std::vector<AMRA_Star::Node> solutionPath_;



    void calculate_and_send_next_goal()
    {
        if (solutionPath_.empty()){
            amra_star_->search();
            solutionPath_ = amra_star_->getSolution();
        }

        if (current_pose_index_ < solutionPath_.size())
        {
            const auto& current_node = solutionPath_[current_pose_index_];

            auto goal_msg = MoveToPose::Goal();
            goal_msg.pose.position.x = current_node.position[0] / scaleSensorMapRobot; // sensor -> real world 
            goal_msg.pose.position.y = current_node.position[1] / scaleSensorMapRobot;
            goal_msg.pose.position.z = current_node.position[2] / scaleSensorMapRobot;

            goal_msg.pose.orientation.w = current_node.orientation.getW();
            goal_msg.pose.orientation.x = current_node.orientation.getX();
            goal_msg.pose.orientation.y = current_node.orientation.getY();
            goal_msg.pose.orientation.z = current_node.orientation.getZ();

            // send desired pose and set callback
            auto send_goal_options = rclcpp_action::Client<MoveToPose>::SendGoalOptions();

            send_goal_options.goal_response_callback = std::bind(&MoveToPoseActionClient::goal_response_callback, this, std::placeholders::_1);
            send_goal_options.result_callback = std::bind(&MoveToPoseActionClient::result_callback, this, std::placeholders::_1);
            send_goal_options.feedback_callback = std::bind(&MoveToPoseActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

            this->action_client_->async_send_goal(goal_msg, send_goal_options); //may need sync.
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Task finished, all trajectory has been sent.");
        }
    }

    // goal response
    void goal_response_callback(rclcpp_action::ClientGoalHandle<MoveToPose>::SharedPtr goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Target rejected");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Target accepted");
        }
    }


    // feedback
    void feedback_callback(
        rclcpp_action::ClientGoalHandle<MoveToPose>::SharedPtr, 
        const std::shared_ptr<const MoveToPose::Feedback> feedback)
    {
        // current pose
        //const auto& current_pose = feedback->pose;
        
        // print current pose
        // RCLCPP_INFO(this->get_logger(), 
        //             "Current position: [x: %f, y: %f, z: %f], Orientation: [w: %f, x: %f, y: %f, z: %f]",
        //             current_pose.position.x, current_pose.position.y, current_pose.position.z,
        //             current_pose.orientation.w, current_pose.orientation.x, 
        //             current_pose.orientation.y, current_pose.orientation.z);

        // print progress(error)
        RCLCPP_INFO(this->get_logger(), "Current progress(error): %f%%", feedback->progress * 100);
    }


    // result
    void result_callback(const rclcpp_action::ClientGoalHandle<MoveToPose>::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Current target reached.");
            
            // check error, if too large then recalculate path
            float error_threshold = 0.05; 
            if (result.result->pose_error > error_threshold)
            {
                RCLCPP_WARN(this->get_logger(), "error to big (%f), recalculate trajectory.", result.result->pose_error);
                this->recalculate_path_with_current_pose();
            }
            else
            {
                if (++current_pose_index_ < solutionPath_.size())
                {
                    this->calculate_and_send_next_goal(); // send next pose
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Current trajecory finished, you can start a new goal");
                    this->ask_for_new_goal(); // Ask for next task
                }
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Current target can not reach, replanning the trajectory.");
            this->recalculate_path_with_current_pose();
        }
    }


    void recalculate_path_with_current_pose()
    {
        RCLCPP_INFO(this->get_logger(), "Replaning trajectory, start with current pose.");

        // get current pose
        geometry_msgs::msg::Pose current_pose = get_current_pose();

        //set pose
        std::array<int, 3> new_start_position = {
            static_cast<int>(current_pose.position.x * scaleSensorMapRobot),
            static_cast<int>(current_pose.position.y * scaleSensorMapRobot),
            static_cast<int>(current_pose.position.z * scaleSensorMapRobot)
        };

        std::array<double, 4> new_start_orientation = {
            current_pose.orientation.w, current_pose.orientation.x, 
            current_pose.orientation.y, current_pose.orientation.z
        };
        
        // replan the path
        amra_star_->reSearchPath(new_start_position, new_start_orientation, final_goal_position_, final_goal_orientation_);
        solutionPath_ = amra_star_->getSolution();
        // set new goal
        current_pose_index_ = 0;
        this->calculate_and_send_next_goal();
    }

    geometry_msgs::msg::Pose get_current_pose()
    {
        tf2_ros::Buffer tf_buffer(this->get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer);

        geometry_msgs::msg::Pose current_pose;

        try
        {
            // From "panda.panda_link0_0" to "panda.panda_link7_0" transformation
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer.lookupTransform("panda.panda_link0_0", "panda.panda_link7_0", tf2::TimePointZero);

            // Extract position
            current_pose.position.x = transform_stamped.transform.translation.x;
            current_pose.position.y = transform_stamped.transform.translation.y;
            current_pose.position.z = transform_stamped.transform.translation.z;

            // Extract orientation
            current_pose.orientation = transform_stamped.transform.rotation;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
        }

        return current_pose;
    }

    void ask_for_new_goal()
    {
        RCLCPP_INFO(this->get_logger(), "Task accomplished, please add new pose: (format: x y z w qx qy qz):");
        //Need GUI
    }


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToPoseActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
