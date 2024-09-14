#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp> 
#include "my_robot_msgs/action/move_to_pose.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>        
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>  

#include <Eigen/Dense>
#include <thread>


class MoveToPoseActionServer : public rclcpp::Node
{
public:
    using MoveToPose = my_robot_msgs::action::MoveToPose;
    using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

    MoveToPoseActionServer() : Node("move_to_pose_action_server")
    {
        // create action server
        action_server_ = rclcpp_action::create_server<MoveToPose>(
            this,
            "move_to_pose",
            std::bind(&MoveToPoseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveToPoseActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&MoveToPoseActionServer::handle_accepted, this, std::placeholders::_1));

        // pose_publisher (check action files in my robot msgs)
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("calculated_pose", 10);//10ms

        // initial pinocchio
        std::string urdf_filename = ament_index_cpp::get_package_share_directory("panda_description") + "/urdf/panda.urdf";
        pinocchio::urdf::buildModel(urdf_filename, model_);
        data_ = pinocchio::Data(model_);
        // joint state publisher
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        // initial pose
        current_q_ = pinocchio::neutral(model_);

        // for (size_t i = 0; i < model_.frames.size(); ++i) { //print all the frames(names)
        //     RCLCPP_INFO(this->get_logger(), "Frame %zu: %s", i, model_.frames[i].name.c_str());
        // }

        // RCLCPP_INFO(this->get_logger(), "number of the joint (nq): %d, FoD (nv): %d", model_.nq, model_.nv);


    }

private:
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_; 

    pinocchio::Model model_;
    pinocchio::Data data_;
    Eigen::VectorXd current_q_; 


    void publishJointStates(const Eigen::VectorXd & q_new) {
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = this->get_clock()->now();

        // Traverse all the joints，begin with 1，because 0 is fixed joint: "universe"
        for (size_t i = 1; i < model_.names.size(); ++i) {
            // check if the joint is revolute joint
            if (model_.joints[i].shortname() == "JointModelRZ") { 
                // add name
                joint_state_msg.name.push_back(model_.names[i]);
                
                // add joint angle，q_new[i-1] 
                joint_state_msg.position.push_back(q_new[i-1]);
                // RCLCPP_INFO(this->get_logger(), "Added joint: %s with position %f", model_.names[i].c_str(), q_new[i-1]);
                
            } else {
                // RCLCPP_INFO(this->get_logger(), "Skipped non-revolute joint: %s", model_.names[i].c_str());
            }
    }

    joint_state_publisher_->publish(joint_state_msg);

    // RCLCPP_INFO(this->get_logger(), "Published joint state with %zu joints.", joint_state_msg.name.size());
    }

    // Handle for new goals
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveToPose::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(this->get_logger(), "Pose requists accepted.");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Handle cancel requist 
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        (void) goal_handle;
        RCLCPP_INFO(this->get_logger(), "Cancel requist accepted.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Handel accepted requist
    void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        std::thread{
            std::bind(&MoveToPoseActionServer::execute, this, std::placeholders::_1),
            goal_handle}.detach();
    }



    // execution pose requist
    void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        const auto goal = goal_handle->get_goal(); // definition check my_robot_msgs
        auto feedback = std::make_shared<MoveToPose::Feedback>();
        auto result = std::make_shared<MoveToPose::Result>();

        RCLCPP_INFO(this->get_logger(), "IK begins.");

        // get pose
        Eigen::Vector3d target_position(goal->pose.position.x, goal->pose.position.y, goal->pose.position.z);
        Eigen::Quaterniond target_orientation(
            goal->pose.orientation.w, 
            goal->pose.orientation.x, 
            goal->pose.orientation.y, 
            goal->pose.orientation.z);

        pinocchio::SE3 target_SE3(target_orientation.toRotationMatrix(), target_position);

        // current pose 
        Eigen::VectorXd q = current_q_;

        // check q size 
        // if (q.size() != model_.nq) {
        //     RCLCPP_ERROR(this->get_logger(), "Joint q size (%ld) and nq (%d) not match", q.size(), model_.nq);
        //     return;
        // }

        // GET EE
        const std::string ee_frame_name = "panda.panda_link7_0.panda_joint8.body";  // From URDF
        const pinocchio::FrameIndex frame_id = model_.getFrameId(ee_frame_name);
        RCLCPP_INFO(this->get_logger(), "EE ID: %ld", static_cast<long int>(frame_id));

        // if (frame_id == (pinocchio::FrameIndex)(-1) || frame_id >= model_.nframes) {
        //     RCLCPP_ERROR(this->get_logger(), "Invalid EE%s", ee_frame_name.c_str());
        //     return;
        // }

        // parameters of IK (from official demo)
        const double eps  = 1e-4;  // tolerence
        const int IT_MAX  = 1000;  // max iteration
        const double DT   = 1e-1;  // step
        const double damp = 1e-6;  // damp

        pinocchio::Data::Matrix6x J(6, model_.nv);
        J.setZero();

        bool success = false;

        typedef Eigen::Matrix<double, 6, 1> Vector6d;
        Vector6d err;

        Eigen::VectorXd v(model_.nv); //for new joint pose adjustment q = q - DT*v

        // calculate init error
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacement(model_, data_, frame_id);
        const pinocchio::SE3 &current_SE3 = data_.oMf[frame_id];
        const pinocchio::SE3 initial_dMi = target_SE3.actInv(current_SE3);
        Eigen::VectorXd initial_error = pinocchio::log6(initial_dMi).toVector();
        double initial_error_norm = initial_error.norm();
        if (initial_error_norm < eps)
            initial_error_norm = eps;

        for (int i = 0; i < IT_MAX; ++i)
        {
            // check cancel requist
            if (goal_handle->is_canceling())
            {
                result->pose_error = (data_.oMf[frame_id].translation() - target_position).norm();
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Target cancelled");
                return;
            }

            // FK
            pinocchio::forwardKinematics(model_, data_, q);

            // Update frame
            pinocchio::updateFramePlacement(model_, data_, frame_id);

            // get EE 
            const pinocchio::SE3 &current_SE3 = data_.oMf[frame_id];

            // error
            const pinocchio::SE3 dMi = target_SE3.actInv(current_SE3);
            err = pinocchio::log6(dMi).toVector();

            double error_norm = err.norm();

            // check convergence
            if (error_norm < eps)
            {
                success = true;
                break;
            }

            // Jaconian
            pinocchio::computeFrameJacobian(model_, data_, q, frame_id, J);

            // new joint pose v
            Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
            JJt.diagonal().array() += damp;
            v.noalias() = - J.transpose() * JJt.ldlt().solve(err);

            // update q
            q = pinocchio::integrate(model_, q, v * DT);

            // feedback
            feedback->pose.position.x = current_SE3.translation().x();
            feedback->pose.position.y = current_SE3.translation().y();
            feedback->pose.position.z = current_SE3.translation().z();
            Eigen::Quaterniond current_orientation(current_SE3.rotation());
            feedback->pose.orientation.w = current_orientation.w();
            feedback->pose.orientation.x = current_orientation.x();
            feedback->pose.orientation.y = current_orientation.y();
            feedback->pose.orientation.z = current_orientation.z();

            feedback->progress = 1.0 - (error_norm / initial_error_norm);
            if (feedback->progress < 0.0)
                feedback->progress = 0.0;
            if (feedback->progress > 1.0)
                feedback->progress = 1.0;

            goal_handle->publish_feedback(feedback);

            // publish joint pose
            publishJointStates(q);

            // Simulate actuator
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "IK successed, error: %f", err.norm());

            // publish pose
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = "world";
            pose_msg.pose = feedback->pose;
            pose_publisher_->publish(pose_msg);

            // update q
            current_q_ = q;

            // result published
            result->pose_error = err.norm();
            goal_handle->succeed(result);
        }
        else
        {
            result->pose_error = err.norm();
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "IK divergence, target failed");
        }
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
