#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"  
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class PoseSubscriber : public rclcpp::Node {
public:
    PoseSubscriber() : Node("pose_subscriber") {
        // 初始化订阅者
        subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "target_pose", 10,
            std::bind(&PoseSubscriber::poseCallback, this, std::placeholders::_1));

        // 初始化关节状态订阅者
        // joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        //     "joint_states", 10,
        //     std::bind(&PoseSubscriber::jointStateCallback, this, std::placeholders::_1));

        // 动态查找 URDF 文件路径
        std::string urdf_file = ament_index_cpp::get_package_share_directory("panda_description") + "/urdf/panda.urdf";

        // 初始化 Pinocchio 模型
        pinocchio::urdf::buildModel(urdf_file, model_);
        data_ = pinocchio::Data(model_);

        // 初始化 TF 广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 初始化 JointState 发布器
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // 初始化关节配置向量q的尺寸
        q_ = pinocchio::neutral(model_);

        RCLCPP_INFO(this->get_logger(), "PoseSubscriber node has started.");

    }

private:
    // void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {

    //     /*
    //     // 打印 model_.names 中的所有关节名称
    //     RCLCPP_INFO(this->get_logger(), "Model joint names:");
    //     for (const auto& name : model_.names) {
    //         RCLCPP_INFO(this->get_logger(), "%s", name.c_str());
    //     }

    //     // 打印 msg->name 中的所有关节名称
    //     RCLCPP_INFO(this->get_logger(), "Received joint state names:");
    //     for (const auto& name : msg->name) {
    //         RCLCPP_INFO(this->get_logger(), "%s", name.c_str());
    //     }
    //     */

    //     for (size_t i = 0; i < msg->name.size(); ++i) {
    //         std::cout <<"msg_size: "<<msg->name.size()<<std::endl;

    //         if (model_.joints[i].shortname() == "JointModelRZ") {
    //             q_[i-1] = msg->position[i-1];
    //             RCLCPP_INFO(this->get_logger(), "Updated q_[%zu] to %f", i-1, q_[i-1]);
    //         }
    //         else 
    //         {
    //             RCLCPP_INFO(this->get_logger(), "Skipped update non-revolute joint: %s", model_.names[i].c_str());
    //             std::cout <<"msg_position: "<<i<<std::endl;
    //         }

    //     }
    // }


    void publishJointStates(const Eigen::VectorXd & q_new) {
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = this->get_clock()->now();

        // 遍历模型中的关节，从索引 1 开始，因为索引 0 是固定关节 "universe"
        for (size_t i = 1; i < model_.names.size(); ++i) {
            // 使用 Pinocchio 提供的关节类型检查方法来过滤非 revolute 关节
            if (model_.joints[i].shortname() == "JointModelRZ") { 
                // 添加可动关节的名称
                joint_state_msg.name.push_back(model_.names[i]);
                
                // 添加对应的关节角度，q_new[i-1] 对应实际关节状态
                joint_state_msg.position.push_back(q_new[i-1]);
                RCLCPP_INFO(this->get_logger(), "Added joint: %s with position %f", model_.names[i].c_str(), q_new[i-1]);
                
            } else {
                RCLCPP_INFO(this->get_logger(), "Skipped non-revolute joint: %s", model_.names[i].c_str());
            }
        }

        // 发布过滤后的 JointState 消息
        joint_state_publisher_->publish(joint_state_msg);

        RCLCPP_INFO(this->get_logger(), "Published joint state with %zu joints.", joint_state_msg.name.size());
    }






    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received a target_pose message.");

        // 使用 Pinocchio 进行逆运动学计算
        Eigen::VectorXd v(6); // 期望末端位姿速度
        pinocchio::SE3 desired_pose(
            Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z),
            Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z)
        );

        // 正向运动学计算当前位姿
        pinocchio::forwardKinematics(model_, data_, q_);
        pinocchio::computeJointJacobians(model_, data_); // 计算所有关节的雅可比矩阵
        pinocchio::SE3 current_pose = data_.oMi[model_.njoints-1]; // 获取末端执行器当前位姿

        // 计算末端执行器的位姿误差
        pinocchio::SE3 error_pose = desired_pose.actInv(current_pose);

        // 创建一个 Eigen::MatrixXd 来存储雅可比矩阵
        Eigen::MatrixXd jacobian(6, model_.nv); // 6行，模型的自由度列数
        std::cout <<"model_nv: "<<model_.nv<<std::endl;
        pinocchio::getJointJacobian(model_, data_, model_.njoints-1, pinocchio::LOCAL, jacobian);

        // 计算关节更新量
        double alpha = 0.1;  // 调整这个值以控制每次更新的步进大小
        v = pinocchio::log6(error_pose).toVector();
        q_= q_ - alpha * jacobian.transpose() * v;

        // 输出调试信息
        RCLCPP_INFO(this->get_logger(), "Current Pose: [%f, %f, %f]", current_pose.translation()[0], current_pose.translation()[1], current_pose.translation()[2]);
        RCLCPP_INFO(this->get_logger(), "Desired Pose: [%f, %f, %f]", msg->position.x, msg->position.y, msg->position.z);
        RCLCPP_INFO(this->get_logger(), "q_: [%f, %f, %f, %f, %f, %f, %f]", q_[0], q_[1], q_[2], q_[3], q_[4], q_[5], q_[6]);
        //RCLCPP_INFO(this->get_logger(), "q_new: [%f, %f, %f, %f, %f, %f, %f]", q_new[0], q_new[1], q_new[2], q_new[3], q_new[4], q_new[5], q_new[6]);

        // 创建并发布关节状态消息
        // sensor_msgs::msg::JointState joint_state_msg;
        // joint_state_msg.header.stamp = this->get_clock()->now();
        // joint_state_msg.name = model_.names; // 设置关节名称
        // joint_state_msg.position = std::vector<double>(q_.data(), q_.data() + q_.size()); // 将计算得到的关节角度赋值给消息
        // joint_state_publisher_->publish(joint_state_msg);
        publishJointStates(q_);
        //q_ = q_new;

        // 将当前位姿发布到TF
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "panda.panda_link0_0";
        transformStamped.child_frame_id = "panda.panda_link7_0";
        transformStamped.transform.translation.x = current_pose.translation()[0];
        transformStamped.transform.translation.y = current_pose.translation()[1];
        transformStamped.transform.translation.z = current_pose.translation()[2];
        Eigen::Quaterniond quat(current_pose.rotation());
        transformStamped.transform.rotation.x = quat.x();
        transformStamped.transform.rotation.y = quat.y();
        transformStamped.transform.rotation.z = quat.z();
        transformStamped.transform.rotation.w = quat.w();

        tf_broadcaster_->sendTransform(transformStamped);
    }


    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_;
    //rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    pinocchio::Model model_;
    pinocchio::Data data_;
    Eigen::VectorXd q_; // 当前的关节状态向量
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseSubscriber>());
    rclcpp::shutdown();
    return 0;
}
