#include<rclcpp/rclcpp.hpp>
#include<moveit/move_group_interface/move_group_interface.h>
#include<example_interfaces/msg/bool.hpp>
#include<example_interfaces/msg/float64_multi_array.hpp>
#include<test_robot_interfaces/msg/pose_command.hpp>

// 类型别名
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using Floatarray = example_interfaces::msg::Float64MultiArray;
using PoseCmd = test_robot_interfaces::msg::PoseCommand;
using namespace std::placeholders;

// 自定义类 Commander（用来统一管理机械臂+夹爪）
class Commander
{
public:
    // 构造函数，传入ROS2节点指针
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;  // 保存节点对象

        // 创建机械臂规划组 "arm" 的MoveGroupInterface对象
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(0.1);
        arm_->setMaxAccelerationScalingFactor(0.1);

        // 创建夹爪规划组 "gripper" 的MoveGroupInterface对象
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

        open_gripper_sub_ = node_->create_subscription<Bool>("open_gripper", 10 , std::bind(&Commander::openGripperCallback, this, _1));

        // 编译期已知消息类型，用普通订阅即可（generic_subscription 是给运行时才知道类型的场景用的）
        joint_cmd_sub_ = node_->create_subscription<Floatarray>("joint_command", 10, std::bind(&Commander::jointCmdCallback, this, _1));

        pose_cmd_sub_ = node_->create_subscription<PoseCmd>("pose_command", 10, std::bind(&Commander::poseCmdCallback, this, _1));
    }



    // 函数1：跳转到预定义命名位姿
    void goToNamedTarget(const std::string &name)
    {
        // 设置规划起点 = 当前机械臂实时状态（非常重要！防止规划起点是旧状态）
        arm_->setStartStateToCurrentState();
        // 设置目标：命名目标，这个名字必须是MoveIt配置里预定义好的（比如home、pregrasp）
        arm_->setNamedTarget(name);
        // 调用上一课写好的通用函数：规划 + 执行
        planAndExecute(arm_);
    }

    // 函数2：跳转到指定关节角度目标
    void goToJointTarget(const std::vector<double> &joints)
    {
        arm_->setStartStateToCurrentState();
        // 设置目标：各个关节的角度（单位：弧度！）
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
    }

    // 函数3：跳转到笛卡尔位姿目标（x,y,z + 欧拉角roll pitch yaw）
    void goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path=false)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();

        if(!cartesian_path)
        {
            arm_->setPoseTarget(target_pose);
            planAndExecute(arm_);
        }
        else
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;    
            waypoints.push_back(target_pose.pose);
            // 定义轨迹消息，用来存放规划出来的整条轨迹
            moveit_msgs::msg::RobotTrajectory trajectory;

            // 核心函数：笛卡尔路径规划
            // 参数：路点列表，步长0.01m，输出轨迹
            double fraction = arm_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

            // fraction == 1 代表 100%规划成功
            if (fraction == 1) 
            {
                arm_->execute(trajectory); // 执行这条轨迹
            }

        }

    }    

    void openGripper()    // 打开夹爪
    {
        gripper_->setStartStateToCurrentState();   // 规划起点设为夹爪当前状态
        gripper_->setNamedTarget("gripper_open");  
        planAndExecute(gripper_);                  // 调用之前封装好的规划+执行函数
    }

    void closeGripper()    // 关闭夹爪
    {
        gripper_->setStartStateToCurrentState();
        if (!gripper_->setNamedTarget("gripper_close")) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set gripper_close target");
            return;
        }
        planAndExecute(gripper_);
    }


private:

    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        // 定义Plan结构体，用来保存规划出来的轨迹方案
        MoveGroupInterface::Plan plan;

        // 调用plan()进行运动规划，返回错误码，判断是否规划成功
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        // 如果规划成功，就执行这条轨迹
        if (success) {
            interface->execute(plan);
        }
    }

    void openGripperCallback(const Bool &msg)
    {
        if(msg.data)
        {
           openGripper(); 
        }
        else
        {
            closeGripper();
        }
    }

    void jointCmdCallback(const Floatarray &msg)
    {
        auto joints = msg.data; 

        if(joints.size() == 6)
        {
            goToJointTarget(joints);
        }
    }

    void poseCmdCallback(const PoseCmd &msg)
    {
        goToPoseTarget(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.cartesian_path);
    }

    std::shared_ptr<rclcpp::Node> node_;       // ROS2节点
    std::shared_ptr<MoveGroupInterface> arm_;    // 机械臂规划组对象
    std::shared_ptr<MoveGroupInterface> gripper_;// 夹爪规划组对象

    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<Floatarray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
};

// 主函数
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // ROS2初始化
    auto node = std::make_shared<rclcpp::Node>("commander"); // 创建名字叫commander的节点
    auto commander = Commander(node); // 实例化Commander类
    rclcpp::spin(node);  // 节点开始循环，保持运行（后面你就可以在这里加Subscriber回调）
    rclcpp::shutdown();  // 关闭ROS2
    return 0;
}
