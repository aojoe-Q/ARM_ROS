#include<rclcpp/rclcpp.hpp>
#include<moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner =std::thread([&executor]() { executor.spin();});

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(0.1);
    arm.setMaxAccelerationScalingFactor(0.1);    // 10% during commissioning

    auto gripper = moveit::planning_interface::MoveGroupInterface(node, "gripper");

    //name goal

    // arm.setStartStateToCurrentState();          //设定起始姿态
    // arm.setNamedTarget("pose_1");               //设定目标位置

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS); //计划

    // if(success1)
    // {
    //     arm.execute(plan1);                 //执行
    // }

    // arm.setStartStateToCurrentState();          //设定起始姿态
    // arm.setNamedTarget("home");               //设定目标位置

    // moveit::planning_interface::MoveGroupInterface::Plan plan2;
    // bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS); //计划

    // if(success2)
    // {
    //     arm.execute(plan2);                 //执行
    // }

//-------------------------------------------------------------------------------------------------

    //joint goal 
    
    // std::vector<double> joints = { 1.5, 0.5, 0.0, 1.5, 0.0, -0.7 };

    // arm.setStartStateToCurrentState();
    // arm.setJointValueTarget(joints);

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS); //计划

    // if(success1)
    // {
    //     arm.execute(plan1);                 //执行
    // }

//-------------------------------------------------------------------------------------------------

    //pose goal

    tf2::Quaternion q;
    q.setRPY(3.14, 0.0, 0.0);
    q = q.normalize();


    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = 0.7;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 0.4;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS); //计划

    if(success1)
    {
        arm.execute(plan1);                 //执行
    }

    //cartesian path

    // 定义路点集合，存放笛卡尔位姿（末端Pose）
    std::vector<geometry_msgs::msg::Pose> waypoints;    
    geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose;// 获取机械臂当前末端位姿
    pose1.position.z += -0.2;
    waypoints.push_back(pose1);
    geometry_msgs::msg::Pose pose2 = pose1;
    pose2.position.y += 0.2;
    waypoints.push_back(pose2);
    geometry_msgs::msg::Pose pose3 = pose2;
    pose3.position.y += -0.2;
    pose3.position.z += 0.2;
    waypoints.push_back(pose3);    // 把这个目标点加入路点列表

    // 定义轨迹消息，用来存放规划出来的整条轨迹
    moveit_msgs::msg::RobotTrajectory trajectory;

    // 核心函数：笛卡尔路径规划
    // 参数：路点列表，步长0.01m，输出轨迹
    double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    // fraction == 1 代表 100%规划成功
    if (fraction == 1) 
    {
        arm.execute(trajectory); // 执行这条轨迹
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
