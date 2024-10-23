#include<string>
#include<ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

int main(int argc,char **argv)
{
    ros::init(argc,argv,"moveit_cartesian_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("ur5_arm");

    std::string end_effector_name=arm.getEndEffectorLink();

    std::string reference_frame="base_link";

    arm.setPoseReferenceFrame(reference_frame);

    arm.allowReplanning(true);

    arm.setGoalOrientationTolerance(0.001);\
    arm.setGoalPositionTolerance(0.01);

    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.4);

    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    geometry_msgs::Pose start_pose=arm.getCurrentPose(end_effector_name).pose;

    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(start_pose);

    start_pose.position.z -= 0.2;
	waypoints.push_back(start_pose);

    start_pose.position.x += 0.1;
	waypoints.push_back(start_pose);

    start_pose.position.y += 0.1;
	waypoints.push_back(start_pose);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = 0.0;
    int maxtries = 100;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数

    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;
        
        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }
    
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");

	    // 生成机械臂的运动规划数据
	    moveit::planning_interface::MoveGroupInterface::Plan plan;
	    plan.trajectory_ = trajectory;

	    // 执行运动
	    arm.execute(plan);
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

	ros::shutdown(); 
	return 0;
}