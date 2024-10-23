#include<string>
#include<ros/ros.h>
#include<moveit/move_group_interface/move_group_interface.h>


int main(int argc,char **argv)
{
    ros::init(argc,argv,"moveit_ik_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("ur5_arm");

    std::string end_effector_link=arm.getEndEffectorLink();

    std::string reference_frame ="base_link";
    arm.setPoseReferenceFrame(reference_frame);

    arm.allowReplanning(true);
    
    arm.setGoalPositionTolerance(0.01);
    arm.setGoalOrientationTolerance(0.01);
    arm.setMaxVelocityScalingFactor(0.4);
    arm.setMaxAccelerationScalingFactor(0.2);

    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = 0.70692;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.70729;

    target_pose.position.x = 0.5;
    target_pose.position.y = 0.01;
    target_pose.position.z = 0.01;   

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode success = arm.plan(plan);

    ROS_INFO("plan (pose goal) %s",success== moveit::core::MoveItErrorCode::SUCCESS?"":"failed");

    if(success== moveit::core::MoveItErrorCode::SUCCESS)
        arm.execute(plan);
    sleep(1);
    
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);
    
    ros::shutdown();

    return 0;
}