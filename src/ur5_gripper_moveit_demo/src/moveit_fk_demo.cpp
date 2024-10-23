#include<ros/ros.h>
#include<moveit/move_group_interface/move_group_interface.h>

int main(int argc,char* argv[])
{
    ros::init (argc,argv,"moveit_ik_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("ur5_arm");

    arm.setGoalJointTolerance(0.001);
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);

    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    double targetPose[6]={0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125};
    std::vector<double>  target_group(6);
    target_group[0]=targetPose[0];
    target_group[1]=targetPose[1];
    target_group[2]=targetPose[2];
    target_group[3]=targetPose[3];
    target_group[4]=targetPose[4];
    target_group[5]=targetPose[5];

    arm.setJointValueTarget(target_group);
    arm.move();
    sleep(1);

    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    ros::shutdown();

    return 0;
}