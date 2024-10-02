#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ar3_move_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ROS_INFO_STREAM_NAMED("ar3", "Planning frame: " << move_group.getPlanningFrame());
    ROS_INFO_STREAM_NAMED("ar3", "End effector link: " << move_group.getEndEffectorLink());
    ROS_INFO_STREAM_NAMED("ar3", "Available Planning Group:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    // Planning to a Pose goal
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
    move_group.getCurrentState()->printStateInfo();
    // std::cout << current_pose.pose.position.x << " "
    //  << current_pose.pose.position.y << " "
    //  << current_pose.pose.position.z << std::endl;

    /*Tool Center Point (TCP) */
    double tcp_roll = .0, tcp_pitch = .0, tcp_yaw = .0;
    nh.getParam("roll", tcp_roll);
    nh.getParam("pitch", tcp_pitch);
    nh.getParam("yaw", tcp_yaw);
    const double DEG_TO_RAD = M_PI / 180.0;
    tcp_roll *= DEG_TO_RAD;
    tcp_pitch *= DEG_TO_RAD;
    tcp_yaw *= DEG_TO_RAD;

    tf2::Quaternion q;
    q.setRPY(tcp_roll, tcp_pitch, tcp_yaw); // Create this quaternion from roll/pitch/yaw (in radians)
    q.normalize();
    ROS_INFO_STREAM("RPY: " << tcp_roll << " " << tcp_pitch << " " << tcp_yaw);
    // print q
    ROS_INFO_STREAM("q: " << q.getX() << " " << q.getY() << " " << q.getZ() << " " << q.getW());
    geometry_msgs::Pose target;
    target.orientation.w = q.getW();
    target.orientation.x = q.getX();
    target.orientation.y = q.getY();
    target.orientation.z = q.getZ();
    // target.position = current_pose.pose.position;
    target.position.x = current_pose.pose.position.x;
    target.position.y = current_pose.pose.position.y;
    target.position.z = current_pose.pose.position.z;

    move_group.setPoseTarget(target);
    move_group.setPlanningTime(5.0);

    moveit::planning_interface::MoveGroupInterface::Plan ar3_plan;
    if (move_group.plan(ar3_plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO_STREAM("Plan succes. Execute...");
        move_group.move();
        ROS_INFO_STREAM("Execute is Done");
    }
    else
    {
        ROS_ERROR_STREAM("Plan Error");
    }

    ros::shutdown();
    return 0;
}
