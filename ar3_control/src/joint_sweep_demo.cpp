/*********************************************************************
 * Joint sweep demo for the REDAR3 / AR3 arm.
 *
 * Demonstrates the range of motion of every joint of the
 * 'manipulator' planning group. Starting from the 'ready' state
 * (SRDF, used as the home pose), each joint is swept to both sides of
 * home (scaled by amplitude_scale) and returned to home after every
 * sweep. Targets are clipped to the actual joint limits and,
 * optionally, to a window around home: range_fraction limits the
 * allowed excursion to a given fraction of the joint's full range
 * (e.g. 0.3 keeps the joint within a 30%-wide window centered on
 * home). Joint limits are read from the robot model - nothing is
 * hardcoded. A failed step is retried once with a reduced amplitude
 * before it is skipped.
 *
 * Private parameters:
 *   step_mode       (bool,   default true)  - wait for the 'next'
 *                   button in the RvizVisualToolsGui panel before each
 *                   motion; set to false to run automatically
 *   amplitude_scale (double, default 0.5)   - fraction of each
 *                   joint's range used as sweep amplitude, (0, 1]
 *   range_fraction  (double, default 1.0)   - allowed sweep window
 *                   around home as a fraction of each joint's full
 *                   range, (0, 1]; 1.0 disables the home-relative
 *                   limitation
 *   velocity_scale  (double, default 0.3)   - max velocity and
 *                   acceleration scaling factor for the trajectories
 *   planning_time   (double, default 5.0)   - planning timeout per
 *                   step in seconds
 *********************************************************************/

#include <cmath>
#include <string>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_model/joint_model.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_sweep_demo");
  ros::NodeHandle node_handle;
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";

  // Parameters
  bool step_mode = true;
  double amplitude_scale = 0.5;
  double range_fraction = 1.0;
  double velocity_scale = 0.3;
  double planning_time = 5.0;
  nh.param("step_mode", step_mode, true);
  nh.param("amplitude_scale", amplitude_scale, 0.5);
  nh.param("range_fraction", range_fraction, 1.0);
  nh.param("velocity_scale", velocity_scale, 0.3);
  nh.param("planning_time", planning_time, 5.0);

  if (amplitude_scale <= 0.0 || amplitude_scale > 1.0)
  {
    ROS_WARN_NAMED("joint_sweep", "amplitude_scale (%.2f) is out of (0, 1], falling back to 0.5", amplitude_scale);
    amplitude_scale = 0.5;
  }

  if (range_fraction <= 0.0 || range_fraction > 1.0)
  {
    ROS_WARN_NAMED("joint_sweep", "range_fraction (%.2f) is out of (0, 1], falling back to 1.0 (no home-relative limit)",
                   range_fraction);
    range_fraction = 1.0;
  }

  // MoveIt setup
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setPlanningTime(planning_time);
  move_group.setMaxVelocityScalingFactor(velocity_scale);
  move_group.setMaxAccelerationScalingFactor(velocity_scale);

  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ROS_INFO_NAMED("joint_sweep", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("joint_sweep", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Visualization
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  if (step_mode)
    visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.00;
  visual_tools.publishText(text_pose, "Joint Sweep Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Neutral pose: the 'ready' named state from the SRDF,
  // fall back to the current state if it is missing
  moveit::core::RobotState state(*move_group.getCurrentState());
  if (!state.setToDefaultValues(joint_model_group, "ready"))
  {
    ROS_WARN_NAMED("joint_sweep", "Named state 'ready' not found, using the current state as neutral pose");
    state = *move_group.getCurrentState();
  }
  std::vector<double> neutral;
  state.copyJointGroupPositions(joint_model_group, neutral);

  // Joint limits straight from the robot model, nothing is hardcoded
  const std::vector<const robot_state::JointModel*>& joints = joint_model_group->getActiveJointModels();
  ROS_INFO_NAMED("joint_sweep", "Sweeping %zu joints, amplitude scale %.2f, range fraction %.2f", joints.size(),
                 amplitude_scale, range_fraction);
  for (std::size_t i = 0; i < joints.size(); ++i)
    ROS_INFO_NAMED("joint_sweep", "  %s: [%.3f, %.3f] rad", joints[i]->getName().c_str(),
                   joints[i]->getVariableBounds()[0].min_position_, joints[i]->getVariableBounds()[0].max_position_);

  // Plan, visualize and execute one joint-space step
  auto move_to = [&](const std::vector<double>& target, const std::string& label) -> bool {
    move_group.setJointValueTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_WARN_NAMED("joint_sweep", "Planning FAILED for '%s'", label.c_str());
      return false;
    }
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, label, rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group->getLinkModel("link_6"),
                                       joint_model_group, rvt::LIME_GREEN);
    visual_tools.trigger();
    if (step_mode)
      visual_tools.prompt("Press 'next' to execute: " + label);
    if (move_group.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_WARN_NAMED("joint_sweep", "Execution FAILED for '%s'", label.c_str());
      return false;
    }
    return true;
  };

  if (step_mode)
    visual_tools.prompt("Press 'next' to start the joint sweep demo");

  const std::string dir_labels[2] = { "+", "-" };
  int step = 0;

  for (std::size_t i = 0; i < joints.size() && ros::ok(); ++i)
  {
    const robot_state::JointModel* joint = joints[i];
    if (joint->getVariableCount() != 1)
    {
      ROS_WARN_NAMED("joint_sweep", "Skipping %s: not a single-DOF joint", joint->getName().c_str());
      continue;
    }

    const double min_pos = joints[i]->getVariableBounds()[0].min_position_;
    const double max_pos = joints[i]->getVariableBounds()[0].max_position_;
    const double amplitude = amplitude_scale * 0.5 * (max_pos - min_pos);

    // Sweep window around the home pose: in each direction the joint may
    // leave home by at most range_fraction of the travel available towards
    // that limit, so the total window is range_fraction of the full range.
    // With range_fraction = 1.0 the joint can reach both limits.
    const double window[2] = { range_fraction * (max_pos - neutral[i]),   // towards the upper limit
                               range_fraction * (neutral[i] - min_pos) };  // towards the lower limit

    ROS_INFO_NAMED("joint_sweep", "Sweeping %s, range [%.3f, %.3f] rad, amplitude %.3f rad, window [%.3f, %.3f] rad",
                   joint->getName().c_str(), min_pos, max_pos, amplitude, neutral[i] - window[1],
                   neutral[i] + window[0]);

    for (int dir = 0; dir < 2 && ros::ok(); ++dir)
    {
      std::vector<double> target = neutral;
      const double excursion = std::min(amplitude, window[dir]);
      target[i] = (dir == 0) ? neutral[i] + excursion : neutral[i] - excursion;
      target[i] = std::max(min_pos, std::min(max_pos, target[i]));

      if (std::fabs(target[i] - neutral[i]) < 1e-4)
      {
        ROS_WARN_NAMED("joint_sweep", "%s: no motion possible in '%s' direction, skipping",
                       joint->getName().c_str(), dir_labels[dir].c_str());
        continue;
      }

      ++step;
      const std::string label = "Step " + std::to_string(step) + ": " + joint->getName() + " " + dir_labels[dir];
      ROS_INFO_NAMED("joint_sweep", "%s -> %.3f rad", label.c_str(), target[i]);

      if (!move_to(target, label))
      {
        // Retry once with a reduced amplitude before giving up on this direction
        std::vector<double> reduced = neutral;
        reduced[i] = neutral[i] + 0.6 * (target[i] - neutral[i]);
        if (!move_to(reduced, label + " (reduced)"))
          ROS_WARN_NAMED("joint_sweep", "Giving up on '%s', skipping", label.c_str());
      }

      // Back to the neutral pose before the next sweep
      move_to(neutral, "Return to neutral");
    }
  }

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Sweep Demo Complete", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  if (step_mode)
    visual_tools.prompt("Press 'next' to end the demo");

  ROS_INFO_NAMED("joint_sweep", "Demo finished, %d steps executed", step);

  ros::shutdown();
  return 0;
}
