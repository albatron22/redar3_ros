# ar3_control
This module contains sample code for motion planning with the MoveIt API. There is currently a demo for the move group interface which has been adapted for the AR3 from the tutorials. There is much more to the MoveIt API, do check out the [official MoveIt tutorials](https://ros-planning.github.io/moveit_tutorials/index.html).

## Move Group Interface Demo
This demo is a simplified version of the official tutorial and covers various ways of specifying and planning for a goal - namely pose goals, joint-space goals, Cartesian paths and planning with path constraints. This demo has modified goals that are valid for the AR3 so that you can try it out with the simulator or your own arm. This should be sufficient to get started with basic tasks. The official tutorial also covers adding objects to the environment, which I've chosen to leave out, so you are encouraged to go through that as well.

* **rviz_tools_gui**  
  You will need to add the rviz_tools_gui to step through the demo. You can do so with `Panels -> Add New Panel -> RvizVisualToolsGui`.

* **Marker Array**  
  `Add -> By display type -> MarkerArray` to view the poses and trajectories.

## Joint Sweep Demo
A demo node that shows the range of motion of all six joints of the `manipulator` group. Starting from the `ready` state (defined in the SRDF, used as the home pose) it sweeps each joint from 1 to 6 to both sides of home and returns to the home pose after every sweep. Joint limits are read from the robot model (nothing is hardcoded), target positions are clipped to the actual limits and to the optional home-relative window, and a failed step is retried once with a reduced amplitude before being skipped.

### Run
1. Start any bringup layer first (see the top-level README for details):
   ```
   roslaunch ar3_moveit_config demo.launch    # RViz-only, no arm
   ```
2. Start the demo:
   ```
   roslaunch ar3_control joint_sweep_demo.launch
   ```
   In the default step mode, press `next` in the RvizVisualToolsGui panel in RViz before every motion.

3. Or run it automatically with a limited range (30% of each joint's range around home):
   ```
   roslaunch ar3_control joint_sweep_demo.launch step_mode:=false range_fraction:=0.3
   ```

Parameters (all overridable as launch arguments):

* `step_mode` (default `true`) — wait for the `next` button in the RvizVisualToolsGui panel before every motion. Set to `false` (`step_mode:=false`) to run the whole sequence automatically.
* `amplitude_scale` (default `0.5`) — fraction of each joint's range used as the sweep amplitude, in `(0, 1]`.
* `range_fraction` (default `1.0`) — limits the sweep to a window around the home pose as a fraction of each joint's full range, in `(0, 1]`. For example, `range_fraction:=0.3` keeps every joint within a 30%-wide window centered on home even if `amplitude_scale` is larger; `1.0` allows reaching both limits.
* `velocity_scale` (default `0.3`) — velocity/acceleration scaling factor applied to planned trajectories.
* `planning_time` (default `5.0`) — planning timeout per step, in seconds.