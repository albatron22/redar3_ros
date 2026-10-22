# AGENTS.md

## What this repo is
- Catkin workspace `src/` for the **REDAR3** 6-axis arm — a fork of `ongdexter/ar3_core` (ROS 1, ros_control + MoveIt) adapted for a Teensy 4.1 controller running modified AR4 firmware v4.4.
- The repo root **is** the workspace's `src/`. The workspace root is the parent dir (`~/ar3_ros1_ws`): run `catkin_make` and `source ~/ar3_ros1_ws/devel/setup.bash` from there, not from this repo.
- The top of `README.md` (Russian section) documents all fork-specific changes vs upstream.

## Environment / Docker dev container
- This machine has **only ROS 2 Jazzy** installed natively (`/opt/ros/jazzy`). ROS 1 Noetic for this repo runs in the dev container defined in `docker/` (`docker-compose.yml` + `Dockerfile` + `entrypoint.sh`).
- Workflow: `xhost +local:` (once per X session, needed for rviz) → `docker compose -f docker/docker-compose.yml up -d` → `docker compose -f docker/docker-compose.yml exec ar3 bash`. Inside: `catkin_make` (no args — builds all packages).
- The **entire workspace** `~/ar3_ros1_ws` is mounted at `/home/albert/catkin_ws`, so `build/` and `devel/` land on the host next to `src/`, and rebuilds need no image changes. Rebuild the image only after editing `docker/Dockerfile` (e.g. to enable Gazebo: uncomment `ros-noetic-gazebo-ros-pkgs`).
- The container uses `network_mode: host` (ROS master/nodes share the host network) and `privileged: true` (all `/dev/ttyACM*`, `/dev/ttyUSB*` visible; survives USB re-plug — no per-device mapping needed).
- Native `catkin_make` on the host **will not work** (no ROS 1, no `/opt/ros/noetic`) — don't try.

## Build & run
- No tests, CI, or lint config exist in this repo — don't invent verification commands beyond `catkin_make`.
- Running takes two launch layers, in order:
  1. Arm: real → `roslaunch ar3_hardware_interface ar3_hardware_bringup.launch`, sim → `roslaunch ar3_gazebo ar3_gazebo_bringup.launch`
  2. MoveIt + RViz → `roslaunch ar3_moveit_config ar3_moveit_bringup.launch`
  - RViz-only demo (no arm): `roslaunch ar3_moveit_config demo.launch`
  - Programmatic MoveIt demo: `roslaunch ar3_control move_group_demo.launch` (after the two layers above)
- By default, hardware bringup **runs joint calibration, which physically moves the arm**. Skip it with `use_existing_calibrations:=true`.
- Serial access requires `dialout` group membership (already set up in the container). Port/baud: `/dev/ttyACM0` @ 9600 in `ar3_hardware_interface/config/hardware_driver.yaml`.
- GUI in the container needs `xhost +local:` on the host **once per X session** — after a host reboot, rviz fails with "could not connect to display" until it's re-run.

## Serial protocol — do not "fix" without the real firmware
- `ar3_hardware_drivers/src/TeensyDriver.cpp` speaks the RedAR3 v4.4 protocol: `UP...` init (expects `Done\n`), `RP\n` position query (reply starts directly with `A…`, angles in degrees, no `JP` header), `RJ...` move command.
- The Teensy sketches in `ar3_microcontrollers/` are **legacy upstream AR3 firmware** using the old `MT`/`JP` protocol — they are **not compatible** with this driver. The actual Teensy 4.1 sketch is not in this repo (protocol analysis notes are in the author's Obsidian).
- The arm has **no encoders**: feedback arrives already in degrees. `encoder_steps_per_deg` in `hardware_driver.yaml` is dead config (parsed, never used — conversions are pass-throughs).
- Joint 1 is sign-inverted in both command and feedback inside `TeensyDriver::update()` (the code comments even say "why? Dont know..."). Keep both inversions symmetric or RViz and the arm will disagree.
- `setStepperSpeed()` is a stub; the `RJ` message hardcodes speed/accel (`Sp50Ac20...`). `joint_limits.yaml` max_velocity/acceleration affect MoveIt planning only, not actual stepper speeds.

## Config quirks
- `ar3_hardware_interface/config/joint_offsets.yaml`: these offsets are **not** HMI calibration values — they're tuned so the RViz model matches the real robot (note `joint_3: 90.0`).
- `ar3_moveit_config/config/joint_limits.yaml` is also loaded by hardware bringup (under `/ar3`, values in radians) to feed velocity/accel limits. The README warns that casually editing limits has broken the RViz model before (J2 incident).
- `hardware_interface.yaml` `loop_hz: 50` was raised deliberately for smooth motion; the node also `ROS_INFO`s every joint position each cycle, so it's very chatty by design.
- `ar3_description/urdf/ar3.urdf` is xacro-generated — edit `ar3.urdf.xacro` (real arm) or `ar3_gazebo.urdf.xacro` (sim, carries the Gazebo plugins), never the `.urdf`.

## App-level notes
- MoveIt planning group is `manipulator`; end-effector link is `link_6`.
- `ar3_control` has two executables: `move_group_demo` (interactive step-through demo) and `ar3_move_node` (moves to a pose given by `roll`/`pitch`/`yaw` params in **degrees**, set in `ar3_move_node.launch`).
- Default branch is `dev`.
