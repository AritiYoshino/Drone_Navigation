#!/usr/bin/env bash
export LD_LIBRARY_PATH=/home/amov/third_party/ceres-cuda/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH}
set -e

RS_LAUNCH="/home/amov/realsense_ws/src/realsense-ros/realsense2_camera/launch/rs_camera_ego.launch"
VINS_CFG="/home/amov/vinsf_gpu_ws/src/VINS-Fusion/config/realsense_d435i/px4_d435_vins_gpu.yaml"
VINS_CHECK="/home/amov/check_vins_gravity.py"

gnome-terminal --window -e \
"bash -lc 'source /opt/ros/melodic/setup.bash; roslaunch mavros px4_ego.launch; exec bash'" \
--tab -e \
"bash -lc 'sleep 7; source /home/amov/realsense_ws/devel/setup.bash; roslaunch \"$RS_LAUNCH\" unite_imu_method:=linear_interpolation; exec bash'" \
--tab -e "bash -lc 'sleep 20; source /home/amov/realsense_ws/devel/setup.bash; source /home/amov/vinsf_gpu_ws/devel/setup.bash; rosrun vins vins_node \"$VINS_CFG\" __ns:=/uav1; exec bash'" \
--tab -e "bash -lc 'source /opt/ros/melodic/setup.bash; source /home/amov/Fast-Drone-250-master/devel/setup.bash; until timeout 5 rostopic echo -n 1 /mavros/imu/data_raw >/dev/null 2>&1; do sleep 1; done; until timeout 5 rostopic echo -n 1 /uav1/vins_estimator/imu_propagate >/dev/null 2>&1; do sleep 1; done; if python \"$VINS_CHECK\" --settle 8 --duration 2 --threshold-deg 2 --max-resets 3; then roslaunch px4ctrl run_ctrl_with_vins_bridge.launch; else echo \"[startup] VINS gravity check failed; px4ctrl blocked.\"; fi; exec bash'" \
--tab -e "bash -lc 'source /opt/ros/melodic/setup.bash; source /home/amov/Fast-Drone-250-master/devel/setup.bash; until timeout 5 rostopic echo -n 1 /uav1/vins_estimator/imu_propagate >/dev/null 2>&1; do sleep 1; done; until timeout 5 rostopic echo -n 1 /uav1/camera/depth/image_rect_raw >/dev/null 2>&1; do sleep 1; done; roslaunch ego_planner single_run_in_exp.launch; exec bash'" \
--tab -e "bash -lc 'source /opt/ros/melodic/setup.bash; source /home/amov/Fast-Drone-250-master/devel/setup.bash; until timeout 5 rostopic echo -n 1 /uav1/vins_estimator/imu_propagate >/dev/null 2>&1; do sleep 1; done; until timeout 5 rostopic echo -n 1 /uav1/camera/depth/image_rect_raw >/dev/null 2>&1; do sleep 1; done; roslaunch ego_planner rviz.launch; exec bash'" \
--tab -e "bash -lc 'source /opt/ros/melodic/setup.bash; source /home/amov/Fast-Drone-250-master/devel/setup.bash; until rosnode info /drone_0_ego_planner_node >/dev/null 2>&1; do sleep 1; done; rosrun ego_planner realtime_goal.py; exec bash'"
