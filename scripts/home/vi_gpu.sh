#!/usr/bin/env bash
export LD_LIBRARY_PATH=/home/amov/third_party/ceres-cuda/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH}
set -e

RS_LAUNCH="/home/amov/realsense_ws/src/realsense-ros/realsense2_camera/launch/rs_camera_ego.launch"
VINS_CFG="/home/amov/vinsf_gpu_ws/src/VINS-Fusion/config/realsense_d435i/px4_d435_vins_gpu.yaml"
VINS_CHECK="/home/amov/check_vins_gravity.py"

gnome-terminal --window -e \
"bash -lc 'source /opt/ros/melodic/setup.bash; roslaunch mavros px4_ego.launch; exec bash'" \
--tab -e \
"bash -lc 'sleep 6; source /home/amov/realsense_ws/devel/setup.bash; roslaunch \"$RS_LAUNCH\" unite_imu_method:=linear_interpolation; exec bash'" \
--tab -e "bash -lc 'sleep 20; source /home/amov/realsense_ws/devel/setup.bash; source /home/amov/vinsf_gpu_ws/devel/setup.bash; rosrun vins vins_node \"$VINS_CFG\" __ns:=/uav1; exec bash'" \
--tab -e 'bash -c "sleep 6; source /home/amov/vinsf_gpu_ws/devel/setup.bash; roslaunch vins vins_rviz.launch; exec bash"'
