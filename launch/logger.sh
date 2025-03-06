source /opt/ros/humble/setup.bash
source /home/ubuntu/ouxt_automation/robotx_ws/install/local_setup.bash
ros2 bag record -o /home/ubuntu/auto_logger/rosbag/$(date '+%Y-%m-%d-%H-%M-%S') -d 10 -s mcap --storage-config-file /home/ubuntu/ouxt_automation/.github/workflows/docker/auto_logger/settings/storage_options.yaml /wamv/sensors/cameras/front_camera_sensor/image_raw/compressed /wamv/sensors/lidars/front_lidar_sensor/velodyne_packets
