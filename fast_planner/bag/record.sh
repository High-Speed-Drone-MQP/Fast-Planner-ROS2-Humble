# camera, map, range, path, traj, cmd
# /vins_estimator/camera_pose

# TODO: change rosbag to ros2?
rosbag record /pcl_render_node/camera_pose /sdf_map/occupancy_inflate /sdf_map/update_range /planning_vis/topo_path /planning_vis/trajectory /planning/state 