# robot_localization_igvc

Fuses wheel odometry + IMU + GPS into a stable pose estimate

Produces:
Local odometry (odom frame) → smooth, drift-free short term
Global pose (map frame) → GPS-anchored, long term
Provides the map ↔ odom ↔ base_link ↔ utm TF tree required by Nav2

Nodes used:

ekf_node (local)
Frame: odom
Inputs: wheel odometry + IMU
Output: /odometry/filtered_local

ekf_node (global)
Frame: map
Inputs: wheel odometry + GPS odometry + IMU
Output: /odometry/filtered_global
navsat_transform_node
Converts GPS (lat/lon) → UTM → map frame
Publishes utm → map TF

Frames
map → global, GPS-anchored frame
odom → locally smooth frame
base_link → robot body frame
imu_link, gps_link → sensor frames
utm → intermediate Cartesian GPS frame
