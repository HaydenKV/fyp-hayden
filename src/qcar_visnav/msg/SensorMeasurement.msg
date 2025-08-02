# Unified Sensor Measurement Message
Header header

# Measurement type
string measurement_type    # "imu", "odometry", "lidar"

# IMU data (if applicable)
sensor_msgs/Imu imu_data

# Odometry data (if applicable)  
nav_msgs/Odometry odom_data

# LiDAR data (if applicable)
sensor_msgs/LaserScan lidar_data

# Processing metadata
bool is_valid
float64 processing_time