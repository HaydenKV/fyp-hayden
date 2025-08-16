#ifndef QCAR_VISNAV_ESTIMATION_UTILS_ODOMETRY_BUFFER_H
#define QCAR_VISNAV_ESTIMATION_UTILS_ODOMETRY_BUFFER_H

#include <deque>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>

namespace qcar_nav {

/**
 * @brief Timestamped odometry entry for the buffer
 */
struct OdometryEntry {
  ros::Time timestamp;
  double vx, vy, r;           // body-frame velocities and yaw rate
  Eigen::Matrix3d covariance; // 3x3 covariance for [vx, vy, r]
  
  OdometryEntry() : vx(0), vy(0), r(0) {
    covariance.setIdentity();
  }
  
  OdometryEntry(const ros::Time& t, double vx_, double vy_, double r_) 
    : timestamp(t), vx(vx_), vy(vy_), r(r_) {
    covariance.setIdentity();
  }
};

/**
 * @brief Ring buffer for storing timestamped odometry data
 * Used for LiDAR deskewing and temporal interpolation
 */
class OdometryBuffer {
public:
  /**
   * @brief Constructor
   * @param max_size Maximum number of entries to store (default: 200 for ~2s at 100Hz)
   */
  explicit OdometryBuffer(size_t max_size = 200);
  
  /**
   * @brief Add new odometry entry to buffer
   * @param timestamp Time of the measurement
   * @param vx Body-frame velocity x (m/s)
   * @param vy Body-frame velocity y (m/s)
   * @param r Yaw rate (rad/s)
   * @param cov 3x3 covariance matrix for [vx, vy, r]
   */
  void addEntry(const ros::Time& timestamp, double vx, double vy, double r,
                const Eigen::Matrix3d& cov = Eigen::Matrix3d::Identity());
  
  /**
   * @brief Add odometry from ROS message
   * @param odom_msg ROS odometry message
   */
  void addEntry(const nav_msgs::Odometry& odom_msg);
  
  /**
   * @brief Get interpolated odometry at specific timestamp
   * @param timestamp Desired timestamp
   * @param entry Output interpolated entry
   * @return true if interpolation successful, false if timestamp out of range
   */
  bool getInterpolated(const ros::Time& timestamp, OdometryEntry& entry) const;
  
  /**
   * @brief Get odometry entry closest to timestamp (no interpolation)
   * @param timestamp Desired timestamp
   * @param entry Output closest entry
   * @return true if entry found, false if buffer empty
   */
  bool getClosest(const ros::Time& timestamp, OdometryEntry& entry) const;
  
  /**
   * @brief Get all entries between two timestamps
   * @param start_time Start timestamp (inclusive)
   * @param end_time End timestamp (inclusive)
   * @return Vector of entries in time range
   */
  std::vector<OdometryEntry> getRange(const ros::Time& start_time, 
                                      const ros::Time& end_time) const;
  
  /**
   * @brief Integrate motion between two timestamps
   * @param start_time Start timestamp
   * @param end_time End timestamp
   * @param delta_x Output integrated displacement x (m)
   * @param delta_y Output integrated displacement y (m)
   * @param delta_yaw Output integrated yaw change (rad)
   * @return true if integration successful
   */
  bool integrateMotion(const ros::Time& start_time, const ros::Time& end_time,
                       double& delta_x, double& delta_y, double& delta_yaw) const;
  
  /**
   * @brief Get size of buffer
   */
  size_t size() const { return buffer_.size(); }
  
  /**
   * @brief Check if buffer is empty
   */
  bool empty() const { return buffer_.empty(); }
  
  /**
   * @brief Clear all entries
   */
  void clear() { buffer_.clear(); }
  
  /**
   * @brief Get oldest timestamp in buffer
   */
  ros::Time getOldestTime() const;
  
  /**
   * @brief Get newest timestamp in buffer
   */
  ros::Time getNewestTime() const;
  
  /**
   * @brief Get buffer time span in seconds
   */
  double getTimeSpan() const;

private:
  std::deque<OdometryEntry> buffer_;
  size_t max_size_;
  
  /**
   * @brief Find iterator to entry with timestamp <= target
   * @param timestamp Target timestamp
   * @return Iterator to entry (may be end() if none found)
   */
  std::deque<OdometryEntry>::const_iterator findLowerBound(const ros::Time& timestamp) const;
  
  /**
   * @brief Linear interpolation between two entries
   * @param entry1 Earlier entry
   * @param entry2 Later entry
   * @param timestamp Target timestamp (must be between entry1 and entry2)
   * @return Interpolated entry
   */
  OdometryEntry interpolate(const OdometryEntry& entry1, const OdometryEntry& entry2,
                           const ros::Time& timestamp) const;
};

} // namespace qcar_nav

#endif // QCAR_VISNAV_ESTIMATION_UTILS_ODOMETRY_BUFFER_H