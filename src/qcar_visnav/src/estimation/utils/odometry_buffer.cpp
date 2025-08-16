#include "qcar_visnav/estimation/utils/odometry_buffer.h"
#include <algorithm>
#include <ros/ros.h>

namespace qcar_nav {

OdometryBuffer::OdometryBuffer(size_t max_size) : max_size_(max_size) {
}

void OdometryBuffer::addEntry(const ros::Time& timestamp, double vx, double vy, double r,
                              const Eigen::Matrix3d& cov) {
  OdometryEntry entry(timestamp, vx, vy, r);
  entry.covariance = cov;
  
  // Insert in chronological order
  auto it = std::upper_bound(buffer_.begin(), buffer_.end(), entry,
                            [](const OdometryEntry& a, const OdometryEntry& b) {
                              return a.timestamp < b.timestamp;
                            });
  buffer_.insert(it, entry);
  
  // Remove oldest entries if buffer exceeds max size
  while (buffer_.size() > max_size_) {
    buffer_.pop_front();
  }
}

void OdometryBuffer::addEntry(const nav_msgs::Odometry& odom_msg) {
  // Extract velocities (assuming twist is in body frame)
  double vx = odom_msg.twist.twist.linear.x;
  double vy = odom_msg.twist.twist.linear.y;
  double r = odom_msg.twist.twist.angular.z;
  
  // Extract covariance for velocities [vx, vy, r]
  Eigen::Matrix3d cov;
  cov(0, 0) = odom_msg.twist.covariance[0];   // vx variance
  cov(0, 1) = odom_msg.twist.covariance[1];   // vx-vy covariance
  cov(0, 2) = odom_msg.twist.covariance[5];   // vx-r covariance
  cov(1, 0) = odom_msg.twist.covariance[6];   // vy-vx covariance
  cov(1, 1) = odom_msg.twist.covariance[7];   // vy variance
  cov(1, 2) = odom_msg.twist.covariance[11];  // vy-r covariance
  cov(2, 0) = odom_msg.twist.covariance[30];  // r-vx covariance
  cov(2, 1) = odom_msg.twist.covariance[31];  // r-vy covariance
  cov(2, 2) = odom_msg.twist.covariance[35];  // r variance
  
  addEntry(odom_msg.header.stamp, vx, vy, r, cov);
}

bool OdometryBuffer::getInterpolated(const ros::Time& timestamp, OdometryEntry& entry) const {
  if (buffer_.empty()) {
    return false;
  }
  
  // Check if timestamp is within buffer range
  if (timestamp < buffer_.front().timestamp || timestamp > buffer_.back().timestamp) {
    return false;
  }
  
  // Find bracketing entries
  auto it_upper = std::upper_bound(buffer_.begin(), buffer_.end(), timestamp,
                                  [](const ros::Time& t, const OdometryEntry& e) {
                                    return t < e.timestamp;
                                  });
  
  if (it_upper == buffer_.begin()) {
    // Exact match or before first entry
    entry = buffer_.front();
    return true;
  }
  
  auto it_lower = it_upper - 1;
  
  if (it_upper == buffer_.end()) {
    // After last entry
    entry = buffer_.back();
    return true;
  }
  
  // Interpolate between lower and upper
  entry = interpolate(*it_lower, *it_upper, timestamp);
  return true;
}

bool OdometryBuffer::getClosest(const ros::Time& timestamp, OdometryEntry& entry) const {
  if (buffer_.empty()) {
    return false;
  }
  
  // Find closest entry by timestamp
  auto closest_it = std::min_element(buffer_.begin(), buffer_.end(),
                                    [&timestamp](const OdometryEntry& a, const OdometryEntry& b) {
                                      return std::abs((a.timestamp - timestamp).toSec()) <
                                             std::abs((b.timestamp - timestamp).toSec());
                                    });
  
  entry = *closest_it;
  return true;
}

std::vector<OdometryEntry> OdometryBuffer::getRange(const ros::Time& start_time, 
                                                    const ros::Time& end_time) const {
  std::vector<OdometryEntry> result;
  
  for (const auto& entry : buffer_) {
    if (entry.timestamp >= start_time && entry.timestamp <= end_time) {
      result.push_back(entry);
    }
  }
  
  return result;
}

bool OdometryBuffer::integrateMotion(const ros::Time& start_time, const ros::Time& end_time,
                                     double& delta_x, double& delta_y, double& delta_yaw) const {
  delta_x = 0.0;
  delta_y = 0.0;
  delta_yaw = 0.0;
  
  if (buffer_.empty() || end_time <= start_time) {
    return false;
  }
  
  // Get all entries in time range
  auto entries = getRange(start_time, end_time);
  if (entries.size() < 2) {
    return false;
  }
  
  // Integrate using trapezoidal rule
  for (size_t i = 1; i < entries.size(); ++i) {
    const auto& prev = entries[i-1];
    const auto& curr = entries[i];
    
    double dt = (curr.timestamp - prev.timestamp).toSec();
    if (dt <= 0.0) continue;
    
    // Average velocities for trapezoidal integration
    double vx_avg = 0.5 * (prev.vx + curr.vx);
    double vy_avg = 0.5 * (prev.vy + curr.vy);
    double r_avg = 0.5 * (prev.r + curr.r);
    
    // Integrate motion in body frame
    delta_x += vx_avg * dt;
    delta_y += vy_avg * dt;
    delta_yaw += r_avg * dt;
  }
  
  return true;
}

ros::Time OdometryBuffer::getOldestTime() const {
  if (buffer_.empty()) {
    return ros::Time(0);
  }
  return buffer_.front().timestamp;
}

ros::Time OdometryBuffer::getNewestTime() const {
  if (buffer_.empty()) {
    return ros::Time(0);
  }
  return buffer_.back().timestamp;
}

double OdometryBuffer::getTimeSpan() const {
  if (buffer_.size() < 2) {
    return 0.0;
  }
  return (getNewestTime() - getOldestTime()).toSec();
}

std::deque<OdometryEntry>::const_iterator OdometryBuffer::findLowerBound(const ros::Time& timestamp) const {
  return std::lower_bound(buffer_.begin(), buffer_.end(), timestamp,
                         [](const OdometryEntry& e, const ros::Time& t) {
                           return e.timestamp < t;
                         });
}

OdometryEntry OdometryBuffer::interpolate(const OdometryEntry& entry1, const OdometryEntry& entry2,
                                         const ros::Time& timestamp) const {
  double dt_total = (entry2.timestamp - entry1.timestamp).toSec();
  if (dt_total <= 0.0) {
    return entry1;  // Avoid division by zero
  }
  
  double dt_partial = (timestamp - entry1.timestamp).toSec();
  double alpha = dt_partial / dt_total;  // interpolation factor [0, 1]
  
  // Clamp alpha to [0, 1]
  alpha = std::max(0.0, std::min(1.0, alpha));
  
  OdometryEntry result;
  result.timestamp = timestamp;
  
  // Linear interpolation of velocities
  result.vx = (1.0 - alpha) * entry1.vx + alpha * entry2.vx;
  result.vy = (1.0 - alpha) * entry1.vy + alpha * entry2.vy;
  result.r = (1.0 - alpha) * entry1.r + alpha * entry2.r;
  
  // Simple covariance interpolation (could be improved)
  result.covariance = (1.0 - alpha) * entry1.covariance + alpha * entry2.covariance;
  
  return result;
}

} // namespace qcar_nav