#pragma once
#include <ros/ros.h>
#include <deque>
#include <string>
#include <vector>
#include <limits>

#include <nav_msgs/Odometry.h>
#include <qcar_visnav/ConeArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Core>

#include "particle.h"
#include "motion_model.h"
#include "data_association.h"

namespace qcar_visnav { namespace slam {

/**
 * @brief FastSLAM 2.0 Implementation for QCar
 * 
 * ALGORITHM OVERVIEW:
 * 1. Each particle maintains its own belief about:
 *    - Robot trajectory (x, y, yaw over time)
 *    - Map of landmarks (cone locations + uncertainties)
 * 
 * 2. Motion Update: Propagate particles using odometry
 *    - Input: Robot velocity commands (vx, vy, r)
 *    - Output: Updated particle poses with motion noise
 * 
 * 3. Measurement Update: Update particle weights based on cone observations
 *    - Input: Confirmed tracked cones from perception pipeline
 *    - Process: Data association + landmark updates + weight computation
 *    - Output: Particle weights reflecting observation likelihood
 * 
 * 4. Resampling: Maintain particle diversity
 *    - Remove low-weight particles, duplicate high-weight ones
 */
class FastSLAM2 {
public:
  FastSLAM2(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void spinOnce();

private:
  // =============================================================================
  // ROS INTERFACE
  // =============================================================================
  
  // SUBSCRIBERS
  ros::Subscriber sub_odom_;             // Robot odometry (configurable: /odom or /qcar/ekf/odom)
  ros::Subscriber sub_cones_;            // Tracked cone observations (/tracked_cones)
  
  // PUBLISHERS  
  ros::Publisher  pub_particles_;        // Particle arrows for RViz (/slam/particles)
  ros::Publisher  pub_particles_posearray_; // Particle poses (/slam/particles_pose)
  ros::Publisher  pub_map_markers_;      // Landmark spheres for RViz (/slam/map_markers)
  ros::Publisher  pub_slam_odom_;        // Best particle pose (/slam/odom)
  ros::Publisher  pub_slam_odom_mean_;   // Mean particle pose (/slam/odom_mean)
  
  // TRANSFORM HANDLING
  tf2_ros::Buffer tfbuf_;                // TF buffer for coordinate transforms
  tf2_ros::TransformListener tfl_;       // TF listener

  // =============================================================================
  // CONFIGURATION PARAMETERS  
  // =============================================================================
  
  // COORDINATE FRAMES
  std::string map_frame_, odom_frame_, base_frame_, lidar_frame_;
  
  // PARTICLE FILTER PARAMETERS
  int    N_{120};                        // Number of particles (more = better accuracy, slower)
  double neff_ratio_{0.4};               // Resample when Neff < ratio * N (diversity maintenance)
  
  // DATA ASSOCIATION PARAMETERS
  double chi2_gate_{16.27};              // Mahalanobis distance gate (larger = more associations)
  
  // LANDMARK MANAGEMENT PARAMETERS
  int    confirm_hits_{2};               // Observations needed to confirm landmark (reliability filter)
  double min_new_lm_dist_{0.35};         // Min distance to create new landmark (avoid duplicates) [m]
  double merge_R_scale_{4.0};            // Uncertainty inflation when merging nearby landmarks
  
  // INITIALIZATION BEHAVIOR
  bool   seed_from_params_{true};        // Initialize from config file vs. first odometry message
  bool   overwrite_with_odom_on_first_msg_{true}; // Snap to first /odom pose regardless of config
  double init_x_{0.0}, init_y_{0.0}, init_yaw_{0.0}; // Initial pose from config
  double spread_x_{0.02}, spread_y_{0.02}, spread_yaw_{0.01}; // Initial particle spread
  
  // ODOMETRY INTERPRETATION  
  bool odom_twist_in_world_{false};      // false = body frame, true = world frame velocities
  std::string odom_topic_;               // Configurable: "/odom" or "/qcar/ekf/odom"

  // =============================================================================
  // SLAM STATE
  // =============================================================================
  
  // PARTICLE COLLECTION
  std::vector<Particle> P_;              // The particle set (each has pose + landmark map)
  
  // ALGORITHM COMPONENTS
  MotionModel     motion_;               // Handles particle propagation with noise
  DataAssociation assoc_;                // Handles observation-to-landmark matching
  
  // INITIALIZATION FLAGS
  bool particles_initialized_{false};    // Have we initialized particle poses?
  bool snapped_to_first_odom_{false};   // Have we used first /odom message?
  
  // MOTION INTEGRATION
  struct OdomStamped { 
    ros::Time t; 
    double vx, vy, r;                   // Body-frame velocities and yaw rate
  };
  std::deque<OdomStamped> odom_buf_;     // Ring buffer of recent odometry (for smooth integration)
  ros::Time last_prop_stamp_{ros::Time(0)}; // Last time we propagated particles
  
  // OUTPUT CACHING (for publishing)
  int    best_idx_{0};                   // Index of highest-weight particle
  double mean_x_{0.0}, mean_y_{0.0}, mean_yaw_{0.0}; // Weighted mean particle pose

  // =============================================================================
  // CORE ALGORITHM CALLBACKS
  // =============================================================================
  
  /**
   * @brief Process odometry messages
   * FUNCTIONALITY:
   * - Store velocity commands in ring buffer for motion integration
   * - Initialize particles from first pose (if configured)
   * 
   * @param msg Odometry message (from /odom or /qcar/ekf/odom)
   */
  void cbOdom(const nav_msgs::Odometry::ConstPtr& msg);
  
  /**
   * @brief Process tracked cone observations (MAIN SLAM UPDATE)
   * FUNCTIONALITY:
   * 1. Propagate particles to observation time using stored odometry
   * 2. Transform cone observations from LiDAR frame to ODOM frame  
   * 3. For each particle: associate observations with landmarks, update map
   * 4. Update particle weights based on observation likelihood
   * 5. Resample particles if diversity gets too low
   * 6. Publish visualization and pose estimates
   * 
   * @param msg ConeArray with confirmed tracked cones
   */
  void cbCones(const qcar_visnav::ConeArray::ConstPtr& msg);

  // =============================================================================
  // PARTICLE MANAGEMENT
  // =============================================================================
  
  /**
   * @brief Initialize particle poses around given location
   * @param x, y, yaw Center pose
   * @param sx, sy, syaw Gaussian spread parameters
   */
  void initializeParticlesFrom(double x, double y, double yaw,
                               double sx, double sy, double syaw);
  
  /**
   * @brief Propagate all particles forward in time using odometry
   * PROCESS:
   * 1. Find odometry data between last_prop_stamp_ and target time t
   * 2. For each time segment: interpolate velocities, apply motion model
   * 3. Motion model adds Gaussian noise to each particle (uncertainty growth)
   * 
   * @param t Target time to propagate to
   */
  void propagateParticlesTo(const ros::Time& t);
  
  /**
   * @brief Interpolate odometry between two samples
   * @param a Earlier odometry sample
   * @param b Later odometry sample  
   * @param s Target interpolation time
   * @return Interpolated odometry at time s
   */
  OdomStamped interpTwist(const OdomStamped& a,
                          const OdomStamped& b,
                          const ros::Time& s) const;

  // =============================================================================
  // COORDINATE TRANSFORMS
  // =============================================================================
  
  /**
   * @brief Convert world-frame velocities to body-frame for given yaw
   * USAGE: When odometry provides world-frame twist, convert per-particle
   * 
   * @param yaw Robot orientation [rad]
   * @param vx_w, vy_w World-frame velocities [m/s]
   * @param vx_b, vy_b Output: body-frame velocities [m/s]
   */
  static inline void worldToBody(double yaw,
                                 double vx_w, double vy_w,
                                 double& vx_b, double& vy_b) {
    const double c = std::cos(yaw), s = std::sin(yaw);
    vx_b =  c*vx_w + s*vy_w;    // Forward velocity in body frame
    vy_b = -s*vx_w + c*vy_w;    // Lateral velocity in body frame
  }

  // =============================================================================
  // LANDMARK UTILITIES
  // =============================================================================
  
  /**
   * @brief Find nearest landmark in particle's map (for duplicate detection)
   * @param p Particle with landmark map
   * @param z Observation position [x,y] in ODOM
   * @param out_dist_sq Output: squared Euclidean distance to nearest landmark
   * @return Index of nearest landmark, or -1 if map is empty
   */
  static int nearestLandmarkIdx(const Particle& p,
                                const Eigen::Vector2d& z,
                                double& out_dist_sq);
  
  /**
   * @brief Core measurement update (THE HEART OF FASTSLAM)
   * PROCESS FOR EACH PARTICLE:
   * 1. For each cone observation:
   *    a) Try to associate with existing landmarks (Mahalanobis gating)
   *    b) If associated: EKF update of landmark position + weight update
   *    c) If not associated: check for merge vs. create new landmark
   * 2. Update particle weight based on observation likelihood
   * 3. After all particles: normalize weights and resample if needed
   * 
   * @param t Observation timestamp
   * @param zs Cone positions in ODOM frame [x,y]
   * @param Rs Cone observation uncertainties (2x2 covariance matrices)
   */
  void processMeasurementAt(const ros::Time& t,
                            const std::vector<Eigen::Vector2d>& zs,
                            const std::vector<Eigen::Matrix2d>& Rs);

  // =============================================================================
  // VISUALIZATION & OUTPUT
  // =============================================================================
  
  /**
   * @brief Publish RViz markers and pose estimates
   * OUTPUTS:
   * - Particle arrows (all particles)
   * - Particle pose array (all particles) 
   * - Landmark spheres (best particle's confirmed landmarks)
   * - SLAM odometry (best particle pose)
   * - SLAM odometry mean (weighted average pose)
   * 
   * @param t Current timestamp
   */
  void publishViz(const ros::Time& t);
};

}} // namespace