#ifndef QCAR_VISNAV_MEASUREMENT_EVENTS_H
#define QCAR_VISNAV_MEASUREMENT_EVENTS_H

#include "qcar_visnav/estimator/event_queue.h"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

namespace qcar_visnav {

// IMU Event - equivalent to MATLAB MeasurementAccelerometer
class IMUEvent : public Event {
public:
    sensor_msgs::Imu::ConstPtr imu_data;
    bool need_to_simulate;       // Equivalent to MATLAB needToSimulate
    std::string update_method;   // Equivalent to MATLAB updateMethod
    
    IMUEvent(double t) : Event(t, EventType::IMU_EVENT), need_to_simulate(true), 
                        update_method("NewtonTrustEig") {}
    
    bool process(EKFProcessor& processor, SystemParams& system) override;
    std::string getProcessString() const override { return "IMU measurement update:"; }
    
    // Set sensor data
    void setSensorData(const sensor_msgs::Imu::ConstPtr& data) { imu_data = data; }
};

// Odometry Event - equivalent to MATLAB MeasurementEncoder  
class OdometryEvent : public Event {
public:
    nav_msgs::Odometry::ConstPtr odom_data;
    bool need_to_simulate;
    std::string update_method;
    
    OdometryEvent(double t) : Event(t, EventType::ODOMETRY_EVENT), need_to_simulate(true),
                             update_method("NewtonTrustEig") {}
    
    bool process(EKFProcessor& processor, SystemParams& system) override;
    std::string getProcessString() const override { return "Odometry measurement update:"; }
    
    void setSensorData(const nav_msgs::Odometry::ConstPtr& data) { odom_data = data; }
};

// LiDAR Event - equivalent to MATLAB MeasurementLiDAR
class LiDAREvent : public Event {
public:
    sensor_msgs::LaserScan::ConstPtr lidar_data;
    bool need_to_simulate;
    std::string update_method;
    
    LiDAREvent(double t) : Event(t, EventType::LIDAR_EVENT), need_to_simulate(true),
                          update_method("BFGSTrustSqrtInv") {}
    
    bool process(EKFProcessor& processor, SystemParams& system) override;
    std::string getProcessString() const override { return "LiDAR measurement update:"; }
    
    void setSensorData(const sensor_msgs::LaserScan::ConstPtr& data) { lidar_data = data; }
};

// Control Event - equivalent to MATLAB ControlEvent
class ControlEvent : public Event {
public:
    bool use_gradient;
    bool calc_predicted_state;
    std::string problem_type;
    
    ControlEvent(double t) : Event(t, EventType::CONTROL_EVENT), use_gradient(false),
                            calc_predicted_state(true), problem_type("SquaredErrorCostWithBoundConstraints") {}
    
    bool process(EKFProcessor& processor, SystemParams& system) override;
    std::string getProcessString() const override { return "Control event:"; }
};

} // namespace qcar_visnav

#endif // QCAR_VISNAV_MEASUREMENT_EVENTS_H