#include "qcar_visnav/estimator/event_queue.h"
#include "qcar_visnav/estimator/measurement_events.h"
#include "qcar_visnav/estimator/ekf_processor.h"

namespace qcar_visnav {

EventQueue::EventQueue() {
    // Initialize empty queue
}

void EventQueue::schedulePeriodicEvents(double t_sim) {
    // Clear existing events
    clear();
    
    // Schedule events with precise timing - from MATLAB runSim.m
    double dt_ctrl = 1.0/50.0;   // 50 Hz control
    double dt_imu = 1.0/50.0;    // 50 Hz IMU
    double dt_lidar = 1.0/10.0;  // 10 Hz LiDAR
    
    ROS_INFO("Scheduling events for %.2f seconds simulation", t_sim);
    
    // Schedule IMU events - equivalent to: for t = t_imu_events
    scheduleIMUEvents(t_sim, dt_imu);
    
    // Schedule odometry events  
    scheduleOdometryEvents(t_sim, dt_imu); // Same rate as IMU
    
    // Schedule LiDAR events
    scheduleLiDAREvents(t_sim, dt_lidar);
    
    // Schedule control events
    scheduleControlEvents(t_sim, dt_ctrl);
    
    ROS_INFO("Scheduled %zu total events", queue_.size());
}

void EventQueue::scheduleIMUEvents(double t_sim, double dt_imu) {
    for (double t = 0.0; t <= t_sim; t += dt_imu) {
        auto event = std::make_shared<IMUEvent>(t);
        event->need_to_simulate = true;
        event->update_method = "NewtonTrustEig"; // From MATLAB
        event->verbosity = 1;
        addEvent(event);
    }
    ROS_DEBUG("Scheduled IMU events: 0 to %.2f with dt=%.4f", t_sim, dt_imu);
}

void EventQueue::scheduleOdometryEvents(double t_sim, double dt_odom) {
    for (double t = 0.0; t <= t_sim; t += dt_odom) {
        auto event = std::make_shared<OdometryEvent>(t);
        event->need_to_simulate = true;
        event->update_method = "NewtonTrustEig";
        event->verbosity = 1;
        addEvent(event);
    }
    ROS_DEBUG("Scheduled Odometry events: 0 to %.2f with dt=%.4f", t_sim, dt_odom);
}

void EventQueue::scheduleLiDAREvents(double t_sim, double dt_lidar) {
    for (double t = 0.0; t <= t_sim; t += dt_lidar) {
        auto event = std::make_shared<LiDAREvent>(t);
        event->need_to_simulate = true;
        event->update_method = "BFGSTrustSqrtInv"; // From MATLAB
        event->verbosity = 1;
        addEvent(event);
    }
    ROS_DEBUG("Scheduled LiDAR events: 0 to %.2f with dt=%.4f", t_sim, dt_lidar);
}

void EventQueue::scheduleControlEvents(double t_sim, double dt_ctrl) {
    for (double t = 0.0; t <= t_sim; t += dt_ctrl) {
        auto event = std::make_shared<ControlEvent>(t);
        event->verbosity = 1;
        addEvent(event);
    }
    ROS_DEBUG("Scheduled Control events: 0 to %.2f with dt=%.4f", t_sim, dt_ctrl);
}

void EventQueue::addEvent(std::shared_ptr<Event> event) {
    queue_.push(event);
}

bool EventQueue::processNextEvent(EKFProcessor& processor, SystemParams& system) {
    if (queue_.empty()) {
        return false;
    }
    
    auto event = queue_.top();
    queue_.pop();
    
    // Update system time
    system.time = event->time;
    
    // Process event - equivalent to MATLAB [obj, system] = process(obj, system)
    if (event->verbosity > 0) {
        ROS_DEBUG("[t=%07.3fs] %s", event->time, event->getProcessString().c_str());
    }
    
    // Time update first - equivalent to MATLAB system.predict(obj.time)
    processor.predict(event->time);
    
    // Event-specific processing - equivalent to MATLAB obj.update(system)
    bool success = event->process(processor, system);
    
    if (event->verbosity > 0 && success) {
        ROS_DEBUG(" done");
    } else if (!success) {
        ROS_WARN(" failed");
    }
    
    return success;
}

bool EventQueue::empty() const {
    return queue_.empty();
}

size_t EventQueue::size() const {
    return queue_.size();
}

double EventQueue::nextEventTime() const {
    if (queue_.empty()) return -1.0;
    return queue_.top()->time;
}

void EventQueue::clear() {
    while (!queue_.empty()) {
        queue_.pop();
    }
}

} // namespace qcar_visnav