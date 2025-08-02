#ifndef QCAR_VISNAV_EVENT_QUEUE_H
#define QCAR_VISNAV_EVENT_QUEUE_H

#include <queue>
#include <memory>
#include <ros/ros.h>
#include "qcar_visnav/math/system_params.h"

namespace qcar_visnav {

// Forward declarations
class EKFProcessor;

enum class EventType {
    IMU_EVENT = 0,
    ODOMETRY_EVENT = 1, 
    LIDAR_EVENT = 2,
    CONTROL_EVENT = 3
};

// Base Event class - matches MATLAB Event.m
class Event {
public:
    double time;                 // Event time
    EventType type;              // Event type
    bool save_system_state;      // Whether to save state
    int verbosity;               // Debug verbosity level
    
    Event(double t, EventType et) : time(t), type(et), save_system_state(true), verbosity(1) {}
    virtual ~Event() = default;
    
    // Pure virtual - equivalent to MATLAB update(obj, system)
    virtual bool process(EKFProcessor& processor, SystemParams& system) = 0;
    
    // Virtual method for debug string - equivalent to MATLAB getProcessString()
    virtual std::string getProcessString() const = 0;
    
    // Comparison for priority queue (earlier events have higher priority)
    bool operator>(const Event& other) const {
        return time > other.time;
    }
};

// Event comparator for priority queue
struct EventComparator {
    bool operator()(const std::shared_ptr<Event>& a, const std::shared_ptr<Event>& b) {
        return a->time > b->time; // Earlier events have higher priority
    }
};

// Priority event queue manager - equivalent to MATLAB event_queue
class EventQueue {
public:
    EventQueue();
    
    // Schedule events (equivalent to MATLAB event creation)
    void schedulePeriodicEvents(double t_sim);
    void addEvent(std::shared_ptr<Event> event);
    
    // Process next event
    bool processNextEvent(EKFProcessor& processor, SystemParams& system);
    
    // Queue status
    bool empty() const;
    size_t size() const;
    double nextEventTime() const;
    
    // Clear queue
    void clear();
    
private:
    std::priority_queue<std::shared_ptr<Event>, 
                       std::vector<std::shared_ptr<Event>>, 
                       EventComparator> queue_;
    
    // Event scheduling helpers
    void scheduleIMUEvents(double t_sim, double dt_imu);
    void scheduleOdometryEvents(double t_sim, double dt_odom);
    void scheduleLiDAREvents(double t_sim, double dt_lidar);
    void scheduleControlEvents(double t_sim, double dt_ctrl);
};

} // namespace qcar_visnav

#endif // QCAR_VISNAV_EVENT_QUEUE_H