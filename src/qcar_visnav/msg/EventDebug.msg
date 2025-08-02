# Event Debug Message - for timing analysis
Header header

# Event queue status
float64 current_time
int32 queue_size
float64 next_event_time

# Event processing statistics  
int32 events_processed
float64 average_processing_time
float64 max_processing_time

# Current event info
string current_event_type
bool event_success
float64 event_duration