# Estimator State Message - matches MATLAB state vector exactly
Header header

# State vector: [u; v; r; wF; wR; delta; N; E; psi; gyro]
float64 u          # Forward velocity (m/s)
float64 v          # Lateral velocity (m/s)
float64 r          # Yaw rate (rad/s)
float64 wF         # Front wheel speed (rad/s)
float64 wR         # Rear wheel speed (rad/s)
float64 delta      # Steering angle (rad)
float64 N          # North position (m)
float64 E          # East position (m)  
float64 psi        # Heading angle (rad)
float64 gyro       # Gyro bias (rad/s)

# State covariance (10x10 flattened)
float64[100] covariance

# Estimation quality metrics
bool state_valid
float64 innovation_norm
int32 measurement_count