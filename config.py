
# V3 LITE CONFIGURATION

# Simulation
HOST = "localhost"
PORT = 2000
TIMEOUT = 10.0
SYNC_MODE = True
FIXED_DELTA_SECONDS = 0.033  # 30 Hz (Stable)

# Radar (Optimized for 30Hz)
RADAR_RANGE = 100.0
RADAR_FOV_AZIMUTH = 30.0   # Wider FOV for better detection
RADAR_FOV_ELEVATION = 10.0  # Wider vertical
RADAR_POINTS_PER_SECOND = 10000 # More points for reliability

# Control
TARGET_SPEED_KMH = 7.0 # Reduced speed to prevent getting stuck/missing logic
LOOKAHEAD_BASE = 8.0
EMERGENCY_DIST = 10.0
AVOID_DIST = 45.0 # Earlier reaction (User req)

# Actors
EGO_FILTER = 'vehicle.tesla.model3'
OBSTACLE_FILTER = 'vehicle.nissan.patrol'
