
import math

class RadarProcessor:
    def __init__(self):
        self.last_dist = float('inf')
        self.alpha = 0.3  # Smoothing factor

    def process(self, radar_data):
        if not radar_data:
            return self.last_dist, 0.0

        min_dist = float('inf')
        target_vel = 0.0
        
        for detect in radar_data:
            dist = detect.depth
            if dist < min_dist:
                min_dist = dist
                target_vel = detect.velocity

        if min_dist == float('inf'):
            # IMPORTANT: Reset smoothing if road is clear!
            self.last_dist = float('inf')
            return float('inf'), 0.0

        # Exponential Smoothing
        if self.last_dist == float('inf'):
            self.last_dist = min_dist
        else:
            self.last_dist = self.alpha * min_dist + (1 - self.alpha) * self.last_dist
            
        return self.last_dist, target_vel
