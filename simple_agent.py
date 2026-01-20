
import carla
import math
import queue
import time
import config

class SimpleAgent:
    """Robust autonomous driving agent with debug output."""
    
    def __init__(self, world, ego):
        self.world = world
        self.ego = ego
        self.map = world.get_map()
        
        # Sensors
        self.radar_queue = queue.Queue()
        self.radar = self._setup_radar()
        
        # State
        self.state = "CRUISE"
        self.lane_change_until = 0
        self.cooldown_until = 0
        self.lane_change_dir = None  # 'left' or 'right'
        
    def _setup_radar(self):
        bp = self.world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(config.RADAR_FOV_AZIMUTH))
        bp.set_attribute('vertical_fov', str(config.RADAR_FOV_ELEVATION))
        bp.set_attribute('range', str(config.RADAR_RANGE))
        bp.set_attribute('points_per_second', str(config.RADAR_POINTS_PER_SECOND))
        
        tf = carla.Transform(carla.Location(x=2.5, z=1.0))
        sensor = self.world.spawn_actor(bp, tf, attach_to=self.ego)
        sensor.listen(self.radar_queue.put)
        return sensor

    def destroy(self):
        if self.radar: self.radar.destroy()

    def _get_obstacle_dist(self):
        """Get closest obstacle distance (with filters)."""
        min_dist = 999.0
        point_count = 0
        
        while not self.radar_queue.empty():
            data = self.radar_queue.get()
            for det in data:
                # CRITICAL: Ignore self-detection (anything < 3m is likely ego vehicle)
                if det.depth < 3.0:
                    continue
                    
                point_count += 1
                # Only consider forward-facing points within lane width
                lateral = abs(det.depth * math.sin(det.azimuth))
                if lateral < 2.5:  # Within ~lane width
                    if det.depth < min_dist:
                        min_dist = det.depth
        
        # Debug: Show radar detection (only when detecting real obstacles)
        if point_count > 0 and min_dist < 100:
            print(f"📡 Radar: {point_count} pts, closest: {min_dist:.1f}m")
        
        return min_dist

    def tick(self):
        """Main control loop."""
        now = time.time()
        loc = self.ego.get_location()
        wp = self.map.get_waypoint(loc)
        
        # If no waypoint, just drive forward
        if not wp:
            print("⚠️ No waypoint found - driving forward")
            return self._drive_forward()
        
        obstacle_dist = self._get_obstacle_dist()
        
        # STATE: CRUISE
        if self.state == "CRUISE":
            # Check for obstacles (only if not in cooldown)
            if now > self.cooldown_until and obstacle_dist < config.AVOID_DIST:
                left = wp.get_left_lane()
                right = wp.get_right_lane()
                
                can_left = left and left.lane_type == carla.LaneType.Driving
                can_right = right and right.lane_type == carla.LaneType.Driving
                
                if can_left:
                    self.state = "LANE_CHANGE"
                    self.lane_change_dir = 'left'
                    self.lane_change_until = now + 3.0
                    print(f"🚗 Lane Change LEFT! Obstacle at {obstacle_dist:.1f}m")
                elif can_right:
                    self.state = "LANE_CHANGE"
                    self.lane_change_dir = 'right'
                    self.lane_change_until = now + 3.0
                    print(f"🚗 Lane Change RIGHT! Obstacle at {obstacle_dist:.1f}m")
            
            # Follow lane normally
            return self._follow_lane(wp)
            
        # STATE: LANE_CHANGE
        elif self.state == "LANE_CHANGE":
            if now >= self.lane_change_until:
                print("✅ Lane change complete!")
                self.state = "CRUISE"
                self.cooldown_until = now + 5.0
                self.lane_change_dir = None
                return self._follow_lane(wp)
            
            # Steer towards target lane
            target_lane = None
            if self.lane_change_dir == 'left':
                target_lane = wp.get_left_lane()
            else:
                target_lane = wp.get_right_lane()
            
            if target_lane and target_lane.lane_type == carla.LaneType.Driving:
                return self._steer_to_lane(target_lane)
            else:
                # Lost target lane - abort and continue
                return self._follow_lane(wp)
        
        return self._follow_lane(wp)

    def _follow_lane(self, wp):
        """Follow current lane with fallback."""
        # Try to get next waypoint at different distances
        for dist in [10.0, 5.0, 3.0]:
            wps = wp.next(dist)
            if wps:
                return self._steer_towards(wps[0].transform.location)
        
        # No waypoints found - just drive forward
        return self._drive_forward()

    def _steer_to_lane(self, target_lane):
        """Steer towards target lane."""
        # Aim 10m ahead in target lane for smooth merge
        ahead = target_lane.next(10.0)
        if ahead:
            return self._steer_towards(ahead[0].transform.location)
        return self._steer_towards(target_lane.transform.location)

    def _steer_towards(self, target_loc):
        """Pure Pursuit steering with FIXED throttle."""
        loc = self.ego.get_location()
        yaw = math.radians(self.ego.get_transform().rotation.yaw)
        
        dx = target_loc.x - loc.x
        dy = target_loc.y - loc.y
        
        # Transform to vehicle frame
        x_local = math.cos(-yaw)*dx - math.sin(-yaw)*dy
        y_local = math.sin(-yaw)*dx + math.cos(-yaw)*dy
        
        # Pure Pursuit curvature
        L = 8.0
        curvature = 2.0 * y_local / (L**2)
        steer = max(-0.5, min(0.5, curvature))
        
        # ALWAYS apply throttle (unless at target speed)
        vel = self.ego.get_velocity()
        speed = 3.6 * math.sqrt(vel.x**2 + vel.y**2)
        
        # More aggressive throttle control
        if speed < config.TARGET_SPEED_KMH * 0.8:
            throttle = 0.7  # Strong acceleration if slow
        elif speed < config.TARGET_SPEED_KMH:
            throttle = 0.4  # Gentle acceleration if close to target
        else:
            throttle = 0.0  # Coast if at speed
        
        return carla.VehicleControl(throttle=throttle, steer=steer)

    def _drive_forward(self):
        """Fallback - drive straight with throttle."""
        vel = self.ego.get_velocity()
        speed = 3.6 * math.sqrt(vel.x**2 + vel.y**2)
        
        if speed < config.TARGET_SPEED_KMH * 0.8:
            throttle = 0.7
        elif speed < config.TARGET_SPEED_KMH:
            throttle = 0.4
        else:
            throttle = 0.0
            
        return carla.VehicleControl(throttle=throttle, steer=0.0)
