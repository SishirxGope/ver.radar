import math
import carla
import config

class RoadFollower:
    def __init__(self, world, ego):
        self.world = world
        self.ego = ego
        self.last_steer = 0.0

    def apply(self):
        loc = self.ego.get_location()
        wp = self.world.get_map().get_waypoint(loc)
        
        # Dynamic lookahead
        vel = self.ego.get_velocity()
        speed = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2) # km/h
        
        lookahead = 8.0 + 0.3 * (speed / 3.6)
        lookahead = min(15.0, max(5.0, lookahead))

        next_wps = wp.next(lookahead)
        if not next_wps: return # fast fail
        target = next_wps[0].transform.location

        yaw = math.radians(self.ego.get_transform().rotation.yaw)

        dx = target.x - loc.x
        dy = target.y - loc.y

        x =  math.cos(-yaw)*dx - math.sin(-yaw)*dy
        y =  math.sin(-yaw)*dx + math.cos(-yaw)*dy

        curvature = 2 * y / (lookahead * lookahead)
        steer = max(-0.7, min(0.7, curvature))

        # Slew Rate Limiter (Smoothness)
        delta = steer - self.last_steer
        delta = max(-0.05, min(0.05, delta))
        steer = self.last_steer + delta
        self.last_steer = steer

        # Speed Controller (Simple P)
        if speed < config.TARGET_SPEED_KMH:
            throttle = 0.6 # Increased to ensure start on hills
        else:
            throttle = 0.0

        control = carla.VehicleControl()
        control.throttle = throttle
        control.steer = steer
        control.brake = 0.0

        self.ego.apply_control(control)
