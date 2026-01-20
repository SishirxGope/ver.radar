import carla
import math

class PathFollower:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.path = []
        self.index = 0
        self.lookahead = 8.0   # meters
        self.last_steer = 0.0

    def set_path(self, path):
        self.path = path
        self.index = 0
        self.last_steer = 0.0

    def has_path(self):
        return self.path and self.index < len(self.path)

    def tick(self):
        if not self.has_path():
            return False

        loc = self.vehicle.get_location()
        yaw = math.radians(self.vehicle.get_transform().rotation.yaw)
        
        # Dynamic lookahead
        vel = self.vehicle.get_velocity()
        speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        # Lookahead = 8.0 + 0.3 * speed
        self.lookahead = 8.0 + 0.3 * speed
        self.lookahead = min(15.0, max(5.0, self.lookahead))

        # Find lookahead point
        target = None
        for i in range(self.index, len(self.path)):
            p = self.path[i]
            # Handle both carla.Location objects and tuples
            # User snippet uses p directly assuming it has .x .y .z
            # I will ensure p is treated as such.
            wp = carla.Location(x=p.x, y=p.y, z=p.z)
            if loc.distance(wp) > self.lookahead:
                target = wp
                self.index = i
                break

        if target is None:
            # If no point is far enough, target the LAST point
            last_point = self.path[-1]
            last_wp = carla.Location(x=last_point.x, y=last_point.y, z=last_point.z)
            
            if loc.distance(last_wp) < 3.0:
                # We reached the end
                self.path = []
                # print("✅ Path Completed")
                return False
            else:
                # Continue driving to the end
                target = last_wp
                self.index = len(self.path) - 1

        # Transform target to vehicle coordinates
        dx = target.x - loc.x
        dy = target.y - loc.y

        # Rotate into vehicle frame
        x =  math.cos(-yaw)*dx - math.sin(-yaw)*dy
        y =  math.sin(-yaw)*dx + math.cos(-yaw)*dy

        if x <= 0.001:
            return True

        # Pure pursuit curvature
        L = self.lookahead
        curvature = 2 * y / (L * L)

        steer = max(-0.7, min(0.7, curvature))

        # Slew Rate Limiter (Max delta 0.05 per frame)
        # 0.05 @ 60Hz = 3.0 units/sec = Full lock in ~0.25s (Smooth)
        delta = steer - self.last_steer
        delta = max(-0.05, min(0.05, delta))
        steer = self.last_steer + delta
        self.last_steer = steer

        control = carla.VehicleControl()
        control.throttle = 0.45
        control.steer = steer
        control.brake = 0.0

        self.vehicle.apply_control(control)
        return True
