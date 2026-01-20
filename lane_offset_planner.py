import carla
import math

class LaneOffsetPlanner:
    def __init__(self, world):
        self.world = world

    def generate_path(self, start_location, offset, length=80.0, step=2.0):
        m = self.world.get_map()
        wp = m.get_waypoint(start_location)

        path = []

        dist = 0.0
        while dist < length:
            next_wps = wp.next(step)
            if not next_wps:
                break
            wp = next_wps[0]

            transform = wp.transform
            yaw = math.radians(transform.rotation.yaw)

            # Lateral offset (left/right)
            ox = -math.sin(yaw) * offset
            oy =  math.cos(yaw) * offset

            loc = transform.location
            shifted = carla.Location(
                x = loc.x + ox,
                y = loc.y + oy,
                z = loc.z
            )

            path.append((shifted.x, shifted.y))
            dist += step

        return path
