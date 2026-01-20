import numpy as np
from scipy.interpolate import splprep, splev
import math
import carla

class BSplinePlanner:
    def generate_path(self, current, target, offset=6.0):
        # Align to road, not target
        import carla
        client = carla.Client("localhost", 2000)
        world = client.get_world()
        m = world.get_map()
        ego_wp = m.get_waypoint(current)
        yaw = math.radians(ego_wp.transform.rotation.yaw)
        
        c = math.cos(yaw)
        s = math.sin(yaw)
        
        # Helper to rotation
        def rotate_point(lx, ly):
            # Rotate local point (lx, ly) by yaw and add to current
            wx = current.x + (lx * c - ly * s)
            wy = current.y + (lx * s + ly * c)
            return carla.Location(x=wx, y=wy, z=current.z)
            
        # P0: Current
        p0 = current
        
        # P1: Forward 15m
        p1 = rotate_point(15.0, 0.0)
        
        # P2: Forward 35m, Side offset
        p2 = rotate_point(35.0, offset)
        
        # P3: Forward 70m, Side offset
        p3 = rotate_point(70.0, offset)
        
        points = [p0, p1, p2, p3]
        return self._bspline(points)

    def _bspline(self, points):
        x = [p.x for p in points]
        y = [p.y for p in points]
        
        # If we have duplicate points, splprep might fail, but with 12m spacing it's fine.
        pts = np.vstack([x, y])
        
        try:
            # k=3 needs 4 points. We have 4.
            tck, _ = splprep(pts, k=3, s=0.0)
        except:
            # Fallback to k=2 if something weird happens (though 4 pts is enough for k=3 cubic)
            tck, _ = splprep(pts, k=2, s=0.0)
            
        u = np.linspace(0, 1, 30)
        out = splev(u, tck)
        
        return list(zip(out[0], out[1]))
