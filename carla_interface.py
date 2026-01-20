
import carla
import queue
import config
import math
import random
import time

class CarlaInterface:
    def __init__(self):
        self.client = None
        self.world = None
        self.ego_vehicle = None
        self.radar_sensor = None
        self.radar_queue = queue.Queue()
        self.actor_list = []

    def setup_world(self):
        self.client = carla.Client(config.HOST, config.PORT)
        self.client.set_timeout(config.TIMEOUT)
        self.world = self.client.get_world()
        
        # Nuclear Cleanup: Destroy ALL existing vehicles ("Ghosts")
        # This prevents the <10m collisions on restart
        print("☢️ Nuclear Cleanup: Removing old actors...")
        commands = []
        for actor in self.world.get_actors().filter('vehicle.*'):
            commands.append(carla.command.DestroyActor(actor))
        for actor in self.world.get_actors().filter('sensor.*'):
            commands.append(carla.command.DestroyActor(actor))
        self.client.apply_batch(commands)
        
        # Apply 60Hz Sync Mode
        settings = self.world.get_settings()
        settings.synchronous_mode = config.SYNC_MODE
        settings.fixed_delta_seconds = config.FIXED_DELTA_SECONDS
        self.world.apply_settings(settings)

        # Graphical Optimization (Safe)
        try:
            self.world.unload_map_layer(carla.MapLayer.Buildings)
            self.world.unload_map_layer(carla.MapLayer.Foliage)
            self.world.unload_map_layer(carla.MapLayer.Props)
            self.world.unload_map_layer(carla.MapLayer.ParkedVehicles)
        except:
            pass

    def spawn_ego_vehicle(self):
        bp = self.world.get_blueprint_library().filter(config.EGO_VEHICLE_FILTER)[0]
        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)  # Randomize Start
        
        map = self.world.get_map()
        
        # Smart Filter: Find a Multi-Lane Road
        for sp in spawn_points:
            wp = map.get_waypoint(sp.location)
            
            # Check for valid lane neighbors (Left or Right)
            left = wp.get_left_lane()
            right = wp.get_right_lane()
            
            has_left = left and left.lane_type == carla.LaneType.Driving
            has_right = right and right.lane_type == carla.LaneType.Driving
            
            if has_left or has_right:
                self.ego_vehicle = self.world.try_spawn_actor(bp, sp)
                if self.ego_vehicle:
                    print(f"✅ Spawned on Multi-Lane Road (Left: {has_left}, Right: {has_right})")
                    break
                
        if not self.ego_vehicle:
            # Fallback to any point if strict spawning fails
            print("⚠️ Could not find multi-lane road. Spawning at random point.")
            for sp in spawn_points:
                self.ego_vehicle = self.world.try_spawn_actor(bp, sp)
                if self.ego_vehicle: break

        if not self.ego_vehicle:
            raise RuntimeError("Could not spawn Ego Vehicle!")
            
        self.actor_list.append(self.ego_vehicle)
        
        # Initial Camera Setup
        self.update_spectator()

    def attach_radar(self):
        bp = self.world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(config.RADAR_FOV_AZIMUTH))
        bp.set_attribute('vertical_fov', str(config.RADAR_FOV_ELEVATION))
        bp.set_attribute('range', str(config.RADAR_RANGE))
        bp.set_attribute('points_per_second', str(config.RADAR_POINTS_PER_SECOND))
        
        # Sensor tick 0.0 matches simulation step
        bp.set_attribute('sensor_tick', '0.0')

        transform = carla.Transform(carla.Location(x=2.5, z=1.6))
        self.radar_sensor = self.world.spawn_actor(bp, transform, attach_to=self.ego_vehicle)
        self.radar_sensor.listen(self.radar_queue.put)
        self.actor_list.append(self.radar_sensor)

    def get_latest_radar_data(self):
        """
        DRAINS THE QUEUE to ensure we always get the freshest data.
        Prevents lag accumulation.
        """
        data = None
        while not self.radar_queue.empty():
            data = self.radar_queue.get()
        return data

    def update_spectator(self):
        if not self.ego_vehicle: return
        
        spectator = self.world.get_spectator()
        ego_t = self.ego_vehicle.get_transform()
        
        yaw = math.radians(ego_t.rotation.yaw)
        x = ego_t.location.x - 10 * math.cos(yaw)
        y = ego_t.location.y - 10 * math.sin(yaw)
        z = ego_t.location.z + 5.0
        
        target_loc = carla.Location(x=x, y=y, z=z)
        target_rot = carla.Rotation(pitch=-20, yaw=ego_t.rotation.yaw, roll=0)
        
        # LERP Smoothing
        current_t = spectator.get_transform()
        alpha = 0.1
        
        new_loc = carla.Location(
            x=current_t.location.x * (1-alpha) + target_loc.x * alpha,
            y=current_t.location.y * (1-alpha) + target_loc.y * alpha,
            z=current_t.location.z * (1-alpha) + target_loc.z * alpha
        )
        
        spectator.set_transform(carla.Transform(new_loc, target_rot))

    def cleanup(self):
        print("🧹 Cleaning up actors...")
        for actor in self.actor_list:
            if actor.is_alive:
                actor.destroy()
        
        # Reset server settings
        if self.world:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)
