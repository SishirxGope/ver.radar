
import carla
import random
import time

class ObstacleSpawner:
    def __init__(self, world, ego_vehicle):
        self.world = world
        self.ego = ego_vehicle
        self.actors = []
        self.last_spawn_time = time.time()
        self.spawn_interval = 8.0  # Seconds between spawns
        
        # Immediate Spawn
        self._spawn()

    def tick(self):
        self._cleanup()
        self._spawn()

    def _cleanup(self):
        # Remove obstacles that are behind the ego vehicle (passed)
        if not self.ego: return
        
        ego_loc = self.ego.get_location()
        ego_fwd = self.ego.get_transform().get_forward_vector()
        
        active_actors = []
        for actor in self.actors:
            if not actor.is_alive: continue
            
            # Check relative position
            to_actor = actor.get_location() - ego_loc
            dot = ego_fwd.x * to_actor.x + ego_fwd.y * to_actor.y
            
            # If > 15m behind, destroy
            if dot < -15.0:
                print("♻️ Garbage Collected Obstacle")
                actor.destroy()
            else:
                active_actors.append(actor)
        
        self.actors = active_actors

    def _spawn(self):
        # Hard limit: Max 2 obstacles at a time
        if len(self.actors) >= 2:
            return

        if time.time() - self.last_spawn_time < self.spawn_interval:
            return
            
        self.last_spawn_time = time.time()
        
        ego_wp = self.world.get_map().get_waypoint(self.ego.get_location())
        # Spawn 100m ahead (Increased from 80m)
        next_wps = ego_wp.next(100.0)
        
        if not next_wps: return
        
        target_wp = next_wps[0]
        bp = random.choice(self.world.get_blueprint_library().filter("vehicle.*"))
        
        transform = target_wp.transform
        transform.location.z += 0.5
        
        vehicle = self.world.try_spawn_actor(bp, transform)
        if vehicle:
            vehicle.set_simulate_physics(False) # Static obstacle
            self.actors.append(vehicle)
            print("✅ Spawned Obstacle")

    def cleanup(self):
        for a in self.actors:
            if a.is_alive: a.destroy()
