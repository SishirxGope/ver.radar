
import carla
import random
import time
import config

def setup_world(client):
    """Resets world settings and performs Nuclear Cleanup."""
    world = client.get_world()
    
    # 1. Nuclear Cleanup
    print("☢️  Nuclear Cleanup: Destroying all actors...")
    batch = []
    for actor in world.get_actors().filter('vehicle.*'):
        batch.append(carla.command.DestroyActor(actor))
    for actor in world.get_actors().filter('sensor.*'):
        batch.append(carla.command.DestroyActor(actor))
    if batch: client.apply_batch(batch)

    # 2. Sync Settings (30Hz)
    settings = world.get_settings()
    settings.synchronous_mode = config.SYNC_MODE
    settings.fixed_delta_seconds = config.FIXED_DELTA_SECONDS
    # Optimize rendering (Quick & Dirty)
    settings.max_substep_delta_time = 0.02
    settings.max_substeps = 10
    world.apply_settings(settings)
    
    # Spectrum Setup
    try:
        world.unload_map_layer(carla.MapLayer.Foliage)
        world.unload_map_layer(carla.MapLayer.Buildings)
        world.unload_map_layer(carla.MapLayer.ParkedVehicles)
    except: pass

    return world

def spawn_safe_ego(world):
    """Spawns Ego ONLY on multi-lane roads."""
    bp = world.get_blueprint_library().filter(config.EGO_FILTER)[0]
    points = world.get_map().get_spawn_points()
    random.shuffle(points)
    
    ego = None
    curr_map = world.get_map()
    
    for sp in points:
        wp = curr_map.get_waypoint(sp.location)
        # Check Neighbors
        left = wp.get_left_lane()
        right = wp.get_right_lane()
        
        has_left = left and left.lane_type == carla.LaneType.Driving
        has_right = right and right.lane_type == carla.LaneType.Driving
        
        if (has_left or has_right):
            ego = world.try_spawn_actor(bp, sp)
            if ego:
                print(f"✅ Safe Spawn: Multi-Lane (L:{has_left} R:{has_right})")
                return ego
                
    raise RuntimeError("❌ Could not find a safe multi-lane spawn point!")

def spawn_obstacle(world, ego, distance=100.0):
    """Spawns a static obstacle ahead."""
    bp = world.get_blueprint_library().filter(config.OBSTACLE_FILTER)[0]
    bp.set_attribute('role_name', 'obstacle')
    
    ego_loc = ego.get_location()
    wp = world.get_map().get_waypoint(ego_loc)
    
    # Scan ahead
    targets = wp.next(distance)
    if not targets: return None
    
    target_wp = targets[0]
    transform = target_wp.transform
    transform.location.z += 0.5 # Drop prevention
    
    obs = world.try_spawn_actor(bp, transform)
    if obs:
        obs.set_simulate_physics(True) # Physics on but Handbrake
        control = carla.VehicleControl(hand_brake=True)
        obs.apply_control(control)
        print(f"⚠️  Obstacle Spawned at {distance}m")
    
    return obs

def update_spectator(world, ego):
    """Updates spectator camera to follow ego vehicle."""
    spectator = world.get_spectator()
    transform = ego.get_transform()
    
    # Position: Behind and above
    # Using simple math to avoid complex vector operations if possible, 
    # but CARLA transforms make this easy.
    
    # -5m behind, +2.5m up
    location = transform.location - 5 * transform.get_forward_vector()
    location.z += 2.5
    
    # Look at vehicle
    rotation = transform.rotation
    rotation.pitch = -15.0 # Look down slightly
    
    spectator.set_transform(carla.Transform(location, rotation))
