import random
import time
import carla

class TrafficSpawner:
    def __init__(self, world, ego_vehicle):
        self.world = world
        self.ego = ego_vehicle
        self.blueprints = world.get_blueprint_library().filter("vehicle.*")
        self.spawned = []
        self.last_spawn_time = 0.0

        self.SPAWN_INTERVAL = 5.0
        self.MAX_VEHICLES = 10
        self.MIN_DISTANCE = 20.0
        self.MAX_DISTANCE = 60.0

    def tick(self):
        now = time.time()

        if now - self.last_spawn_time < self.SPAWN_INTERVAL:
            return

        self.last_spawn_time = now

        if len(self.spawned) >= self.MAX_VEHICLES:
            return

        self.try_spawn_near_ego()

    def try_spawn_near_ego(self):
        ego_tf = self.ego.get_transform()
        ego_loc = ego_tf.location

        world_map = self.world.get_map()
        ego_wp = world_map.get_waypoint(ego_loc)

        if ego_wp is None:
            return

        direction = random.choice(["forward", "forward", "forward", "backward"])
        dist = random.uniform(self.MIN_DISTANCE, self.MAX_DISTANCE)

        if direction == "forward":
            candidates = ego_wp.next(dist)
        else:
            candidates = ego_wp.previous(dist)

        if not candidates:
            return

        wp = random.choice(candidates)

        lane_choice = random.choice(["same", "left", "right"])

        if lane_choice == "left" and wp.get_left_lane():
            wp = wp.get_left_lane()
        elif lane_choice == "right" and wp.get_right_lane():
            wp = wp.get_right_lane()

        transform = wp.transform
        transform.location.z += 0.5

        bp = random.choice(self.blueprints)

        vehicle = self.world.try_spawn_actor(bp, transform)

        if vehicle:
            vehicle.set_autopilot(True)
            self.spawned.append(vehicle)
            print("🚗 Spawned traffic vehicle")
