
import carla
import time
import config
import utils
from simple_agent import SimpleAgent

def main():
    client = carla.Client(config.HOST, config.PORT)
    client.set_timeout(config.TIMEOUT)
    
    print("🚀 Ver.RADAR V3 (Lite) Starting...")
    
    try:
        # 1. Setup
        world = utils.setup_world(client)
        ego = utils.spawn_safe_ego(world)
        agent = SimpleAgent(world, ego)
        
        utils.spawn_obstacle(world, ego, distance=150.0)
        
        print("✅ System Online. Stable 30Hz Loop.")
        
        # 2. Loop
        frame = 0
        clock = time.time()
        last_spawn_time = time.time()
        
        while True:
            # Physics
            world.tick()
            utils.update_spectator(world, ego)
            
            # Periodic Spawning (Every 20s, further away)
            if time.time() - last_spawn_time > 20.0:
                 utils.spawn_obstacle(world, ego, distance=80.0)
                 last_spawn_time = time.time()
            
            # Agent Logic
            control = agent.tick()
            ego.apply_control(control)
            
            # Stats (Every 1s)
            frame += 1
            if frame % 30 == 0:
                now = time.time()
                fps = 30.0 / (now - clock)
                clock = now
                
                v = ego.get_velocity()
                spd = 3.6 * (v.x**2 + v.y**2)**0.5
                print(f"⏱️ FPS: {fps:.1f} | Spd: {spd:.1f} | State: {agent.state}")

    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"CRITICAL: {e}")
    finally:
        print("🧹 Cleanup...")
        if 'agent' in locals(): agent.destroy()
        utils.setup_world(client) # Re-runs nuclear cleanup
        print("👋 Done.")

if __name__ == "__main__":
    main()
