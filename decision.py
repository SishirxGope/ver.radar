import config

class DecisionEngine:
    def __init__(self):
        self.current_state = "NORMAL"

    def decide(self, distance, relative_velocity):
        """
        Decides the state based on distance and relative velocity.
        """
        if distance is None or relative_velocity is None:
            self.current_state = "NORMAL"
            ttc = float('inf')
            print(f"State: {self.current_state}, Dist: None, TTC: inf")
            return self.current_state, ttc

        # Compute closing speed
        closing_speed = max(-relative_velocity, 0.1)
        
        ttc = distance / closing_speed
        
        # State Machine Logic
        if distance < 8.0:
            self.current_state = "EMERGENCY"
        elif distance < 15.0:
            self.current_state = "BRAKE"
        else:
            self.current_state = "NORMAL"
            
        # print(f"State: {self.current_state}, Dist: {distance:.2f}, TTC: {ttc:.2f}")
        
        return self.current_state, ttc
