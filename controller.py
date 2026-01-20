
import carla


class Controller:
    def __init__(self):
        pass

    def get_control(self, state, ttc):
        """
        Generates control signal based on state.
        
        Args:
            state: decision.State
            ttc: float (Time To Collision)
            
        Returns:
            carla.VehicleControl
        """
        control = carla.VehicleControl()

        # DO NOT TOUCH control.steer !!!

        if state == "EMERGENCY":
            control.throttle = 0.0
            control.brake = 1.0
        elif state == "BRAKE":
            control.throttle = 0.0
            control.brake = min(1.0, max(0.2, 3.0 / max(ttc, 0.1)))
        elif state == "NORMAL":
            control.throttle = 0.4
            control.brake = 0.0
            
        return control
