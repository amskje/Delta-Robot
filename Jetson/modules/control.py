import math
import comms
import kinematics
from typing import List, Tuple
from dataclasses import dataclass

@dataclass
class ControlConfig:
    # Base parameters
    WAYPOINTS: int = 5
    INITIAL_POSITION: list[float] = kinematics.Position(3.373, 0.184, 257.886)  # Initial position after goHome()

def config() -> ControlConfig:
    return ControlConfig()


class DeltaRobotController:
    def __init__(self, serial_comm: comms.SerialComm):
        self.serial = serial_comm  # Instance of SerialComm
        self.current_pos = [0.0, 0.0, 0.0]  # Track robot position in mm

    def send_angles_sequence(self, angles_list):
        self.serial.send_message("POSITION")
        if not self.serial.wait_for_ack("READY"):
            print("[Error] Arduino not ready for POSITION.")
            return False

        for angles in angles_list:
            a1, a2, a3 = angles
            msg = f"ANGLES {a1:.2f},{a2:.2f},{a3:.2f}"
            self.serial.send_message(msg)

        self.serial.send_message("GO")
        if not self.serial.wait_for_ack("DONE"):
            print("[Error] Arduino did not complete GO.")
            return False
        
        return True

    def twist_delivery(self, target_pos: Tuple[float, float, float], dropoff_pos: Tuple[float, float, float]):
        """
        Executes a full delivery sequence from current position to target, then drop-off.
        
        Args:
            target_pos (List[int]): [x, y, z] coordinates of pickup target
            dropoff_pos (List[int]): [x, y, z] coordinates of drop-off target
        """
        print(f"[Control] Planning move to pickup at {target_pos}...")

        # === Phase 1: Plan path to pickup
        pickup_angles = []
        kinematics.plan_linear_move(self.current_pos[0], self.current_pos[1], self.current_pos[2],
                                    target_pos[0], target_pos[1], target_pos[2], pickup_angles)

        if not self._send_angles_sequence(pickup_angles):
            return False

        # === Pickup operation
        self.serial.send_message("PUMP_ON")
        self.serial.send_message("PICK_UP")
        if not self.serial.wait_for_ack("PICKED_UP"):
            print("[Error] Pickup not acknowledged.")
            return False
        
        self.current_pos = list(target_pos)

        print(f"[Control] Planning move to drop-off at {dropoff_pos}...")

        # === Phase 2: Plan path to drop-off
        dropoff_angles = []
        kinematics.plan_linear_move(
            *target_pos,
            *dropoff_pos,
            dropoff_angles,
            waypoints=config().WAYPOINTS
        )

        if not self._send_angles_sequence(dropoff_angles):
            return False

        # === Release
        self.serial.send_message("PUMP_OFF")

        # Update robot state
        self.current_pos = list(dropoff_pos)
        print("[Control] Delivery complete.")
        return True