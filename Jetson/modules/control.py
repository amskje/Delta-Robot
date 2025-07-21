from . import comms
from . import kinematics
from typing import List, Tuple
from dataclasses import dataclass

HOME_X = 3.373
HOME_Y = 0.184
HOME_Z = 257.886

@dataclass
class ControlConfig:
    # Base parameters
    WAYPOINTS: int = 5 # Minimum 2
    WAYPOINTS_DOWN: int = 5 #Minimum 2
    DOWN_MM: int = 36 #Total mm robot can move down after hitting target pos
    INITIAL_POSITION: kinematics.Position = kinematics.Position(HOME_X, HOME_Y, HOME_Z)  # Initial position after goHome()

def config() -> ControlConfig:
    return ControlConfig()


class DeltaRobotController:
    def __init__(self, serial_comm: comms.SerialComm):
        self.serial = serial_comm  # Instance of SerialComm
        self.current_pos = [HOME_X, HOME_Y, HOME_Z] # Track robot position in mm


    def send_angles_sequence(self, angles_list, angles_down_list, down_included):
        self.serial.send_message("POSITION")
        if not self.serial.wait_for_ack("READY"):
            print("[Error] Arduino not ready for POSITION.")
            return False

        for angles in angles_list:
            a1, a2, a3 = angles
            msg = f"ANGLES {int(a1)}, {int(a2)}, {int(a3)}"
            self.serial.send_message(msg)

        if (down_included == True):
            #Send the moving down angles 
            self.serial.send_message("DOWN")
            if not self.serial.wait_for_ack("READY"):
                print("[Error] Arduino not ready for down POSITION.")
                return False
            
            for angles_down in angles_down_list:
                a1, a2, a3 = angles_down
                msg = f"ANGLES {int(a1)}, {int(a2)}, {int(a3)}"
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
        down_angles = []
         
        kinematics.plan_linear_move(self.current_pos[0], self.current_pos[1], self.current_pos[2],
                                    target_pos[0], target_pos[1], target_pos[2], pickup_angles, waypoints=config().WAYPOINTS)

        #Pre calculated steps for moving down if twist is not picked up, maby change the number 25
        #kan kanskje sende den 6 cm ned, også i arduino code ta å bruke 1/3 av way punktene av gangen
        kinematics.plan_linear_move(target_pos[0], target_pos[1], target_pos[2],
                                    target_pos[0], target_pos[1], target_pos[2]+config().DOWN_MM, down_angles, waypoints=config().WAYPOINTS_DOWN)



        #Her får man done for arduino hvis den har plukket opp twsiten
        if not self.send_angles_sequence(pickup_angles, down_angles, down_included=True):
            return False

        # === Pickup operation


        #self.serial.send_message("PUMP_ON")
        
        #if not self.serial.wait_for_ack("PICKED_UP"):
         #   print("[Error] Pickup not acknowledged.")
         #   return False
        
        self.current_pos = list(target_pos) #husk, må gjøre slik at etter roboten har plukket opp må den vite akkuret hvor langt ned den har gått
        
        print(f"[Control] Planning move to drop-off at {dropoff_pos}...")

        # === Phase 2: Plan path to drop-off
        
        dropoff_angles = []
        kinematics.plan_linear_move(
            *target_pos,
            *dropoff_pos,
            dropoff_angles,
            waypoints=config().WAYPOINTS
        )


        # === Release
        #self.serial.send_message("PUMP_OFF")
        self.serial.send_message("DROPP")



        if not self.send_angles_sequence(dropoff_angles, [], down_included=False):
            return False
        

        
    
        # Update robot state
        self.current_pos = list(dropoff_pos)
        print("[Control] Delivery complete.")
        return True