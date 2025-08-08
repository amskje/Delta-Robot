from . import comms
from . import kinematics
from typing import List, Tuple
from dataclasses import dataclass
import numpy as np
import json
import cv2

HOME_X = 3.038
HOME_Y = 0.1655
HOME_Z = -247.34

@dataclass
class ControlConfig:
    # Base parameters
    WAYPOINTS: int = 10 # Minimum 2
    WAYPOINTS_DOWN: int = 10 #Minimum 2
    DOWN_MM: int = 63 #Total mm robot can move down after hitting target pos
    DOWN_DAIM_MM: int = 75
    DOWN_NOTTI_MM: int = 50
    INITIAL_POSITION: kinematics.Position = kinematics.Position(HOME_X, HOME_Y, HOME_Z)  # Initial position after goHome()

def config() -> ControlConfig:
    return ControlConfig()

class DeltaRobotController:
    def __init__(self, serial_comm: comms.SerialComm):
        self.serial = serial_comm  # Instance of SerialComm
        self.current_pos = [HOME_X, HOME_Y, HOME_Z] # Track robot position in mm
        with open('homography_ROBOT_WORLD.json', 'r') as file:
            H_robot = json.load(file)

        self.H_robot_inv = np.linalg.inv(H_robot)

        print(H_robot)

    
    def correct_target(self, x_desired, y_desired):
        """Transform a target point from real-world into robot coordinates"""
        pt = np.array([[[x_desired, y_desired]]], dtype=np.float32)
        corrected = cv2.perspectiveTransform(pt, self.H_robot_inv)
        return corrected[0][0]

    def check_abort(self, abort_flag, msg=""):
        if abort_flag and abort_flag.is_set():
            self.go_to_pos((-120, 80, -305), abort_flag=None) 
            print(f"[Control] ABORT detected {msg}. Retreating...")
            return True
        return False


    def send_angles_sequence(self, angles_list, angles_down_list, down_included, abort_flag=None):
        self.serial.send_message("POSITION")
        ack = self.serial.wait_for_ack("READY")
        if ack != True:
            print(f"[Error] Arduino not ready for POSITION. Got: {ack}")
            return ack  # Can be unexpected msg or False (timeout)

        for angles in angles_list:
            if self.check_abort(abort_flag, "before pickup"):
                return False, "ABORTED"
            a1, a2, a3 = angles
            msg = f"ANGLES {int(a1)}, {int(a2)}, {int(a3)}"
            self.serial.send_message(msg)

        if down_included:
            self.serial.send_message("DOWN")
            ack = self.serial.wait_for_ack("READY")
            if ack != True:
                print(f"[Error] Arduino not ready for DOWN. Got: {ack}")
                return ack
            
            for angles_down in angles_down_list:
                if self.check_abort(abort_flag, "before pickup"):
                                return "ABORTED"
                a1, a2, a3 = angles_down
                msg = f"ANGLES {int(a1)}, {int(a2)}, {int(a3)}"
                self.serial.send_message(msg)

        self.serial.send_message("GO")
        ack = self.serial.wait_for_ack("DONE")
        if ack != True:
            print(f"[Error] Arduino did not complete GO. Got: {ack}")
            return ack

        return "SUCCESS"


    def twist_delivery(self, target_pos: Tuple[float, float, float], dropoff_pos: Tuple[float, float, float], twist_type: str, include_dropoff: bool = True, abort_flag=None):
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

        # Map robot coordinates to real world
        x_corrected, y_corrected = self.correct_target(target_pos[0], target_pos[1])
        
        kinematics.plan_linear_move(self.current_pos[0], self.current_pos[1], self.current_pos[2],
                                    x_corrected, y_corrected, target_pos[2], pickup_angles, waypoints=config().WAYPOINTS)


        #if statement som endrer down_mm baser på twist!!!!!!!!!!!!!!!!!!!!!!!!!!
        #Plan the moving down waypoints
        if (twist_type == 'Daim'):   
            kinematics.plan_linear_move(x_corrected, y_corrected, target_pos[2],
                            x_corrected, y_corrected, target_pos[2]-config().DOWN_DAIM_MM, down_angles, waypoints=config().WAYPOINTS_DOWN)
        elif (twist_type == 'Notti'):
            kinematics.plan_linear_move(x_corrected, y_corrected, target_pos[2],
                            x_corrected, y_corrected, target_pos[2]-config().DOWN_NOTTI_MM, down_angles, waypoints=config().WAYPOINTS_DOWN)

        else:
            kinematics.plan_linear_move(x_corrected, y_corrected, target_pos[2],
                            x_corrected, y_corrected, target_pos[2]-config().DOWN_MM, down_angles, waypoints=config().WAYPOINTS_DOWN)



        
        pickup_result = self.send_angles_sequence(pickup_angles, down_angles, down_included=True)


        if pickup_result != "SUCCESS":
            if pickup_result in ["NOT_PICKED_UP", "DROPPED", "ABORTED"]:
                print(f"[Control] Twist delivery not completed, message recived {pickup_result}. Moving to fallback position.")
                self.go_to_pos((0, 0, -305))
                self.go_to_pos((-120, 80, -305))
                self.current_pos = [-120, 80, -305] 

                return False, pickup_result

            else:
                print(f"[Control] Error during pickup: {pickup_result}")
                return False, pickup_result
        else: 
            # Update robot position
            self.current_pos = [x_corrected, y_corrected, target_pos[2] ] 

        

       


        # === Phase 2: Plan path to drop-off
        
        print(f"[Control] Planning move to drop-off at {dropoff_pos}...")

        # Adjust dropoff pos with H
        x_dropoff_corrected, y_dropoff_corrected = self.correct_target(dropoff_pos[0], dropoff_pos[1])
        

        if include_dropoff:
            dropoff_angles = []
            kinematics.plan_linear_move(
                *target_pos,
                x_dropoff_corrected, y_dropoff_corrected, dropoff_pos[2],
                dropoff_angles,
                waypoints=config().WAYPOINTS
            )


        # === Release
        self.serial.send_message("DROPP")

        pickup_result = self.send_angles_sequence(dropoff_angles, [], down_included=False, abort_flag=abort_flag)
        if pickup_result != "SUCCESS":
            if pickup_result in ["NOT_PICKED_UP", "DROPPED", "ABORTED"]:
                print(f"[Control] Twist delivery not completed, message recived {pickup_result}. Moving to fallback position.")
                self.go_to_pos((0, 0, -305))
                self.go_to_pos((-120, 80, -305))
                self.current_pos = [-120, 80, -305] 

                return False, pickup_result

            else:
                print(f"[Control] Error during pickup: {pickup_result}")
                return False, pickup_result
        else:
            self.current_pos = list(dropoff_pos)
            # Update robot state
            print("[Control] Delivery complete.")
            return True, "SUCCESS"

        


       
        
        


 
    def go_to_pos(self, move_pos: Tuple[float, float, float],  abort_flag=None):
        """
        Moves the robot to choosen x, y and z coordinates
        
        Args:
            move_pos (List[int]): [x, y, z] coordinates of point to move to
        """
        print(f"[Control] Planning move to pickup at {move_pos}...")

        down_angles = []
        move_angles = []

        x_corrected, y_corrected = self.correct_target(move_pos[0], move_pos[1])
 
        kinematics.plan_linear_move(self.current_pos[0], self.current_pos[1], self.current_pos[2],
                                    x_corrected, y_corrected, move_pos[2], move_angles, waypoints=config().WAYPOINTS)

        if not self.send_angles_sequence(move_angles, down_angles, down_included=False, abort_flag=abort_flag):
            return False

        self.current_pos = [x_corrected, y_corrected, move_pos[2]] 
        
        print("[Control] Move complete.")
        return True
        