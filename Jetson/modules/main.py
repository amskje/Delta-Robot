import vision
import comms
import control
from enum import Enum, auto

class RobotState(Enum):
    IDLE = auto()
    DELIVERING = auto()
    ERROR = auto()
    PAUSED = auto()
    RESETTING = auto()

state = RobotState.IDLE

order = None


def main():
    ROS = comms.ROSComm()
    serial = comms.SerialComm()
    controller = control.DeltaRobotController(serial)
    model = vision.init_yolo()


    while ROS.node.ok():
        ROS.spin_once(timeout_sec=0.1)

        if state == RobotState.IDLE:
            print("[Main] Robot is idle. Waiting for commands...")
            msg = ROS.get_latest_message()
            if msg in vision.class_names:
                order = msg
                ROS.clear_message()
                state = RobotState.DELIVERING
            else:
                print("[Main] No valid order received. Waiting for next command...")
        
        if state == RobotState.DELIVERING:
            matches = vision.detect_target(model, order)
            # Choose target, now it just picks the first match
            if matches:
                controller.twist_delivery(
                    target_pos=(matches[0][1], matches[0][2], 250),  # x_cm, y_cm, z=250
                    dropoff_pos=(0, 0, 250)  # Example drop-off position
                )

        if state == RobotState.ERROR:
            print("[Main] Error state detected. Resetting robot...")
        elif state == RobotState.PAUSED:
            print("[Main] Robot is paused. Waiting for resume command...")
        elif state == RobotState.RESETTING:
            print("[Main] Resetting robot, but nothing because not implemented yet :)")
            
    
if __name__ == '__main__':
    main()