import time
import rclpy
from enum import Enum, auto

import modules.vision as vision
import modules.comms as comms
import modules.control as control
import modules.kinematics as kinematics

#Teset, for middlertidig keyboard knapp d
import sys
import select
#Teset, for middlertidig keyboard knapp d


class RobotState(Enum):
    IDLE = auto()
    DELIVERING = auto()
    ERROR = auto()
    PAUSED = auto()
    RESETTING = auto()


def log(msg: str):
    """Prints a log message with timestamp."""
    print(f"[{time.strftime('%H:%M:%S')}] {msg}")


def main():
    # Initialize modules
    ROS = comms.ROSComm()
    serial = comms.SerialComm()
    controller = control.DeltaRobotController(serial)
    model = vision.init_yolo()

    # Initial state
    state = RobotState.IDLE
    order = None

    log("System initialized. Entering main loop...")

    while rclpy.ok():
        ROS.spin_once(timeout_sec=0.1)

        if state == RobotState.IDLE:
            log("Robot is idle. Waiting for commands...")

            #Teset, for middlertidig keyboard knapp d

            # Check for 'd' key to simulate "daim" message
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.readline().strip()
                if key.lower() == 'd':
                    log("Keyboard input 'd' detected. Simulating 'daim' message.")
                    order = 'Daim'
                    state = RobotState.DELIVERING
                    continue

            #Teset, for middlertidig keyboard knapp d
            


            msg = ROS.get_latest_message()
            if msg in vision.class_names:
                order = msg
                ROS.clear_message()
                state = RobotState.DELIVERING
            else:
                time.sleep(0.5)  # Prevent rapid idle polling

        elif state == RobotState.DELIVERING:
            log(f"Received order for: {order}")
            matches = []
            retry_count = 0

            # Retry YOLO detection up to 15 times
            while not matches and retry_count < 15:
                matches = vision.detect_target(model, order)
                
                if not matches:
                    log(f"No target found on attempt {retry_count + 1}. Retrying...")
                    retry_count += 1
                    time.sleep(1)

            if matches:
                target_x, target_y = matches[0][1], matches[0][2]
                log(f"Target detected at x={target_x:.2f} cm, y={target_y:.2f} cm")

                success = controller.twist_delivery(
                    target_pos=(target_y, target_x, 250), #test, byttet om rekkefølgen på x og y
                    dropoff_pos=(0, 0, 250)
                )

                state = RobotState.IDLE if success else RobotState.ERROR
            else:
                log("Failed to detect target after retries. Returning to IDLE.")
                state = RobotState.IDLE

        elif state == RobotState.ERROR:
            log("Error state detected. Awaiting manual reset or shutdown.")
            time.sleep(1)  # Idle delay during error state

        elif state == RobotState.PAUSED:
            log("Robot is paused. Awaiting resume command.")
            time.sleep(1)

        elif state == RobotState.RESETTING:
            log("Resetting robot...")
            # Optional: controller.reset_robot() if implemented
            state = RobotState.IDLE

        else:
            log("Unknown state. Switching to ERROR.")
            state = RobotState.ERROR


if __name__ == '__main__':
    main()
