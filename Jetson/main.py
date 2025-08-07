import time
import rclpy
from enum import Enum, auto

import modules.vision as vision
import modules.comms as comms
import modules.control as control
import modules.kinematics as kinematics


import numpy as np
import threading

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
    vision_state = vision.VisionState()

    arduino_ready = False
    rpi_ready = False

    vision_conf = vision.config()
    model = vision.init_yolo(vision_conf.MODEL_PATH)    
    vision.start_inference_thread(model, vision_conf, vision_state)  

    vision.wait_for_inference_ready(vision_state, timeout=10.0)

    #while not (arduino_ready and rpi_ready):
    while not (arduino_ready):
        # Poll Arduino
        if not arduino_ready:
            line = serial.conn.readline().decode().strip()
            if "Finished setup" in line:
                log("✅ Arduino is ready.")
                arduino_ready = True

        # Poll RPi message
        ROS.spin_once(timeout_sec=0.1)
        msg = ROS.get_latest_message()
        if not rpi_ready and msg == "RPi_READY":
            log("✅ RPi is ready.")
            rpi_ready = True
            ROS.clear_message()

        time.sleep(0.1)
  
    #Show live videofeed
    #threading.Thread(target=vision.show_live_detections, daemon=True).start()

    #Move robot to position to take picture
    controller.go_to_pos(move_pos = (0, 0, -305))
    controller.go_to_pos(move_pos = (-120, 80, -305))


    # Initial state
    state = RobotState.IDLE
    order = None 

    ROS.send_message("SETUP_FINISHED")
    log(" Setup complete. Sent confirmation to RPi.")  

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
                    order = 'Banan'
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

            # Retry YOLO detection up to 4 times
            while not matches and retry_count < 4:
                matches = vision.detect_target(order, vision_conf, vision_state)
                if not matches:
                    log(f"No target found on attempt {retry_count + 1}. Retrying...")
                    
                    # Move robot to a different position for the next attempt
                    if retry_count == 0:
                        controller.go_to_pos(move_pos=(120, 80, -305))
                    if retry_count == 1:
                        controller.go_to_pos(move_pos=(120, -80, -305))
                    if retry_count == 2:
                        controller.go_to_pos(move_pos=(-120, -80, -305))
                    if retry_count == 3:
                        controller.go_to_pos(move_pos=(-120, 80, -305))
                    retry_count += 1
                    time.sleep(1)

            if matches:
                target_x, target_y = matches[0][1], matches[0][2]
                log(f"Target detected at x={target_x:.2f} cm, y={target_y:.2f} cm")

                success = controller.twist_delivery(
                    target_pos=(target_x, target_y, -305),
                    dropoff_pos=(-120, 80, -305)
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
