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


import time
import psutil




class RobotState(Enum):
    IDLE = auto()
    DELIVERING = auto()
    ERROR = auto()
    PAUSED = auto()
    RESETTING = auto()


def log(msg: str):
    """Prints a log message with timestamp."""
    print(f"[{time.strftime('%H:%M:%S')}] {msg}")
    
def ros_spin_thread(ros_comm):
    while rclpy.ok():
        ros_comm.spin_once(timeout_sec=0.05)

def abort_listener(ROS, abort_flag, serial):
    while True:
        msg = ROS.get_latest_message()
        if msg == "ABORT":
            log("⚠️ ABORT received in background thread")
            abort_flag.set()
            serial.send_message("ABORT")
            #serial.wait_for_ack("ABORTED")
            ROS.clear_message()
        time.sleep(0.05)


def main():
    abort_flag = threading.Event()
    stop_event = threading.Event()
    arduino_ready = False
    rpi_ready = False

    # Initialize modules

    ROS = comms.ROSComm()

    threading.Thread(target=ros_spin_thread, args=(ROS,), daemon=True).start()


    serial = comms.SerialComm()

    threading.Thread(target=abort_listener, args=(ROS, abort_flag, serial), daemon=True).start()

    controller = control.DeltaRobotController(serial)
    vision_state = vision.VisionState()
    vision_conf = vision.config()
    model = vision.init_yolo(vision_conf.MODEL_PATH)

    cap, _ = vision.start_inference_thread(model, vision_conf, vision_state, stop_event)
    vision.wait_for_inference_ready(vision_state, timeout=20.0)


    ROS.send_message("REBOOT")

    while not (arduino_ready and rpi_ready):
    #while not (arduino_ready):
        # Poll Arduino
        if not arduino_ready:
            line = serial.conn.readline().decode().strip()
            if "Finished setup" in line:
                log("✅ Arduino is ready.")
                arduino_ready = True


        # Poll RPi message
        ROS.spin_once(timeout_sec=0.1)
        msg = ROS.get_latest_message()
        if not rpi_ready and msg == "PI_READY":
            log("✅ PI is ready.")
            rpi_ready = True
            ROS.clear_message()

        time.sleep(0.1)


    #Move robot to position to take picture
    controller.go_to_pos(move_pos = (0, 0, -305), abort_flag=abort_flag)
    controller.go_to_pos(move_pos = (-120, 80, -305), abort_flag=abort_flag)


    # Initial state
    state = RobotState.IDLE
    order = None 

    ROS.send_message("SETUP_FINISHED")
    log(" Setup complete. Sent confirmation to RPi.")  

    log("System initialized. Entering main loop...")

    try:

        while rclpy.ok():

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
                    elif key.lower() == 'j':
                        log("Keyboard input 'j' detected. Simulating 'Japp' message.")
                        order = 'Japp'
                        state = RobotState.DELIVERING
                        continue
                    if key.lower() == 'c':
                        log("Keyboard input 'c' detected. Simulating 'Crisp' message.")
                        order = 'Crisp'
                        state = RobotState.DELIVERING
                        continue
                    if key.lower() == 'g':
                        log("Keyboard input 'g' detected. Simulating 'golden' message.")
                        order = 'Golden'
                        state = RobotState.DELIVERING
                        continue
                    if key.lower() == 'n':
                        log("Keyboard input 'n' detected. Simulating 'notti' message.")
                        order = 'Notti'
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
                            controller.go_to_pos(move_pos=(120, 80, -305), abort_flag=abort_flag)
                        if retry_count == 1:
                            controller.go_to_pos(move_pos=(120, -80, -305), abort_flag=abort_flag)
                        if retry_count == 2:
                            controller.go_to_pos(move_pos=(-120, -80, -305), abort_flag=abort_flag)
                        if retry_count == 3:
                            controller.go_to_pos(move_pos=(-120, 80, -305), abort_flag=abort_flag)
                        retry_count += 1
                        time.sleep(1)

                if matches:
                    target_x, target_y = matches[0][1], matches[0][2]
                    log(f"Target detected at x={target_x:.2f} cm, y={target_y:.2f} cm")

                    result_msg = ""
                    success, result_msg = controller.twist_delivery(
                        target_pos=(target_x, target_y, -305),
                        dropoff_pos=(-120, 80, -305),
                        twist_type=order,
                        abort_flag=abort_flag
                    )


                    if success:
                        log(f" Delivery succeeded: {result_msg}")
                        ROS.send_message("DELIVERED")
                        state = RobotState.IDLE
                    else:
                        if result_msg in ["NOT_PICKED_UP", "DROPPED", "LIMIT"]:
                            log(f" Twist delivery failed: {result_msg}. Restarting delivery loop...")
                            controller.retreat_home(abort_flag=abort_flag)
                            ROS.send_message("LOST")
                            state = RobotState.IDLE  # Retry from the beginning
                        elif result_msg == "ABORTED":
                            log(f" Twist delivery failed: {result_msg}. Restarting delivery loop...")
                            controller.retreat_home(abort_flag=abort_flag)
                            ROS.send_message("COMPLETE_ABORT")
                            abort_flag.clear()
                            state = RobotState.IDLE

                        else:
                            log(f" Critical delivery failure: {result_msg}")
                            state = RobotState.ERROR
                                    
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

        
        pass

    except KeyboardInterrupt:
        log("Keyboard interupt recived (ctrl+c)")
    finally:

        log("Shutting down system.")
        ROS.send_message("SHUTDOWN")

        stop_event.set()

        time.sleep(1)

        serial.close()
        log("Serial connection closed")

        if cap:
            cap.release()
            log("Camera relased")


        ROS.shutdown()
        log("Ros shutdown")


if __name__ == '__main__':
    main()
