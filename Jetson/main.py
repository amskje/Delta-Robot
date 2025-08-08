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
    
def log_resource(tag):
    cpu = psutil.cpu_percent(interval=None)
    mem = psutil.virtual_memory()
    print(f"[{tag}] CPU: {cpu:.1f}% | RAM used: {mem.used / 1e6:.1f} MB")


def serial_listener(serial):
    while True:
        serial.read_serial_responses()
        time.sleep(0.1)

    
def ros_spin_thread(ros_comm):
    while rclpy.ok():
        ros_comm.spin_once(timeout_sec=0.05)


def abort_listener(ROS, abort_flag):
    while True:
        ROS.spin_once(timeout_sec=0.1)  # Short spin to check for new messages
        msg = ROS.get_latest_message()
        if msg == "ABORTED":
            log("⚠️ Received ABORTED from RPi. Triggering emergency stop...")
            serial.send_message("ABORT")  # Send emergency stop to Arduino
            abort_flag.set()  # Signal Python logic to stop current task
            ROS.clear_message()  # Clear the message to prevent re-processing




def main():
    abort_flag = threading.Event()
    arduino_ready = False
    rpi_ready = False

    # Initialize modules
    start = time.time()
    print("⏳ Init ROS...")
    ROS = comms.ROSComm()
    threading.Thread(target=abort_listener, args=(ROS, abort_flag), daemon=True).start()

    threading.Thread(target=ros_spin_thread, args=(ROS,), daemon=True).start()


    log_resource("ROSComm init")
    print(f"✅ ROS init done in {time.time() - start:.2f}s\n")

    start = time.time()
    print("⏳ Init Serial...")
    serial = comms.SerialComm()
    log_resource("SerialComm init")
    print(f"✅ Serial init done in {time.time() - start:.2f}s\n")

    start = time.time()
    print("⏳ Init Controller...")
    controller = control.DeltaRobotController(serial)
    log_resource("Controller init")
    print(f"✅ Controller init done in {time.time() - start:.2f}s\n")

    start = time.time()
    print("⏳ Init VisionState...")
    vision_state = vision.VisionState()
    log_resource("VisionState init")
    print(f"✅ VisionState init done in {time.time() - start:.2f}s\n")

    start = time.time()
    print("⏳ Init Config...")
    vision_conf = vision.config()
    log_resource("Vision config")
    print(f"✅ Vision config done in {time.time() - start:.2f}s\n")

    start = time.time()
    print("⏳ Loading YOLO model...")
    model = vision.init_yolo(vision_conf.MODEL_PATH)
    log_resource("YOLO model load")
    print(f"✅ Model loaded in {time.time() - start:.2f}s\n")

    start = time.time()
    print("⏳ Starting inference thread...")
    vision.start_inference_thread(model, vision_conf, vision_state)
    log_resource("Start inference thread")
    print(f"✅ Thread started in {time.time() - start:.2f}s\n")

    start = time.time()
    print("⏳ Waiting for inference readiness...")
    vision.wait_for_inference_ready(vision_state, timeout=20.0)
    log_resource("Inference ready")
    print(f"✅ Inference ready in {time.time() - start:.2f}s\n")

    ROS.send_message("REBOOT")

    while not (arduino_ready and rpi_ready):
    #while not (arduino_ready):
        # Poll Arduino
        if not arduino_ready:
            line = serial.conn.readline().decode().strip()
            if "Finished setup" in line:
                log("✅ Arduino is ready.")
                arduino_ready = True

                #threading.Thread(target=serial_listener, args=(serial,), daemon=True).start()

        # Poll RPi message
        ROS.spin_once(timeout_sec=0.1)
        msg = ROS.get_latest_message()
        if not rpi_ready and msg == "PI_READY":
            log("✅ PI is ready.")
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
        #ROS.spin_once(timeout_sec=0.1)

        # Check for ROS abort message
        msg = ROS.get_latest_message()
        if msg == "ABORT":
            log(" Received ABORT from RPi. Triggering emergency stop...")
            serial.send_message("ABORT") 
            abort_flag.set()  # Signal to interrupt delivery
            ROS.clear_message()


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

                result_msg = ""
                success, result_msg = controller.twist_delivery(
                    target_pos=(target_x, target_y, -305),
                    dropoff_pos=(-120, 80, -305),
                    twist_type=order,
                    abort_flag=abort_flag
                )

                #legge til her at hvis ikke sucsess, send melding til rpi
                #og start main loop på nytt
                #state = RobotState.IDLE if success else RobotState.ERROR


                if success:
                    log(f" Delivery succeeded: {result_msg}")
                    ROS.send_message("DELIVERD")
                    state = RobotState.IDLE

                else:
                    if result_msg in ["NOT_PICKED_UP", "DROPPED"]:
                        log(f" Twist failed: {result_msg}. Restarting delivery loop...")
                        ROS.send_message("LOST")
                        state = RobotState.IDLE  # Retry from the beginning
                    elif result_msg == "ABORTED":
                        log(f" Twist failed: {result_msg}. Restarting delivery loop...")
                        controller.go_to_pos((0, 0, -305))
                        controller.go_to_pos((-120, 80, -305))  # Already done in twist_delivery for safety
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

    log("Shutting down system.")
    ROS.send_message("SHUTDOWN")



if __name__ == '__main__':
    main()
