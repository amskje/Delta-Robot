import time
import rclpy
from enum import Enum, auto

import modules.vision as vision
import modules.comms as comms
import modules.control as control
import modules.kinematics as kinematics


import numpy as np
import threading
import sys
import time

#Teset, for middlertidig keyboard knapp d
import sys
import select
#Teset, for middlertidig keyboard knapp d


import psutil

RESTART_EXIT_CODE = 42


class RobotState(Enum):
    IDLE = auto()
    DELIVERING = auto()
    ERROR = auto()
    PAUSED = auto()
    RESETTING = auto()
    MANUAL = auto()


def log(msg: str):
    """Prints a log message with timestamp."""
    print(f"[{time.strftime('%H:%M:%S')}] {msg}")
    
def ros_spin_thread(ros_comm):
    while rclpy.ok():
        ros_comm.spin_once(timeout_sec=0.05)

def abort_listener(ROS, abort_flag, serial, state):
    while True:
        msg = ROS.get_latest_message()
        if msg == "ABORT":
            log("⚠️ ABORT received in background thread")
            abort_flag.set()
            if (state == RobotState.DELIVERING):
                serial.send_message("ABORT")
            ROS.clear_message()
        time.sleep(0.05)

def restart_program(cap, stop_event, serial, ROS):
    try:
        if stop_event:
            stop_event.set()
        time.sleep(0.2)

        if cap:
            try: cap.release()
            except: pass
        
        if serial:
            try: serial.close()
            except: pass
        
        if ROS:
            try: ROS.shutdown()
            except: pass
        
    except Exception as e:
        print(f"[Restart] cleanup error: {e}")



def main():
    abort_flag = threading.Event()
    stop_event = threading.Event()
    arduino_ready = False
    rpi_ready = False

    # Initial state
    state = RobotState.IDLE
    order = None 

    # Initialize modules

    ROS = comms.ROSComm()

    threading.Thread(target=ros_spin_thread, args=(ROS,), daemon=True).start()


    serial = comms.SerialComm()

    threading.Thread(target=abort_listener, args=(ROS, abort_flag, serial, state), daemon=True).start()

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


    

    ROS.send_message("SETUP_FINISHED")
    log(" Setup complete. Sent confirmation to RPi.")  

    log("System initialized. Entering main loop...")

    try:

        while rclpy.ok():

            msg = ROS.get_latest_message()
            if msg == "RESTART_APP":
                log("RESTART_APP recived - restarting service")
                ROS.clear_message()

                try:
                    abort_flag.set()
                    serial.send_message("ABORT")
                    serial.wait_for_ack("ABORTED")
                except Exception as e:
                    print(f"[Restart] abort note: {e}")
                
                restart_program(cap, stop_event, serial, ROS)
                sys.exit(RESTART_EXIT_CODE)
                

            if state == RobotState.IDLE:
                log("Robot is idle. Waiting for commands...")

                # Check if abort flag was set during previous cleanup
                if abort_flag.is_set():
                    log("⚠️ Abort flag still set in IDLE. Clearing.")
                    abort_flag.clear()
                    ROS.clear_message()  # Clear any leftover message
                    time.sleep(0.5)
                    continue

                

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
                    if key.lower() == 'b':
                        log("Keyboard input 'b' detected. Simulating 'banan' message.")
                        order = 'Banan'
                        state = RobotState.DELIVERING
                        continue
                    if key.lower() == 'l':
                        log("Keyboard input 'l' detected. Simulating 'lakris' message.")
                        order = 'Lakris'
                        state = RobotState.DELIVERING
                        continue
                    if key.lower() == 'm':
                        log("Keyboard input 'm' detected. Simulating 'marsipan' message.")
                        order = 'Marsipan'
                        state = RobotState.DELIVERING
                        continue
                    if key.lower() == 'f':
                        log("Keyboard input 'f' detected. Simulating 'fransk' message.")
                        order = 'Fransk'
                        state = RobotState.DELIVERING
                        continue
                    if key.lower() == 't':
                        log("Keyboard input 't' detected. Simulating 'toffe' message.")
                        order = 'Toffee'
                        state = RobotState.DELIVERING
                        continue
                    if key.lower() == 'e':
                        log("Keyboard input 'e' detected. Simulating 'eclairs' message.")
                        order = 'Eclairs'
                        state = RobotState.DELIVERING
                        continue
                    if key.lower() == 'o':
                        log("Keyboard input 'o' detected. Simulating 'cocos' message.")
                        order = 'Cocos'
                        state = RobotState.DELIVERING
                        continue
                    if key.lower() == 'k':
                        log("Keyboard input 'k' detected. Simulating 'karamell' message.")
                        order = 'Karamell'
                        state = RobotState.DELIVERING
                        continue

                #Teset, for middlertidig keyboard knapp d
                


                msg = ROS.get_latest_message()
                if msg in vision.class_names:
                    order = msg
                    ROS.clear_message()
                    state = RobotState.DELIVERING
                elif msg == "MANUAL":
                    log("Entering manual mode.")
                    serial.send_message("MANUAL")
                    state = RobotState.MANUAL
                    ROS.clear_message()
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
                    matches.sort(key=lambda m: m[3], reverse=True)
                    best_class, target_x, target_y, best_conf = matches[0]
                    log(f"Selected {best_class} (conf={best_conf:.2f}) at x={target_x:.2f}, y={target_y:.2f}")


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
                            log(f" Critical delivery failure: {result_msg}. Unexpected message, moving home and returning to idle.")
                            serial.send_message("ABORT")
                            serial.wait_for_ack("ABORTED")
                            controller.retreat_home(abort_flag=abort_flag)
                            state = RobotState.IDLE
                                    
                else:
                    log("Failed to detect target after retries. Returning to IDLE.")
                    #Either emphty of the type twist or could not found it
                    ROS.send_message("EMPTY")
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
            
            elif state == RobotState.MANUAL:
                msg = ROS.get_latest_message()

                if msg:
                    ROS.clear_message()

                    if msg == "PUMP_ON":
                        serial.send_message("PUMP_ON")
                        log("Manual: PUMP ON")

                    elif msg == "PUMP_OFF":
                        serial.send_message("PUMP_OFF")
                        log("Manual: PUMP OFF")

                    elif msg.startswith("Z "):
                        z = float(msg.split()[1])

                        angles = []
                        kinematics.plan_linear_move(controller.current_pos[0], controller.current_pos[1], controller.current_pos[2],
                                                    controller.current_pos[0], controller.current_pos[1], -355 + z*50, angles, waypoints=2)
                        a1, a2, a3 = angles[-1] # Sending only the goal waypoint
                        msg = f"ANGLES {int(a1)}, {int(a2)}, {int(a3)}"
                        serial.send_message(msg)
                        controller.current_pos = (controller.current_pos[0], controller.current_pos[1], -355 + z*50)
                        print(f"Sent Angles {int(a1)}, {int(a2)}, {int(a3)}")

                    elif msg.startswith("WAYPOINT "):
                        coord_str = msg.split()[1]
                        x_str, y_str = coord_str.split(",")
                        x = float(x_str)
                        y = float(y_str)
                        angles = []

                        x_corrected, y_corrected = controller.correct_target(-y * 140, x*140, controller.current_pos[2])

                        kinematics.plan_linear_move(controller.current_pos[0], controller.current_pos[1], controller.current_pos[2],
                                                     x_corrected, y_corrected, controller.current_pos[2], angles, waypoints=2)
                        a1, a2, a3 = angles[-1] # Sending only the goal waypoint
                        msg = f"ANGLES {int(a1)}, {int(a2)}, {int(a3)}"
                        serial.send_message(msg)
                        controller.current_pos = (x_corrected, y_corrected, controller.current_pos[2])
                        print(f"Sent Angles {int(a1)}, {int(a2)}, {int(a3)}")

                    elif msg == "EXIT_MANUAL":
                        log("Exiting manual mode. Returning to IDLE.")
                        state = RobotState.IDLE
                        serial.send_message("AUTOMATIC")
                        serial.wait_for_ack("EXIT_MANUAL")
                        controller.go_to_pos(move_pos = (0, 0, -305), abort_flag=abort_flag)
                        controller.go_to_pos(move_pos = (-120, 80, -305), abort_flag=abort_flag)

                    else:
                        log(f"Unknown manual message: {msg}")

                time.sleep(0.05)  # Prevent tight loop


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
