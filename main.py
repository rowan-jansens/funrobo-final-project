# main.py
"""
Main Application Script
----------------------------
Coordinates gamepad input and robot control.
"""

import sys, os
import time
import threading
import traceback
import numpy as np

# Extend system path to include script directory
sys.path.append(os.path.join(os.getcwd(), 'scripts'))

from hiwonder import HiwonderRobot
from gamepad_control import GamepadControl
from utils import EndEffector


# Initialize components
cmdlist = []    # Stores recent gamepad commands
gpc = GamepadControl()
robot = HiwonderRobot()


def monitor_gamepad():
    """ Continuously reads gamepad inputs and stores the latest command. """
    try:
        while True:
            if len(cmdlist) > 2:
                cmdlist.pop(0)  # Retain only the latest two commands
            cmdlist.append(gpc.get_gamepad_cmds())
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("[INFO] Gamepad monitoring stopped.")


def shutdown_robot():
    print("\n[INFO] Shutting down the robot safely...")

    # Stop motors and reset servos to a safe position
    # robot.stop_motors()
    robot.set_joint_values(robot.home_position, duration=600)
    time.sleep(1.5)  # Allow time for servos to reposition

    # Close communication interfaces
    print("[INFO] Closing hardware interfaces...")
    # robot.board.close()
    # robot.servo_bus.close()

    print("[INFO] Shutdown complete. Safe to power off.")


def read_angles_from_file(filepath):
    """Read joint angles from a text file."""
    angles_list = []
    try:
        with open(filepath, 'r') as f:
            for line in f:
                # Parse 6 angles from each line
                angles = [float(x) for x in line.strip().split(',')]
                if len(angles) != 6:
                    raise ValueError(f"Each line must contain 6 angles: {line}")
                angles_list.append(angles)
        return angles_list
    except Exception as e:
        print(f"Error reading angles file: {e}")
        return None


def main():
    """ Main loop that reads gamepad commands and updates the robot accordingly. """
    try:
        # Home the robot first
        robot.set_joint_values(robot.home_position, duration=600)
        time.sleep(1.5)  # Allow time for servos to reposition
        
        # Read and execute pre-recorded movements
        angles_file = "pick_up.csv"
        movement_delay = 0  # Time between movements in seconds
        
        print("\nExecuting pre-recorded movements...")
        angles_list = read_angles_from_file(angles_file)
        if angles_list:
            for i, angles in enumerate(angles_list):
                print(f"Movement {i+1}/{len(angles_list)}: {angles}")
                robot.set_joint_values(np.array(angles) * (11/9), duration=500)
                time.sleep(movement_delay)
                if (i == 1):
                    time.sleep(1)

        
        # Start the gamepad monitoring thread
        gamepad_thread = threading.Thread(target=monitor_gamepad, daemon=True)
        gamepad_thread.start()
        
        control_interval = 0.1  # Seconds per control cycle
        
        while True:
            cycle_start = time.time()

            if cmdlist:
                latest_cmd = cmdlist[-1]
                robot.set_robot_commands(latest_cmd)

            elapsed = time.time() - cycle_start
            remaining_time = control_interval - elapsed
            if remaining_time > 0:
                time.sleep(remaining_time)
            
    except KeyboardInterrupt:
        print("\n[INFO] Keyboard Interrupt detected. Initiating shutdown...")
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
        traceback.print_exc()
    finally:
        shutdown_robot()




if __name__ == "__main__":
    main()


