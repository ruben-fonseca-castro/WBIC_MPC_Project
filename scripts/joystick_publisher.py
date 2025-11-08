import time
import lcm
from inputs import get_gamepad, UnpluggedError 
import os

# Import the new LCM type
# Note: This assumes you ran 'export PYTHONPATH=$PYTHONPATH:/home/davidho/arc-bridge'
from arc_bridge.lcm_msgs import xbox_command_t

# --- Configuration ---
XBOX_COMMAND_CHANNEL = "XBOX_COMMAND"
MAX_JOY_VAL = 32768.0
MAX_TRIG_VAL = 255.0

def main():
    print("Finding gamepad...")
    try:
        get_gamepad()
    except UnpluggedError:  # <-- FIXED EXCEPTION
        print("Error: No gamepad found. Plug one in and re-run.")
        print("Note: You may need to press a button on the controller for it to be detected.")
        return

    print("Gamepad found. Starting publisher...")
    
    lc = lcm.LCM()
    msg = xbox_command_t()
    
    # Set deadzone for sticks
    deadzone = 0.1
    
    try:
        while True:
            # Read all available events
            events = get_gamepad()
            if not events:
                time.sleep(0.01) # No new events, sleep briefly
                continue

            for event in events:
                if event.ev_type == 'Absolute':
                    val = event.state / MAX_JOY_VAL
                    if abs(val) < deadzone:
                        val = 0.0
                        
                    if event.code == 'ABS_X':
                        msg.left_stick_x = val
                    elif event.code == 'ABS_Y':
                        msg.left_stick_y = val
                    elif event.code == 'ABS_RX':
                        msg.right_stick_x = val
                    elif event.code == 'ABS_RY':
                        msg.right_stick_y = val
                        
                elif event.ev_type == 'Key':
                    val = event.state
                    if event.code == 'BTN_SOUTH': # A
                        msg.a = val
                    elif event.code == 'BTN_EAST': # B
                        msg.b = val
                    elif event.code == 'BTN_WEST': # X
                        msg.x = val
                    elif event.code == 'BTN_NORTH': # Y
                        msg.y = val
                    elif event.code == 'BTN_TL': # LB
                        msg.lb = val
                    elif event.code == 'BTN_TR': # RB
                        msg.rb = val

            # Publish the message with the latest state
            msg.timestamp = int(time.time() * 1_000_000) # Microseconds
            lc.publish(XBOX_COMMAND_CHANNEL, msg.encode())

    except KeyboardInterrupt:
        print("Stopping publisher.")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    # This is a common fix for the 'inputs' library
    os.environ['INPUT_API'] = 'evdev'
    main()