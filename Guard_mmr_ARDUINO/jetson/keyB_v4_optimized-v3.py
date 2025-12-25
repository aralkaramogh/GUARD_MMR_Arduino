import serial
import curses
import time
import sys
import re

# ===== Changes in this script =====
# 1. Added KEY_TIMEOUT constant to define when to auto-stop after key release.
# 2. Added last_valid_key_time and is_moving state to track held keys.
# 3. Implemented logic to detect key release (if current_time - last_valid_key_time > KEY_TIMEOUT).
# 4. Added curses.flushinp() after sending commands to clear the input buffer and prevent lag.
# 5. Changed serial read to read all waiting bytes (ser.read(ser.in_waiting)) instead of line-by-line to prevent buffer build-up.
# 6. Added logic to send 'x' (STOP) command automatically when key is released.
# 7. Increased ser.timeout slightly for connection, but kept read loops non-blocking.

# ===== Configuration =====
# Check your port name using 'ls /dev/tty*' on Jetson
# It is usually /dev/ttyACM0 for Arduino Uno
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200
KEY_TIMEOUT = 0.1 # Time in seconds to wait before auto-stopping (Hold-to-run logic)

def main(stdscr):
    # Setup the screen for keyboard input
    curses.curs_set(0)
    stdscr.nodelay(True) # Don't block waiting for input
    stdscr.clear()
    
    # UI Header
    stdscr.addstr(0, 0, "=== Jetson to Arduino Uno Controller ===")
    stdscr.addstr(2, 0, "Controls:")
    stdscr.addstr(3, 2, "WASD / Arrows : Drive")
    stdscr.addstr(4, 2, "Space / X     : STOP")
    stdscr.addstr(5, 2, "Q / Z         : Fwd/Back Speed (+/-)")
    stdscr.addstr(6, 2, "E / C         : Turn Speed (+/-)")
    stdscr.addstr(7, 2, "H             : Reset Speeds (Default 10)")
    stdscr.addstr(8, 2, "Esc           : Quit App")
    
    stdscr.addstr(10, 0, f"Connecting to {SERIAL_PORT}...")
    stdscr.refresh()

    # Connect to Arduino
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01) # Faster timeout
        time.sleep(2) # Wait for Arduino auto-reset to finish
        stdscr.addstr(10, 0, f"Status: Connected to {SERIAL_PORT}      ")
    except Exception as e:
        stdscr.addstr(10, 0, f"Error: {str(e)}")
        stdscr.addstr(12, 0, "Press any key to exit.")
        stdscr.nodelay(False)
        stdscr.getch()
        return

    last_key = -1
    # Parsed current speeds (updated from Arduino serial output)
    forward_speed = None
    turn_speed = None
    last_motion_speed = None
    
    # State variables for Hold-to-Run logic
    last_valid_key_time = 0
    is_moving = False
    
    while True:
        try:
            current_time = time.time()
            # --- Read inbound serial lines (non-blocking & fast) ---
            try:
                # Read ALL pending data to clear buffer (fixes lag)
                if ser.in_waiting > 0:
                    raw_data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                    if raw_data:
                        lines = raw_data.split('\n')
                        # Process all lines to track speed state
                        for line in lines:
                            line = line.strip()
                            if not line: continue
                            
                            # Parse common messages for current speeds
                            m = re.search(r'Forward/Backward Speed[^0-9-]*(\-?\d+)', line)
                            if m: forward_speed = int(m.group(1))
                            m = re.search(r'Initial Forward/Backward Speed[^0-9-]*(\-?\d+)', line)
                            if m: forward_speed = int(m.group(1))
                            m = re.search(r'Turning Speed[^0-9-]*(\-?\d+)', line)
                            if m: turn_speed = int(m.group(1))
                            m = re.search(r'Speed:\s*(\-?\d+)', line)
                            if m: last_motion_speed = int(m.group(1))
                            
                            # Parse Reset
                            m = re.search(r'Reset.*reset to\s*(\d+)', line, re.IGNORECASE)
                            if m:
                                val = int(m.group(1))
                                forward_speed = val
                                turn_speed = val
                                last_motion_speed = None
                        
                        # Show only the very last message to keep UI clean
                        last_line = lines[-1].strip()
                        if last_line:
                            stdscr.addstr(14, 0, f"Arduino: {last_line[:70]:70}  ")
                        
                        # Update the compact speed display
                        stdscr.addstr(16, 0, f"Fwd Speed: {forward_speed if forward_speed is not None else '--':>3}%   Turn: {turn_speed if turn_speed is not None else '--':>3}%   ")
                        if last_motion_speed is not None:
                            stdscr.addstr(17, 0, f"Last Cmd Speed: {last_motion_speed:>3}   ")
                        stdscr.refresh()
            except Exception:
                pass

            key = stdscr.getch()
            
            # Reduce CPU usage slightly, but keep it responsive
            if key == -1:
                # time.sleep(0.02) # Removed sleep here to allow fast loop for auto-stop check, rely on nodelay
                pass

            cmd = None
            status_text = ""

            # If a key is currently pressed
            if key != -1:
                last_valid_key_time = current_time
                is_moving = True
                
                # Map Keys to Characters defined in Arduino code
                if key == ord('w') or key == curses.KEY_UP:
                    cmd = 'w'
                    status_text = "FORWARD"
                elif key == ord('s') or key == curses.KEY_DOWN:
                    cmd = 's'
                    status_text = "BACKWARD"
                elif key == ord('a') or key == curses.KEY_LEFT:
                    cmd = 'a'
                    status_text = "LEFT TURN"
                elif key == ord('d') or key == curses.KEY_RIGHT:
                    cmd = 'd'
                    status_text = "RIGHT TURN"
                elif key == ord(' ') or key == ord('x'): # Spacebar or X
                    cmd = 'x'
                    status_text = "STOP"
                    is_moving = False # Explicit stop
                
                # Speed Controls (Split System)
                elif key == ord('q'):
                    cmd = 'q'
                    status_text = "FWD SPEED UP"
                elif key == ord('z'):
                    cmd = 'z'
                    status_text = "FWD SPEED DOWN"
                elif key == ord('e'):
                    cmd = 'e'
                    status_text = "TURN SPEED UP"
                elif key == ord('c'):
                    cmd = 'c'
                    status_text = "TURN SPEED DOWN"
                
                # Reset
                elif key == ord('h'):
                    cmd = 'h'
                    status_text = "RESET (Speeds -> 10)"

                # Quit App (Use Esc instead of Q)
                elif key == 27: # ESC key
                    # SAFETY: Stop robot before quitting
                    ser.write(b'x')
                    break
            
            # Logic for Auto-Stop (Release Detection)
            else:
                if is_moving and (current_time - last_valid_key_time > KEY_TIMEOUT):
                    cmd = 'x'
                    status_text = "AUTO-STOP"
                    is_moving = False

            # Send to Arduino
            if cmd:
                # Send encoding
                ser.write(cmd.encode())
                
                # CRITICAL: Flush input buffer to prevent lag/ghost keys
                curses.flushinp()
                
                # Update UI
                stdscr.addstr(12, 0, f"Last Command: {status_text} ({cmd})      ")
                stdscr.refresh()
            
            # Small sleep to prevent 100% CPU usage
            time.sleep(0.01)

        except Exception as e:
            # Handle serial disconnections gracefully
            stdscr.addstr(16, 0, f"Runtime Error: {str(e)}")
            break

    # Cleanup
    if ser and ser.is_open:
        # Final safety stop just in case loop broke unexpectedly
        try:
            ser.write(b'x') 
        except:
            pass
        ser.close()

if __name__ == "__main__":
    try:
        curses.wrapper(main)
    except Exception as e:
        print(f"Failed to initialize curses: {e}")