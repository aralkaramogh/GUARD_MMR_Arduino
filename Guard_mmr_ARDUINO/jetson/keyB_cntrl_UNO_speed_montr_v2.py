import serial
import curses
import time
import sys
import re

# ===== Configuration =====
# Check your port name using 'ls /dev/tty*' on Jetson
# It is usually /dev/ttyACM0 for Arduino Uno
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200

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
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2) # Wait for Arduino auto-reset to finish
        stdscr.addstr(10, 0, f"Status: Connected to {SERIAL_PORT}      ")
    except Exception as e:
        stdscr.addstr(10, 0, f"Error: {str(e)}")
        stdscr.addstr(12, 0, "Press any key to exit.")
        stdscr.nodelay(False)
        stdscr.getch()
        return

    last_key = -1
    last_cmd_time = 0
    # Parsed current speeds (updated from Arduino serial output)
    forward_speed = None
    turn_speed = None
    last_motion_speed = None
    
    while True:
        try:
            # --- Read inbound serial lines (non-blocking) ---
            try:
                while ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        break
                    # Show raw message (trimmed)
                    stdscr.addstr(14, 0, f"Arduino: {line[:70]:70}  ")
                    # Parse common messages for current speeds
                    m = re.search(r'Forward/Backward Speed[^0-9-]*(\-?\d+)', line)
                    if m:
                        forward_speed = int(m.group(1))
                    m = re.search(r'Initial Forward/Backward Speed[^0-9-]*(\-?\d+)', line)
                    if m:
                        forward_speed = int(m.group(1))
                    m = re.search(r'Turning Speed[^0-9-]*(\-?\d+)', line)
                    if m:
                        turn_speed = int(m.group(1))
                    m = re.search(r'Speed:\s*(\-?\d+)', line)
                    if m:
                        last_motion_speed = int(m.group(1))
                    # Parse Reset messages like: "Reset: Forward/Backward and Turning speeds reset to 10."
                    m = re.search(r'Reset.*reset to\s*(\d+)', line, re.IGNORECASE)
                    if m:
                        val = int(m.group(1))
                        forward_speed = val
                        turn_speed = val
                        last_motion_speed = None
                    # Update the compact speed display
                    stdscr.addstr(16, 0, f"Fwd Speed: {forward_speed if forward_speed is not None else '--':>3}%   Turn: {turn_speed if turn_speed is not None else '--':>3}%   ")
                    if last_motion_speed is not None:
                        stdscr.addstr(17, 0, f"Last Cmd Speed: {last_motion_speed:>3}   ")
                    stdscr.refresh()
            except Exception:
                pass

            key = stdscr.getch()
            
            # Reduce CPU usage
            if key == -1:
                time.sleep(0.05)
                continue

            cmd = None
            status_text = ""

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

            # Send to Arduino
            if cmd:
                # Send encoding
                ser.write(cmd.encode())
                
                # Update UI
                stdscr.addstr(12, 0, f"Last Command: {status_text} ({cmd})      ")
                stdscr.refresh()

                # Try to capture any immediate response (useful for Reset which prints the new speeds)
                timeout = time.time() + 0.5  # wait up to 0.5s for reply
                while time.time() < timeout:
                    if ser.in_waiting == 0:
                        time.sleep(0.01)
                        continue
                    try:
                        response = ser.readline().decode('utf-8', errors='ignore').strip()
                    except Exception:
                        break
                    if not response:
                        continue
                    stdscr.addstr(14, 0, f"Arduino: {response[:70]:70}  ")
                    # Parse same messages we handle in the inbound block
                    m = re.search(r'Forward/Backward Speed[^0-9-]*(\-?\d+)', response)
                    if m:
                        forward_speed = int(m.group(1))
                    m = re.search(r'Initial Forward/Backward Speed[^0-9-]*(\-?\d+)', response)
                    if m:
                        forward_speed = int(m.group(1))
                    m = re.search(r'Turning Speed[^0-9-]*(\-?\d+)', response)
                    if m:
                        turn_speed = int(m.group(1))
                    m = re.search(r'Speed:\s*(\-?\d+)', response)
                    if m:
                        last_motion_speed = int(m.group(1))
                    m = re.search(r'Reset.*reset to\s*(\d+)', response, re.IGNORECASE)
                    if m:
                        val = int(m.group(1))
                        forward_speed = val
                        turn_speed = val
                        last_motion_speed = None
                    # Update compact display immediately
                    stdscr.addstr(16, 0, f"Fwd Speed: {forward_speed if forward_speed is not None else '--':>3}%   Turn: {turn_speed if turn_speed is not None else '--':>3}%   ")
                    if last_motion_speed is not None:
                        stdscr.addstr(17, 0, f"Last Cmd Speed: {last_motion_speed:>3}   ")
                    stdscr.refresh()
                    # Stop early if we parsed a reset message so UI is updated quickly
                    if re.search(r'Reset.*reset to\s*(\d+)', response, re.IGNORECASE):
                        break

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