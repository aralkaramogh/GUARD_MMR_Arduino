import serial
import curses
import time
import sys
import re

# ===== CRITICAL FIX =====
# Root cause: Curses nodelay(True) returns a key ONCE when pressed, then -1 on holds.
# Previous timeout-based release was killing the motor after 0.15s of held key.
# 
# NEW APPROACH: Keep REPEATING the last command continuously until a NEW key is pressed.
# This makes the motor spin smoothly as long as the key is held, matching user expectation.

# ===== Configuration =====
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
COMMAND_REPEAT_RATE = 0.05  # Send same command every 50ms to keep motor running

def main(stdscr):
    # Setup the screen for keyboard input
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.clear()
    
    # UI Header
    stdscr.addstr(0, 0, "=== Jetson to Arduino Uno Controller (Fixed) ===")
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
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        time.sleep(2)
        stdscr.addstr(10, 0, f"Status: Connected to {SERIAL_PORT}      ")
    except Exception as e:
        stdscr.addstr(10, 0, f"Error: {str(e)}")
        stdscr.addstr(12, 0, "Press any key to exit.")
        stdscr.nodelay(False)
        stdscr.getch()
        return

    # Speed tracking from Arduino
    forward_speed = None
    turn_speed = None
    last_motion_speed = None
    
    # CONTINUOUS COMMAND SYSTEM
    current_command = None  # The command being held (kept repeating)
    last_sent_time = 0
    
    while True:
        try:
            current_time = time.time()
            
            # --- Read serial data (non-blocking) ---
            try:
                if ser.in_waiting > 0:
                    raw_data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                    if raw_data:
                        lines = raw_data.split('\n')
                        for line in lines:
                            line = line.strip()
                            if not line:
                                continue
                            
                            # Parse speed messages
                            m = re.search(r'Forward/Backward Speed[^0-9-]*(\-?\d+)', line)
                            if m: forward_speed = int(m.group(1))
                            m = re.search(r'Initial Forward/Backward Speed[^0-9-]*(\-?\d+)', line)
                            if m: forward_speed = int(m.group(1))
                            m = re.search(r'Turning Speed[^0-9-]*(\-?\d+)', line)
                            if m: turn_speed = int(m.group(1))
                            m = re.search(r'Speed:\s*(\-?\d+)', line)
                            if m: last_motion_speed = int(m.group(1))
                            m = re.search(r'Reset.*reset to\s*(\d+)', line, re.IGNORECASE)
                            if m:
                                val = int(m.group(1))
                                forward_speed = val
                                turn_speed = val
                                last_motion_speed = None
                        
                        last_line = lines[-1].strip()
                        if last_line:
                            stdscr.addstr(14, 0, f"Arduino: {last_line[:70]:70}  ")
                        stdscr.addstr(16, 0, f"Fwd Speed: {forward_speed if forward_speed is not None else '--':>3}%   Turn: {turn_speed if turn_speed is not None else '--':>3}%   ")
                        if last_motion_speed is not None:
                            stdscr.addstr(17, 0, f"Last Cmd Speed: {last_motion_speed:>3}   ")
                        stdscr.refresh()
            except Exception:
                pass

            # --- KEY INPUT DETECTION ---
            key = stdscr.getch()
            status_text = ""
            
            # Only process when a NEW key is pressed (key != -1)
            if key != -1:
                new_command = None
                
                # Movement keys
                if key == ord('w') or key == curses.KEY_UP:
                    new_command = 'w'
                    status_text = "FORWARD"
                elif key == ord('s') or key == curses.KEY_DOWN:
                    new_command = 's'
                    status_text = "BACKWARD"
                elif key == ord('a') or key == curses.KEY_LEFT:
                    new_command = 'a'
                    status_text = "LEFT TURN"
                elif key == ord('d') or key == curses.KEY_RIGHT:
                    new_command = 'd'
                    status_text = "RIGHT TURN"
                
                # Stop commands
                elif key == ord(' ') or key == ord('x'):
                    new_command = 'x'
                    status_text = "STOP"
                
                # Speed controls
                elif key == ord('q'):
                    new_command = 'q'
                    status_text = "FWD SPEED UP"
                elif key == ord('z'):
                    new_command = 'z'
                    status_text = "FWD SPEED DOWN"
                elif key == ord('e'):
                    new_command = 'e'
                    status_text = "TURN SPEED UP"
                elif key == ord('c'):
                    new_command = 'c'
                    status_text = "TURN SPEED DOWN"
                
                # Reset
                elif key == ord('h'):
                    new_command = 'h'
                    status_text = "RESET (Speeds -> 10)"
                
                # Quit
                elif key == 27:  # ESC
                    ser.write(b'x')
                    break
                
                # Update current command (ONLY on new key press)
                if new_command:
                    current_command = new_command
                    last_sent_time = 0  # Force immediate send
                    stdscr.addstr(12, 0, f"Last Command: {status_text} ({new_command})      ")
                    stdscr.refresh()

            # --- CONTINUOUS COMMAND SENDING ---
            # Keep sending the current command repeatedly (essential for smooth motor control!)
            if current_command and (current_time - last_sent_time > COMMAND_REPEAT_RATE):
                ser.write(current_command.encode())
                last_sent_time = current_time
            
            # Minimal sleep to prevent CPU spin
            time.sleep(0.001)

        except Exception as e:
            stdscr.addstr(16, 0, f"Runtime Error: {str(e)}")
            break

    # Cleanup
    if ser and ser.is_open:
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