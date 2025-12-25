import serial
import curses
import time
import sys
import re

# ===== Key Optimizations =====
# 1. KEY STATE TRACKING: Only send commands on state change (press/release), not every frame
# 2. COMMAND DEDUPLICATION: Track last_command_sent to prevent duplicate sends
# 3. REDUCED SERIAL WRITES: Only write when command actually changes
# 4. IMPROVED AUTO-STOP: Better timing logic for smooth release detection
# 5. FASTER LOOP: Removed unnecessary sleep, uses optimized timing instead
# 6. CLEANER STATE MACHINE: Separated "key input" from "command dispatch"

# ===== Configuration =====
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
KEY_RELEASE_TIMEOUT = 0.15  # Time before auto-stop after key release
KEY_REPEAT_COOLDOWN = 0.05  # Minimum time between sending same command (for speed controls)

def main(stdscr):
    # Setup the screen for keyboard input
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.clear()
    
    # UI Header
    stdscr.addstr(0, 0, "=== Jetson to Arduino Uno Controller (Optimized) ===")
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
    
    # KEY STATE TRACKING (CRITICAL FIX)
    current_key = None  # Currently pressed key
    prev_key = None     # Previously pressed key
    last_key_time = time.time()
    
    # COMMAND TRACKING
    last_command_sent = None  # Prevents sending duplicate commands
    last_command_time = 0
    
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
            
            # Store current key state
            if key != -1:
                current_key = key
                last_key_time = current_time
            else:
                # Check for timeout-based key release
                if current_key is not None and (current_time - last_key_time > KEY_RELEASE_TIMEOUT):
                    current_key = None

            # --- COMMAND GENERATION (Only on state change) ---
            cmd = None
            status_text = ""
            
            # Only process if key state changed (press or release)
            if current_key != prev_key:
                
                if current_key is not None:
                    # KEY PRESS
                    if current_key == ord('w') or current_key == curses.KEY_UP:
                        cmd = 'w'
                        status_text = "FORWARD"
                    elif current_key == ord('s') or current_key == curses.KEY_DOWN:
                        cmd = 's'
                        status_text = "BACKWARD"
                    elif current_key == ord('a') or current_key == curses.KEY_LEFT:
                        cmd = 'a'
                        status_text = "LEFT TURN"
                    elif current_key == ord('d') or current_key == curses.KEY_RIGHT:
                        cmd = 'd'
                        status_text = "RIGHT TURN"
                    elif current_key == ord(' ') or current_key == ord('x'):
                        cmd = 'x'
                        status_text = "STOP"
                    elif current_key == ord('q'):
                        cmd = 'q'
                        status_text = "FWD SPEED UP"
                    elif current_key == ord('z'):
                        cmd = 'z'
                        status_text = "FWD SPEED DOWN"
                    elif current_key == ord('e'):
                        cmd = 'e'
                        status_text = "TURN SPEED UP"
                    elif current_key == ord('c'):
                        cmd = 'c'
                        status_text = "TURN SPEED DOWN"
                    elif current_key == ord('h'):
                        cmd = 'h'
                        status_text = "RESET (Speeds -> 10)"
                    elif current_key == 27:  # ESC
                        ser.write(b'x')
                        break
                
                else:
                    # KEY RELEASE - Send stop command
                    cmd = 'x'
                    status_text = "AUTO-STOP (Release)"
                
                prev_key = current_key

            # --- COMMAND DISPATCH (Deduplication) ---
            if cmd:
                # For movement (w, a, s, d): Send only once per press
                # For speed controls (q, z, e, c): Allow repeated sends after cooldown
                is_speed_control = cmd in ['q', 'z', 'e', 'c']
                
                should_send = (last_command_sent != cmd) or \
                              (is_speed_control and (current_time - last_command_time > KEY_REPEAT_COOLDOWN))
                
                if should_send:
                    ser.write(cmd.encode())
                    last_command_sent = cmd
                    last_command_time = current_time
                    
                    # Update UI
                    stdscr.addstr(12, 0, f"Last Command: {status_text} ({cmd})      ")
                    stdscr.refresh()
            
            # Minimal sleep to prevent CPU spin (10ms = responsive enough for motor control)
            time.sleep(0.005)

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