import serial
import curses
import time
import sys
import re

# ===== Configuration =====
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200
KEY_TIMEOUT = 0.15 # Time (seconds) before assuming key release

def main(stdscr):
    # Setup terminal
    curses.curs_set(0)
    stdscr.nodelay(True) 
    stdscr.clear()
    
    # UI Header
    stdscr.addstr(0, 0, "=== SSH ROBOT CONTROLLER ===")
    stdscr.addstr(2, 0, "Controls (Hold to Move):")
    stdscr.addstr(3, 4, "W / A / S / D")
    stdscr.addstr(4, 0, "Commands:")
    stdscr.addstr(5, 4, "Q/Z : Fwd Speed +/-")
    stdscr.addstr(6, 4, "E/C : Turn Speed +/-")
    stdscr.addstr(7, 4, "H   : Reset")
    stdscr.addstr(8, 4, "ESC : Quit")
    
    stdscr.addstr(10, 0, f"Connecting to {SERIAL_PORT}...")
    stdscr.refresh()

    # Connect to Arduino
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
        time.sleep(2) 
        stdscr.addstr(10, 0, f"Status: Connected!              ")
    except Exception as e:
        stdscr.addstr(10, 0, f"Error: {str(e)}")
        stdscr.addstr(12, 0, "Press any key to exit.")
        stdscr.nodelay(False)
        stdscr.getch()
        return

    # State
    last_key = -1
    last_valid_key_time = 0
    is_moving = False
    last_sent_movement_cmd = None  # Track last sent movement command to avoid re-sending
    
    # Feedback variables
    fwd_speed = 10
    turn_speed = 10
    last_sent_cmd = "STOP"

    while True:
        try:
            # 1. Read Serial Feedback (Non-blocking)
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        # Parse Speeds
                        m = re.search(r'Speed.*:\s*(\-?\d+)', line)
                        if m:
                            # Just a visual update, logic handled by Arduino
                            pass
                        
                        # Show raw feedback
                        stdscr.addstr(14, 0, f"Arduino: {line[:60]:60}")
                except:
                    pass

            # 2. Read Keyboard
            key = stdscr.getch()
            current_time = time.time()
            
            cmd_to_send = None
            status_text = ""
            is_movement_cmd = False  # Track if this is a movement command

            # --- logic for Hold-to-Run ---
            # If a key is pressed
            if key != -1:
                last_valid_key_time = current_time
                is_moving = True
                
                # Movement
                if key == ord('w') or key == curses.KEY_UP:
                    cmd_to_send = 'w'
                    status_text = "FORWARD"
                    is_movement_cmd = True
                elif key == ord('s') or key == curses.KEY_DOWN:
                    cmd_to_send = 's'
                    status_text = "BACKWARD"
                    is_movement_cmd = True
                elif key == ord('a') or key == curses.KEY_LEFT:
                    cmd_to_send = 'a'
                    status_text = "LEFT"
                    is_movement_cmd = True
                elif key == ord('d') or key == curses.KEY_RIGHT:
                    cmd_to_send = 'd'
                    status_text = "RIGHT"
                    is_movement_cmd = True
                
                # Instant Commands (Speed/Reset) - Always send these
                elif key == ord('q'): cmd_to_send = 'q'; status_text = "SPEED UP"
                elif key == ord('z'): cmd_to_send = 'z'; status_text = "SPEED DOWN"
                elif key == ord('e'): cmd_to_send = 'e'; status_text = "TURN UP"
                elif key == ord('c'): cmd_to_send = 'c'; status_text = "TURN DOWN"
                elif key == ord('h'): cmd_to_send = 'h'; status_text = "RESET"
                
                # Stop/Quit
                elif key == ord(' ') or key == ord('x'): 
                    cmd_to_send = 'x'
                    status_text = "STOP"
                    is_moving = False # Explicit stop
                elif key == 27: # ESC
                    ser.write(b'x')
                    break

            # --- Logic for Auto-Stop (Release Detection) ---
            else:
                # If no key pressed for KEY_TIMEOUT seconds, and we were moving, STOP.
                if is_moving and (current_time - last_valid_key_time > KEY_TIMEOUT):
                    cmd_to_send = 'x'
                    status_text = "AUTO-STOP"
                    is_moving = False
                    last_sent_movement_cmd = None  # Clear cache on stop

            # 3. Send Command - Only send if: (a) it's a new movement, (b) it's NOT a movement (speed/reset), or (c) it's stop
            send_now = False
            if cmd_to_send:
                if is_movement_cmd:
                    # Only send if this movement is different from last sent movement
                    if cmd_to_send != last_sent_movement_cmd:
                        send_now = True
                        last_sent_movement_cmd = cmd_to_send
                else:
                    # Non-movement commands (speed, reset, stop) are always sent
                    send_now = True
                    last_sent_movement_cmd = None  # Reset movement cache on non-movement cmd

            if send_now:
                ser.write(cmd_to_send.encode())
                last_sent_cmd = f"{status_text} ({cmd_to_send})"
                # Clear input buffer to prevent lag build-up
                curses.flushinp() 

            # 4. Update UI
            stdscr.addstr(12, 0, f"Last Cmd: {last_sent_cmd:20}")
            stdscr.refresh()
            
            # Small sleep to prevent CPU hogging
            time.sleep(0.02)

        except Exception as e:
            stdscr.addstr(16, 0, f"Error: {str(e)}")
            break

    # Cleanup
    if ser and ser.is_open:
        ser.write(b'x')
        ser.close()

if __name__ == "__main__":
    try:
        curses.wrapper(main)
    except Exception as e:
        print(f"Error: {e}")