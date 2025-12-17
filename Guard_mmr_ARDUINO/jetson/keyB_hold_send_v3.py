import serial
import curses
import time
import sys
import re

"""
================================================================================
                    HOLD & SEND CONTROLLER v3 (Smooth Start)
================================================================================

DESCRIPTION:
  This is a Jetson Nano keyboard controller for Arduino-based robot movement.
  It implements a "Hold-to-Move" paradigm: movement commands are sent ONLY when
  the user holds down a movement key (W/A/S/D). Once the key is released, an
  immediate STOP command is sent. This eliminates start-stop jerk seen in repeat
  send systems.

HOW IT WORKS:
  1. Key Press Detection:
     - When W/A/S/D is pressed (and not already active), the movement command
       is sent to Arduino immediately.
     - The system tracks currently_moving_cmd to know which direction is active.
     - If the same key is pressed again, NO command is re-sent (smooth continuity).

  2. Key Release Detection:
     - When NO key is pressed (getch() returns -1), the system detects key release.
     - If a movement was active, a STOP ('x') command is sent immediately.
     - This creates smooth deceleration without jerk.

  3. Speed & Reset Controls:
     - Instant commands (Q/Z/E/C/H for speed adjust and reset) are always sent.
     - These do NOT interrupt ongoing movement.
     - Speed feedback is captured and displayed on screen.

  4. Serial Feedback:
     - Continuously reads Arduino output to parse current speeds and motion state.
     - Displays Fwd Speed, Turn Speed, and Last Cmd Speed on the UI.
     - Parses Reset messages to update both speeds atomically.

CONTROL SCHEME:
  Movement (Hold Down to Move):
    W / Up Arrow    -> Move FORWARD (hold down)
    S / Dn Arrow    -> Move BACKWARD (hold down)
    A / Lt Arrow    -> Turn LEFT (hold down)
    D / Rt Arrow    -> Turn RIGHT (hold down)
    
  Speed Adjustment (Instant, works mid-motion):
    Q               -> Increase Fwd/Back speed (+5%)
    Z               -> Decrease Fwd/Back speed (-5%)
    E               -> Increase Turn speed (+5%)
    C               -> Decrease Turn speed (-5%)
    
  Special Commands:
    H               -> Reset speeds to default (10%)
    Space / X       -> Manual STOP (one-time)
    Esc             -> Quit application (safety stop first)

SMOOTHNESS FEATURES:
  - Movement command sent ONLY on key press state change (not repeatedly).
  - Stop sent immediately on key release (no artificial delay).
  - Speed changes don't interrupt motion.
  - No initial jerk because we never send the same movement command twice.

UI DISPLAY:
  Line 12: Last Command sent (shows what was transmitted)
  Line 14: Raw Arduino feedback (latest message from board)
  Line 16: Current Speeds (Fwd % and Turn %)
  Line 17: Last motion speed executed by robot

HARDWARE:
  - Arduino Uno with 4 motors controlled via JKBLD300 drivers.
  - Serial connection at 115200 baud.
  - Default serial port: /dev/ttyACM0 (change SERIAL_PORT if needed).

================================================================================
"""


# ===== Configuration =====
SERIAL_PORT = '/dev/ttyACM0'  # Serial port for Arduino connection
BAUD_RATE = 115200           # Baud rate (must match Arduino)


def main(stdscr):
    # ===== MAIN CONTROLLER FUNCTION =====
    # Setup curses (terminal UI library) for keyboard input capture
    curses.curs_set(0)
    stdscr.nodelay(True)  # Non-blocking getch() so loop doesn't freeze waiting for key
    stdscr.clear()
    
    # ===== DISPLAY UI HEADER =====
    # Show controls and connection status on screen
    stdscr.addstr(0, 0, "=== Hold & Send Controller (Smooth Start) ===")
    stdscr.addstr(2, 0, "Controls (HOLD to move):")
    stdscr.addstr(3, 2, "WASD / Arrows : Drive (hold down)")
    stdscr.addstr(4, 2, "Space / X     : STOP (one-time)")
    stdscr.addstr(5, 2, "Q / Z         : Fwd/Back Speed (+/-)")
    stdscr.addstr(6, 2, "E / C         : Turn Speed (+/-)")
    stdscr.addstr(7, 2, "H             : Reset Speeds")
    stdscr.addstr(8, 2, "Esc           : Quit")
    
    stdscr.addstr(10, 0, f"Connecting to {SERIAL_PORT}...")
    stdscr.refresh()

    # Connect to Arduino
    ser = None
    try:
        # Open serial port with 1s timeout for read operations
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for Arduino auto-reset (bootloader delay)
        stdscr.addstr(10, 0, f"Status: Connected to {SERIAL_PORT}      ")
    except Exception as e:
        stdscr.addstr(10, 0, f"Error: {str(e)}")
        stdscr.addstr(12, 0, "Press any key to exit.")
        stdscr.nodelay(False)
        stdscr.getch()
        return

    # ===== STATE VARIABLES =====
    # Track the currently active movement command to avoid re-sending the same one
    # (This is the KEY to smooth motion - no repeated "forward" commands)
    currently_moving_cmd = None  # Currently active movement command (w/s/a/d) or None
    
    # ===== PARSED SPEEDS FROM ARDUINO FEEDBACK =====
    # These are updated as Arduino sends serial messages about speed changes
    forward_speed = None   # Current forward/backward speed %
    turn_speed = None      # Current turning speed %
    last_motion_speed = None  # Last speed used for motion command
    
    while True:
        try:
            # ========================================
            # PHASE 1: READ SERIAL FEEDBACK (Non-blocking)
            # ========================================
            # This continuously reads any pending Arduino messages and updates the UI.
            # Parse for speed values and motion confirmations.
            try:
                while ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        break
                    # Display raw message on screen for debugging
                    stdscr.addstr(14, 0, f"Arduino: {line[:70]:70}  ")
                    
                    # PARSE: Forward/Backward Speed (captured when speed changes)
                    # Example: "Forward/Backward Speed Increased: 15"
                    m = re.search(r'Forward/Backward Speed[^0-9-]*(\-?\d+)', line)
                    if m:
                        forward_speed = int(m.group(1))
                    
                    # PARSE: Initial speed report (on startup)
                    # Example: "Initial Forward/Backward Speed (user): 10"
                    m = re.search(r'Initial Forward/Backward Speed[^0-9-]*(\-?\d+)', line)
                    if m:
                        forward_speed = int(m.group(1))
                    
                    # PARSE: Turning Speed (captured when speed changes)
                    # Example: "Turning Speed Increased: 20"
                    m = re.search(r'Turning Speed[^0-9-]*(\-?\d+)', line)
                    if m:
                        turn_speed = int(m.group(1))
                    
                    # PARSE: Last motion speed (from Cmd output)
                    # Example: "Cmd: Forward | Speed: 10"
                    m = re.search(r'Speed:\s*(\-?\d+)', line)
                    if m:
                        last_motion_speed = int(m.group(1))
                    
                    # PARSE: Reset message (sets both speeds at once)
                    # Example: "Reset: Forward/Backward and Turning speeds reset to 10."
                    m = re.search(r'Reset.*reset to\s*(\d+)', line, re.IGNORECASE)
                    if m:
                        val = int(m.group(1))
                        forward_speed = val
                        turn_speed = val
                        last_motion_speed = None
                    
                    # UPDATE UI: Display parsed speeds on screen
                    stdscr.addstr(16, 0, f"Fwd Speed: {forward_speed if forward_speed is not None else '--':>3}%   Turn: {turn_speed if turn_speed is not None else '--':>3}%   ")
                    if last_motion_speed is not None:
                        stdscr.addstr(17, 0, f"Last Cmd Speed: {last_motion_speed:>3}   ")
                    stdscr.refresh()
            except Exception:
                pass

            # ========================================
            # PHASE 2: READ KEYBOARD INPUT (Non-blocking)
            # ========================================
            # Detect key presses. Returns -1 if no key pressed (non-blocking).
            key = stdscr.getch()
            
            if key == -1:
                # ========================================
                # NO KEY PRESSED: DETECT RELEASE (Smooth Stop)
                # ========================================
                # When getch() returns -1, no key is currently held.
                # If we were previously moving, send STOP command now.
                if currently_moving_cmd is not None:
                    ser.write(b'x')  # Send stop command
                    stdscr.addstr(12, 0, f"Last Command: KEY RELEASED -> STOP      ")
                    currently_moving_cmd = None  # Clear active command
                stdscr.refresh()
                time.sleep(0.05)  # Reduce CPU usage during idle wait
                continue

            # ========================================
            # KEY IS PRESSED: MAP TO COMMAND
            # ========================================
            # Prepare variables for command logic
            cmd_to_send = None
            status_text = ""
            is_movement = False

            # --- MOVEMENT COMMANDS (W/A/S/D) ---
            # These are sent ONLY when the key changes state (e.g., idle->W, W->A, etc.)
            # This avoids repeated "forward" commands and ensures smooth continuous motion.
            if key == ord('w') or key == curses.KEY_UP:
                # Check if this is a NEW movement (not already moving forward)
                if currently_moving_cmd != 'w':
                    cmd_to_send = 'w'
                    status_text = "FORWARD (hold)"
                    is_movement = True
                    currently_moving_cmd = 'w'
            elif key == ord('s') or key == curses.KEY_DOWN:
                if currently_moving_cmd != 's':
                    cmd_to_send = 's'
                    status_text = "BACKWARD (hold)"
                    is_movement = True
                    currently_moving_cmd = 's'
            elif key == ord('a') or key == curses.KEY_LEFT:
                if currently_moving_cmd != 'a':
                    cmd_to_send = 'a'
                    status_text = "LEFT TURN (hold)"
                    is_movement = True
                    currently_moving_cmd = 'a'
            elif key == ord('d') or key == curses.KEY_RIGHT:
                if currently_moving_cmd != 'd':
                    cmd_to_send = 'd'
                    status_text = "RIGHT TURN (hold)"
                    is_movement = True
                    currently_moving_cmd = 'd'
            
            # --- INSTANT COMMANDS (Q/Z/E/C/H) ---
            # Speed and reset commands are ALWAYS sent (no caching).
            # These work even during movement (speed change on-the-fly).
            elif key == ord('q'):
                cmd_to_send = 'q'
                status_text = "FWD SPEED UP"
                currently_moving_cmd = None  # Clear movement cache
            elif key == ord('z'):
                cmd_to_send = 'z'
                status_text = "FWD SPEED DOWN"
                currently_moving_cmd = None
            elif key == ord('e'):
                cmd_to_send = 'e'
                status_text = "TURN SPEED UP"
                currently_moving_cmd = None
            elif key == ord('c'):
                cmd_to_send = 'c'
                status_text = "TURN SPEED DOWN"
                currently_moving_cmd = None
            elif key == ord('h'):
                cmd_to_send = 'h'
                status_text = "RESET (Speeds -> 10)"
                currently_moving_cmd = None
            
            # --- STOP & QUIT ---
            elif key == ord(' ') or key == ord('x'):
                cmd_to_send = 'x'
                status_text = "STOP (one-time)"
                currently_moving_cmd = None
            elif key == 27:  # ESC key
                # SAFETY: Always stop robot before quitting application
                ser.write(b'x')
                break

            # ========================================
            # PHASE 3: SEND COMMAND TO ARDUINO
            # ========================================
            # Only send if a command was mapped above.
            if cmd_to_send:
                ser.write(cmd_to_send.encode())
                stdscr.addstr(12, 0, f"Last Command: {status_text} ({cmd_to_send})      ")
                
                # Try to capture immediate response (Reset, speed changes)
                if not is_movement:  # Only wait for non-movement commands
                    timeout = time.time() + 0.3
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
                        # Parse response
                        m = re.search(r'Forward/Backward Speed[^0-9-]*(\-?\d+)', response)
                        if m:
                            forward_speed = int(m.group(1))
                        m = re.search(r'Turning Speed[^0-9-]*(\-?\d+)', response)
                        if m:
                            turn_speed = int(m.group(1))
                        m = re.search(r'Reset.*reset to\s*(\d+)', response, re.IGNORECASE)
                        if m:
                            val = int(m.group(1))
                            forward_speed = val
                            turn_speed = val
                            last_motion_speed = None
                        # Update display
                        stdscr.addstr(16, 0, f"Fwd Speed: {forward_speed if forward_speed is not None else '--':>3}%   Turn: {turn_speed if turn_speed is not None else '--':>3}%   ")
                        if last_motion_speed is not None:
                            stdscr.addstr(17, 0, f"Last Cmd Speed: {last_motion_speed:>3}   ")
                        stdscr.refresh()
                        # Exit early on reset message
                        if re.search(r'Reset.*reset to\s*(\d+)', response, re.IGNORECASE):
                            break

            stdscr.refresh()
            time.sleep(0.02)  # Small sleep to reduce CPU usage

        except Exception as e:
            # Handle serial disconnections gracefully
            stdscr.addstr(18, 0, f"Runtime Error: {str(e)}")
            break

    # Cleanup
    if ser and ser.is_open:
        # Final safety stop
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
