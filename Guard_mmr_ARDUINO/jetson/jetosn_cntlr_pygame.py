import pygame
import serial
import time
import sys
import re

# ===== Configuration =====
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200

# Screen settings (Pygame needs a window to capture keyboard focus)
SCREEN_WIDTH = 640
SCREEN_HEIGHT = 480
BG_COLOR = (30, 30, 30)
TEXT_COLOR = (200, 200, 200)
ACCENT_COLOR = (0, 255, 0)

# ===== Pygame Setup =====
pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Jetson Robot Controller")
font = pygame.font.Font(None, 32)
small_font = pygame.font.Font(None, 24)

def draw_text(surface, text, x, y, color=TEXT_COLOR, font_obj=font):
    text_surface = font_obj.render(text, True, color)
    surface.blit(text_surface, (x, y))

def main():
    # --- Connect to Arduino ---
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2) # Allow reset
        connection_status = f"Connected: {SERIAL_PORT}"
    except Exception as e:
        connection_status = f"Error: {str(e)}"
        print(connection_status)
        # We continue even if connection fails to show the UI, but commands won't send.

    # State variables
    forward_speed = 10
    turn_speed = 10
    last_cmd = "STOP"
    arduino_feedback = ""
    
    # Track keys currently held down
    keys_pressed = {
        'w': False, 's': False, 'a': False, 'd': False
    }

    running = True
    clock = pygame.time.Clock()

    while running:
        # 1. Event Handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            # --- Key Down (Press) ---
            if event.type == pygame.KEYDOWN:
                char = event.unicode.lower()
                key_code = event.key

                # Movement Keys
                if key_code == pygame.K_w: keys_pressed['w'] = True
                elif key_code == pygame.K_s: keys_pressed['s'] = True
                elif key_code == pygame.K_a: keys_pressed['a'] = True
                elif key_code == pygame.K_d: keys_pressed['d'] = True
                
                # Command Keys (Single Trigger)
                cmd_to_send = None
                
                if key_code == pygame.K_q: cmd_to_send = 'q' # Fwd Speed Up
                elif key_code == pygame.K_z: cmd_to_send = 'z' # Fwd Speed Down
                elif key_code == pygame.K_e: cmd_to_send = 'e' # Turn Speed Up
                elif key_code == pygame.K_c: cmd_to_send = 'c' # Turn Speed Down
                elif key_code == pygame.K_h: 
                    cmd_to_send = 'h' # Reset
                    forward_speed = 10
                    turn_speed = 10
                    last_cmd = "RESET"
                elif key_code == pygame.K_SPACE or key_code == pygame.K_x:
                    cmd_to_send = 'x' # Hard Stop
                    
                elif key_code == pygame.K_ESCAPE:
                    running = False

                if cmd_to_send and ser and ser.is_open:
                    ser.write(cmd_to_send.encode())

            # --- Key Up (Release) ---
            if event.type == pygame.KEYUP:
                key_code = event.key
                if key_code == pygame.K_w: keys_pressed['w'] = False
                elif key_code == pygame.K_s: keys_pressed['s'] = False
                elif key_code == pygame.K_a: keys_pressed['a'] = False
                elif key_code == pygame.K_d: keys_pressed['d'] = False

        # 2. Continuous Logic (Hold Handling)
        if ser and ser.is_open:
            current_move = None
            
            # Priority: Turning overrides Straight, Straight overrides Stop
            # (Simple logic, can be mixed if Arduino supports diagonal)
            if keys_pressed['a']: current_move = 'a'
            elif keys_pressed['d']: current_move = 'd'
            elif keys_pressed['w']: current_move = 'w'
            elif keys_pressed['s']: current_move = 's'
            else:
                # If ALL movement keys are released, send Stop (x) once
                # We check if we *were* moving to avoid spamming 'x' unnecessarily
                if last_cmd not in ["STOP", "RESET", "SPEED_CMD"]:
                    current_move = 'x'

            # Send command if it exists
            if current_move:
                ser.write(current_move.encode())
                
                # Update UI state
                if current_move == 'w': last_cmd = "FORWARD"
                elif current_move == 's': last_cmd = "BACKWARD"
                elif current_move == 'a': last_cmd = "LEFT"
                elif current_move == 'd': last_cmd = "RIGHT"
                elif current_move == 'x': last_cmd = "STOP"

        # 3. Read Serial Feedback
        if ser and ser.is_open and ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    arduino_feedback = line[:60] # Truncate for display
                    
                    # Parse Speeds from Arduino feedback
                    # "Forward/Backward Speed Increased: 20"
                    m = re.search(r'Speed.*:\s*(\-?\d+)', line)
                    if m:
                        val = int(m.group(1))
                        # Identify which speed it was based on context or just update variables based on key presses
                        # Since parsing exact lines is tricky without exact tags, 
                        # we rely on the Arduino confirming the reset or increment.
                        pass # For now, we trust the key press logic for UI updates 
                             # or parse specific strings if strict sync is needed.
                             
                    # Special Case: Sync on Reset
                    if "Reset" in line:
                        forward_speed = 10
                        turn_speed = 10

            except Exception:
                pass

        # 4. Draw UI
        screen.fill(BG_COLOR)
        
        y = 20
        draw_text(screen, "=== JETSON CONTROLLER ===", 20, y, ACCENT_COLOR)
        y += 40
        draw_text(screen, f"Status: {connection_status}", 20, y)
        y += 40
        draw_text(screen, f"Last Command: {last_cmd}", 20, y, (255, 255, 0))
        y += 40
        draw_text(screen, f"Arduino Says: {arduino_feedback}", 20, y, (150, 150, 150), small_font)
        
        # Controls Help
        y += 60
        draw_text(screen, "CONTROLS:", 20, y, ACCENT_COLOR, small_font)
        y += 25
        draw_text(screen, "WASD - Move (Hold)", 20, y, TEXT_COLOR, small_font)
        y += 25
        draw_text(screen, "Q / Z - Fwd Speed (+/-)", 20, y, TEXT_COLOR, small_font)
        y += 25
        draw_text(screen, "E / C - Turn Speed (+/-)", 20, y, TEXT_COLOR, small_font)
        y += 25
        draw_text(screen, "H     - Reset All", 20, y, TEXT_COLOR, small_font)
        y += 25
        draw_text(screen, "ESC   - Quit", 20, y, TEXT_COLOR, small_font)

        pygame.display.flip()
        clock.tick(30) # Limit to 30 FPS to save CPU

    # Cleanup
    if ser and ser.is_open:
        ser.write(b'x') # Safety Stop
        ser.close()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()