import sys
import time
import socket
import json

# Try to import pygame, exit with helpful message if missing
try:
    import pygame
except ImportError:
    print("Error: Pygame is not installed.")
    print("Please run: pip install pygame")
    sys.exit(1)

def main():
    pygame.init()
    pygame.joystick.init()

    # Check for connected controllers
    count = pygame.joystick.get_count()
    if count == 0:
        print("No game controllers detected. Please connect your PS5 controller via Bluetooth/USB.")
        # We will loop and wait for a controller
        while pygame.joystick.get_count() == 0:
            time.sleep(1.0)
            pygame.joystick.init()
        count = pygame.joystick.get_count()

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Connected to controller: {js.get_name()}")
    print("Streaming inputs to local port 5005 via UDP...")

    # UDP Setup
    udp_ip = "127.0.0.1"
    udp_port = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    clock = pygame.time.Clock()
    running = True

    try:
        while running:
            # Handle pygame events to refresh joystick state
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Read all axes
            num_axes = js.get_numaxes()
            axes = [round(js.get_axis(i), 3) for i in range(num_axes)]

            # Read all buttons
            num_buttons = js.get_numbuttons()
            buttons = [js.get_button(i) for i in range(num_buttons)]

            # Read all hats (D-pad)
            num_hats = js.get_numhats()
            hats = []
            for i in range(num_hats):
                hat = js.get_hat(i)
                hats.extend([hat[0], hat[1]]) # add x and y

            # Build data packet
            packet = {
                "axes": axes,
                "buttons": buttons,
                "hats": hats
            }

            # Serialize and send
            data_str = json.dumps(packet)
            sock.sendto(data_str.encode('utf-8'), (udp_ip, udp_port))

            # Print debug info to console when there is active input
            has_active_axis = any(abs(a) > 0.15 for a in axes)
            has_active_btn = any(b == 1 for b in buttons)
            has_active_hat = any(h != 0 for h in hats)
            if has_active_axis or has_active_btn or has_active_hat:
                print(f"[DEBUG_CONTROLLER] Axes: {axes} | Buttons: {buttons} | Hats: {hats}")

            # Limit rate to 50Hz (20ms) for responsive, low-latency control
            clock.tick(50)

    except KeyboardInterrupt:
        print("\nExiting controller script.")
    finally:
        js.quit()
        pygame.quit()

if __name__ == "__main__":
    main()
