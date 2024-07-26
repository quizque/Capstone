from queue import Queue
import threading
from pygame.locals import *
import pygame
import socket
import struct
import numpy as np
import time


serial_data_queue = Queue()


# Network configuration
host = "localhost"  # Update this with the IP address or hostname of the server
port = 12345  # Update this with the port number of the server

# Create a socket object
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the server
sock.connect((host, port))


def network_thread():
    try:
        while True:
            # Receive data from the server
            data_bytes = sock.recv(36)
            data = struct.unpack("fffffffff", data_bytes)

            # Parse the quaternion
            q = list(data)
            # print(q)

            serial_data_queue.put(q)
            if serial_data_queue.qsize() > 1:
                serial_data_queue.get()

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        sock.close()


# Convert millibar to feet
def mbar_to_feet(pressure):
    return (1 - (pressure / 1013.25) ** 0.190284) * 145366.4


def main():

    # Initialize Pygame
    pygame.init()
    display = (520, 700)
    screen = pygame.display.set_mode(display)

    clock = pygame.time.Clock()

    logo = pygame.image.load("logo.png").convert_alpha()
    logo_rect = logo.get_rect(center=(display[0] // 2, display[1] // 2))
    font_large = pygame.font.SysFont("Arial", 65)

    font_header = pygame.font.SysFont("Arial", 45)
    font_small = pygame.font.SysFont("Arial", 20)

    data_rate_graph_screen = pygame.Surface((520, 250))
    data_rate_graph_screen_prev = pygame.Surface((520, 250))

    latest_data = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    time_since_last_update = time.time() - 1

    first_data = True
    ref_height = 0
    smoothed_ref_height = 0
    height_max = 0
    height_min = 0

    height_graph_screen = pygame.Surface((520, 250))
    height_graph_screen_prev = pygame.Surface((520, 250))

    G_max = 2
    G_min = -2

    G_graph_screen = pygame.Surface((520, 250))
    G_graph_screen_prev = pygame.Surface((520, 250))

    last_feet = 0

    read_GPS = False

    # Main loop
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

        if mbar_to_feet(latest_data[4]) - ref_height > height_max and not first_data:
            height_max = mbar_to_feet(latest_data[4]) - ref_height
        if mbar_to_feet(latest_data[4]) - ref_height < height_min and not first_data:
            height_min = mbar_to_feet(latest_data[4]) - ref_height

        if -latest_data[5] > G_max and not first_data:
            G_max = -latest_data[5]
        if -latest_data[5] < G_min and not first_data:
            G_min = -latest_data[5]

        if not serial_data_queue.empty():
            latest_data = serial_data_queue.get()
            time_since_last_update = time.time()

            if first_data:
                ref_height = mbar_to_feet(latest_data[4])
                first_data = False

        dt = time.time() - time_since_last_update
        if dt < 0.0001:
            dt = 0.01

        smoothed_ref_height = 0.6 * smoothed_ref_height + 0.4 * mbar_to_feet(
            latest_data[4]
        )

        screen.fill((77, 77, 77))
        screen.fill((50, 50, 50), (0, 0, 520, 250))
        screen.blit(logo, (-50, 0))

        text_surface = font_large.render("DATA TERMINAL", True, (255, 255, 255))
        screen.blit(text_surface, (50, 150))

        #########################################################################################
        # stats

        text_surface = font_header.render("RADIO DATA", True, (255, 255, 255))
        screen.blit(text_surface, (10, 250))

        text_surface = font_small.render(
            f"RSSI: {latest_data[7]} dBm", True, (255, 255, 255)
        )
        screen.blit(text_surface, (10, 300))

        text_surface = font_small.render(
            f"SNR: {latest_data[8]} dB", True, (255, 255, 255)
        )
        screen.blit(text_surface, (10, 320))

        text_surface = font_small.render(
            f"Last update: {dt:.2f}s",
            True,
            (255, 255, 255),
        )
        screen.blit(text_surface, (10, 340))

        text_surface = font_small.render(
            f"Data rate: {(42/dt):.2f}bytes/s",
            True,
            (255, 255, 255),
        )
        screen.blit(text_surface, (10, 360))

        data_rate_graph_screen_prev.blit(data_rate_graph_screen, (0, 0))
        data_rate_graph_screen.fill((50, 50, 50))
        data_rate_graph_screen.blit(data_rate_graph_screen_prev, (1, 0))
        pygame.draw.rect(
            data_rate_graph_screen,
            (0, 255, 0),
            (0, 250 - round(((42 / dt) / 4200) * 250), 3, 3),
        )
        text_surface = font_header.render(
            f"4.2 Kbytes/s",
            True,
            (255, 255, 255),
        )
        data_rate_graph_screen_prev.blit(data_rate_graph_screen, (0, 0))
        data_rate_graph_screen_prev.blit(text_surface, (310, 0))
        text_surface = font_header.render(
            f"0 bytes/s",
            True,
            (255, 255, 255),
        )
        data_rate_graph_screen_prev.blit(text_surface, (370, 200))

        # screen.blit(data_rate_graph_screen_prev, (0, 400))
        scaled_screen = pygame.transform.scale(data_rate_graph_screen_prev, (260, 120))
        screen.blit(scaled_screen, (240, 260))
        pygame.draw.rect(screen, (255, 255, 255), (240, 260, 260, 120), 1)

        pygame.draw.rect(screen, (255, 255, 255), (0, 390, 520, 3))

        #########################################################################################
        # stats

        text_surface = font_header.render("ROCKET DATA", True, (255, 255, 255))
        screen.blit(text_surface, (10, 390))

        text_surface = font_small.render(
            f"Pressure: {latest_data[4]:.2f} mbar", True, (255, 255, 255)
        )
        screen.blit(text_surface, (10, 440))

        text_surface = font_small.render(
            f"Height: {smoothed_ref_height-ref_height:.2f} ft",
            True,
            (255, 255, 255),
        )
        screen.blit(text_surface, (10, 460))

        text_surface = font_small.render(
            f"Temperature: {latest_data[6]:.2f} C",
            True,
            (255, 255, 255),
        )
        screen.blit(text_surface, (10, 480))

        height_graph_screen_prev.blit(height_graph_screen, (0, 0))
        height_graph_screen.fill((50, 50, 50))
        height_graph_screen.blit(height_graph_screen_prev, (1, 0))
        pygame.draw.rect(
            height_graph_screen,
            (0, 255, 0),
            (
                0,
                250
                - round(
                    np.interp(
                        mbar_to_feet(latest_data[4]) - smoothed_ref_height,
                        [height_min, height_max],
                        [0, 250],
                    )
                ),
                3,
                3,
            ),
        )
        text_surface = font_header.render(
            f"{height_max:.2f} ft",
            True,
            (255, 255, 255),
        )
        height_graph_screen_prev.blit(height_graph_screen, (0, 0))
        height_graph_screen_prev.blit(text_surface, (380, 0))
        text_surface = font_header.render(
            f"{height_min:.2f} ft",
            True,
            (255, 255, 255),
        )
        height_graph_screen_prev.blit(text_surface, (380, 200))

        scaled_screen = pygame.transform.scale(height_graph_screen_prev, (260, 120))
        screen.blit(scaled_screen, (240, 440))
        pygame.draw.rect(screen, (255, 255, 255), (240, 440, 260, 120), 1)

        text_surface = font_small.render(
            f"G-Force: {-latest_data[5]:.2f} G",
            True,
            (255, 255, 255),
        )
        screen.blit(text_surface, (10, 500))

        G_graph_screen_prev.blit(G_graph_screen, (0, 0))
        G_graph_screen.fill((50, 50, 50))
        G_graph_screen.blit(G_graph_screen_prev, (1, 0))
        pygame.draw.rect(
            G_graph_screen,
            (0, 255, 0),
            (
                0,
                round(
                    np.interp(
                        -latest_data[5],
                        [G_min, G_max],
                        [249, 1],
                    )
                ),
                3,
                3,
            ),
        )
        text_surface = font_header.render(
            f"{G_max:.2f} G",
            True,
            (255, 255, 255),
        )
        G_graph_screen_prev.blit(G_graph_screen, (0, 0))
        G_graph_screen_prev.blit(text_surface, (380, 0))
        text_surface = font_header.render(
            f"{G_min:.2f} G",
            True,
            (255, 255, 255),
        )
        G_graph_screen_prev.blit(text_surface, (380, 200))

        scaled_screen = pygame.transform.scale(G_graph_screen_prev, (260, 120))
        screen.blit(scaled_screen, (240, 575))
        pygame.draw.rect(screen, (255, 255, 255), (240, 575, 260, 120), 1)

        #########################################################################################
        # derived stats

        text_surface = font_small.render(
            f"Vertical Speed: {(smoothed_ref_height-last_feet)/dt:.2f} ft/s",
            True,
            (255, 255, 255),
        )
        screen.blit(text_surface, (10, 520))
        last_feet = smoothed_ref_height

        text_surface = font_small.render(
            f"Max G: {G_max:.2f} G",
            True,
            (255, 255, 255),
        )
        screen.blit(text_surface, (10, 540))

        text_surface = font_small.render(
            f"Max height G: {height_max:.2f} G",
            True,
            (255, 255, 255),
        )
        screen.blit(text_surface, (10, 560))

        if not read_GPS:
            text_surface = font_small.render(
                f"GPS Lock: DISABLED",
                True,
                (255, 255, 255),
            )
            screen.blit(text_surface, (10, 580))
            text_surface = font_small.render(
                f"GPS Sats: DISABLED",
                True,
                (255, 255, 255),
            )
            screen.blit(text_surface, (10, 600))
            text_surface = font_small.render(
                f"GPS Lat: DISABLED",
                True,
                (255, 255, 255),
            )
            screen.blit(text_surface, (10, 620))
            text_surface = font_small.render(
                f"GPS Lon: DISABLED",
                True,
                (255, 255, 255),
            )
            screen.blit(text_surface, (10, 640))

        pygame.display.flip()
        clock.tick(60)


if __name__ == "__main__":

    network_thread = threading.Thread(target=network_thread)
    network_thread.daemon = True
    network_thread.start()

    main()
