from queue import Queue
import threading
import time
import serial
import numpy as np
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *


serial_data_queue = Queue()


# Serial port configuration
serial_port = "COM4"  # Update this with your serial port
baud_rate = 115200

# Open the serial port
ser = serial.Serial(serial_port, baud_rate)


def serial_thread():
    try:
        while True:
            # Read a line from the serial port
            ser.reset_input_buffer()
            line = ser.readline().decode("utf-8").strip()
            # print(f"Received: {line}")

            if len(line.split(",")) != 4:
                continue

            # Parse the quaternion
            q = list(map(float, line.split(",")))

            serial_data_queue.put(q)
            if serial_data_queue.qsize() > 1:
                serial_data_queue.get()

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        ser.close()


def quaternion_to_rotation_matrix(q):
    q0, q1, q2, q3 = q
    R = np.array(
        [
            [
                1 - 2 * (q2**2 + q3**2),
                2 * (q1 * q2 - q0 * q3),
                2 * (q1 * q3 + q0 * q2),
                0,
            ],
            [
                2 * (q1 * q2 + q0 * q3),
                1 - 2 * (q1**2 + q3**2),
                2 * (q2 * q3 - q0 * q1),
                0,
            ],
            [
                2 * (q1 * q3 - q0 * q2),
                2 * (q2 * q3 + q0 * q1),
                1 - 2 * (q1**2 + q2**2),
                0,
            ],
            [0, 0, 0, 1],  # Add an extra row for a 4x4 matrix
        ]
    )
    return R


def main_3d_display():
    import pygame

    # Function to load an OBJ file
    def load_obj(filename):
        vertices = []
        faces = []
        texcoords = []
        texcoord_indices = []
        with open(filename) as f:
            for line in f:
                if line.startswith("v "):
                    vertices.append(list(map(float, line.strip().split()[1:])))
                elif line.startswith("vt "):
                    texcoords.append(list(map(float, line.strip().split()[1:])))
                elif line.startswith("f "):
                    face = []
                    tex_index = []
                    for vertex in line.strip().split()[1:]:
                        v, t, _ = vertex.split("/")
                        face.append(int(v) - 1)
                        tex_index.append(int(t) - 1)
                    faces.append(face)
                    texcoord_indices.append(tex_index)
        return (
            np.array(vertices, dtype=np.float32),
            np.array(faces, dtype=np.int32),
            np.array(texcoords, dtype=np.float32),
            np.array(texcoord_indices, dtype=np.int32),
        )

    # Function to load the texture
    def load_texture(filename):
        texture_surface = pygame.image.load(filename)
        texture_data = pygame.image.tostring(texture_surface, "RGB", 1)
        width, height = texture_surface.get_rect().size
        glEnable(GL_TEXTURE_2D)
        texture_id = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, texture_id)
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGB,
            width,
            height,
            0,
            GL_RGB,
            GL_UNSIGNED_BYTE,
            texture_data,
        )
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
        return texture_id

    # Function to draw the loaded model with texture
    def draw_model(vertices, faces, texcoords, texcoord_indices, texture_id):
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, texture_id)
        glBegin(GL_TRIANGLES)
        for i, face in enumerate(faces):
            for j, vertex in enumerate(face):
                glTexCoord2fv(texcoords[texcoord_indices[i][j]])
                glVertex3fv(vertices[vertex])
        glEnd()
        glDisable(GL_TEXTURE_2D)

    # Initialize Pygame
    pygame.init()
    display = (600, 600)
    screen = pygame.display.set_mode(display, DOUBLEBUF | OPENGLBLIT)
    hidden_screen = pygame.Surface(display, flags=pygame.SRCALPHA)
    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -5)

    # Load your 3D model (replace 'your_model.obj' with the path to your model file)
    vertices, faces, texcoords, texcoord_indices = load_obj("rocket.obj")
    global texture_id
    # Load your texture (replace 'your_texture.png' with the path to your texture file)
    texture_id = load_texture("rusty_metal_sheet_diff_1k.jpg")

    clock = pygame.time.Clock()

    # Main loop
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

        bg_img = pygame.Surface(display)
        bg_img.fill((255, 255, 255))

        hidden_screen.blit(bg_img, (0, 0))

        if not serial_data_queue.empty():
            serial_data = serial_data_queue.get()
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            quaternion = serial_data_queue.get()
            rotation_matrix = quaternion_to_rotation_matrix(quaternion)
            # Apply the rotation matrix
            glLoadIdentity()
            gluPerspective(90, (display[0] / display[1]), 0.1, 50.0)
            glScalef(0.1, 0.1, 0.1)
            glTranslatef(0.0, 0.0, -5)
            # Convert the numpy array to a list and flatten it
            glRotate(-90, 1, 0, 0)
            glMultMatrixf(np.linalg.inv(rotation_matrix).T.flatten().tolist())

        draw_model(vertices, faces, texcoords, texcoord_indices, texture_id)

        pygame.display.flip()
        clock.tick(60)

        screen.blit(hidden_screen, (0, 0))


def main_2d_display():
    import pygame

    # Initialize Pygame and draw square
    pygame.init()
    display = (600, 600)
    screen = pygame.display.set_mode(display)
    clock = pygame.time.Clock()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        screen.fill((255, 255, 255))
        pygame.draw.rect(screen, (0, 128, 255), (100, 100, 400, 400))
        pygame.display.flip()
        clock.tick(60)


if __name__ == "__main__":

    serial_thread = threading.Thread(target=serial_thread)
    serial_thread.daemon = True
    serial_thread.start()
    main_3d_display_thread = threading.Thread(target=main_3d_display)
    main_3d_display_thread.daemon = False
    main_3d_display_thread.start()

    # main_2d_display_thread = threading.Thread(target=main_2d_display)
    # main_2d_display_thread.daemon = False
    # main_2d_display_thread.start()
    # # main_3d_display()
    # # main_2d_display()
