import argparse
import math
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import imgui
from imgui.integrations.pygame import PygameRenderer
from pyglm import glm
from functions import bvh_parser, motion_adapter, get_rotation_matrix
from draw_humanoid import draw_humanoid
from utils import draw_axes, set_lights
from tkinter import Tk, filedialog
import numpy as np

center = glm.vec3(0, 0, 0)
eye = glm.vec3(60, 180, 600)
upVector = glm.vec3(0, 1, 0)

distance = glm.length(eye - center)
yaw = math.atan2(eye.x - center.x, eye.z - center.z)
pitch = math.asin((eye.y - center.y) / distance)

last_x, last_y = 0, 0
is_rotating = False
is_translating = False
stop = False

frame_idx = 0
frame_len = None
root = None
motion_frames = None
selected_joint = None
loaded_file_path = None


def imgui_joint_tree(joint):
    global selected_joint
    node_open = imgui.tree_node(joint.name)
    imgui.set_item_allow_overlap()
    if imgui.is_item_clicked():
        selected_joint = joint
        print("hi")
    if node_open:
        for child in joint.children:
            imgui_joint_tree(child)
        imgui.tree_pop()

def imgui_joint_control():
    global selected_joint
    imgui.separator()
    imgui.text("Selected Joint Info:")
    if selected_joint: #아직 석원이의 코딩이 안끝나서 다 고쳐지진 않았어여.
        imgui.text(f"Name: {selected_joint.name}")
        if hasattr(selected_joint, "rotation"):
            pos = selected_joint.rotation
            cha = selected_joint.channels

            text = ''
            for a, b in zip(pos, cha):
                text += f'{b}: {a}\n'
            imgui.text(text)
    else:
        imgui.text("No joint selected.")

def define_root_transform(joint):
    if joint.name in ("Hips","hip"):
        M = joint.kinetics
        yaw = extract_y_rotation(M)
        Rotation_yaw = np.array([
            [np.cos(yaw),0,np.sin(yaw),0],
            [0,1,0,0],
            [-np.sin(yaw),0,np.cos(yaw),0],
            [0,0,0,1]
        ], dtype=np.float32)

        Rotation_yaw[0][3] = root_position[0]
        Rotation_yaw[2][3] = root_position[2]

        return Rotation_yaw
    else:
        raise ValueError("Root joint error: joint name is not Hips or hip.")


def extract_y_rotation(matrix):
    forward = np.array([matrix[0, 2], matrix[1, 2], matrix[2, 2]], dtype=np.float32)
    forward[1] = 0.0
    norm = np.linalg.norm(forward)
    if norm > 0:
        forward /= norm
    else:
        forward = np.array([0, 0, 1], dtype=np.float32)
    # atan2(x, z) 사용: xz 평면에서의 yaw
    yaw = math.atan2(forward[0], forward[2])
    return yaw

def draw_T_axes(root_transform, axis_length = 10.0):
    matrix_np = np.array(root_transform, dtype=np.float32).T.flatten()
    glPushMatrix()
    glMultMatrixf(matrix_np)
    glLineWidth(2.0)
    glBegin(GL_LINES)
    # X-axis in red.
    glColor3f(1, 0, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(axis_length, 0, 0)
    # Y-axis in green.
    glColor3f(0, 1, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(0, axis_length, 0)
    # Z-axis in blue.
    glColor3f(0, 0, 1)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, axis_length)

    glEnd()
    glPopMatrix()

def update_eye():
    global eye, distance, yaw, pitch, center
    max_pitch = math.radians(89)
    pitch = max(-max_pitch, min(pitch, max_pitch))
    offset_x = distance * math.sin(yaw) * math.cos(pitch)
    offset_y = distance * math.sin(pitch)
    offset_z = distance * math.cos(yaw) * math.cos(pitch)
    eye = center + glm.vec3(offset_x, offset_y, offset_z)

def resize(width, height):
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, width / height, 0.1, 5000.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def handle_mouse_motion(event):
    global last_x, last_y, yaw, pitch, center, eye, distance, is_rotating, is_translating
    xpos, ypos = event.pos
    dx = xpos - last_x
    dy = ypos - last_y
    last_x, last_y = xpos, ypos

    if is_rotating:
        sensitivity = 0.005
        yaw -= dx * sensitivity
        pitch += dy * sensitivity
        update_eye()
    elif is_translating:
        sensitivity = 0.005 * distance
        view_dir = glm.normalize(center - eye)
        right = glm.normalize(glm.cross(view_dir, glm.vec3(0, 1, 0)))
        up = glm.vec3(0, 1, 0)
        translation = (-dx * right + dy * up) * sensitivity
        center += translation
        update_eye()

def handle_mouse_button(event):
    global last_x, last_y, is_rotating, is_translating
    if event.type == pygame.MOUSEBUTTONDOWN:
        last_x, last_y = event.pos
        if event.button == 1:
            is_rotating = True
        elif event.button == 3:
            is_translating = True
        if event.button in (4, 5):
            handle_mouse_wheel(event)
    elif event.type == pygame.MOUSEBUTTONUP:
        if event.button == 1:
            is_rotating = False
        elif event.button == 3:
            is_translating = False

def handle_mouse_wheel(event):
    global distance
    zoom_sensitivity = 0.1
    if hasattr(event, 'y'):
        distance -= zoom_sensitivity * event.y * distance
    else:
        if event.button == 4:
            distance -= zoom_sensitivity * distance
        elif event.button == 5:
            distance += zoom_sensitivity * distance
    distance = max(distance, 0.1)
    update_eye()

def render():
    global root, frame_idx, root_position
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    gluLookAt(eye.x, eye.y, eye.z,
              center.x, center.y, center.z,
              upVector.x, upVector.y, upVector.z)
    draw_axes()
    root_position, _ = motion_adapter(root, motion_frames[frame_idx])
    draw_humanoid(root_position, root)
    draw_T_axes(define_root_transform(root), 30)

def main():
    global frame_idx, frame_len, root, motion_frames, last_x, last_y, stop, loaded_file_path
    Tk().withdraw()
    pygame.init()
    size = (800, 600)
    screen = pygame.display.set_mode(size, pygame.DOUBLEBUF | pygame.OPENGL | pygame.RESIZABLE)
    pygame.display.set_caption("BVH Viewer with ImGui Control Panel")

    glEnable(GL_DEPTH_TEST)
    set_lights()
    resize(*size)

    imgui.create_context()
    impl = PygameRenderer()

    clock = pygame.time.Clock()
    previous_time = pygame.time.get_ticks() / 1000.0
    frame_duration = 1 / 60.0
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                continue

            impl.process_event(event)
            io = imgui.get_io()

            if event.type == pygame.MOUSEWHEEL:
                if io.want_capture_mouse:
                    continue
                else:
                    handle_mouse_wheel(event)

            if event.type == pygame.MOUSEMOTION:
                if not io.want_capture_mouse:
                    handle_mouse_motion(event)

            if event.type in (pygame.MOUSEBUTTONDOWN, pygame.MOUSEBUTTONUP):
                if not io.want_capture_mouse:
                    handle_mouse_button(event)

            if event.type == pygame.VIDEORESIZE:
                size = event.size
                screen = pygame.display.set_mode(size, pygame.DOUBLEBUF | pygame.OPENGL | pygame.RESIZABLE)
                resize(*size)

        io.display_size = pygame.display.get_surface().get_size()

        current_time = pygame.time.get_ticks() / 1000.0
        delta_time = current_time - previous_time
        if not stop:
            if delta_time >= frame_duration:
                frame_idx = (frame_idx + 1) % frame_len
                previous_time = current_time

        # ImGui 프레임
        imgui.new_frame()
        imgui.set_next_window_position(10, 10)  # (X, Y)

        imgui.begin("Control Panel")
        changed, value = imgui.slider_int("Slider", frame_idx+1, 0, frame_len)
        if imgui.button("play/pause", 100, 30):
            stop = (stop + 1)%2
        if changed:
            frame_idx = value-1
        imgui.separator()
        imgui.text("Joint Tree:")
        if root:
            imgui_joint_tree(root)
        imgui_joint_control()
        imgui.end()

        # BVH_File Loader
        imgui.set_next_window_position(550, 10, condition=imgui.ONCE)  # (X, Y)
        imgui.set_next_window_size(200, 100, condition=imgui.ONCE)  # Width, Height
        imgui.begin("BVH Loader")
        if imgui.button("Open Folder/Directory"):
            file_path = filedialog.askopenfilename(
                title="Select BVH file",
                filetypes=[("BVH Files", "*.bvh")]
            )
            if file_path:
                loaded_file_path = file_path
                print("File selected:", file_path)
                root, motion_frames = bvh_parser(file_path)
                check_bvh_structure(root, is_root=True)
                frame_len = len(motion_frames)
                frame_idx = 0

        if loaded_file_path:
            imgui.text("Loaded: {}".format(loaded_file_path.split("/")[-1]))
        imgui.end()
        render()

        imgui.render()
        impl.render(imgui.get_draw_data())
        pygame.display.flip()
        clock.tick(60)

    impl.shutdown()
    pygame.quit()


def check_bvh_structure(joint, is_root=False):
    if is_root:
        if len(joint.channels) != 6:
            raise ValueError(f"Root joint '{joint.name}' must have 6 channels, found {len(joint.channels)}")
        for channel in joint.channels[:3]:
            if "position" not in channel.lower():
                raise ValueError(
                    f"Root joint '{joint.name}' first three channels must be position channels, found '{channel}'")
        for channel in joint.channels[3:]:
            if "rotation" not in channel.lower():
                raise ValueError(
                    f"Root joint '{joint.name}' last three channels must be rotation channels, found '{channel}'")
    else:
        if joint.channels:
            if len(joint.channels) != 3:
                for channel in joint.channels[3:]:
                    if "rotation" not in channel.lower():
                        raise ValueError(f"Joint '{joint.name}' channel must be a rotation channel, found '{channel}'")
            else:
                for channel in joint.channels:
                    if "rotation" not in channel.lower():
                        raise ValueError(f"Joint '{joint.name}' channel must be a rotation channel, found '{channel}'")

    for child in joint.children:
        check_bvh_structure(child, is_root=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file_path")
    args = parser.parse_args()

    root, motion_frames = bvh_parser(args.file_path)
    check_bvh_structure(root, is_root=True)

    frame_len = len(motion_frames)
    main()