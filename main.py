import argparse
import glfw
from OpenGL.GL import *
from OpenGL.GLU import *
import math
import glm
from functions import *
from utils import draw_axes, set_lights
from draw_humanoid import *


center = glm.vec3(0, 0, 0)
eye = glm.vec3(20, 60, 200)
upVector = glm.vec3(0, 1, 0)

distance = glm.length(eye - center)
yaw = math.atan2(eye.x - center.x, eye.z - center.z)
pitch = math.asin((eye.y - center.y) / distance)


last_x, last_y = 0, 0
is_rotating = False
is_translating = False

frame_idx = 0
frame_len = None
root = None
motion_frames = None

# GLFW Window
window = None


def update_eye():
    """카메라 담당 함수"""
    global eye, distance, yaw, pitch, center
    max_pitch = math.radians(89)
    pitch = max(-max_pitch, min(pitch, max_pitch))

    offset_x = distance * math.sin(yaw) * math.cos(pitch)
    offset_y = distance * math.sin(pitch)
    offset_z = distance * math.cos(yaw) * math.cos(pitch)
    eye = center + glm.vec3(offset_x, offset_y, offset_z)


def resize(window, w, h):
    glViewport(0, 0, w, h)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, w / h, 0.1, 500.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def mouse(window, button, action, mods):
    """마우스 버튼 이벤트 처리"""
    global last_x, last_y, is_rotating, is_translating

    if action == glfw.PRESS:
        last_x, last_y = glfw.get_cursor_pos(window)

        if button == glfw.MOUSE_BUTTON_LEFT:
            is_rotating = True
        elif button == glfw.MOUSE_BUTTON_RIGHT:
            is_translating = True

    elif action == glfw.RELEASE:
        if button == glfw.MOUSE_BUTTON_LEFT:
            is_rotating = False
        elif button == glfw.MOUSE_BUTTON_RIGHT:
            is_translating = False


def motion(window, xpos, ypos):
    """마우스 드래그 이벤트 처리"""
    global last_x, last_y, yaw, pitch, center, eye, distance

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


def mouse_wheel(window, x_offset, y_offset):
    """마우스 휠 이벤트 처리"""
    global distance
    zoom_sensitivity = 0.1
    distance -= zoom_sensitivity * y_offset * distance
    distance = max(distance, 0.1)
    update_eye()


def render():
    """렌더링 처리 함수"""
    global root, frame_idx

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()

    gluLookAt(eye.x, eye.y, eye.z,
              center.x, center.y, center.z,
              upVector.x, upVector.y, upVector.z)

    draw_axes()
    root_position, root_node = motion_adapter(root, motion_frames[frame_idx])
    draw_humanoid(root_position, root)


def main():
    global window, frame_idx, frame_len, root, motion_frames

    # Initialize GLFW
    if not glfw.init():
        raise Exception("GLFW could not be initialized")

    window = glfw.create_window(800, 600, "BVH Viewer", None, None)
    if not window:
        glfw.terminate()
        raise Exception("GLFW window creation failed")

    glfw.make_context_current(window)
    glEnable(GL_DEPTH_TEST)
    set_lights()

    # 콜백 함수들
    glfw.set_framebuffer_size_callback(window, resize)
    glfw.set_cursor_pos_callback(window, motion)
    glfw.set_mouse_button_callback(window, mouse)
    glfw.set_scroll_callback(window, mouse_wheel)

    # projection matrix 초기에 설정해줘야됨, 이거 안하면 처음에 검정화면 되더라
    width, height = glfw.get_framebuffer_size(window)
    resize(window, width, height)

    # FPS 정의
    previous_time = glfw.get_time()
    frame_duration = 1 / 60

    while not glfw.window_should_close(window):
        current_time = glfw.get_time()
        delta_time = current_time - previous_time

        if delta_time >= frame_duration:
            frame_idx = (frame_idx + 1) % frame_len
            previous_time = current_time

        render()

        glfw.swap_buffers(window)
        glfw.poll_events()

    glfw.terminate()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file_path")

    args = parser.parse_args()
    root, motion_frames = bvh_parser(args.file_path)
    frame_len = len(motion_frames)

    main()