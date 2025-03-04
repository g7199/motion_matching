import argparse
from functions import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from utils import draw_axes, set_lights
from draw_humanoid import *
import math
from pyglm import glm

center = glm.vec3(0, 0, 0)
eye = glm.vec3(20, 60, 200)
upVector = glm.vec3(0, 1, 0)

distance = glm.length(eye - center)
yaw = math.atan2(eye.x - center.x, eye.z - center.z)
pitch = math.asin((eye.y - center.y) / distance)


last_x, last_y = 0, 0
is_rotating = False
is_translating = False
args = None
root = None
frame_len=None
frame_idx=0

def display():
    global root, frame_idx
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()

    # 카메라 설정
    gluLookAt(eye.x, eye.y, eye.z,
              center.x, center.y, center.z,
              upVector.x, upVector.y, upVector.z)

    draw_axes()
    root_position, root_node = motion_adapter(root, motion_frames[frame_idx])
    draw_humanoid(root_position, root)
    glutSwapBuffers()

def update_eye():
    """카메라 담당 함수"""
    global eye, distance, yaw, pitch, center
    # 90° 넘어가면 뒤집히는거 방지
    max_pitch = math.radians(89)
    if pitch > max_pitch:
        pitch = max_pitch
    if pitch < -max_pitch:
        pitch = -max_pitch

    # Offset 계산
    offset_x = distance * math.sin(yaw) * math.cos(pitch)
    offset_y = distance * math.sin(pitch)
    offset_z = distance * math.cos(yaw) * math.cos(pitch)
    eye = center + glm.vec3(offset_x, offset_y, offset_z)



def resize(w, h):
    glViewport(0, 0, w, h)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, w / h, 0.1, 500.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def motion(x, y):
    """마우스 드래그 이벤트 처리"""
    global last_x, last_y, yaw, pitch, center, eye, distance

    dx = x - last_x
    dy = y - last_y
    last_x, last_y = x, y

    if is_rotating:
        sensitivity = 0.005
        yaw -= dx * sensitivity
        pitch += dy * sensitivity
        update_eye()

    elif is_translating:
        sensitivity = 0.005 * distance  # scale panning with distance
        view_dir = glm.normalize(center - eye)
        right = glm.normalize(glm.cross(view_dir, glm.vec3(0, 1, 0)))
        up = glm.vec3(0, 1, 0)
        translation = (-dx * right + dy * up) * sensitivity
        center += translation
        update_eye()

    glutPostRedisplay()


def mouse(button, state, x, y):
    """마우스 버튼 이벤트 처리"""
    global last_x, last_y, is_rotating, is_translating

    last_x, last_y = x, y

    if button == GLUT_LEFT_BUTTON:
        is_rotating = (state == GLUT_DOWN)
    elif button == GLUT_RIGHT_BUTTON:
        is_translating = (state == GLUT_DOWN)


def mouse_wheel(button, direction, x, y):
    """마우스 휠 이벤트 처리 (Zoom In/Out)"""
    global distance
    zoom_sensitivity = 0.1
    if direction > 0:
        distance -= zoom_sensitivity * distance
    else:
        distance += zoom_sensitivity * distance
    if distance < 0.1:
        distance = 0.1
    update_eye()
    glutPostRedisplay()


def update(value):
    global frame_idx, frame_len
    glutPostRedisplay()
    frame_idx += 1

    if frame_idx == frame_len:
        frame_idx = 0

    glutTimerFunc(16, update, 0)


def main():
    glutInit()
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(800, 600)
    glutCreateWindow(b"Camera Control Example")

    glEnable(GL_DEPTH_TEST)
    set_lights()

    glutReshapeFunc(resize)
    glutDisplayFunc(display)
    glutMouseFunc(mouse)
    glutMotionFunc(motion)
    glutMouseWheelFunc(mouse_wheel)
    glutTimerFunc(16, update, 0)

    glutMainLoop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file_path")

    args = parser.parse_args()
    root, motion_frames = bvh_parser(args.file_path)
    frame_len = len(motion_frames)

    main()
