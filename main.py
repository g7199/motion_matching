import argparse
import math
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
import imgui
from imgui.integrations.pygame import PygameRenderer
from pyglm import glm
import numpy as np

from BVH_Parser import bvh_parser, check_bvh_structure
from Transforms import motion_adapter, get_pelvis_virtual, extract_yaw_rotation
from Rendering import draw_humanoid, draw_virtual_root_axis
from utils import draw_axes, set_lights
import Events
import UI

state = {
        'center': glm.vec3(0, 0, 0),
        'eye': glm.vec3(60, 180, 600),
        'upVector': glm.vec3(0, 1, 0),
        'distance': glm.length(glm.vec3(60, 180, 600) - glm.vec3(0, 0, 0)),
        'yaw': math.atan2(60, 600),
        'pitch': math.asin((180) / glm.length(glm.vec3(60, 180, 600))),
        'last_x': 0,
        'last_y': 0,
        'is_rotating': False,
        'is_translating': False,
        'stop': False,
        'frame_idx': 0,
        'frame_len': None,
        'root': None,
        'motion_frames': None,
        'loaded_file_path': None
    }

def resize(width, height):
    """
    glViewport사이즈를 조절하는 함수
    :param width: 너비
    :param height: 높이
    """
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, width / height, 0.1, 5000.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def main():
    """
    BVH_Viewer 의 main loop
    """
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
                if not io.want_capture_mouse:
                    Events.handle_mouse_wheel(event, state)
            if event.type == pygame.MOUSEMOTION:
                if not io.want_capture_mouse:
                    Events.handle_mouse_motion(event, state)
            if event.type in (pygame.MOUSEBUTTONDOWN, pygame.MOUSEBUTTONUP):
                if not io.want_capture_mouse:
                    Events.handle_mouse_button(event, state)
            if event.type == pygame.VIDEORESIZE:
                size = event.size
                screen = pygame.display.set_mode(size, pygame.DOUBLEBUF | pygame.OPENGL | pygame.RESIZABLE)
                resize(*size)

        io.display_size = pygame.display.get_surface().get_size()
        current_time = pygame.time.get_ticks() / 1000.0
        delta_time = current_time - previous_time
        if not state['stop']:
            if delta_time >= frame_duration and state['motion_frames']:
                state['frame_idx'] = (state['frame_idx'] + 1) % state['frame_len']
                previous_time = current_time

        imgui.new_frame()
        UI.draw_control_panel(state)
        UI.draw_file_loader(state)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        gluLookAt(state['eye'].x, state['eye'].y, state['eye'].z,
                  state['center'].x, state['center'].y, state['center'].z,
                  state['upVector'].x, state['upVector'].y, state['upVector'].z)
        draw_axes()
        if state['motion_frames'] and state['root']:
            root_position, _ = motion_adapter(state['root'], state['motion_frames'][state['frame_idx']])
            
            draw_humanoid(root_position, state['root'])
            
            hip_node = state['root'].children[0]
            hip_kinetics = get_pelvis_virtual(hip_node.kinetics)
            # global translation을 제거한 kinetics를 생성합니다.
            local_kinetics = hip_kinetics.copy()
            local_kinetics = extract_yaw_rotation(local_kinetics, root_position*np.array([1,0,1]))
            
            draw_virtual_root_axis(local_kinetics)

        imgui.render()
        impl.render(imgui.get_draw_data())
        pygame.display.flip()
        clock.tick(60)

    impl.shutdown()
    pygame.quit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file_path")
    args = parser.parse_args()


    root, motion_frames = bvh_parser(args.file_path)
    state['frame_len'] = len(motion_frames)
    check_bvh_structure(root, is_root=True)

    state['root'] = root
    state['motion_frames'] = motion_frames


    main()
