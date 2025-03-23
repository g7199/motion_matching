import argparse
import math
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
import imgui
from imgui.integrations.pygame import PygameRenderer
from pyglm import glm
import numpy as np
from scipy.spatial.transform import Rotation as R

from BVH_Parser import bvh_parser, check_bvh_structure
from Transforms import motion_adapter, get_pelvis_virtual, extract_yaw_rotation, translation_matrix, inverse_matrix, extract_xz_plane
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
    'pitch': math.asin(180 / glm.length(glm.vec3(60, 180, 600))),
    'last_x': 0,
    'last_y': 0,
    'is_rotating': False,
    'is_translating': False,
    'stop': False,
    'frame_idx': 1,
    'frame_len': None,
    'root': None,
    'motion_frames': None,
    'second_frame_len': None,
    'second_root': None,
    'second_motion_frames': None,
    'loaded_file_path': None,
    'global_offset': np.eye(4)
}


def resize(width, height):
    """
    glViewport 사이즈를 조절하는 함수.
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
    BVH Viewer의 메인 루프.
    첫 번째 모션이 끝나면 두 번째 모션의 root가 첫 번째 모션의 마지막 포즈에 맞춰 retargeting되고,
    모션 데이터가 swap되어 무한 반복하도록 구성됩니다.
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

        if not state['stop'] and state['motion_frames']:
            # 현재 모션 프레임 업데이트
            prev_frame = state['frame_idx']
            state['frame_idx'] += 1

            # 만약 현재 모션이 마지막 프레임이면, 두 번째 모션으로 retargeting 후 swap
            if prev_frame == state['frame_len'] - 1:
                if state['second_motion_frames'] is not None:
                    # 첫 번째 모션의 마지막 프레임에서의 global root 포즈 계산
                    T_final_global = state['root'].kinetics
                    T_final_local = state['root'].children[0].kinetics

                    hip_node_2 = state['second_root'].children[0]
                    init_position, _ = motion_adapter(hip_node_2, state['second_motion_frames'][1])

                    # pelvis에 대한 global 행렬
                    T_init_global = translation_matrix(init_position) @ hip_node_2.kinetics
                    T_init_local  = get_pelvis_virtual(T_init_global)
                    T_root_init = T_init_global @ inverse_matrix(T_init_local)

                    T_offset_global = T_final_global @ inverse_matrix(T_root_init)
                    state['global_offset'] = T_offset_global

                    # 모션 데이터, root, frame length 등을 swap
                    state['motion_frames'], state['second_motion_frames'] = state['second_motion_frames'], state['motion_frames']
                    state['root'], state['second_root'] = state['second_root'], state['root']
                    state['frame_len'], state['second_frame_len'] = state['second_frame_len'], state['frame_len']
                    state['frame_idx'] = 1  # 새 모션은 처음부터 시작

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
            # 현재 모션의 root 포즈 계산 및 캐릭터 그리기
            hip_node = state['root'].children[0]
            root_position, _ = motion_adapter(hip_node, state['motion_frames'][state['frame_idx']])

            # pelvis의 글로벌 변환행렬, 로컬 변환행렬을 분리(가상의 루트/골반)
            T_global_pelvis = translation_matrix(root_position) @ hip_node.kinetics
            T_local_pelvis  = get_pelvis_virtual(T_global_pelvis)
            T_root_current  = T_global_pelvis @ inverse_matrix(T_local_pelvis)

            # 오프셋 적용(블렌딩용) → ‘새로운’ 루트/골반 행렬
            T_new_root   = state['global_offset'] @ T_root_current

            # 실제로 캐릭터에 적용
            state['root'].kinetics        = T_new_root
            state['root'].children[0].kinetics = T_local_pelvis

            # 캐릭터 그리기
            draw_humanoid(root_position, state['root'])

            # 바닥면(수평면)에서의 Yaw만 추출하여 축 그리기
            pure_yaw = extract_xz_plane(state['root'].kinetics @ hip_node.kinetics)
            draw_virtual_root_axis(pure_yaw)

        imgui.render()
        impl.render(imgui.get_draw_data())
        pygame.display.flip()
        clock.tick(60)

    impl.shutdown()
    pygame.quit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file_path1")
    parser.add_argument("file_path2")
    args = parser.parse_args()

    print(args)

    # 첫 번째 모션 파싱
    root, motion_frames = bvh_parser(args.file_path1)
    state['frame_len'] = len(motion_frames)
    check_bvh_structure(root, is_root=True)

    # 두 번째 모션 파싱
    second_root, second_motion_frames = bvh_parser(args.file_path2)
    state['second_frame_len'] = len(second_motion_frames)
    check_bvh_structure(second_root, is_root=True)

    # state에 저장 (키 이름은 swap 시에도 일관되게 사용)
    state['root'] = root
    state['motion_frames'] = motion_frames
    state['second_root'] = second_root
    state['second_motion_frames'] = second_motion_frames

    main()
