import tkinter as tk
tk.Tk().withdraw()
import math
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
import imgui
from imgui.integrations.pygame import PygameRenderer
from pyglm import glm

from bvh_controller import parse_bvh, get_preorder_joint_list
from Rendering import draw_humanoid, draw_virtual_root_axis, draw_matching_features
from utils import draw_axes, set_lights, random_color
from virtual_transforms import extract_xz_plane
import Events
import UI
from feature_extractor import MotionKDTree

state = {
    'center': glm.vec3(0, 0, 0),
    'eye': glm.vec3(20, 60, 200) * 0.5,
    'upVector': glm.vec3(0, 1, 0),
    'distance': glm.length(glm.vec3(20, 60, 200)*0.5 - glm.vec3(0, 0, 0)),
    'yaw': math.atan2(20, 200),
    'pitch': math.asin(180 / glm.length(glm.vec3(20, 60, 200))),
    'last_x': 0,
    'last_y': 0,
    'is_rotating': False,
    'is_translating': False,
    'stop': False,
    # motions: 파일 로더를 통해 추가된 여러 BVH 모션 정보 목록
    # 각 항목은 'name', 'root', 'motion', 'frame_len', 'visible', 'frame_idx'를 포함합니다.
    'motions': [],
    # 파일 다이얼로그 호출 플래그 (파일 로더 창에서 사용)
    'open_file_dialog': False
}

def resize(width, height):
    glViewport(0, 0, width - 300, height - 200)  # side panel과 control panel만큼 영역 축소
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, (width - 300) / (height - 200), 0.1, 5000.0)
    glMatrixMode(GL_MODELVIEW)

def init_motion(file_path):
    root, motion = parse_bvh(file_path)
    joint_order = get_preorder_joint_list(root)
    motion.build_quaternion_frames(joint_order)
    motion.apply_velocity_feature(root)
    virtual_root = motion.apply_virtual(root)
    motion.apply_future_feature()
    new_entry = {
        'name': file_path.split("/")[-1],
        'root': virtual_root,
        'motion': motion,
        'frame_len': motion.frames,
        'visible': True,
        'frame_idx': 1,
        'color': random_color()
    }
    state['motions'].append(new_entry)
    print("File loaded:", file_path)


def main():
    pygame.init()
    size = (800, 600)
    screen = pygame.display.set_mode(size, pygame.DOUBLEBUF | pygame.OPENGL | pygame.RESIZABLE)
    pygame.display.set_caption("BVH Viewer")
    glEnable(GL_DEPTH_TEST)
    set_lights()
    resize(*size)

    imgui.create_context()
    impl = PygameRenderer()

    clock = pygame.time.Clock()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                continue
            impl.process_event(event)
            io = imgui.get_io()
            if event.type == pygame.MOUSEWHEEL and not io.want_capture_mouse:
                Events.handle_mouse_wheel(event, state)
            if event.type == pygame.MOUSEMOTION and not io.want_capture_mouse:
                Events.handle_mouse_motion(event, state)
            if event.type in (pygame.MOUSEBUTTONDOWN, pygame.MOUSEBUTTONUP) and not io.want_capture_mouse:
                Events.handle_mouse_button(event, state)
            if event.type == pygame.VIDEORESIZE:
                size = event.size
                screen = pygame.display.set_mode(size, pygame.DOUBLEBUF | pygame.OPENGL | pygame.RESIZABLE)

        # 매 프레임 사이즈 갱신
        width, height = size[0], size[1]
        side_width = int(width * 0.25)
        bottom_height = int(height * 0.25)

        # --- GL viewport와 projection 설정 (매 프레임 한번만 설정) ---
        glViewport(0, bottom_height, width - side_width, height - bottom_height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, (width - side_width) / (height - bottom_height), 0.1, 5000.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # --- GL 렌더링 (반드시 UI 이전) ---
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        gluLookAt(state['eye'].x, state['eye'].y, state['eye'].z,
                  state['center'].x, state['center'].y, state['center'].z,
                  state['upVector'].x, state['upVector'].y, state['upVector'].z)
        draw_axes()

        if state.get('motions'):
            for motion_entry in state['motions']:
                if motion_entry.get('visible', True):
                    if not state['stop']:
                        motion_entry['frame_idx'] = (motion_entry['frame_idx'] + 1) % motion_entry['frame_len']
                    frame_idx = motion_entry['frame_idx']
                    motion_entry['motion'].apply_to_skeleton(frame_idx, motion_entry['root'])
                    draw_humanoid(motion_entry['root'], motion_entry['color'])
                    if motion_entry['root'].children:
                        draw_virtual_root_axis(
                            extract_xz_plane(
                                motion_entry['root'].kinematics *
                                motion_entry['root'].children[0].kinematics
                            ), motion_entry['color']
                        )
                        draw_matching_features(motion_entry['root'], motion_entry['motion'].feature_frames[frame_idx])

        # --- ImGui 렌더링 영역 ---
        io.display_size = width, height
        imgui.new_frame()
        viewport = imgui.get_main_viewport()

        imgui.render()
        impl.render(imgui.get_draw_data())

        pygame.display.flip()
        clock.tick(60)

        # --- 파일 다이얼로그 처리 ---
        if state.get('open_file_dialog'):
            from tkinter import filedialog
            file_path = filedialog.askopenfilename(
                title="Select BVH file",
                filetypes=[("BVH Files", "*.bvh")]
            )
            if file_path:
                init_motion(file_path)
            state['open_file_dialog'] = False

    impl.shutdown()
    pygame.quit()


if __name__ == "__main__":

    file_path = "bvh/data/002/02_08.bvh"
    root_path = './bvh/data/002' 

    init_motion(file_path)
    tree = MotionKDTree(root_path)

    main()
