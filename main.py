import tkinter as tk
import math
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
import imgui
from imgui.integrations.pygame import PygameRenderer
from pyglm import glm
from bvh_controller import parse_bvh, get_preorder_joint_list, FeatureFrame, get_joint_chains_from_root, connect
from Rendering import draw_humanoid, draw_virtual_root_axis, draw_matching_features
from utils import draw_axes, set_lights, random_color
from virtual_transforms import extract_xz_plane
import Events
import UI
from feature_extractor import MotionKDTree
import numpy as np
import copy


tk.Tk().withdraw()

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
    'motions': [],
    'open_file_dialog': False
}

def resize(width, height):
    glViewport(0, 0, width - 300, height - 200)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, (width - 300) / (height - 200), 0.1, 5000.0)
    glMatrixMode(GL_MODELVIEW)

def make_default_feature_frame(virtual_root, feature_frame):
    joint_chains = get_joint_chains_from_root(virtual_root)
    zero = glm.vec3(0)
    root_name = virtual_root.children[0].name
    feature_frame.velocity[root_name] = glm.vec3(0)
    for chain in joint_chains:
        joint_name = chain[-1].name
        feature_frame.velocity[joint_name] = zero
        feature_frame.site_positions[joint_name] = zero
    for _ in range(3):
        feature_frame.future_position.append(zero)
        feature_frame.future_orientation.append(zero)
    return feature_frame

def make_current_feature_frame(vr, motion, idx, delta_time, hip_velocity, current_pos, current_dir, turn_rate):
    feature_frame = copy.deepcopy(motion.feature_frames[idx])
    feature_frame.future_position = []
    feature_frame.future_orientation = []

    feature_frame.velocity["Hips"] = hip_velocity
    inv_root_tf = glm.inverse(vr.kinematics)

    # future prediction (turning & moving forward)
    steps = [20, 40, 60]
    dir = glm.normalize(current_dir)
    speed = glm.length(hip_velocity)

    for k in steps:
        t = k * delta_time
        angle = turn_rate * t
        rot_quat = glm.angleAxis(angle, glm.vec3(0, 1, 0))  # 회전 축: Y축

        # 목표 방향과 slerp blending
        target_dir = glm.normalize(rot_quat * dir)
        q_current = glm.quat(glm.vec3(0, 0, 1), dir)
        q_target = glm.quat(glm.vec3(0, 0, 1), target_dir)
        q_blended = glm.slerp(q_current, q_target, 0.5)
        rotated_dir = glm.normalize(q_blended * glm.vec3(0, 0, 1))

        predicted_pos = current_pos + rotated_dir * speed * t

        # local 기준 상대 위치/방향으로 변환
        rel_pos = glm.vec3(inv_root_tf * glm.vec4(predicted_pos - current_pos, 0.0))
        rel_dir = glm.normalize(glm.vec3(inv_root_tf * glm.vec4(rotated_dir, 0.0)))

        feature_frame.future_position.append(rel_pos)
        feature_frame.future_orientation.append(rel_dir)

    return feature_frame


def init_motion(file_path):
    root, motion = parse_bvh(file_path)
    joint_order = get_preorder_joint_list(root)
    motion.build_quaternion_frames(joint_order)
    virtual_root = motion.apply_virtual(root)
    motion.apply_velocity_feature(virtual_root)
    motion.apply_future_feature()
    cur_feature = make_default_feature_frame(virtual_root, FeatureFrame())
    new_entry = {
        'name': file_path.split("/")[-1],
        'root': virtual_root,
        'motion': motion,
        'frame_len': motion.frames,
        'visible': True,
        'frame_idx': 1,
        'color': random_color(),
        'controller': Events.InputController(),
        'current_feature': cur_feature,
        'count': 0
    }
    state['motions'].append(new_entry)
    print("File loaded:", file_path)

def attatch_motion(root, motion):
    joint_order = get_preorder_joint_list(root)
    motion.build_quaternion_frames(joint_order)
    virtual_root = motion.apply_virtual(root)
    motion.apply_velocity_feature(virtual_root)
    motion.apply_future_feature()
    cur_feature = make_default_feature_frame(virtual_root, FeatureFrame())
    return cur_feature

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

    search_interval = 50
    motion_penalty = 5.0

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

        width, height = size[0], size[1]
        side_width = int(width * 0.25)
        bottom_height = int(height * 0.25)
        glViewport(0, bottom_height, width - side_width, height - bottom_height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, (width - side_width) / (height - bottom_height), 0.1, 5000.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        gluLookAt(state['eye'].x, state['eye'].y, state['eye'].z,
                  state['center'].x, state['center'].y, state['center'].z,
                  state['upVector'].x, state['upVector'].y, state['upVector'].z)
        draw_axes()

        if state.get('motions'):
            for motion_entry in state['motions']:
                if motion_entry.get('visible', True):
                    controller = motion_entry['controller']
                    keys = pygame.key.get_pressed()
                    controller.update(keys)  

                    if not state['stop']:
                        motion_entry['frame_idx'] = motion_entry['frame_idx'] + 1

                    frame_idx = motion_entry['frame_idx']
                    motion_entry['motion'].apply_to_skeleton(frame_idx, motion_entry['root'])
                    pos, dir, turn_rate = controller.update_virtual_kinematics(motion_entry['root'], motion_entry['motion'].frame_time)
                    motion_entry['current_feature'] = make_current_feature_frame(
                        motion_entry['root'],
                        motion_entry['motion'],
                        motion_entry['frame_idx'],
                        motion_entry['motion'].frame_time,
                        controller.current_velocity,
                        pos, dir, turn_rate
                    )
                    draw_humanoid(motion_entry['root'], motion_entry['color'])
                    if motion_entry['root'].children:
                        draw_virtual_root_axis(
                            extract_xz_plane(
                                motion_entry['root'].kinematics *
                                motion_entry['root'].children[0].kinematics
                            ), motion_entry['color']
                        )
                        draw_matching_features(motion_entry['root'], motion_entry['current_feature'])
                    motion_entry['count'] += 1

                    if frame_idx + 21 > motion_entry['frame_len']:
                        motion_entry['count'] = 1000
                    if motion_entry['count'] >= search_interval:
                        dist, [matched_motion, matched_idx, path], query_vec = tree.search_frame(motion_entry['current_feature'], motion_entry['motion'].quaternion_frames[frame_idx])

                        if frame_idx + 21 < motion_entry['frame_len']:
                            current_next_feature = motion_entry['motion'].feature_frames[frame_idx + 1]
                            current_next_joint = motion_entry['motion'].quaternion_frames[frame_idx + 1]
                            current_next_vec = tree.extract_feature_vector(current_next_feature, current_next_joint)
                            current_next_vec = tree.normalize(current_next_vec)
                            dist_current = tree.distance_function(query_vec, current_next_vec)
                        else:
                            dist_current = float('inf')

                        if dist + motion_penalty < dist_current:
                            new_motion = connect(motion_entry['motion'][motion_entry['frame_idx']:motion_entry['frame_idx']+20],
                                                matched_motion[matched_idx:], 0, transition_frames=20)

                            motion_entry['motion'] = new_motion
                            motion_entry['name'] = path.split("/")[-1]
                            motion_entry['frame_idx'] = 0
                            motion_entry['frame_len'] = new_motion.frames
                            motion_entry['count'] = 0
                            motion_entry['current_feature'] = attatch_motion(motion_entry['root'].children[0], new_motion)

                            print(f"[SMART MATCH] {path} @ {matched_idx} (better distance: {dist:.2f} < {dist_current:.2f})")
                        else:
                            motion_entry['count'] = 0


        io.display_size = width, height
        imgui.new_frame()
        imgui.render()
        impl.render(imgui.get_draw_data())
        pygame.display.flip()
        clock.tick(60)

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
    file_path = "./bvh/data/exp/slow_walk.bvh"
    root_path = './bvh/data/exp'
    init_motion(file_path)
    tree = MotionKDTree(root_path)
    main()