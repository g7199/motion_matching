import tkinter as tk
import math
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
import imgui
from imgui.integrations.pygame import PygameRenderer
from pyglm import glm
from bvh_controller import parse_bvh, get_preorder_joint_list, FeatureFrame, get_joint_chains_from_root
from Rendering import draw_humanoid, draw_virtual_root_axis, draw_matching_features
from utils import draw_axes, set_lights, random_color
from virtual_transforms import extract_xz_plane
import Events
import UI
from feature_extractor import MotionKDTree
import numpy as np


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

def make_current_feature_frame(virtual_root, before_feature_frame, delta_time, hip_velocity, current_pos, current_dir, turn_rate):
    feature_frame = FeatureFrame()
    joint_chains = get_joint_chains_from_root(virtual_root)
    root_name = virtual_root.children[0].name
    feature_frame.velocity[root_name] = hip_velocity
    for chain in joint_chains:
        global_transform = glm.mat4(virtual_root.kinematics)
        for joint in chain:
            global_transform *= joint.kinematics
        pos = global_transform * glm.vec4(0, 0, 0, 1)
        feature_frame.site_positions[chain[-1].name] = glm.vec3(pos.x, pos.y, pos.z)
    for joint_name in feature_frame.site_positions:
        v = feature_frame.site_positions[joint_name] - before_feature_frame.site_positions[joint_name]
        feature_frame.velocity[joint_name] = v / delta_time
    steps = [20, 40, 60]
    dir = glm.normalize(current_dir)
    speed = glm.length(hip_velocity)
    for k in steps:
        t = k * delta_time
        angle = turn_rate * t
        rot_quat = glm.angleAxis(angle, glm.vec3(0, 1, 0))
        target_dir = glm.normalize(rot_quat * dir)
        q_current = glm.quat(glm.vec3(0, 0, 1), dir)
        q_target = glm.quat(glm.vec3(0, 0, 1), target_dir)
        slerp_ratio = 0.5
        q_blended = glm.slerp(q_current, q_target, slerp_ratio)
        rotated_dir = glm.normalize(q_blended * glm.vec3(0, 0, 1))
        predicted_pos = current_pos + rotated_dir * speed * t
        rel_pos = glm.vec3(glm.inverse(virtual_root.kinematics) * glm.vec4(predicted_pos - current_pos, 0))
        rel_dir = glm.normalize(glm.vec3(glm.inverse(virtual_root.kinematics) * glm.vec4(rotated_dir, 0)))
        feature_frame.future_position.append(rel_pos)
        feature_frame.future_orientation.append(rel_dir)
    return feature_frame

def init_motion(file_path):
    root, motion = parse_bvh(file_path)
    joint_order = get_preorder_joint_list(root)
    motion.build_quaternion_frames(joint_order)
    motion.apply_velocity_feature(root)
    virtual_root = motion.apply_virtual(root)
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
        'last_matched_frame': -10
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

    search_interval = 10
    distance_threshold = 60.0

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
                        motion_entry['frame_idx'] = (motion_entry['frame_idx'] + 1) % motion_entry['frame_len']

                    frame_idx = motion_entry['frame_idx']
                    motion_entry['motion'].apply_to_skeleton(frame_idx, motion_entry['root'])
                    pos, dir, turn_rate = controller.update_virtual_kinematics(motion_entry['root'], motion_entry['motion'].frame_time)
                    motion_entry['current_feature'] = make_current_feature_frame(
                        motion_entry['root'],
                        motion_entry['current_feature'],
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

                    if frame_idx - motion_entry['last_matched_frame'] >= search_interval:
                        query_vec = tree.extract_feature_vector(motion_entry['current_feature'])
                        dist, idx = tree.tree.query(query_vec)

                        if dist < distance_threshold:
                            matched_motion, matched_idx, path = tree.index_map[idx]
                            matched_vec = tree.feature_vectors[idx]

                            motion_entry['motion'] = matched_motion
                            motion_entry['frame_idx'] = matched_idx
                            motion_entry['last_matched_frame'] = frame_idx

                            print(f"[MATCH] {path} @ {matched_idx} (distance: {dist:.2f})")

                            # --- 추가 출력: 벡터 비교 ---
                            np.set_printoptions(precision=3, suppress=True, linewidth=120)
                            print("▶ query_vec:")
                            print(query_vec)

                            print("▶ matched_vec:")
                            print(matched_vec)

                            print("▶ diff (query - matched):")
                            print(query_vec - matched_vec)

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
    file_path = "./bvh/data/002/02_03.bvh"
    root_path = './bvh/data/exp'
    init_motion(file_path)
    tree = MotionKDTree(root_path)
    main()