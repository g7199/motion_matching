import math
from pygame.locals import *
import pygame
from pyglm import glm

class InputController:
    def __init__(self, max_speed=40.0, turn_strength=1.0):
        self.max_speed = max_speed
        self.turn_strength = turn_strength
        self.current_forward = glm.vec3(0, 0, 1)
        self.input_state = {'W': False, 'A': False, 'S': False, 'D': False}
        self.prev_pos = glm.vec3(0, 0, 0)
        self.acceleration = 2.0 
        self.deceleration = 3.0 
        self.current_velocity = glm.vec3(0)

    def update(self, keys):
        self.input_state['W'] = keys[pygame.K_w]
        self.input_state['A'] = keys[pygame.K_a]
        self.input_state['S'] = keys[pygame.K_s]
        self.input_state['D'] = keys[pygame.K_d]

    def compute_velocity(self, delta_time):
        vel = glm.vec3(0)
        if self.input_state['W']: vel += glm.vec3(0, 0, -1)
        if self.input_state['S']: vel += glm.vec3(0, 0, 1)
        if self.input_state['A']: vel += glm.vec3(-1, 0, 0)
        if self.input_state['D']: vel += glm.vec3(1, 0, 0)

        if glm.length(vel) > 0:
            vel = glm.normalize(vel) * self.max_speed
            self.current_velocity += (vel - self.current_velocity) * self.acceleration * delta_time
        else:
            self.current_velocity -= self.current_velocity * self.deceleration * delta_time
            
        return self.current_velocity

    def compute_vel_forward(self, delta_time):
        vel = self.compute_velocity(delta_time)
        turn_rate = 0.0

        if glm.length(vel) > 0:
            desired = glm.normalize(vel)

            # 회전 전에 이전 방향 저장
            prev_forward = glm.normalize(self.current_forward)

            # 방향 보간
            q_current = glm.quat(glm.vec3(0, 0, 1), prev_forward)
            q_target = glm.quat(glm.vec3(0, 0, 1), desired)

            speed = glm.length(vel)
            slerp_amount = min(0.2, speed * 0.02)

            q_new = glm.slerp(q_current, q_target, slerp_amount)
            self.current_forward = glm.normalize(q_new * glm.vec3(0, 0, 1))

            # 회전 각도 계산
            dot = glm.clamp(glm.dot(prev_forward, self.current_forward), -1.0, 1.0)
            angle = glm.acos(dot)
            cross = glm.cross(prev_forward, self.current_forward)
            if cross.y < 0:
                angle *= -1
            turn_rate = angle / delta_time

        return vel, self.current_forward, turn_rate
    
    def update_virtual_kinematics(self, virtual_root, delta_time):
        velocity, direction, turn_rate = self.compute_vel_forward(delta_time)
        new_pos = self.prev_pos + velocity * delta_time
        base_forward = glm.vec3(0,0,1)
        rot_quat = glm.quat(base_forward, glm.normalize(self.current_forward))
        virtual_root.kinematics = glm.translate(glm.mat4(1.0), new_pos) * glm.mat4_cast(rot_quat)
        self.prev_pos = new_pos

        return new_pos, direction, turn_rate


def update_eye(center, distance, yaw, pitch):
    """
    변화하는 input값에 따라 카메라를 update하는 함수입니다
    :param center: 카메라 중심값
    :param distance: 카메라 거리값
    :param yaw: 카메라 yaw 값 (rotation)
    :param pitch: 카메라 pitch 값 (rotation)
    """
    max_pitch = math.radians(89)
    pitch = max(-max_pitch, min(pitch, max_pitch))
    offset_x = distance * math.sin(yaw) * math.cos(pitch)
    offset_y = distance * math.sin(pitch)
    offset_z = distance * math.cos(yaw) * math.cos(pitch)
    return center + glm.vec3(offset_x, offset_y, offset_z)

def handle_mouse_motion(event, state):
    """
    마우스 입력을 처리하는 함수입니다.
    :param event: 이벤트 (드래그)를 분기점으로 하기 위한 param
    :param state: main에서 프로그램을 관리하기 위한 global 한 딕셔너리
    """
    xpos, ypos = event.pos
    dx = xpos - state['last_x']
    dy = ypos - state['last_y']
    state['last_x'], state['last_y'] = xpos, ypos

    if state['is_rotating']:
        sensitivity = 0.005
        state['yaw'] -= dx * sensitivity
        state['pitch'] += dy * sensitivity
        state['eye'] = update_eye(state['center'], state['distance'], state['yaw'], state['pitch'])
    elif state['is_translating']:
        sensitivity = 0.005 * state['distance']
        view_dir = glm.normalize(state['center'] - state['eye'])
        right = glm.normalize(glm.cross(view_dir, glm.vec3(0, 1, 0)))
        up = glm.vec3(0, 1, 0)
        translation = (-dx * right + dy * up) * sensitivity
        state['center'] += translation
        state['eye'] = update_eye(state['center'], state['distance'], state['yaw'], state['pitch'])

def handle_mouse_button(event, state):
    """
    마우스 클릭을 처리하는 함수입니다.
    :param event: 이벤트 (클릭)를 분기점으로 하기 위한 param
    :param state: main에서 프로그램을 관리하기 위한 global 한 딕셔너리
    """
    if event.type == pygame.MOUSEBUTTONDOWN:
        state['last_x'], state['last_y'] = event.pos
        if event.button == 1:
            state['is_rotating'] = True
        elif event.button == 3:
            state['is_translating'] = True
        if event.button in (4, 5):
            handle_mouse_wheel(event, state)
    elif event.type == pygame.MOUSEBUTTONUP:
        if event.button == 1:
            state['is_rotating'] = False
        elif event.button == 3:
            state['is_translating'] = False

def handle_mouse_wheel(event, state):
    """
    마우스 스크롤을 처리하는 함수입니다.
    :param event: 이벤트 (스크롤)를 분기점으로 하기 위한 param
    :param state: main에서 프로그램을 관리하기 위한 global 한 딕셔너리
    """
    zoom_sensitivity = 0.1
    if hasattr(event, 'y'):
        state['distance'] -= zoom_sensitivity * event.y * state['distance']
    else:
        if event.button == 4:
            state['distance'] -= zoom_sensitivity * state['distance']
        elif event.button == 5:
            state['distance'] += zoom_sensitivity * state['distance']
    state['distance'] = max(state['distance'], 0.1)
    state['eye'] = update_eye(state['center'], state['distance'], state['yaw'], state['pitch'])
