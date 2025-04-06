from OpenGL.GL import *
from pyglm import glm
import numpy as np
from utils import draw_colored_cube, draw_colored_sphere, bone_rotation, draw_arrow, draw_undercircle, draw_arrow_from_direction

joint_size = 0.8

#OpenGL_accelerate 사용하면 numpy로 변환해줘야함.
def glm_mat4_to_glf(m: glm.mat4) -> np.ndarray:
    return np.array(m.to_list(), dtype=np.float32).flatten()

def draw_humanoid(root_joint, color):
    """
    Skeleton을 그리기 위한 함수입니다.
    :param root_joint: 그릴 joint (전역 kinematics가 이미 계산되어 있음)
    :param color: RGB 컬러 (tuple 또는 list of 3 floats)
    """
    glPushMatrix()
    glMultMatrixf(glm_mat4_to_glf(root_joint.kinematics))
    draw_joint(root_joint.children[0], color)
    glPopMatrix()

def draw_joint(joint, color):
    """
    Joint를 그리기 위한 함수입니다.
    전역 좌표계의 kinematics를 그대로 적용하고, 관절이면 sphere, 아니라면 뼈대를 그립니다.
    :param color: RGB 컬러
    """
    glPushMatrix()
    glMultMatrixf(glm_mat4_to_glf(joint.kinematics))
    
    if joint.name != "joint_Root":
        draw_colored_sphere(joint_size)
    
    for child in joint.children:
        if joint.name != "joint_Root":
            draw_bone(child.offset, color)
        draw_joint(child, color)
    
    glPopMatrix()

def draw_bone(offset, color):
    """
    Skeleton에서 뼈를 그리기 위한 함수입니다.
    :param offset: 뼈 길이를 구하기 위한 값
    :param color: RGB 컬러
    """
    mid = [offset[0] / 2.0, offset[1] / 2.0, offset[2] / 2.0]
    rot_quat = bone_rotation(glm.vec3(*offset))
    rot_mat = glm.mat4_cast(rot_quat)
    glPushMatrix()
    glTranslatef(*mid)
    glMultMatrixf(np.array(rot_mat, dtype=np.float32).flatten())
    glScalef(joint_size, abs(glm.length(glm.vec3(*offset)) - 2 * joint_size) / 2, joint_size / 3)
    draw_colored_cube(1, color=color)
    glPopMatrix()

def draw_virtual_root_axis(kinematics, color, circle_radius=2, arrow_length=4):
    """
    root Transform에서 조그만한 3차원 축을 그리기 위함입니다.
    virtual root의 위치를 받아 회전만큼 회전하여 pelvis의 회전을 시각적으로 확인합니다.
    """
    glPushMatrix()
    glMultMatrixf(glm_mat4_to_glf(kinematics))
    draw_arrow(circle_radius, arrow_length, color)
    glRotatef(90, 1.0, 0.0, 0.0)
    glColor3f(1.0, 1.0, 1.0)
    draw_undercircle(circle_radius)
    glPopMatrix()

def draw_matching_features(root_joint, frame, circle_radius = 2):
    if frame is not None:
        #glPushMatrix()
        glMultMatrixf(glm_mat4_to_glf(root_joint.kinematics))

        for pos, rot in zip(frame.future_position, frame.future_orientation):
            glPushMatrix()
            glTranslatef(pos.x, pos.y, pos.z)

            draw_arrow_from_direction(rot)
            glRotatef(90, 1.0, 0.0, 0.0)
            # 3. 해당 위치에 구(점) 그리기
            glColor3f(0.0, 1.0, 0.0)  # 초록색 점
            draw_undercircle(circle_radius)

            glPopMatrix()
        #glPopMatrix()