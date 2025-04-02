from OpenGL.GL import *
from pyglm import glm
import numpy as np
from utils import draw_colored_cube, draw_colored_sphere, bone_rotation, draw_arrow, draw_undercircle

joint_size = 3

def glm_mat4_to_glf(m: glm.mat4) -> np.ndarray:
    return np.array(m.to_list(), dtype=np.float32).flatten()

def draw_humanoid(root_joint):
    """
    Skeleton을 그리기 위한 함수입니다.
    :param root_joint: 그릴 joint (전역 kinematics가 이미 계산되어 있음)
    """
    glPushMatrix()
    glMultMatrixf(glm_mat4_to_glf(root_joint.kinematics))
    # root_joint.kinematics를 draw_joint 내에서 적용하므로 여기서는 단순히 재귀 호출합니다.
    draw_joint(root_joint.children[0])
    glPopMatrix()

def draw_joint(joint):
    """
    Joint를 그리기 위한 함수입니다.
    전역 좌표계의 kinematics를 그대로 적용하고, 관절이면 sphere, 아니라면 뼈대를 그립니다.
    """
    glPushMatrix()
    # 이미 전역 변환으로 계산된 joint.kinematics를 적용
    glMultMatrixf(glm_mat4_to_glf(joint.kinematics))
    
    if joint.name != "joint_Root":
        draw_colored_sphere(joint_size)
    
    for child in joint.children:
        # 부모의 local 좌표계에서 뼈를 먼저 그립니다.
        if joint.name != "joint_Root":
            draw_bone(child.offset)
        # 자식 joint는 draw_joint 내부에서 각각 glPush/Pop하여 전역 변환을 적용
        draw_joint(child)
    
    glPopMatrix()

def draw_bone(offset):
    """
    Skeleton에서 뼈를 그리기 위한 함수입니다.
    :param offset: 뼈 길이를 구하기 위한 값
    """
    mid = [offset[0] / 2.0, offset[1] / 2.0, offset[2] / 2.0]
    rot_quat = bone_rotation(glm.vec3(*offset))
    rot_mat = glm.mat4_cast(rot_quat)
    glPushMatrix()
    glTranslatef(*mid)
    glMultMatrixf(np.array(rot_mat, dtype=np.float32).flatten())
    glScalef(joint_size, abs(glm.length(glm.vec3(*offset)) - 2 * joint_size) / 2, joint_size / 3)
    draw_colored_cube(1)
    glPopMatrix()

def draw_virtual_root_axis(kinematics, circle_radius=10, arrow_length=20):
    """
    root Transform에서 조그만한 3차원 축을 그리기 위함입니다.
    virtual root의 위치를 받아 회전만큼 회전하여 pelvis의 회전을 시각적으로 확인합니다.
    """
    glPushMatrix()
    glMultMatrixf(glm_mat4_to_glf(kinematics))
    draw_arrow(circle_radius, arrow_length)
    glRotatef(90, 1.0, 0.0, 0.0)
    glColor3f(1.0, 1.0, 1.0)
    draw_undercircle(10)
    glPopMatrix()
