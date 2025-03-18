from OpenGL.GL import *
from pyglm import glm
import numpy as np
from utils import draw_colored_cube, draw_colored_sphere

joint_size = 3

def draw_humanoid(root_position, root_joint):
    """
    Skeleton을 그리기 위한 함수입니다.
    :param root_position: skeleton을 그리기 시작할 position
    :param root_joint: 그릴 joint
    """
    glPushMatrix()
    glTranslatef(*root_position)
    draw_joint(root_joint)
    glPopMatrix()

def draw_joint(joint):
    """
    Joint를 그리기 위한 함수입니다.
    만약 joint면 관절을 표현하는 sphere를 그리고 아니라면 뼈대를 그립니다.
    """
    glPushMatrix()
    glMultMatrixf(joint.kinetics.T.flatten())
    if joint.name != "joint_Root":
        draw_colored_sphere(joint_size)
    for child in joint.children:
        glPushMatrix()
        if joint.name != "joint_Root":
            draw_bone(child.offset)
        draw_joint(child)
        glPopMatrix()
    glPopMatrix()

def draw_bone(offset):
    """
    Skeleton에서 뼈를 그리기 위한 함수입니다.
    :param offset: 뼈 길이를 구하기 위한 값
    """
    mid = [offset[0] / 2.0, offset[1] / 2.0, offset[2] / 2.0]
    from utils import bone_rotation
    rot_quat = bone_rotation(glm.vec3(*offset))
    rot_mat = glm.mat4_cast(rot_quat)
    glPushMatrix()
    glTranslatef(*mid)
    glMultMatrixf(np.array(rot_mat, dtype=np.float32).flatten())
    glScalef(joint_size, abs(glm.l2Norm(offset) - 2 * joint_size) / 2, joint_size/3)
    draw_colored_cube(1)
    glPopMatrix()

def draw_virtual_root_axis(virtual_root, rotation, axis_length=10.0):
    """
    root Transform T에서 조그만한 3차원 축을 그리기 위함입니다.
    virtual root의 위치를 받아 rotation만큼 회전하여 그려 pelvis의 회전을 시각적으로 확인할 수 있습니다.
    :param virtual_root: 축을 그릴 root
    :param rotation: 적용할 회전값
    :param axis_length: 축 크기 (기본값 10)
    """
    pos = virtual_root.offset  # Assumed to be [x, y, z]
    glPushMatrix()
    glTranslatef(pos[0], pos[1], pos[2])
    glMultMatrixf(rotation)
    glLineWidth(2.0)
    glBegin(GL_LINES)
    # X-axis in red.
    glColor3f(1, 0, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(axis_length, 0, 0)
    # Y-axis in green.
    glColor3f(0, 1, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(0, axis_length, 0)
    # Z-axis in blue.
    glColor3f(0, 0, 1)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, axis_length)
    glEnd()
    glPopMatrix()