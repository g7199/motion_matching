from pyglm import glm
import numpy as np
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

joint_size = 3

from utils import *

def draw_humanoid(root_position, root_joint):
    glPushMatrix()
    glTranslatef(*root_position)
    draw_joint(root_joint)
    glPopMatrix()

def draw_joint(joint):
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
    mid = [offset[0] / 2.0, offset[1] / 2.0, offset[2] / 2.0]
    rot_quat = bone_rotation(glm.vec3(*offset))
    rot_mat = glm.mat4_cast(rot_quat)

    glPushMatrix()
    glTranslatef(*mid)
    glMultMatrixf(np.array(rot_mat, dtype=np.float32).flatten())
    glScalef(joint_size, abs(glm.l2Norm(offset) - 2 * joint_size) / 2, joint_size)
    draw_colored_cube(1)
    glPopMatrix()