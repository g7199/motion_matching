from pyglm import glm
import numpy as np
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

joint_size = 0.3

from utils import *

def draw_humanoid(root_position, root_joint):
    glPushMatrix()
    glTranslatef(*root_position)
    draw_joint(root_joint)
    glPopMatrix()

def draw_joint(joint):
    glPushMatrix()
    glTranslatef(*joint.offset)

    if joint is None:
        return

    if joint.rotation is not None:
        for channel, angle in zip(joint.channels, joint.rotation):
            if "Xrotation" in channel:
                glRotatef(angle, 1, 0, 0)
            elif "Yrotation" in channel:
                glRotatef(angle, 0, 1, 0)
            elif "Zrotation" in channel:
                glRotatef(angle, 0, 0, 1)

    draw_colored_sphere(joint_size)

    for child in joint.children:
        glPushMatrix()
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