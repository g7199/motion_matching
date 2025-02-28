from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

from pyglm import glm

colors = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [1.0, 1.0, 0.0],
    [1.0, 0.0, 1.0],
    [0.0, 1.0, 1.0]
]
vertices = [
    [-1.0, -1.0, 1.0], [1.0, -1.0, 1.0], [1.0, 1.0, 1.0], [-1.0, 1.0, 1.0],
    [-1.0, -1.0, -1.0], [1.0, -1.0, -1.0], [1.0, 1.0, -1.0], [-1.0, 1.0, -1.0],
    [-1.0, -1.0, -1.0], [-1.0, -1.0, 1.0], [-1.0, 1.0, 1.0], [-1.0, 1.0, -1.0],
    [1.0, -1.0, -1.0], [1.0, -1.0, 1.0], [1.0, 1.0, 1.0], [1.0, 1.0, -1.0],
    [-1.0, 1.0, 1.0], [1.0, 1.0, 1.0], [1.0, 1.0, -1.0], [-1.0, 1.0, -1.0],
    [-1.0, -1.0, 1.0], [1.0, -1.0, 1.0], [1.0, -1.0, -1.0], [-1.0, -1.0, -1.0]
]
normals = [
    [0.0, 0.0, 1.0],
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, -1.0, 0.0]
]


def draw_colored_cube(size_x, size_y=-1, size_z=-1):
    if (size_y < 0): size_y = size_x
    if (size_z < 0): size_z = size_x
    glPushMatrix()
    glScaled(size_x, size_y, size_z)
    glBegin(GL_QUADS)
    for i in range(6):
        glColor3fv(colors[i])
        glNormal3fv(normals[i])
        for j in range(4):
            glVertex3fv(vertices[i * 4 + j])
    glEnd()
    glPopMatrix()


def draw_colored_sphere(radius):
    glPushMatrix()
    glColor3fv(colors[0])
    glutSolidSphere(radius, 30, 30)
    glPopMatrix()


def draw_axes():
    glLineWidth(3.0)
    glBegin(GL_LINES)
    len = 20.0

    # X축 - 빨간색
    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(-len, 0.0, 0.0)  # X축의 시작점
    glVertex3f(len, 0.0, 0.0)  # X축의 끝점

    # Y축 - 초록색
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(0.0, -len, 0.0)  # Y축의 시작점
    glVertex3f(0.0, len, 0.0)  # Y축의 끝점

    # Z축 - 파란색
    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(0.0, 0.0, -len)  # Z축의 시작점
    glVertex3f(0.0, 0.0, len)  # Z축의 끝점

    glEnd()


def set_lights():
    # 조명 설정
    glEnable(GL_LIGHTING)  # 조명 활성화
    glEnable(GL_LIGHT0)  # 기본 조명 활성화
    glEnable(GL_DEPTH_TEST)  # 깊이 테스트 활성화

    # 조명 파라미터 설정
    ambient_light = [0.5, 0.5, 0.5, 1.0]  # 주변광
    diffuse_light = [0.2, 0.2, 0.2, 1.0]  # 난반사광
    r = 0.5
    specular_light = [r, r, r, 1.0]  # 정반사광
    position = [5.0, 5.0, 5.0, 0.0]  # 조명 위치

    # 조명 속성 적용
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient_light)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse_light)
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular_light)
    glLightfv(GL_LIGHT0, GL_POSITION, position)

    # 재질(Material) 설정
    glEnable(GL_COLOR_MATERIAL)  # 재질 색상 활성화
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)  # 재질 색상 설정
    specular_reflection = [r, r, r, 1.0]
    glMaterialfv(GL_FRONT, GL_SPECULAR, specular_reflection)  # 정반사 재질
    glMateriali(GL_FRONT, GL_SHININESS, 1)  # 광택 설정


def rotation_between_vectors(v1, v2):
    v1 = glm.normalize(v1)
    v2 = glm.normalize(v2)
    cos_theta = glm.dot(v1, v2)

    rotation_axis = glm.cross(v1, v2)
    s = glm.sqrt((1 + cos_theta) * 2)
    invs = 1 / s
    return glm.quat(s * 0.5,
                    rotation_axis.x * invs,
                    rotation_axis.y * invs,
                    rotation_axis.z * invs)


def bone_rotation(forward):
    originalDir = glm.vec3(0, 1, 0)
    if glm.length2(glm.cross(originalDir, forward)) < 1e-6:
        if glm.dot(originalDir, forward) < 0:
            rot = glm.angleAxis(glm.pi(), glm.vec3(0, 1, 0))
        else:
            rot = glm.quat(1, 0, 0, 0)
    else:
        rot = rotation_between_vectors(forward, originalDir)
    return rot