# transforms.py
import numpy as np

def get_rotation_matrix(channel, angle_deg):
    """
    채널에 존재하는 각도를 회전 행렬로 바꿔주는 함수입니다.
    :param channel: 바꿀 채널
    :param angle_deg: 각도 input
    :return: 회전 행렬
    """
    theta = np.deg2rad(angle_deg)
    if "Xrotation" in channel:
        return np.array([
            [1, 0, 0, 0],
            [0, np.cos(theta), -np.sin(theta), 0],
            [0, np.sin(theta),  np.cos(theta), 0],
            [0, 0, 0, 1]
        ])
    elif "Yrotation" in channel:
        return np.array([
            [ np.cos(theta), 0, np.sin(theta), 0],
            [ 0, 1, 0, 0],
            [-np.sin(theta), 0, np.cos(theta), 0],
            [0, 0, 0, 1]
        ])
    elif "Zrotation" in channel:
        return np.array([
            [np.cos(theta), -np.sin(theta), 0, 0],
            [np.sin(theta),  np.cos(theta), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
    else:
        return np.identity(4)

def translation_matrix(offset):
    """
    Translation (x,y,z)를 행렬로 바꿔주는 함수입니다.
    :param offset: (x,y,z) translation vector
    :return: translation 행렬
    """
    tx, ty, tz = offset
    return np.array([
        [1, 0, 0, tx],
        [0, 1, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1]
    ])

def compute_forward_kinetics(node, rotations):
    """
    각 joint별로 Forward Kinemetic을 구현하기 위한 행렬입니다.
    Joint에 저장되어있는 local translation 과 rotation을 적용합니다.
    :param node: 적용할 Node (Joint)
    :param rotations: rotation 값
    :return: Forward Kinetic을 적용한 4x4 행렬
    """
    M = translation_matrix(node.offset)
    channels = node.channels[-3:]  # 마지막 채널 3개가 rotation값
    if rotations is not None:
        for channel, angle in zip(channels, rotations):
            M = M @ get_rotation_matrix(channel, angle)
    return M

def extract_yaw_rotation(kinetics):
    """
    회전행렬에서 yaw값만을 추출하여, offset을 적용한 4x4 행렬을 반환합니다.
    기존 방식 대신 회전행렬의 특정 요소를 이용해 yaw를 안정적으로 계산합니다.
    
    :param kinetics: 4x4 변환 행렬 (회전 포함)
    :param offset: 3차원 offset 벡터 (예: [x, y, z])
    :return: yaw 회전만을 적용한 4x4 행렬
    """
    # 회전 행렬에서 yaw를 직접 추출 (현재 좌표계에 맞게 수정 필요)
    R_mat = kinetics[:3, :3]
    offset = kinetics[:3, 3]
    # 예: R_mat[0,2]와 R_mat[2,2]를 사용 (좌표계에 따라 부호나 순서가 달라질 수 있음)
    yaw = np.arctan2(R_mat[0, 2], R_mat[2, 2])
    
    cos_y = np.cos(yaw)
    sin_y = np.sin(yaw)
    rotation_y = np.array([
        [cos_y, 0, sin_y, offset[0]],
        [0,     1, 0,     offset[1]],
        [-sin_y,0, cos_y, offset[2]],
        [0,     0, 0,     1]
    ], dtype=float)
    return rotation_y

def motion_adapter(root, motion_frame):
    """
    root를 목표로 motion을 적용시키기 위한 함수입니다.
    add_motion 함수를 콜해 적용시키고 root position과 root를 return합니다.
    :param root: 적용할 root
    :param motion_frame: 모션 프레임값
    :return: root_position과 root를 return
    """
    add_motion(root, motion_frame, idx=[0])
    root_position = list(map(float, motion_frame[:3]))

    return root_position, root

def add_motion(node, motion_frame, idx=[0]):
    """
    모션을 root에 재귀적으로 더해주는 함수입니다.
    :param node: 적용할 node, 재귀적으로 작동한다.
    :param motion_frame: 모션프레임
    :param idx: 인덱스
    """
    if not node:
        return

    if node.name != "Site":
        if len(node.channels) == 6:
            # 첫 3개가 position 다음 3개가 rotation
            idx[0] += 3
            rotation = list(map(float, motion_frame[idx[0]:idx[0] + 3]))
            node.kinetics = compute_forward_kinetics(node, rotation)
            idx[0] += 3
        elif len(node.channels) == 3:
            rotation = list(map(float, motion_frame[idx[0]:idx[0] + 3]))
            node.kinetics = compute_forward_kinetics(node, rotation)
            idx[0] += 3

    for child in node.children:
        add_motion(child, motion_frame, idx)

def inverse_matrix(T):
    """
    역행렬 구해주는 함수
    :param T: 역행렬을 구할 행렬
    :return: T의 역행렬
    """
    R = T[:3, :3]
    t = T[:3, 3]
    R_inv = R.T
    t_inv = -R_inv @ t
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    return T_inv

def get_projection(v, onto):
    onto_norm = onto / np.linalg.norm(onto)
    proj = np.dot(v, onto_norm) * onto_norm
    return proj

import numpy as np
from scipy.spatial.transform import Rotation as R

def lookrotation(v, u):
    """
    기존 lookrotation의 cross 기반 구현에
    회전행렬 -> 쿼터니언 변환 후 w<0이면 뒤집는 과정을 추가.
    """
    # 1) 입력 벡터 정규화
    u_hat = u / np.linalg.norm(u)
    v_hat = v / np.linalg.norm(v)

    # 2) 교차곱으로 right축(t_hat) 구하고, up축 재계산
    vxu = np.cross(u_hat, v_hat)
    t_hat = vxu / np.linalg.norm(vxu)
    # forward축 = v_hat
    # up축      = forward x right = v_hat x t_hat

    # 3) 열방향(column) = [right, up, forward]
    rot_mat = np.array([
        t_hat,                     # right
        np.cross(v_hat, t_hat),   # up
        v_hat                     # forward
    ]).T  # shape (3,3)

    # 4) 쿼터니언 변환
    rot_q = R.from_matrix(rot_mat).as_quat()  # [x, y, z, w]

    # 5) w < 0이면 부호 뒤집기
    if rot_q[3] < 0:
        rot_q = -rot_q

    # 6) 보정된 쿼터니언을 다시 회전행렬로
    rot_mat_fixed = R.from_quat(rot_q).as_matrix()
    return rot_mat_fixed


def get_pelvis_virtual(kinetics):

    ap = kinetics[:3, 3]
    ar = kinetics[:3, :3]
    upVector = np.array([0,1,0], dtype=float)
    p = ap - get_projection(ap, upVector)
    f = ar[:, 2]
    r = lookrotation(f-get_projection(f, upVector), upVector)
    ap_transformed = r.T @ (ap - p)
    ar_transformed = r.T @ ar

    next_kinetics = np.eye(4)
    next_kinetics[:3, :3] = ar_transformed
    next_kinetics[:3, 3] = ap_transformed
    
    return next_kinetics

def extract_xz_plane(kinetics):
    R_mat = kinetics[:3, :3]
    offset = kinetics[:3, 3]
    # 예: R_mat[0,2]와 R_mat[2,2]를 사용 (좌표계에 따라 부호나 순서가 달라질 수 있음)
    yaw = np.arctan2(R_mat[0, 2], R_mat[2, 2])
    
    cos_y = np.cos(yaw)
    sin_y = np.sin(yaw)
    rotation_y = np.array([
        [cos_y, 0, sin_y, offset[0]],
        [0,     1, 0,     0        ],
        [-sin_y,0, cos_y, offset[2]],
        [0,     0, 0,     1        ]
    ], dtype=float)
    return rotation_y

def remove_yaw_from_matrix(mat4):
    # 3x3 회전행렬 분리
    R_mat = mat4[:3, :3]

    # yaw 추출 (현재 +Z가 전방이라고 가정)
    yaw = np.arctan2(R_mat[0, 2], R_mat[2, 2])
    # 그 역회전
    inv_yaw = -yaw

    # Y축 회전행렬
    cy = np.cos(inv_yaw)
    sy = np.sin(inv_yaw)
    R_yaw_inv = np.array([
        [cy,  0, sy],
        [ 0,  1,  0],
        [-sy, 0, cy],
    ], dtype=float)

    # mat4 복사
    new_mat = mat4.copy()
    # 새 회전행렬 = (R_yaw_inv) * (원래 R_mat)
    new_rot = R_yaw_inv @ R_mat
    new_mat[:3, :3] = new_rot
    return new_mat