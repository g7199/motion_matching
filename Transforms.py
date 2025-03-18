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

def extract_yaw_rotation(kinetics, offset):
    """
    회전행렬에서 yaw값만을 추출하기 위한 함수입니다.
    XZ평면에 사영된 root Transform T 에 y축 회전을 적용하느넫 사용됩니다.
    :param kinetics: 적용되고 있는 회전
    :return: yaw값만을 담고있는 4x4 행렬
    """
    forward = kinetics[:3, 2].copy()
    forward[1] = 0  # Project onto XZ plane.
    norm = np.linalg.norm(forward)
    if norm != 0:
        forward /= norm
    else:
        forward = np.array([0, 0, 1])

    yaw = -np.arctan2(forward[0], forward[2])
    cos_y = np.cos(yaw)
    sin_y = np.sin(yaw)
    rotation_y = np.array([
        [cos_y, 0, sin_y, offset[0]],
        [0, 1, 0, offset[1]],
        [-sin_y, 0, cos_y, offset[2]],
        [0, 0, 0, 1]
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

def lookrotation(v, u):
    u_hat = u/np.linalg.norm(u)
    v_hat = v/np.linalg.norm(v)

    vxu = np.cross(u_hat, v_hat)
    t_hat = vxu/np.linalg.norm(vxu)

    R = np.array([t_hat, np.cross(v_hat, t_hat), v_hat]).T
    return R

def get_pelvis_virtual(kinetics):

    ap = kinetics[:3, 3]
    ar = kinetics[:3, :3]
    upVector = np.array([0,1,0], dtype=float)
    p = ap - get_projection(ap, upVector)
    f = ar[:, 2]
    r = lookrotation(f-get_projection(f, upVector), upVector)
    ap_transformed = r.T @ (ap - p)
    ar_transformed = r.T

    kinetics = np.eye(4)
    kinetics[:3, :3] = ar_transformed
    kinetics[:3, 3] = ap_transformed
    
    return kinetics