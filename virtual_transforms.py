from pyglm import glm
import math

def get_projection(v: glm.vec3, onto: glm.vec3):
    onto_norm = glm.normalize(onto)
    return glm.dot(v, onto_norm) * onto_norm


def lookrotation(v: glm.vec3, u: glm.vec3) -> glm.quat:
    v_hat = glm.normalize(v)
    u_hat = glm.normalize(u)
    t_hat = glm.normalize(glm.cross(u_hat, v_hat))
    up_corrected = glm.cross(v_hat, t_hat)
    rot_mat = glm.mat3(t_hat, up_corrected, v_hat)
    rot_q = glm.quat_cast(rot_mat)
    if rot_q.w < 0:
        rot_q = -rot_q
    return rot_q


def get_pelvis_virtual(ap: glm.vec3, ar: glm.quat):
    up = glm.vec3(0, 1, 0)
    p = ap - get_projection(ap, up)
    f = ar * glm.vec3(0, 0, 1)
    f_mod = f - get_projection(f, up)
    r = lookrotation(f_mod, up)
    r_inv = glm.inverse(r)
    new_ap = r_inv * (ap - p)
    new_ar = r_inv * ar
    return new_ap, new_ar

prev_r_inv = glm.quat(1, 0, 0, 0)  # 전역에서 초기화해 사용 가능

def get_pelvis_virtual_safe(ap: glm.vec3, ar: glm.quat, 
                            fallback_forward=glm.vec3(0, 0, 1),
                            smooth_ratio=0.2,
                            prev_r_inv_ref=None):
    """
    안정적으로 virtual root를 추출하는 개선된 함수입니다.
    :param ap: pelvis 위치 (world 기준)
    :param ar: pelvis 회전 (world 기준)
    :param fallback_forward: 비정상 시 사용하는 전방 벡터
    :param smooth_ratio: slerp 보간 비율 (0~1, 높을수록 빠르게 변화)
    :param prev_r_inv_ref: 이전 프레임의 r_inv 값을 참조하고 갱신할 리스트 [quat]
    :return: (new_ap, new_ar)
    """
    up = glm.vec3(0, 1, 0)

    # 수직 성분 제거 (위치 기준 평면화)
    p = ap - get_projection(ap, up)

    # 현재 바라보는 방향 벡터 추출 후 수평화
    f = ar * glm.vec3(0, 0, 1)
    f_mod = f - get_projection(f, up)

    # 벡터 길이 체크 후 보정
    if glm.length(f_mod) < 1e-4:
        f_mod = fallback_forward

    # 회전 정렬 및 역변환
    r = lookrotation(f_mod, up)
    r_inv = glm.inverse(r)

    # low-pass filtering (이전 값과 slerp)
    if prev_r_inv_ref is not None:
        r_inv = glm.slerp(prev_r_inv_ref[0], r_inv, smooth_ratio)
        prev_r_inv_ref[0] = r_inv  # 업데이트

    # 회전 기준 정렬
    new_ap = r_inv * (ap - p)
    new_ar = r_inv * ar
    return new_ap, new_ar



def extract_xz_plane(kinetics: glm.mat4) -> glm.mat4:
    R_mat = glm.mat3(kinetics)
    yaw = math.atan2(R_mat[2][0], R_mat[2][2])
    offset = glm.vec3(kinetics[3])
    rotation_y = glm.rotate(glm.mat4(1.0), yaw, glm.vec3(0, 1, 0))
    rotation_y[3].x = offset.x
    rotation_y[3].z = offset.z
    rotation_y[3].y = 0.0
    return rotation_y