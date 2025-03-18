import numpy as np

class Joint:
    """
    관절을 정의하는 Joint Class
    """
    def __init__(self, name):
        self.name = name
        self.channels = []
        self.children = []
        self.offset = [0, 0, 0]
        self.kinetics = np.identity(4, dtype=float)
        self.parent = None

def bvh_parser(file_path):
    """
    BVH_Data를 받아서 parsing하는 함수입니다.
    이때 마지막 단계에서 virtual_root 노드를 추가로 더해 root Transform T로 사용합니다.
    :param file_path: 파일 경로
    """
    stack = []
    root = None
    cur_node = None
    motion = []
    is_motion = False

    try:
        with open(file_path, "r", encoding="utf-8") as file:
            for line in file:
                parts = line.split()
                if not parts:
                    continue

                if is_motion:
                    if parts[0] in ["Frames:", "Frame"]:
                        continue
                    motion_frame = parts[:]  # List of strings (convert later as needed)
                    motion.append(motion_frame)
                    continue

                if parts[0] == "MOTION":
                    is_motion = True

                if parts[0] in ["ROOT", "JOINT", "End"]:
                    node = Joint(parts[1])
                    node.parent = cur_node
                    if not root:
                        root = node
                    if cur_node:
                        cur_node.children.append(node)

                    stack.append(node)
                    cur_node = node

                elif parts[0] == "OFFSET":
                    cur_node.offset = list(map(float, parts[1:]))

                elif parts[0] == "CHANNELS":
                    cur_node.channels = parts[2:]

                elif parts[0] == "}":
                    stack.pop()
                    if stack:
                        cur_node = stack[-1]
        if root is not None:
            # Virtual root 생성
            virtual_root = Joint("virtual_root")
            virtual_root.channels = []  # Motion Channel은 null
            virtual_root.offset = [0, 0, 0]
            virtual_root.children.append(root)
            root.parent = virtual_root
            root = virtual_root
        return root, motion

    except FileNotFoundError:
        print(f"Error: File '{file_path}' does not exist.")

def check_bvh_structure(joint, is_root=False):
    """
    BVH_Data의 구조가 valid한지 재귀적으로 확인하는 함수입니다.
    :param joint: 확인할 joint
    """
    if joint.name == "virtual_root":
        if not joint.children:
            raise ValueError("Virtual root has no children.")
        for child in joint.children:
            check_bvh_structure(child, is_root=True)
        return
    if is_root:
        if len(joint.channels) != 6:
            raise ValueError(f"Root joint '{joint.name}' must have 6 channels, found {len(joint.channels)}")
        for channel in joint.channels[:3]:
            if "position" not in channel.lower():
                raise ValueError(
                    f"Root joint '{joint.name}' first three channels must be position channels, found '{channel}'")
        for channel in joint.channels[3:]:
            if "rotation" not in channel.lower():
                raise ValueError(
                    f"Root joint '{joint.name}' last three channels must be rotation channels, found '{channel}'")
    else:
        if joint.channels:
            if len(joint.channels) != 3:
                for channel in joint.channels[3:]:
                    if "rotation" not in channel.lower():
                        raise ValueError(f"Joint '{joint.name}' channel must be a rotation channel, found '{channel}'")
            else:
                for channel in joint.channels:
                    if "rotation" not in channel.lower():
                        raise ValueError(f"Joint '{joint.name}' channel must be a rotation channel, found '{channel}'")

    for child in joint.children:
        check_bvh_structure(child, is_root=False)

