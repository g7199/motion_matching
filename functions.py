class Joint:
    def __init__(self, name):
        self.name = name
        self.offset = [0, 0, 0]
        self.channels = []
        self.children = []
        self.rotation = None

def bvh_parser(file_path):
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
                    motion_frame = []
                    parts = line.split()

                    for part in parts:
                        motion_frame.append(part)
                    motion.append(motion_frame)
                    continue

                if parts[0] == "MOTION":
                    is_motion = True

                if parts[0] in ["ROOT", "JOINT", "End"]:
                    node = Joint(parts[1])
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

        return root, motion

    except FileNotFoundError:
        print(f"Error: File '{file_path}' does not exist.")

def print_bvh_tree(node, depth=0):
    if not node:
        return
    indent = "  " * depth

    if node.channels:
        print(f"{indent}{node.name} Children:{node.children}")
    else:
        print(f"{indent}{node.name} Children:{node.children}")

    for child in node.children:
        print_bvh_tree(child, depth + 1)

def add_motion(node, motion_frame, idx=[0]):
    if not node:
        return

    if node.name != "Site":
        node.rotation = list(map(float, motion_frame[idx[0]:idx[0]+3]))
        idx[0] += 3

    for child in node.children:
        add_motion(child, motion_frame, idx)

def motion_adapter(root, motion_frame):
    root_position = list(map(float, motion_frame[:3]))
    motion_frame = motion_frame[3:]
    add_motion(root, motion_frame, idx=[0])

    return root_position, root
