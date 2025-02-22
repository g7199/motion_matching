class Joint:
    def __init__(self, name):
        self.name = name
        self.offset = [0, 0, 0]
        self.channels = []
        self.children = []

def bvh_parser(file_path):
    stack = []
    root = None
    cur_node = None

    try:
        with open(file_path, "r", encoding="utf-8") as file:
            for line in file:
                parts = line.split()
                if not parts:
                    continue

                if parts[0] == "MOTION":
                    break

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

        return root

    except FileNotFoundError:
        print(f"Error: File '{file_path}' does not exist.")


def print_bvh_tree(node, depth=0):
    if not node:
        return
    indent = "  " * depth

    if node.channels:
        print(f"{indent}{node.name} (Offset: {node.offset}, Channels: {node.channels})")
    else:
        print(f"{indent}{node.name} (Offset: {node.offset})")

    for child in node.children:
        print_bvh_tree(child, depth + 1)