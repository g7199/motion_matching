from bvh_controller import *
import os

def find_all_bvh_files(root_folder):
    bvh_files = []
    for dirpath, dirnames, filenames in os.walk(root_folder):
        for filename in filenames:
            if filename.lower().endswith('.bvh'):
                full_path = os.path.join(dirpath, filename)
                bvh_files.append(full_path)
    return bvh_files

def read_bvh_file(filepath):
    root, motion = parse_bvh(filepath)
    joint_order = get_preorder_joint_list(root)
    motion.build_quaternion_frames(joint_order)
    motion.apply_velocity_feature(root)
    virtual_root = motion.apply_virtual(root)
    motion.apply_future_feature()

    return motion

root_path = '/path/to/your/folder' 
bvh_paths = find_all_bvh_files(root_path)
