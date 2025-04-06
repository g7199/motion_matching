from bvh_controller import *
import numpy as np
import os
from scipy.spatial import KDTree
from tqdm import tqdm

class MotionKDTree:
    def __init__(self, root_path):
        self.bvh_paths = self.find_all_bvh_files(root_path)
        self.feature_vectors = []
        self.index_map = []
        self.tree = None
        self.build()

    def find_all_bvh_files(self, root_folder):
        bvh_files = []
        for dirpath, _, filenames in os.walk(root_folder):
            for filename in filenames:
                if filename.lower().endswith('.bvh'):
                    full_path = os.path.join(dirpath, filename)
                    bvh_files.append(full_path)
        return bvh_files

    def read_bvh_file(self, filepath):
        root, motion = parse_bvh(filepath)
        joint_order = get_preorder_joint_list(root)
        motion.build_quaternion_frames(joint_order)
        motion.apply_velocity_feature(root)
        motion.apply_virtual(root)
        motion.apply_future_feature()
        return motion

    def extract_feature_vector(self, frame):
        vec = []
        for v in frame.velocity.values():
            vec.extend([v.x, v.y, v.z])
        for v in frame.site_positions.values():
            vec.extend([v.x, v.y, v.z])
        for p in frame.future_position:
            vec.extend([p.x, p.y, p.z])
        for f in frame.future_orientation:
            vec.extend([f.x, f.y, f.z])
        return np.array(vec, dtype=np.float32)

    def build(self):
        print("building KDTree")
        for path in tqdm(self.bvh_paths):
            motion = self.read_bvh_file(path)
            for idx, frame in enumerate(motion.feature_frames):
                if idx:  # skip frame 0 if needed
                    vec = self.extract_feature_vector(frame)
                    self.feature_vectors.append(vec)
                    self.index_map.append((motion, idx, path))
        self.tree = KDTree(np.array(self.feature_vectors))

    def search(self, query_vec):
        _, idx = self.tree.query(query_vec)
        return self.index_map[idx]
    
    def search_frame(self, query_frame):
        query_vec = self.extract_feature_vector(query_frame)
        return self.search(query_vec)
