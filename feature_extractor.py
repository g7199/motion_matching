from bvh_controller import *
import numpy as np
import os
from scipy.spatial import KDTree
from tqdm import tqdm

class MotionKDTree:
    def __init__(self, root_path):
        self.bvh_paths = self.find_all_bvh_files(root_path)
        self.index_map = []
        self.tree = None
        self.mean = None
        self.std = None
        self.feature_vectors = []
        self.weights = []
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
        virtual = motion.apply_virtual(root)
        motion.apply_velocity_feature(virtual)
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
    
    def normalize(self, vec):
        z = (vec - self.mean) / self.std
        return z * self.weights 

    def build(self):
        print("building KDTree")
        feature_vectors = []
        for path in tqdm(self.bvh_paths):
            motion = self.read_bvh_file(path)
            for idx, frame in enumerate(motion.feature_frames):
                if idx:  # skip frame 0 if needed
                    vec = self.extract_feature_vector(frame)
                    feature_vectors.append(vec)
                    self.index_map.append((motion, idx, path))

        self.mean = np.mean(feature_vectors, axis=0)
        self.std = np.std(feature_vectors, axis=0) + 1e-8
        self.weights = self.compute_weights()

        for vec in feature_vectors:
            #print(vec)
            normalized = self.normalize(np.array(vec))
            self.feature_vectors.append(normalized)

        self.tree = KDTree(self.feature_vectors)

    def search(self, query_vec):
        dist, idx = self.tree.query(query_vec)
        return dist, self.index_map[idx], self.normalize(query_vec)
    
    def compute_weights(self):
        # hip velocity: 3, site velocity: 6, site pos: 6, future pos: 9, future ori: remainder
        w = []
        w += [2.0] * 3    # hip velocity
        w += [1.0] * 6    # site velocities
        w += [0.0] * 3    # handling hip position
        w += [0.2] * 6    # site positions
        w += [0.5] * 9    # future position
        w += [1.0] * 9  # future orientation
        return np.array(w, dtype=np.float32)
    
    def search_frame(self, query_frame):
        query_vec = self.extract_feature_vector(query_frame)
        query_vec = self.normalize(query_vec)

        return self.search(query_vec)
