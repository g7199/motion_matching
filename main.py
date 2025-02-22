import argparse
from bvh_parser import *

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file_path")

    args = parser.parse_args()
    root = bvh_parser(args.file_path)
    print_bvh_tree(root)