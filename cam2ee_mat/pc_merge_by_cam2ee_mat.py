import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as Rtools
import copy, json

# transform pc_B from frame{B} to frame{A} and merge two pc sets
def merge_pc(c2f_mat, pc_A, A_mat, pc_B, B_mat):
    cA_T_cB = np.linalg.inv(c2f_mat) @ np.linalg.inv(A_mat) @ B_mat @ c2f_mat
    pc_B_transformed = copy.deepcopy(pc_B).transform(cA_T_cB)
    pc_B_transformed.paint_uniform_color([1, 0.706, 1.0])
    return pc_A + pc_B_transformed

def compute_A2B(c2f_mat, A_mat, B_mat):
    cA_T_cB = np.linalg.inv(c2f_mat) @ np.linalg.inv(A_mat) @ B_mat @ c2f_mat
    return cA_T_cB

if __name__ == "__main__":
    with open('data_combined4.json', 'r') as f:
    # with open('data_pc1_2.json', 'r') as f:
    # with open('data_test1_2.json', 'r') as f:
        config = json.load(f)

    pc_file_paths = config['point_cloud_path']
    flange_mats   = config['flange_mats']
    cam2flange_mat= np.array(config['cam2flange_mat'])
    ref_idx       = config['reference_frame_idx']

    # Compute cA_T_cB
    cA_T_cB = compute_A2B(cam2flange_mat, np.array(flange_mats[0]), np.array(flange_mats[1]))
    print(f"Flange A with respect to Flange B: \n{cA_T_cB}")

    pc_ref = o3d.io.read_point_cloud(pc_file_paths[ref_idx])
    mat_ref = np.array(flange_mats[ref_idx])
    del pc_file_paths[ref_idx]
    del flange_mats[ref_idx]
    
    pc_merged = copy.deepcopy(pc_ref)
    o3d.visualization.draw_geometries([pc_merged])

    for pc_file_path, flange_mat in zip(pc_file_paths, flange_mats):
        print(f"Processing: {pc_file_path}")
        print(f"Camera Matrix:\n{np.array(flange_mat)}")
        pc_to_be_transformed = o3d.io.read_point_cloud(pc_file_path)
        pc_merged = merge_pc(cam2flange_mat, pc_merged, mat_ref, pc_to_be_transformed, np.array(flange_mat))
        o3d.visualization.draw_geometries([pc_merged])



