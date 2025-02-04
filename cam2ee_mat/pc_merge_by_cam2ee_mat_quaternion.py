import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import copy, json

def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    # Create a rotation object using the quaternion
    rotation = R.from_quat([qx, qy, qz, qw])
    
    # Convert the rotation object to a rotation matrix
    rotation_matrix = rotation.as_matrix()
    
    return rotation_matrix

def combine_rotation_translation(rotation_matrix, translation_vector):
    # Create a 4x4 identity matrix
    transformation_matrix = np.eye(4)
    
    # Replace the top-left 3x3 submatrix with the rotation matrix
    transformation_matrix[:3, :3] = rotation_matrix
    
    # Replace the top-right 3x1 submatrix with the translation vector
    transformation_matrix[:3, 3] = translation_vector
    
    return transformation_matrix

def compute_A2B(c2f_mat, A_mat, B_mat):
    cA_T_cB = np.linalg.inv(c2f_mat) @ np.linalg.inv(A_mat) @ B_mat @ c2f_mat
    return cA_T_cB

# transform pc_B from frame{B} to frame{A} and merge two pc sets
def merge_pc(c2f_mat, pc_A, A_mat, pc_B, B_mat):
    cA_T_cB = np.linalg.inv(c2f_mat) @ np.linalg.inv(A_mat) @ B_mat @ c2f_mat
    pc_B_transformed = copy.deepcopy(pc_B).transform(cA_T_cB)
    # pc_B_transformed.paint_uniform_color([1, 0.706, 1.0])
    return pc_A + pc_B_transformed


if __name__ == "__main__":
    #root_path = './2024-11-12'
    # root_path = './2024-11-23'
    root_path = './2025-02-03'
    with open(f'{root_path}/data.json', 'r') as f:
        config = json.load(f)

    pc_file_paths = config['point_cloud_path']
    cam2flange_mat= np.array(config['cam2flange_mat'])
    ref_idx       = config['reference_frame_idx']
    
    flange_poses  = config['flange_pose_quaternion']
    flange_mats = []
    
    for vec in flange_poses:
        flange_rot_mat = quaternion_to_rotation_matrix(vec[3], vec[4], vec[5], vec[6])
        flange_trans_vec = [vec[0], vec[1], vec[2]]
        flange_mat = combine_rotation_translation(flange_rot_mat, flange_trans_vec)
        flange_mats.append(flange_mat)
        # print("Transformation Matrix:")
        # print(flange_mat)
        # if pre_flange_mat.size != 0: 
        #     cA_T_cB = compute_A2B(cam2flange_mat, pre_flange_mat, flange_mat)
        #     print(f"Flange A with respect to Flange B: \n{cA_T_cB}")
        # pre_flange_mat = flange_mat

    pc_ref = o3d.io.read_point_cloud(f'{root_path}/' + pc_file_paths[ref_idx])
    mat_ref = np.array(flange_mats[ref_idx])
    del pc_file_paths[ref_idx]
    del flange_mats[ref_idx]
    
    pc_merged = copy.deepcopy(pc_ref)
    o3d.visualization.draw_geometries([pc_merged])

    for pc_file_path, flange_mat in zip(pc_file_paths, flange_mats):
        print(f"Processing: {pc_file_path}")
        print(f"Flange pose:\n{np.array(flange_mat)}")
        pc_to_be_transformed = o3d.io.read_point_cloud(f'{root_path}/' + pc_file_path)
        pc_merged = merge_pc(cam2flange_mat, pc_merged, mat_ref, pc_to_be_transformed, np.array(flange_mat))
        o3d.visualization.draw_geometries([pc_merged])
    
    o3d.io.write_point_cloud("chessboard.ply", pc_merged)