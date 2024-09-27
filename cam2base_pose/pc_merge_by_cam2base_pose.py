import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as Rtools
import copy, json

def matrix_from_pose_vec(pose_vec):
    translation_vec = pose_vec[:3]
    quaternion_vec = pose_vec[3:]

    rotaion_matrix = Rtools.from_quat(quaternion_vec).as_matrix()
    Tmatrix = np.eye(4)
    Tmatrix[:3, :3] = rotaion_matrix
    Tmatrix[:3, 3] = np.array(translation_vec).reshape(3, )
    print(Tmatrix)
    return Tmatrix # 4 x 4 transformation matrix

# transform pc_B from frame{B} to frame{A} and merge two pc sets
def merge_pc(pc_A, pose_A, pc_B, pose_B):
    Tmatrix_A = matrix_from_pose_vec(pose_A)
    Tmatrix_B = matrix_from_pose_vec(pose_B)

    cA_T_cB = np.linalg.inv(Tmatrix_A) @ Tmatrix_B
    pc_B_transformed = copy.deepcopy(pc_B).transform(cA_T_cB)
    return pc_A + pc_B_transformed

def rectify_pc(pc_A, pose_A, pc_B, pose_B):
    Tmatrix_A = matrix_from_pose_vec(pose_A)
    Tmatrix_B = matrix_from_pose_vec(pose_B)

    cA_T_cB = np.linalg.inv(Tmatrix_A) @ Tmatrix_B
    pc_B_transformed = copy.deepcopy(pc_B).transform(cA_T_cB)
    return pc_B_transformed


if __name__ == "__main__":
    rectify = False
    root_path = './flaw_inspect'
    # root_path = './2024-09-26'
    with open(f'{root_path}/data.json', 'r') as f:
        config = json.load(f)

    pc_file_paths = config['point_cloud_path']
    camera_poses = config['camera_pose']
    ref_idx = config['reference_frame_idx']

    pc_ref = o3d.io.read_point_cloud(f'{root_path}/' + pc_file_paths[ref_idx])
    pose_ref = camera_poses[ref_idx]
    del pc_file_paths[ref_idx]
    del camera_poses[ref_idx]
    
    pc_merged = copy.deepcopy(pc_ref)
    o3d.visualization.draw_geometries([pc_merged])


    for pc_file_path, camera_pose in zip(pc_file_paths, camera_poses):
        print(pc_file_path) 
        print(camera_pose)
        pc_to_be_transformed = o3d.io.read_point_cloud(f'{root_path}/' + pc_file_path)
        if rectify:
            pc_rectified = rectify_pc(pc_merged, pose_ref, pc_to_be_transformed, camera_pose)
            o3d.visualization.draw_geometries([pc_rectified])
            break           
        else:
            pc_merged = merge_pc(pc_merged, pose_ref, pc_to_be_transformed, camera_pose)
            o3d.visualization.draw_geometries([pc_merged])

    if rectify:
        o3d.io.write_point_cloud("rectified.ply", pc_rectified)
    else:
        o3d.io.write_point_cloud("merged.ply", pc_merged)

