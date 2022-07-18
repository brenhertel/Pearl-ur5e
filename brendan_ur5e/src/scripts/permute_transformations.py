import numpy as np
import itertools
from tf.transformations import euler_from_quaternion, quaternion_from_euler, translation_matrix, quaternion_matrix, concatenate_matrices
import copy

pos_x = 0.155200917422
pos_y = -0.589980535566
pos_z = 0.00673944187901

rot_x = 0.735492005144
rot_y = -0.123955186192
rot_z = -0.00833150062263
rot_w = 0.666045950579

sign_permutations = [[1, 1, 1, 1], [1, 1, 1, -1], [1, 1, -1, 1], [1, 1, -1, -1], [1, -1, 1, 1], [1, -1, 1, -1], [1, -1, -1, 1 ], [1, -1, -1, -1 ], [-1, 1, 1, 1], [-1, 1, 1, -1], [-1, 1, -1, 1], [-1, 1, -1, -1 ], [-1, -1, 1, 1 ], [-1, -1, 1, -1 ], [-1, -1, -1, 1], [-1, -1, -1, -1]]

quat_list = [rot_x, rot_y, rot_z, rot_w]

for i in sign_permutations:
    print(i)
    for j in itertools.permutations(quat_list):
        print('quats:')
        new_quat_list = (j[0] * i[0], j[1] * i[1], j[2] * i[2], j[3] * i[3])
        print(new_quat_list)
        print('output:')
        
        translation_mat = translation_matrix((0.185, 0, 0))
        #translation_mat = translation_matrix((msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z))
            
        #rotation_mat = quaternion_matrix((orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w))
        rotation_mat = quaternion_matrix(new_quat_list)
            
        frame_matrix = np.array([[pos_x], [pos_y], [pos_z], [1]])
        #frame_matrix = np.array([[0], [self.x_trans], [0], [1]])
            
        #transform_mat = np.dot(rotation_mat, translation_mat)
        transform_mat = concatenate_matrices(rotation_mat, translation_mat)
        #transform_mat = rotation_mat
        #transform_mat[1,3] = self.x_trans
        new_point = np.matmul(transform_mat, frame_matrix)
        if (new_point[1] < -0.5):
            print(new_point)
