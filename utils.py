from __future__ import print_function

import Leap
import time
import collections
import numpy as np
import math
from numpy import linalg as LA


class Timer:
    def __init__(self, op_name):
        self.op_name = op_name
        self.start = None
        self.end = None

    def __enter__(self):
        print(self.op_name + ': ...')
        self.start = time.time()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.end = time.time()
        print('elapsed time for {0} is {1:.3f} s'.format(self.op_name.lower(),
                                                         self.end - self.start))


class Orientation:
    def __init__(self, bone_obj):
        basis = bone_obj.basis
        x_basis = basis.x_basis
        y_basis = basis.y_basis
        z_basis = basis.z_basis
        self.matrix = Leap.Matrix(x_basis, y_basis, z_basis).to_array_3x3()
        self.matrix = np.reshape(self.matrix, newshape=(3, 3))
        assert self.validate(self.matrix)

    @staticmethod
    def get_rotation_mat(bone_obj):
        basis = bone_obj.basis
        x_basis = basis.x_basis
        y_basis = basis.y_basis
        z_basis = basis.z_basis
        matrix = Leap.Matrix(x_basis, y_basis, z_basis).to_array_3x3()
        matrix = np.reshape(matrix, newshape=(3, 3))
        assert Orientation.validate(matrix)
        return matrix

    @staticmethod
    def validate(matrix, verbose=False):
        ainv = LA.inv(matrix)

        clauses = [custom_close(np.abs(LA.det(matrix)), 1),
                   custom_close(np.dot(matrix, ainv), np.eye(3)),
                   custom_close(np.dot(ainv, matrix), np.eye(3)),
                   custom_close(ainv, matrix.transpose())]

        if verbose and not all(clauses):
            print(LA.det(matrix))
            print(clauses)
            print(ainv)
            print(matrix.transpose())
            print()

        return all(clauses)

    @staticmethod
    def get_intermediate_rotation(prev_rot, next_rot):

        int_matrix = np.matmul(next_rot, prev_rot.transpose())
        # Orientation.validate(int_matrix, verbose=True)
        # if not custom_close(np.matmul(int_matrix, prev_rot), next_rot):
        #     print('wow')
        return int_matrix

    @staticmethod
    def get_quaternion(rot_matrix):
        i = 0.5 * np.sqrt(1 + rot_matrix[0, 0] -
                          rot_matrix[1, 1] -
                          rot_matrix[2, 2])

        j = 1 / (4 * i) * (rot_matrix[0, 1] + rot_matrix[1, 0])
        k = 1 / (4 * i) * (rot_matrix[0, 2] + rot_matrix[2, 0])
        r = 1 / (4 * i) * (rot_matrix[2, 1] - rot_matrix[1, 2])

        return i, j, k, r

    @staticmethod
    def get_or_angles(i, j, k, r):
        roll = math.atan2(2 * (r * i + j * k), 1 - 2 * (i * i + j * j))
        pitch = math.asin(2 * (r * j - k * i))
        yaw = math.atan2(2 * (r * k + i * j), 1 - 2 * (j * j + k * k))
        return roll, pitch, yaw

    @staticmethod
    def get_angles_from_rot(rot_mat):
        """
        https://github.com/spmallick/learnopencv/blob/master/RotationMatrixToEulerAngles/rotm2euler.py
        https://www.learnopencv.com/rotation-matrix-to-euler-angles/
        :param rot_mat:
        :return:
        """

        sy = math.sqrt(rot_mat[0, 0] * rot_mat[0, 0] + rot_mat[1, 0] * rot_mat[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(rot_mat[2, 1], rot_mat[2, 2])
            y = math.atan2(-rot_mat[2, 0], sy)
            z = math.atan2(rot_mat[1, 0], rot_mat[0, 0])
        else:
            x = math.atan2(-rot_mat[1, 2], rot_mat[1, 1])
            y = math.atan2(-rot_mat[2, 0], sy)
            z = 0

        return [math.degrees(angle) for angle in [x, y, z]]


def custom_close(a, b):
    return np.allclose(a, b, rtol=1e-06, atol=1e-06)


def flatten(iterable):
    for item in iterable:
        if not isinstance(item, collections.Iterable):
            yield item
        else:
            for sub_item in flatten(item):
                yield sub_item


def fill_with_tuples(left, right, length):
    assert len(left) == len(right)

    length_with_over = length + 2

    linspaces = [np.linspace(left[i], right[i], length_with_over) for i in range(len(left))]
    filler = [[linspace[i] for linspace in linspaces] for i in range(length_with_over)][1:-1]

    assert length == len(filler)

    return filler
