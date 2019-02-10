from __future__ import print_function
import Leap
import sys
import time
import numpy as np

import myo
import pickle
import math
import math

from arm_band import get_ms, ArmBandListener
from time import sleep, time
from myo import init, Hub, DeviceListener, Feed
from threading import Lock
from collections import deque
from threading import Lock
from datetime import datetime
from numpy import linalg as LA
from utils import Orientation


def angles_from_rot_matrices(prev_rot, next_rot):
    int_rot = Orientation.get_intermediate_rotation(prev_rot=prev_rot, next_rot=next_rot)
    rot_angles = Orientation.get_angles_from_rot(int_rot)
    return rot_angles


def basis_from_bone(bone):
    # def vector_to_list(vector):
        # return [vector.x, vector.y, vector.z]
        # return vector.to_tuple()

    basis = bone.basis
    # x_basis = vector_to_list(basis.x_basis)
    # y_basis = vector_to_list(basis.y_basis)
    # z_basis = vector_to_list(basis.z_basis)

    x_basis = basis.x_basis
    x_basis = [x_basis.x, x_basis.y, x_basis.z]
    y_basis = basis.y_basis
    y_basis = [y_basis.x, y_basis.y, y_basis.z]
    z_basis = basis.z_basis
    z_basis = [z_basis.x, z_basis.y, z_basis.z]

    return x_basis, y_basis, z_basis
    # return bone.basis.to_tuple()


def rot_matrix_from_basis(basis):
    x_basis, y_basis, z_basis = basis
    matrix = Leap.Matrix(Leap.Vector(*x_basis),
                         Leap.Vector(*y_basis),
                         Leap.Vector(*z_basis)).to_array_3x3()
    return np.reshape(matrix, newshape=(3, 3))

    # return np.reshape(basis, newshape=(3, 3))


class LeapFinger:
    finger_bones = ['metacarpals', 'proximal', 'intermediate', 'distal']

    def __init__(self, finger, hand):
        self.bone_basis = dict()
        self.bone_basis[LeapFinger.finger_bones[0]] = basis_from_bone(hand)

        for i, bone_name in enumerate(LeapFinger.finger_bones[1:], start=1):
            bone = finger.bone(i)
            self.bone_basis[bone_name] = basis_from_bone(bone)

    def get_angles(self):
        angles = list()
        for i, bone_name in enumerate(LeapFinger.finger_bones[1:], start=1):
            angles.append(angles_from_rot_matrices(
                rot_matrix_from_basis(self.bone_basis[LeapFinger.finger_bones[i - 1]]),
                rot_matrix_from_basis(self.bone_basis[bone_name])))

        return angles


class LeapHand:
    def __init__(self, hand):
        self.arm_basis = basis_from_bone(hand.arm)
        self.hand_basis = basis_from_bone(hand)

    def get_angles(self):
        return angles_from_rot_matrices(rot_matrix_from_basis(self.arm_basis),
                                        rot_matrix_from_basis(self.hand_basis))


class LeapListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']

    def __init__(self, start_time):
        super(LeapListener, self).__init__()
        self.start_time = start_time
        self.lock = Lock()
        self.hand_queue = deque()

    def on_init(self, controller):
        print("Leap is initialized")

    def on_connect(self, controller):
        print("Leap is connected")

    def on_disconnect(self, controller):
        print("Leap is disconnected")

    def on_exit(self, controller):
        print("Leap exited")

    def on_frame(self, controller):
        ms = get_ms(self.start_time)
        frame = controller.frame()
        if len(frame.hands) > 1:
            print('Only one hand is allowed')
            return

        hand = frame.hands.frontmost

        if not hand.is_right:
            print('No lefties are allowed')
            return

        if hand.is_valid:
            hand_lst = list()
            leap_hand = LeapHand(hand)
            hand_lst.append(leap_hand)

            # print('\r', leap_hand.get_angles(), end='')

            for finger in hand.fingers:
                leap_finger = LeapFinger(finger, hand)
                hand_lst.append(leap_finger)

            with self.lock:
                self.hand_queue.append((ms, hand_lst))

    @staticmethod
    def angles_from_hand(hand):
        return [leap_obj.get_angles() for leap_obj in hand]

    def get_angles_data(self):
        with self.lock:
            print('getting angles')
            return [(ms, LeapListener.angles_from_hand(hand)) for ms, hand in self.hand_queue]


def main():
    start_time = datetime.now()

    controller = Leap.Controller()
    leap_listener = LeapListener(start_time)
    controller.add_listener(leap_listener)

    try:
        print('start')
        sleep(100)
        print('end')
    except KeyboardInterrupt:
        pass
    finally:
        controller.remove_listener(leap_listener)


if __name__ == "__main__":
    main()
