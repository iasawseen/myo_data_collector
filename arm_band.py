from __future__ import print_function

import myo
import pickle
import math
import numpy as np

from time import sleep, time
from myo import init, Hub, DeviceListener, Feed, quaternion
from threading import Lock
from collections import deque
from datetime import datetime
from utils import Orientation


def get_ms(start_time):
    dt = datetime.now() - start_time
    return (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0


class ArmBandListener(DeviceListener):
    def __init__(self, start_time):
        self.start_time = start_time
        self.emg_lock = Lock()
        self.imu_lock = Lock()
        self.emg_queue = deque()
        self.imu_queue = deque()
        self.emg_local_queue = deque(maxlen=4)

    def on_connect(self, device, timestamp, firmware_version):
        device.set_stream_emg(myo.StreamEmg.enabled)

    def on_emg_data(self, device, timestamp, emg_data):
        ms = get_ms(self.start_time)
        # self.emg_local_queue.append(emg_data)
        # emg = np.average(np.array(self.emg_local_queue), axis=0)
        # emg = list(emg)
        # emg = [abs(int(emg_sensor)) for emg_sensor in emg]
        # print('\r', emg, end='')
        with self.emg_lock:
            self.emg_queue.append((ms, emg_data))

    def get_emg_data(self):
        with self.emg_lock:
            return list(self.emg_queue)

    def on_pair(self, myo, *args):
        print('arm_band is on pair')

    def on_unpair(self, myo, *args):
        print('arm_band is on unpair')

    def on_orientation_data(self, device, timestamp, quat):
        ms = get_ms(self.start_time)
        angles = [math.degrees(ang) for ang in Orientation.get_or_angles(quat.x, quat.y, quat.z, quat.w)]
        with self.imu_lock:
            # self.imu_queue.append((ms, (quat.x, quat.y, quat.z, quat.w)))
            self.imu_queue.append((ms, (angles[0], angles[1])))

    def get_orientation_data(self):
        with self.imu_lock:
            return list(self.imu_queue)


def main():
    start_time = datetime.now()

    init('C:\\Users\\Ivan\\PycharmProjects\\myo_arm_band\\lib\myo-sdk-win-0.9.0\\bin')
    listener = ArmBandListener(start_time)
    hub = Hub()

    sleep(10)

    try:
        hub.run(1000, listener)
        print('start')
        sleep(20)
        print('end')
    except KeyboardInterrupt:
        print('\nQuit')
    finally:
        with open('fist.pickle', 'wb') as handle:
            pickle.dump((listener.get_emg_data(),
                         listener.get_orientation_data()), handle)
        hub.shutdown()


if __name__ == '__main__':
    main()
