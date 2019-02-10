from __future__ import print_function
import Leap
import pickle
import argparse
import os

from arm_band import get_ms, ArmBandListener
from time import sleep, time
from myo import init, Hub, DeviceListener, Feed
from datetime import datetime
from leap import LeapListener

DATA_FOLDER = '..\\input_data_raw'


def record_simple_movements():
    sleep(2)
    print('moving thumb')
    sleep(30)
    print('moving index')
    sleep(30)
    print('moving middle')
    sleep(30)
    print('moving ring')
    sleep(30)
    print('moving pinky')
    sleep(30)
    print('fist opening / closing')
    sleep(60)
    print('freestyle')
    sleep(30)
    print('end recording')


def record_sophisticated_movements():
    sleep(2)
    print('fisting')
    sleep(30)
    print('fingering')
    sleep(60)
    print('pitching')
    sleep(30)
    print('palming')
    sleep(30)
    print('thumbing')
    sleep(30)
    print('freestyling')
    sleep(60)
    print('end recording')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--person', type=str)
    parser.add_argument('-t', '--type', type=str)
    parser.add_argument('-s', '--session', type=str)
    parser.add_argument('-i', '--iteration', type=str)
    args = parser.parse_args()

    data_sub_folder = os.path.join(DATA_FOLDER, args.person)

    if not os.path.exists(data_sub_folder):
        os.mkdir(data_sub_folder)

    start_time = datetime.now()

    init('C:\\Users\\Ivan\\PycharmProjects\\myo_arm_band\\lib\myo-sdk-win-0.9.0\\bin')
    arm_band_listener = ArmBandListener(start_time)
    hub = Hub()

    controller = Leap.Controller()
    leap_listener = LeapListener(start_time)
    controller.add_listener(leap_listener)

    sleep(10)

    try:
        hub.run(1000, arm_band_listener)
        print('start recording')

        if args.type == '0':
            record_simple_movements()
        elif args.type == '1':
            record_sophisticated_movements()
        else:
            print('unknown type of recording')
    except KeyboardInterrupt:
        pass
    finally:
        emg_data = arm_band_listener.get_emg_data()
        orientation_data = arm_band_listener.get_orientation_data()
        angles = leap_listener.get_angles_data()

        print('EMG data length: {}\n'
              'Orientation data length: {}'
              '\nAngles data length: {}'.format(len(emg_data), len(orientation_data), len(angles)))

        with open(os.path.join(data_sub_folder,
                  'raw_{}_{}_{}_{}.pickle'.format(args.person, args.type,
                                                  args.session, args.iteration)), 'wb') as handle:
            pickle.dump((emg_data, orientation_data, angles), handle)

        controller.remove_listener(leap_listener)
        hub.shutdown()


if __name__ == "__main__":
    main()
