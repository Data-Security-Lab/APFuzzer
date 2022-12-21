import json
import logging
import multiprocessing
import os
import random
import shutil

import numpy as np
import pandas as pd
# import ray
from pymavlink import mavutil, mavwp
from pymavlink.mavutil import mavserial
# from pyulog import ULog
from func_timeout import func_set_timeout


class GaMavlink(multiprocessing.Process):
    """
    Mainly responsible for initiating the communication link to interact with UAV
    """

    def __init__(self, port=14550, msg_queue=None):
        super(GaMavlink, self).__init__()
        self.msg_queue = msg_queue
        self._master: mavserial = None
        self._port = port

    def connect(self) -> bool:
        """
        Connect drone
        :return:
        """
        logging.info("connect......")
        self._master = mavutil.mavlink_connection('udp:0.0.0.0:{}'.format(self._port))
        try:
            self._master.wait_heartbeat(timeout=30)
        except TimeoutError:
            logging.info("Timeout")
            return False
        # except FloatingPointError:
        #     logging.info("ERROR: Floating point exception - aborting")
        #     return False
        # except MemoryError:
        #     logging.info("ERROR: Segmentation fault - aborting")

        logging.info("Heartbeat from system (system %u component %u)" % (
            self._master.target_system, self._master.target_system))
        # print("ins connect")
        return True

    @func_set_timeout(60)
    def set_mission(self, mission_file, random: bool, timeout=10) -> bool:
        """
        Set mission
        :param random: Out of order
        :param mission_file: mission file
        :param timeout:
        :return: success
        """
        logging.info("set_mission......")
        if not self._master:
            logging.warning('Mavlink handler is not connect!')
            raise ValueError('Connect at first!')

        loader = mavwp.MAVWPLoader()
        loader.load(mission_file)
        logging.debug(f"Load mission file {mission_file}")

        # clear the waypoint
        self._master.waypoint_clear_all_send()

        # send the waypoint count
        self._master.waypoint_count_send(loader.count())

        seq_list = [True] * loader.count()
        try:
            # looping to send each waypoint information
            while True in seq_list:
                msg = self._master.recv_match(type=['MISSION_REQUEST'], blocking=True,
                                              timeout=timeout)
                if msg is not None and seq_list[msg.seq] is True:
                    self._master.mav.send(loader.wp(msg.seq))
                    seq_list[msg.seq] = False
                    logging.info(f'Sending waypoint {msg.seq}')
            mission_ack_msg = self._master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=timeout)
            logging.info('Upload mission finish.')
        except TimeoutError:
            logging.warning('Upload mission timeout!')
            return False
        return True

    def start_mission(self) -> None:
        """
        Arm and start the flight
        :return:
        """
        logging.info("start_mission......")
        if not self._master:
            logging.warning('Mavlink handler is not connect!')
            raise ValueError('Connect at first!')
        # self._master.set_mode_loiter()
        self._master.arducopter_arm()
        self._master.set_mode_auto()  # 竟然没有takeoff这一步?

    def set_param(self, param: str, value: float) -> None:
        """
        set a value of specific parameter
        :param param: name of the parameter
        :param value: float value want to set
        """
        if not self._master:
            raise ValueError('Connect at first!')

        self._master.param_set_send(param, value)
        self.get_param(param)

    def set_params(self, params_dict: dict) -> None:
        """
        set multiple parameter
        :param params_dict: a dict consist of {parameter:values}...
        """
        for param, value in params_dict.items():
            self.set_param(param, value)

    def get_param(self, param: str) -> float:
        """
        get current value of a parameter.
        :param param: name
        :return: value of parameter
        """
        self._master.param_fetch_one(param)
        while True:
            try:
                message = self._master.recv_match(type=['PARAM_VALUE', 'PARM'], blocking=True).to_dict()
                if message['param_id'] == param:
                    logging.debug('name: %s\t value: %f' % (message['param_id'], message['param_value']))
                    break
            except TimeoutError:
                logging.info("Failed to get param...")

        return message['param_value']

    def get_msg(self, type, block=False):
        """
        receive the mavlink message
        :param type:
        :param block:
        :return:
        """
        msg = self._master.recv_match(type=type, blocking=block)
        return msg

    # 新加函数+++++++++++++++
    def get_location(self):
        """
        return: the current location (lat,lon)
        """
        return self._master.location()

    # _____________________
    def set_mode(self, mode: str) -> None:
        """
        Set flight mode
        :param mode: string type of a mode, it will be convert to an int values.
        :return:
        """
        if not self._master:
            logging.warning('Mavlink handler is not connect!')
            raise ValueError('Connect at first!')
        mode_id = self._master.mode_mapping()[mode]

        self._master.mav.set_mode_send(self._master.target_system,
                                       mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                       mode_id)
        while True:
            message = self._master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
            if message['command'] == mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
                logging.debug(f'Mode: {mode} Set successful')
                break


    @staticmethod
    def random_param_value(param_json: dict):
        """
        random create the value
        :param param_json:
        :return:
        """
        out = {}
        for name, item in param_json.items():
            range = item['range']
            step = item['step']
            random_sample = random.randrange(range[0], range[1], step)
            out[name] = random_sample
        return out

    @staticmethod
    def get_default_values(para_dict):
        return para_dict.loc[['default']]

    @staticmethod
    def select_sub_dict(para_dict, param_choice):
        return para_dict[param_choice]

    @staticmethod
    def read_range_from_dict(para_dict):
        # return np.array(para_dict.loc['range'].to_list())
        range_list=[]
        for key in para_dict.keys():
            range_list.append(para_dict[key]["range"])
        return np.array(range_list)


    @staticmethod
    def read_unit_from_dict(para_dict):
        # return para_dict.loc['step'].to_numpy()
        step_list = []
        for key in para_dict.keys():
            step_list.append(para_dict[key]["step"])
        return np.array(step_list)

    # 新加函数

    def run(self):
        """
        loop check
        :return:
        """

        while True:
            msg = self._master.recv_match(type=['STATUSTEXT'], blocking=False)
            if msg is not None:
                msg = msg.to_dict()
                # print(msg2)
                if msg['severity'] in [0, 2]:
                    # self.send_msg_queue.put('crash')
                    logging.info('ArduCopter detect Crash.')
                    self.msg_queue.put('error')
                    break

# 测试以上函数功能, 此文件中的函数大多是无人机基本操作相关
if __name__ == '__main__':
    """首先在终端手动启动ArduPilot"""
    logging.basicConfig(format='%(asctime)s - %(filename)s[line:%(lineno)d] - %(levelname)s: %(message)s',level=logging.INFO)

    test = GaMavlink(14550, None)
    if test.connect():
        print("ok")
        print(test.get_param("PSC_VELZ_IMAX"))
        test.set_mission("mission.txt", random=True)
        test.start_mission()
        # 没有RTL这一步骤（确实这个步骤纯属浪费时间），重新启动后，会初始化在home位置
        # 也可以单独对RTL这一个任务进行测试
