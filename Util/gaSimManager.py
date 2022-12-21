import logging
import math
import multiprocessing
import os
import re
import sys
import threading
import time
import json

import func_timeout.exceptions
import pysnooper
import eventlet
import pexpect
from pexpect import spawn
from pymavlink import mavextra, mavwp
import numpy as np
from Util.gaMavlink import GaMavlink

current_dir = os.path.dirname(os.path.realpath(__file__))
root_dir = os.path.realpath(os.path.join(current_dir, '../'))
ARDUPILOT_LOG_PATH = f"{root_dir}/Log/logs"
AUTOPILOT = "ArduPilot"
SIMULATION = "SITL"
# SIMULATION="Gazebo"

class Location:
    def __init__(self, x, y=None, timeS=0):
        if y is None:
            self.x = x.x
            self.y = x.y
        else:
            self.x = x
            self.y = y
        self.timeS = timeS
        self.npa = np.array([x, y])

    def __sub__(self, other):
        return Location(self.x - other.x, self.y - other.y)

    def __str__(self):
        return f"X: {self.x} ; Y: {self.y}"

    def sum(self):
        return self.npa.sum()

    @classmethod
    def distance(cls, point1, point2):
        return mavextra.distance_lat_lon(point1.x, point1.y,
                                         point2.x, point2.y)


class GaSimManager(object):
    """
    无人机仿真模拟
    """

    def __init__(self, debug: bool = False):
        self._sitl_task = None
        self._mav_monitor: GaMavlink = None
        self._even = None
        self.msg_queue = multiprocessing.Queue()
        # self.msg_queue=None
        if debug:
            logging.basicConfig(format='%(asctime)s - %(filename)s[line:%(lineno)d] - %(levelname)s: %(message)s',
                                level=logging.DEBUG)
        else:
            logging.basicConfig(format='%(asctime)s - %(filename)s[line:%(lineno)d] - %(levelname)s: %(message)s',
                                level=logging.INFO)

    def start_sitl(self) -> None:
        """
        start the simulator
        :return:
        """
        logging.info("start_sitl......")

        if os.path.exists(f"{ARDUPILOT_LOG_PATH}/mav.tlog"):
            os.remove(f"{ARDUPILOT_LOG_PATH}/mav.tlog")

        cmd = None
        if AUTOPILOT == 'ArduPilot':
            if SIMULATION == 'SITL':  # yes
                # 注意每行最后有一个空格 不能加--console參數,否則cannot get console句柄
                # 作者为什么使用--location=AVC_plane:40.072842,-105.230575,1586,80 经度过于怪异
                cmd = f"python3 /home/changzw/ardupilot/Tools/autotest/sim_vehicle.py -N " \
                      f"--out=127.0.0.1:14550 --out=127.0.0.1:14540 -v ArduCopter -w -S 30 "

                """
                --location:use start location from Tools/autotest/locations.txt  （AVC_plane=40.072842,-105.230575,1586,80）
                --out=127.0.0.1:14550 --out=127.0.0.1:14540: create an additional mavlink output
                -v ArduCopter:选择Vehicle类型
                -w: 擦除虚拟EEPROM并为车辆加载正确的默认参数
                -S:set simulation speedup (1 for wall clock time)
                -N:不重新编译
                """
            elif SIMULATION == "BIN":
                cmd_binary = "gnome-terminal -- bash -c '~/ardu-sim/arducopter -w -S --model + --speedup 30 --defaults ~/ardu-sim/parameters/copter.parm; exec bash'"
                # cmd_binary="gnome-terminal -- bash -c '~/ardupilot/build/sitl/bin/arducopter -w -S --model + --speedup 30 --defaults ~/ardu-sim/parameters/copter.parm; exec bash'"
                try:
                    os.system(cmd_binary)
                except TimeoutError:
                    print("timout....可能是端口被占用...")
                # time.sleep(10)
                cmd = "mavproxy.py --master tcp:127.0.0.1:5760 --out 127.0.0.1:14550"
            elif SIMULATION == 'Gazebo':
                cmd = f"python3 /home/changzw/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --map " \
                      f"-w -S 20 "
                # 另外需要开启gazebo
                start_gazebo_cmd = "gnome-terminal -- bash -c 'gazebo --verbose worlds/iris_arducopter_runway.world; exec bash'"
                os.system(start_gazebo_cmd)

            elif SIMULATION == 'Airsim':
                cmd = f"python3 /home/changzw/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f airsim-copter --map " \
                      f"--out=127.0.0.1:14550 --out=127.0.0.1:14540 -S 10"
            try:
                self._sitl_task = pexpect.spawn(cmd, cwd=ARDUPILOT_LOG_PATH, timeout=60, encoding='utf-8')
            except TimeoutError:
                logging.info("启动SITL失败......")

        logging.info(f"Start {AUTOPILOT} --> [{SIMULATION}]")
        if cmd is None:
            raise ValueError('Not support mode or simulator')

    def start_gazebo(self):
        logging.info("start_gazebo......")
        cmd = f"gazebo --verbose worlds/iris_arducopter_runway.world "
        os.popen(cmd)

    def mav_monitor_init(self, drone_i=0):
        """
        init the SITL simulator
        :return:
        """
        logging.info("mav_monitor_init......")
        # self._mav_monitor = GaMavlink(14540 + int(drone_i), self.msg_queue)
        # self._mav_monitor.connect()
        beginTime = time.time()
        if AUTOPILOT == 'ArduPilot':
            while True:  # 这个地方实际上就是判断sitl是否初始化完成，可以用判断是否arm替换
                try:
                    line = self._sitl_task.readline()
                    print("line:", line)
                    if "IMU0 is using GPS" in line:
                        break
                    if line is None:
                        break
                except TimeoutError:
                    logging.info("初始化失败")

                if time.time() - beginTime > 30:
                    logging.info("mav_monitor_init falied.....")
                    break

        self._mav_monitor = GaMavlink(14550 + int(drone_i), self.msg_queue)

    def get_dump_files(self, log_path):
        log_files = os.listdir(log_path)
        dump_list = []

        for file in log_files:
            if "dump" in file:
                dump_list.append(file)
        # dump_file_count = len(dump_list)
        return dump_list

    def mav_monitor_error_new(self, mission):

        logging.info(f'Start error monitor.')
        # Setting
        mission_time_out_th = 30
        result = 'Pass'
        # Waypoint
        loader = mavwp.MAVWPLoader()
        loader.load(mission)
        print("mission_count:", loader.count())
        #
        lpoint1 = Location(loader.wpoints[0])
        lpoint2 = Location(loader.wpoints[1])
        # command=loader.wpoints[4]
        pre_location = Location(loader.wpoints[0])
        # logger
        small_move_num = 0
        deviation_num = 0
        low_lat_num = 0
        # Flag
        start_check = False
        current_mission = 0
        pre_alt = 0
        last_time = 0

        deviation_dis = 0

        # Vibration
        vibration_num = 0

        Takeoff_times = 0  # =2代表 任务结束 AP: Mission: 1 Takeoff

        # Float point Exception

        log_path = ARDUPILOT_LOG_PATH
        dump_list = self.get_dump_files(log_path)

        start_time = time.time()
        while True:
            # time.sleep(0.1)
            # MAVLink Message https://mavlink.io/zh/messages/common.html
            # https://github.com/ArduPilot/mavlink/blob/734d041888/message_definitions/v1.0/common.xml
            status_message = self._mav_monitor.get_msg(["STATUSTEXT"])
            vibration_msg = self._mav_monitor.get_msg(["VIBRATION"])
            position_msg = self._mav_monitor.get_msg(["GLOBAL_POSITION_INT", "MISSION_CURRENT"])

            # System status message
            # 通过系统日志判断异常
            # {0:Emergency, 1:Alert, 2:Critical, 3:Error, 4:Warning, 5:Notice, 6:Info, 7:Debug}
            if status_message is not None and status_message.get_type() == "STATUSTEXT":
                line = status_message.text
                logging.info(f"status_message:{status_message}")

                if "Floating point exception" in line:
                    result = "Floating Point Exception"
                    logging.info(f"Floating point exception{line}")
                    break
                if "Error" in line or "error" in line or "exception" in line:
                    result = "Other Error"
                    logging.info(f"Other Error:{line}")
                    break

                if status_message.severity == 6:
                    if "Disarming motors" in line:
                        # if successful landed, break the loop and return true
                        result = "Pass"
                        logging.info(f"Disarming motors Successful break the loop.")
                        break

                    if "pre-arm fail" in line:  #
                        result = 'PreArm Failed'
                        logging.info(f"PreArm Failed:{line}")
                        break
                    if "SIM Hit ground" in line:
                        speed = re.search(r'([0-9]{1,}[.][0-9]*)', line).group(1)
                        speed = float(speed)
                        if speed > 5.0:
                            result = "Crash"
                            logging.info(f"Crash:{line}")
                            break
                        else:
                            result = "Pass"
                            logging.info(f"Successful break the loop.")
                            break

                elif status_message.severity == 0 or status_message.severity == 2:
                    # Appear error, break loop and return false
                    if "SIM Hit ground at" in line or "Crash" in line:  # 0:Emergency
                        result = 'Crash'
                        logging.info(f"Crash:{line}")
                        break
                    elif "Potential Thrust Loss" in line:  # 0:Emergency
                        result = 'Thrust Loss'
                        logging.info(f"Thrust Loss:{line}")
                        break
                    elif "Yaw Imbalance" in line:  # 0:Emergency
                        result = "Yaw Imbalance"
                        logging.info(f"Yaw Imbalance:{line}")
                        break
                    elif "PreArm" in line:  # 2:Critical 大多是这种情况
                        result = 'PreArm Failed'
                        logging.info(f"PreArm Failed(Critical):{line}")
                        break

            # 判断Unstable movement https://ardupilot.org/copter/docs/common-measuring-vibration.html
            if vibration_msg is not None and vibration_msg.get_type() == "VIBRATION":  # 考虑加次数
                vibration_x = vibration_msg.vibration_x
                vibration_y = vibration_msg.vibration_y
                vibration_z = vibration_msg.vibration_z
                if vibration_x > 60 or vibration_y > 60 or vibration_z > 60:
                    result = "Vibration"
                    logging.info(f"Vibration_1:{vibration_msg}")  # 没有.text属性
                    break
                if vibration_x > 30 or vibration_y > 30 or vibration_z > 30:
                    vibration_num += 1
                # else:
                #     vibration_num=0

                if vibration_num >= 2:
                    result = "Vibration"
                    logging.info(f"Vibration_2:{vibration_msg}")
                    break

            # 获取当前的航路点index
            if position_msg is not None and position_msg.get_type() == "MISSION_CURRENT":
                # print("position_msg:",position_msg)  #
                # current_mission:已路过的点 position_msg.seq: 正在前往的点  从1开始  只打印一次
                if current_mission != int(position_msg.seq) and int(position_msg.seq) != loader.count():
                    logging.info(f"Mission change {current_mission} -> {position_msg.seq}")
                    # 1->2 代码真正执行command2(从0开始)
                    # logging.info(f"position_msg:{position_msg}")
                    # logging.info(f"position_msg:{position_msg}")

                    # 处理因为无法获取完整的status_message造成的无法正确结束SITL的情况
                    if position_msg.seq == 1:
                        Takeoff_times += 1
                    if Takeoff_times == 2:
                        logging.info("failed stop the loop")
                        result = "Pass"
                        break

                    if loader.wpoints[current_mission].x != 0 and loader.wpoints[current_mission].y != 0:
                        lpoint1 = Location(loader.wpoints[current_mission])
                    if loader.wpoints[position_msg.seq].x != 0 and loader.wpoints[position_msg.seq].y != 0:
                        lpoint2 = Location(loader.wpoints[position_msg.seq])

                    # Start Check
                    if int(position_msg.seq) == 2:
                        start_check = True
                    current_mission = int(position_msg.seq)  #

            # 判断deviation和freeze
            elif position_msg is not None and position_msg.get_type() == "GLOBAL_POSITION_INT" and \
                    loader.wp(current_mission).command in [16, 18, 19]:
                # logging.info(f"position_msg----------------:{position_msg}")
                # print("current_mission:",current_mission)
                # if loader.wp(current_mission).command!=16:
                #     continue
                # Check deviation
                position_lat = position_msg.lat * 1.0e-7  # 经度
                position_lon = position_msg.lon * 1.0e-7  # 维度
                alt = position_msg.relative_alt / 1000  # 高度
                time_usec = position_msg.time_boot_ms * 1e-6  # 时间
                position = Location(position_lat, position_lon, time_usec)  # 当前位置坐标

                # Calculate distance
                try:
                    moving_dis = Location.distance(pre_location, position)
                except ValueError:
                    print("ValueError: math domain error")
                    moving_dis = 0

                time_step = position.timeS - pre_location.timeS  # 每次的时间差是多少????
                alt_change = abs(pre_alt - alt)
                # Update position
                pre_location.x = position_lat
                pre_location.y = position_lon
                pre_alt = alt

                if start_check:
                    if alt < 1:
                        low_lat_num += 1
                    else:
                        small_move_num = 0

                    velocity = moving_dis / time_step
                    # logging.debug(f"Velocity {velocity}.")
                    # Is small move?
                    # logging.debug(f"alt_change {alt_change}.")
                    if velocity < 1 and alt_change < 0.1 and small_move_num != 0:
                        logging.debug(f"Small moving {small_move_num}, num++, num now - {small_move_num}.")
                        small_move_num += 1
                    else:
                        small_move_num = 0

                    # Point2line distance
                    a = Location.distance(position, lpoint1)
                    b = Location.distance(position, lpoint2)
                    c = Location.distance(lpoint1, lpoint2)

                    if c != 0:
                        try:
                            p = (a + b + c) / 2
                            deviation_dis = 2 * math.sqrt(
                                p * (p - a) * (p - b) * (p - c) + 0.01) / c  # 点到直线距离,利用了海伦公式求面积
                        except:
                            print("点到直线距离计算失败......")
                    else:
                        deviation_dis = 0
                    # Is deviation ?
                    # logging.debug(f"Point2line distance {deviation_dis}.")
                    if deviation_dis > 10:
                        logging.debug(f"Deviation {deviation_dis}, num++, num now - {deviation_num}.")
                        deviation_num += 1
                    else:
                        deviation_num = 0

                    # deviation
                    if deviation_num > 3:
                        result = 'Deviation'
                        logging.info("check deviation......")
                        break
                    # Threshold; Judgement
                    # Timeout
                    if small_move_num > 6:
                        result = 'Stuck'
                        break
                # ============================ #

            # Timeout Check if stuck at one point
            mid_point_time = time.time()
            last_time = mid_point_time
            if (mid_point_time - start_time) > mission_time_out_th:
                result = 'Stuck'
                break

            # 如果产生dumpcore.sh_arducopter.(*).out文件,说明程序崩溃
            dump_list_new = self.get_dump_files(log_path)
            if len(dump_list) > len(dump_list_new):
                logging.info(f"Other Exception:{dump_list_new}")

        logging.info(f"Monitor Result: {result}")
        return result


    def mav_monitor_connect(self):
        """
        mavlink connect
        :return:
        """
        logging.info("mav_monitor_connect-----")
        return self._mav_monitor.connect()

    def mav_monitor_set_mission(self, mission_file, random: bool = False) -> bool:
        """set mission"""
        logging.info("mav_monitor_set_mission......")
        try:
            return self._mav_monitor.set_mission(mission_file, random)
        except func_timeout.exceptions.FunctionTimedOut as e:
            logging.info("设置飞行任务失败:", mission_file)
            logging.info(e)
            # self.killPort()
            return False

    def mav_monitor_set_param(self, params, values):
        """set drone configuration"""
        for param, value in zip(params, values):
            self._mav_monitor.set_param(param, value)

    def mav_monitor_get_param(self, param):
        """get drone configuration"""
        return self._mav_monitor.get_param(param)

    def mav_monitor_start_mission(self):
        """start mission"""
        self._mav_monitor.start_mission()
        logging.info("mav_monitor_start_mission")

    def start_mav_monitor(self):
        """
        start monitor
        :return:
        """
        logging.info("start_mav_monitor...... ")
        self._mav_monitor.start()

    def stop_sitl(self):
        """stop the simulator"""
        logging.info("stop_sitl......")
        self._sitl_task.sendcontrol('c')
        while True:
            line = self._sitl_task.readline()
            if not line:
                break
        self._sitl_task.close(force=True)

        self.killGazebo()
        self.killArducopter()

    def killGazebo(self):
        cmd = f"bash {root_dir}/Util/shell/killGazebo.sh"
        os.system(cmd)

    def killPort(self):
        logging.info("kill port 5760......")
        cmd = f"bash {root_dir}/Util/shell/killPort.sh"
        os.system(cmd)

    def killArducopter(self):
        cmd = f"bash {root_dir}/Util/shell/killArducopter.sh"
        print(cmd)
        os.system(cmd)

    def get_mav_monitor(self):
        return self._mav_monitor

    def sitl_task(self) -> spawn:
        return self._sitl_task


def threading_send(task):
    t = threading.currentThread()
    while getattr(t, "do_run", True):
        task.send('status GLOBAL_POSITION_INT \n')  # 为了获取位置信息
        time.sleep(0.1)


def mutate_conf_missions(params, values, mission=f"{root_dir}/Util/mission.txt",is_local=False):
    """配置变异 +飞行任务变异"""
    simManager = GaSimManager()

    # 启动飞控系统sitl
    simManager.start_sitl()
    # simManager.start_test_suite("TakeoffAlt")
    # init mav_moniter
    simManager.mav_monitor_init()

    # 连接
    simManager.mav_monitor_connect()

    simManager.mav_monitor_set_param(params, values)

    # set mission
    simManager.mav_monitor_set_mission(mission)
    # start mission
    simManager.mav_monitor_start_mission()
    # moniter error
    if not is_local:
        result = simManager.mav_monitor_error_new(mission)
        logging.info(f"Result------>{result}")
    else:
        result,score = simManager.mav_monitor_error_local(mission)
        logging.info(f"Result------>{result}")
        logging.info(f"Score------>{score}")

    simManager.stop_sitl()


    return result


def run():
    logging.basicConfig(level=logging.INFO)

    startTime = time.time()

    simManager = GaSimManager()


    # --------start gazebo-----------------
    # simManager.start_gazebo()
    # -------------------------------------

    # 启动飞控系统sitl
    simManager.start_sitl()
    # simManager.start_test_suite("TakeoffAlt")
    # init mav_moniter
    simManager.mav_monitor_init()
    # 连接
    simManager.mav_monitor_connect()

    # set mission
    mission = "mission.txt"
    # mission="AVC2013_mission.txt"

    simManager.mav_monitor_set_mission(mission)
    # start mission
    simManager.mav_monitor_start_mission()
    # moniter error
    result,score = simManager.mav_monitor_error_local(mission)
    print("Result------------->", result)
    print("Score------------->", score)
    simManager.stop_sitl()

    print("over________________________")
    endTime = time.time()
    print("运行时间为：", endTime - startTime)


# 测试此文件中的函数功能
if __name__ == '__main__':
    run()
