#!/usr/bin/env python
"""
生成mission.txt文件,存放在coverage/missions文件夹
"""
import logging
import os.path
import random
import time
# import setting
from pymavlink import mavutil
import argparse
import logging
# system-specfiic
# arduCommandTable = {
#     ##NAVIGATION COMMANDS
#     'takeoff'    : 'MAV_CMD_NAV_TAKEOFF',          #param7: Alt
#                                                    #first command
#     'straightwp' : 'MAV_CMD_NAV_WAYPOINT',         #param1: Delay (sec), param5: Lat, param6: Lon, param7: Alt
#     'splinewp'   : 'MAV_CMD_NAV_SPLINE_WAYPOINT',  #param1: Delay (sec), param5: Lat, param6: Lon, param7: Alt
#     'circle'     : 'MAV_CMD_NAV_LOITER_TURNS',     #param1: Turns# (360 degrees), param5: Lat, param6: Lon, param7: Alt
#                                                    #circle flight mode
#     'wait'       : 'MAV_CMD_NAV_LOITER_TIME',      #param1: Time (sec), param5: Lat, param6: Lon, param7: Alt
#                                                    #Loiter flight mode
#     'rtl'        : 'MAV_CMD_NAV_RETURN_TO_LAUNCH', #RTL flight mode
#                                                    #last command
#     'land'       : 'MAV_CMD_NAV_LAND',             #param5: Lat, param6: Lon
#                                                    #Land flight mode
#     'cmdjump'    : 'MAV_CMD_DO_JUMP',              #param1: WP#, param2: Repeat#
#
#     ##CONDITION COMMANDS
#     'delay'      : 'MAV_CMD_CONDITION_DELAY',      #param1: Time (sec)
#     'changealt'  : 'MAV_CMD_CONDITION_CHANGE_ALT', #param1: Rate (m/sec), param7: Alt
#     'distance'   : 'MAV_CMD_CONDITION_DISTANCE',   #param1: Dist (m)
#     'changehead' : 'MAV_CMD_CONDITION_YAW',        #param1: Deg, param3: Dir (add or subtract for Rel), param4: Rel or Abs
#
#     ##DO COMMANDS
#     'changespeed': 'MAV_CMD_DO_CHANGE_SPEED',      #param2: Speed (m/sec)
#     'sethome'    : 'MAV_CMD_DO_SET_HOME'           #param1: Current (1=current,2=in-message), param5: Lat, param6: Lon, param7: Alt
#
#     ##ADDITIONAL DO COMMANDS
#     #MAV_CMD_DO_SET_RELAY,                         #param1: Relay no., param2: Off(0) or On(1)
#     #MAV_CMD_DO_REPEAT_RELAY,                      #param1: Relay no., param2: Repeat#, param3: Delay (sec)
#     #MAV_CMD_DO_SET_SERVO,                         #param1: Servo no., param2: PWM (microsec, typically 1000-to-2000)
#     #MAV_CMD_DO_REPEAT_SERVO,                      #param1: Servo no., param2: PWM (microsec, typically 1000-to-2000), param3: Repeat#, param4: Delay (sec)s
#     #MAV_CMD_DO_SET_ROI,                           #param5: Lat, param6: Lon, param7: Alt (of the fixed ROI i.e., region of interest)
#     #MAV_CMD_DO_DIGICAM_CONFIGURE,                 #param1: Mode (1: ProgramAuto 2: Aperture Priority 3: Shutter Priority 4: Manual 5: IntelligentAuto 6: SuperiorAuto),
#     #                                              #param2: Shutter speed, param3: Aperture, param4: ISO (e.g., 80,100,200), param7: Engine cut-off time
#     #MAV_CMD_DO_DIGICAM_CONTROL,                   #param1: Off(0) or On(1), param4: Focus lock (0: Ignore 1: Unlock 2: Lock), param5: Shutter command (any non-zero value)
#     #MAV_CMD_DO_MOUNT_CONTROL,
#     #MAV_CMD_DO_SET_CAM_TRIGG_DIST,
#     #MAV_CMD_DO_PARACHUTE
# }
import pysnooper
logging.basicConfig(level=logging.DEBUG)

stateTable_all={"Navigation":["takeoff","waypoint","loiter_unlim","loiter_turns","loiter_time","spline_wp","nav_delay","rtl","land"],
                "DO":["jump","change_speed","set_home"],
                "Condition":["condition_delay","condition_distance","condition_yaw"]}

stateStart={"waypoint":16,"takeoff":22}
stateMiddle={"waypoint":16,"loiter_turns":18,"loiter_time":19,"nav_delay":93,
             "change_speed":178,
             "condition_delay":112 ,"condition_distance":114, "condition_yaw":115}
stateMiddle_list={"Navigation":["waypoint","loiter_turns","loiter_time","nav_delay"],
                  "DO"        :["change_speed"],
                  "Condition" :["condition_delay","condition_distance", "condition_yaw"]
                  }
stateEnd={"land":21}

start_time=time.time()
def progress(text):
    """Pretty printer."""
    delta_time = time.time() - start_time
    formatted_text = "****** AT-%06.1f: %s" % (delta_time, text)
    print(formatted_text)
def editWaypoint(missionFile,index,commandLine):
    """修改特定的wp"""
    # 读取修要修改的mission
    mission=open(missionFile,'w').read().split('\n')

    # 修改mission
    mission[index]=commandLine

    # 写入修改
    with open(missionFile,'r') as f:
        f.write(missionFile)


def set_command(i,command,position):
    lat,lon,alt=position[0],position[1],position[2]
    commandline=""
    # Navigation
    if command == '16' or command == 'waypoint':  # 'MAV_CMD_NAV_WAYPOINT' 导航到wp
        commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (i, 0, 3, 16, 0, 0, 0, 0, lat, lon, alt, 1)

    elif command == '17' or command == 'loiter_unlim':

        commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
            i, 0, 3, 17, 0, 0, 0, 0, lat, lon, alt, 1)

    elif command == '18' or command == 'loiter_turns':  # MAV_CMD_NAV_LOITER_TURNS 在特定点悬停X圈
        p1_turns = random.randint(1, 5)  # Turns
        p3_radius = random.randint(1, 5)  # Radius
        commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
        i, 0, 3, 18, p1_turns, p3_radius, 0, 0, lat, lon, alt, 1)

    elif command == '19' or command == 'loiter_time':
        p1_time = random.randint(1, 5)
        commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
            i, 0, 3, 19, p1_time, 0, 0, 0, lat, lon, alt, 1)

        # 偏航检测有问题
    elif command == '82' or command == 'spline_wp':  # MAV_CMD_NAV_SPLINE_WAYPOINT 使用样条路径（样条曲线）导航到wp，经过该点时画了个弧线
        p1_holdTime = random.randint(0, 5)  # hold time
        commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
        i, 0, 3, 82, p1_holdTime, 0, 0, 0, lat, lon, alt, 1)

    elif command == '93' or command == 'nav_delay':  # MAV_CMD_NAV_DELAY 将下一个导航命令延迟几秒或直到指定时间
        p1 = random.randint(1, 5)
        commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
            i, 0, 3, 93, p1, -1, -1, -1, 0, 0, 0, 1)
    # "DO"
    elif (command == '177' or command == 'jump') and i >= 4:  # 多少检测有问题
        p1_wp = random.randint(2, i - 2)
        p2_repeat = random.randint(1, 3)
        commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
        i, 0, 3, 177, p1_wp, p2_repeat, 0, 0, 0, 0, 0, 1)

    elif command == '178' or command == 'change_speed':  # MAV_CMD_DO_CHANGE_SPEED  更改速度/油门
        p2_speed = random.randint(0, 20)
        commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (i, 0, 3, 178, 0, p2_speed, 0, 0, 0, 0, 0, 1)

    elif command == '179' or command == 'set_home':
        p1_current = random.choice([0, 1])
        commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
            i, 0, 3, 179, p1_current, 0, 0, 0, lat, lon, alt, 1)

    # Condition commands
    elif command == '112' or command == 'condition_delay':  # MAV_CMD_CONDITION_DELAY 延迟任务状态机
        p1_time = random.randint(0, 5)  # delay time
        commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
        i, 0, 3, 112, p1_time, 0, 0, 0, 0, 0, 0, 1)

    elif command == '114' or command == 'condition_distance':  # MAV_CMD_CONDITION_DISTANCE 延迟任务状态机，直到下一个NAV点所需距离内
        p1_distance = random.randint(0, 5)
        commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
        i, 0, 3, 114, p1_distance, 0, 0, 0, 0, 0, 0, 1)

    elif command == '115' or command == 'target_yaw':  # 'MAV_CMD_CONDITION_YAW ' : 115,  # Reach a certain target angle.
        p1_angle = random.randint(0, 30)
        p2_angular_speed = random.randint(0, 10)
        p3_direction = random.choice([-1, 1])
        p4_ralative = random.choice([0, 1])
        commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
        i, 0, 3, 115, p1_angle, p2_angular_speed, p3_direction, p4_ralative, 0, 0, 0, 1)
    return commandline

def get_commandType(command):
    if command in stateMiddle_list["Navigation"]:
        return "Navigation"
    elif command in stateMiddle_list["DO"]:
        return "DO"
    elif command in stateMiddle_list["Condition"]:
        return "Condition"

# @pysnooper.snoop()
def generateMission(seq,home_location,mission_len,bounds):
    """
    产生飞行任务
    """
    logging.info("generate missions......")
    # print("bounds:",bounds)
    latMin=bounds[0]
    latMax=bounds[1]
    lonMin=bounds[2]
    lonMax=bounds[3]
    altMin=bounds[4]
    altMax=bounds[5]
    timeMin=bounds[6]
    timeMax=bounds[7]


    output = 'QGC WPL 110\n'
    commandline=""

    # 任务生成需要满足有限状态机
    # 1) Do命令后面必须接Navigation命令
    # 2) Condition命令后必须接Do命令
    pre_command=""  # ["Navigation","DO","Condition"]
    for i in range(mission_len):
        lat=format(random.uniform(latMin,latMax),'.5f')  # 保留5位小数
        lon=format(random.uniform(lonMin,lonMax),'.5f')
        alt=format(random.uniform(altMin,altMax),'.5f')
        position=[lat,lon,alt]

        # 选择command,最后一个任务设置为land或者rtl

        # Start
        # index 0 wp
        if i==0:
            commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % \
                          (0, 1, 0, 16, 0, 0, 0, 0, home_location[0], home_location[1],home_location[2], 1)
            output += commandline
        # index 1 takeoff
        elif i==1:
            commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % \
                          (1, 0, 3, 22, 0, 0, 0, 0, home_location[0], home_location[1], alt, 1)
            output += commandline
            pre_command="Navigation"

        # Middle
        elif i>=2 and i<mission_len-1:
            if pre_command=="Navigation":
                command_list=stateMiddle_list["Navigation"]+stateMiddle_list["DO"]+stateMiddle_list["Condition"]
                command = str(random.choice(command_list))
                commandline=set_command(i,command,position)
                output += commandline
                pre_command=get_commandType(command)

            elif pre_command=="DO":
                command_list = stateMiddle_list["Navigation"] + stateMiddle_list["DO"]
                command = str(random.choice(command_list))
                commandline = set_command(i, command, position)
                output += commandline
                pre_command = get_commandType(command)

            elif pre_command=="Condition":
                command_list = stateMiddle_list["DO"]
                command = str(random.choice(command_list))
                commandline = set_command(i, command, position)
                output += commandline
                pre_command = get_commandType(command)


        # End: land or rtl
        elif i == mission_len - 1:
            # logging.info("i=last......")
            command = str(random.choice(list(stateEnd.values())))

            # 待实现
            if command == '20' or command == 'rtl':  # MAV_CMD_NAV_RETURN_TO_LAUNCH
                commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
                i, 0, 3, 20, 0, 0, 0, 0, 0, 0, 0, 1)

            elif command == '21' or command == 'land':  # MAV_CMD_NAV_LAND
                p1_abortAlt = random.randint(5, 10)  # Minimum target altitude if landing is aborted
                p2_landMode = random.choice([0, 2])  # 枚举值
                p4_yawAngle = random.randint(0, 30)  # Yaw Angle
                commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
                i, 0, 3, 21, 0, 0, 0, 0, lat, lon, 0, 1)
            output += commandline

    # 保存mission
    filename="mission_%s.txt"%(str(seq))
    filepath='./random_missions/'+filename

    if not os.path.exists("random_missions"):
        os.mkdir("random_missions")
    with open(filepath,'w+') as f:
        f.write(output)
    f.close()

def generate():
    home_location = [-35.363262, 149.165237, 584.0]  # 584代表海平面高度
    # [0 1]up down,  [2 3]left right, [4 5] alt (相较于home的高度),  [6 7]time
    bounds = [home_location[0] - 0.0002, home_location[0] + 0.0002, home_location[1] - 0.0003, home_location[1] + 0.0003,
              10, 50, 0, 5]

    for i in range(1000):
        mission_len = random.randint(5, 10)
        print(f"mission_len_{i}:{mission_len}")
        generateMission(i, home_location, mission_len, bounds)


if __name__ == '__main__':
    generate()
    progress("测试结束......")
