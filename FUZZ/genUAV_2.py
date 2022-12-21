import json
import random
import datetime
import csv
import pysnooper
from colorama import Fore, init

init()
import time
import numpy as np
import os
np.set_printoptions(suppress=True)
from sko.GA import GA
import Util.gaSimManager
from Util.gaMavlink import GaMavlink
from Util.gaSimManager import mutate_conf_missions, GaSimManager
import pandas as pd
import matplotlib.pyplot as plt
import logging
cur_dir = os.path.dirname(os.path.realpath(__file__))
root_dir = os.path.realpath(os.path.join(cur_dir, '..'))
logging.basicConfig(format='%(asctime)s - %(filename)s[line:%(lineno)d] - %(levelname)s: %(message)s',
                    level=logging.INFO, filename='../Util/conf_12_12.Log')

class UAVGen(object):
    def __init__(self):
        self.ga = None
        self.gaManager = GaSimManager()
        self.results = []
        self.envs = None  #
        self.is_envs = True #
        self.is_mission = True  #
        self.is_local = False  #
        self.envs_len = 0
        self.params = None
        self.result_path=None




    def Fit(self, result):
        result_list=["PreArm Failed","Deviation","Stuck","Crash","Thrust Loss","Vibration","Yaw Imbalance","Pass"]
        if result=="PreArm Failed":
            return 128
        elif result=="Crash":
            return 4
        elif result=="Thrust Loss":
            return 16
        elif result=="Yaw Imbalance":
            return 16
        elif result=="Vibration":
            return 256
        elif result=="Deviation":
            return 8
        elif result=="Stuck":
            return 8
        elif result == "Pass":
            return 1


    def initPop(self,algorithm):
        self.Chrom = np.random.randint(low=0, high=2, size=(algorithm.size_pop, algorithm.len_chrom))


    def my_selection_roulette_2(self,algorithm):

        FitV = algorithm.FitV
        # print("FitV:",self.Chrom)
        print(Fore.GREEN, "FitV:", FitV, Fore.RESET)
        dict_fit = {}
        for key in FitV:
            dict_fit[key] = dict_fit.get(key, 0) + 1
        for key in dict_fit.keys():
            if dict_fit[key] >= len(FitV) * 0.3:
                algorithm.Chrom[0:int(algorithm.size_pop / 2)] = np.random.randint(low=0, high=2,
                                                                         size=(int(algorithm.size_pop / 2), algorithm.len_chrom))
        print(Fore.GREEN, "FitV:", FitV, Fore.RESET)
        FitV = (FitV - FitV.min()) / (FitV.max() - FitV.min() + 1e-10) + 0.2
        # the worst one should still has a chance to be selected
        sel_prob = FitV / FitV.sum()
        sel_index = np.random.choice(range(algorithm.size_pop), size=algorithm.size_pop, p=sel_prob)
        algorithm.Chrom = algorithm.Chrom[sel_index, :]
        return algorithm.Chrom

    def schaffer(self, values):

        # print(Fore.GREEN, np.array(values) * self.param_value_unit, Fore.RESET)
        values = values * self.param_value_unit


        if self.is_mission:  # 变异飞行任务
            missions_path = f"{root_dir}/Mission/random_missions"
            missions_list = os.listdir(missions_path)
            mission_name = 'mission_' + str(random.randint(0, len(missions_list) - 1)) + '.txt'
            mission = missions_path + '/' + mission_name
        # print("mission:", mission)
        # print(Fore.GREEN,f"Testing:\nvalues:---------------->{values}\n mission:----------------->{mission_name}", Fore.RESET)
            result = mutate_conf_missions(self.params, values, mission)
        else:
            mission_name="mission.txt"
            result = mutate_conf_missions(self.params, values)
        score=self.Fit(result)
        self.results.append(result)

        info = values.tolist()
        info = [round(i, 4) for i in info]  #
        info = info + [result,mission_name,score]
        logging.info(f"{info}")
        # print(Fore.GREEN,"info:", info,Fore.RESET)

        with open(self.result_path, 'a') as f:
            csv_file = csv.writer(f)
            csv_file.writerow(info)
        logging.info("------------------------------")
        return -score  #


    # @pysnooper.snoop()
    def run_gen(self,params_path,result_path,envs_path=None,missions_path=None):
        # info="\n---------------"+datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")+"--------------\n"

        param_dict = json.loads(open(params_path).read())

        #
        if envs_path is not None:
            self.envs=json.loads(open(envs_path).read())
            param_dict.update(self.envs)
            self.envs_len = len(self.envs)

        table_head = list(param_dict.keys()) + ["Result","Mission","Score"]
        with open(result_path, 'a+') as f:
            csv_file = csv.writer(f)
            csv_file.writerow(table_head)
        param_value_range = GaMavlink.read_range_from_dict(param_dict)
        self.param_value_unit = GaMavlink.read_unit_from_dict(param_dict)
        self.params = list(param_dict.keys())
        n_dim = len(list(param_dict.keys()))  # 目标函数的维度
        size_pop = 50  # 种群规模(种群的个体数) 必须是偶数
        max_iter = 400  # 最大迭代次数
        prob_mut = 0.01  # 变异概率
        print("param_value_range:", param_value_range)
        lb = np.array(param_value_range)[:, 0] / self.param_value_unit  # 每个自变量的最小值
        ub = np.array(param_value_range)[:, 1] / self.param_value_unit  # 每个自变量的最大值
        print("lb:", lb)
        print("ub:", ub)

        precision = 1  # 精准度(x的精度)

        self.ga = GA(func=self.schaffer, n_dim=n_dim, size_pop=size_pop, max_iter=max_iter, prob_mut=prob_mut, lb=lb,
                     ub=ub, precision=precision)
        # 注入自定义算子---->灾变
        self.ga.register(operator_name="selection",operator=self.my_selection_roulette_2)
        best_x, best_y = self.ga.run()

        print('best_x:', best_x, '\n', 'best_y:', best_y)
        print("generation_best_X:", self.ga.generation_best_X)
        print("generation_best_Y:", self.ga.generation_best_Y)

    def draw_result(self):
        print("Result:", self.results)

        Y_history = pd.DataFrame(self.ga.all_history_Y)
        print("Y_history:", Y_history)

        fig, ax = plt.subplots(2, 1)  # 两行一列
        ax[0].plot(Y_history.index, Y_history.values, '.', color='red')  # 所有个体(每一代可能多个)
        Y_history.min(axis=1).cummin().plot(kind='line')  # 每一代的最小值
        plt.show()


# todo:
if __name__ == '__main__':
    startTime = time.time()
    print(f"开始fuzz.......{startTime}")

    uavgen = UAVGen()
    params_path= f"{root_dir}/Util/Json/param_ardu_new.json"
    envs_path= f"{root_dir}/Util/Json/param_envs.json"
    missions_path = f"{root_dir}/Mission/random_missions"
    uavgen.result_path=None

    if uavgen.is_envs and uavgen.is_mission:  # 测试环境因素和飞行任务
        uavgen.result_path=f"{root_dir}/Result/ga_12_12.csv"
        uavgen.run_gen(params_path,uavgen.result_path,envs_path=envs_path,missions_path=missions_path)  # 启动遗传算法
    elif uavgen.is_envs:  # 只测试环境因素
        uavgen.result_path = f"{root_dir}/Result/ga_envs.csv"
        uavgen.run_gen(params_path, uavgen.result_path,envs_path=envs_path)
    elif uavgen.is_mission:  # 只测试飞行任务
        uavgen.result_path = f"{root_dir}/Result/ga_missions.csv"
        uavgen.run_gen(params_path, uavgen.result_path,missions_path=missions_path)  # 启动遗传算法
    else:
        uavgen.result_path = f"{root_dir}/Result/ga_12_4.csv"  # 飞行任务和环境都不变异
        uavgen.run_gen(params_path, uavgen.result_path)  # 启动遗传算法


    uavgen.draw_result()  # 绘图

    endTime = time.time()
    print("运行时间为：", endTime - startTime)
