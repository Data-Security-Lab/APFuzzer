import random
import json
import csv
import logging
import time
import os
from Util.gaMavlink import GaMavlink
from Util.gaSimManager import GaSimManager
from Util.gaSimManager import mutate_conf_missions
#
logging.basicConfig(format='%(asctime)s - %(filename)s[line:%(lineno)d] - %(levelname)s: %(message)s',
                    level=logging.INFO, filename='../Util/log/envs_12_12.log')

cur_dir = os.path.dirname(os.path.realpath(__file__))
project_dir = os.path.realpath(os.path.join(cur_dir, '../'))
start_time=time.time()
def progress(text):
    """Pretty printer."""
    delta_time = time.time() - start_time
    formatted_text = "****** AT-%06.1f: %s" % (delta_time, text)
    print(formatted_text)
class RandomFuzzer():
    def __init__(self):
        self.gaMavlink = GaMavlink()
        self.gaManager = GaSimManager()
        # self.envs=Json.loads(open("Util/param_envs.Json").read())
        self.is_envs = True  # 是否加入环境因素
        # self.env_len=0   # 环境配置的个数
        # self.a=1

    def update_params(self,params_dict,envs_dict):
        logging.info("update_params......")
        if self.is_envs is True:
            params_dict.update(envs_dict)
        return params_dict

    def random_generator_single(self, param, params_dict):
        # 获取取值范围
        range_min = params_dict[param]["range"][0]
        range_max = params_dict[param]["range"][1]
        step = params_dict[param]["step"]
        # progress(f"{range_min/step}, {range_max/step}")
        random_value = random.randint(range_min/step, range_max/step)
        return [param, random_value*step]

    # 一次对n个配置变异
    def random_generator_multiple(self, params_dict: dict):
        param_select=list(params_dict.keys())
        key_values = {}
        for param in param_select:
            key, value = self.random_generator_single(param, params_dict)
            key_values[key] = value

        params=list(key_values.keys())
        values=list(key_values.values())
        values = [round(i, 4) for i in values]
        progress(f"变异后的配置:{values}")
        return params,values  # 返回一个list

    def run(self, params_dict):
        # params = list(params_dict.keys())
        params,values = self.random_generator_multiple(params_dict)
        result = mutate_conf_missions(params, values)
        return values, 0, result
    def run_multiple_missions(self,params_dict):
        """飞行任务变异+配置变异"""
        params,values = self.random_generator_multiple(params_dict)
        missions_path=project_dir+'/Mission/random_missions'
        missions_list=os.listdir(missions_path)
        mission_name='mission_'+str(random.randint(0,len(missions_list)-1))+'.txt'
        mission=missions_path+'/'+mission_name
        progress(f"mission:{mission}")
        result = mutate_conf_missions(params, values, mission)
        return values, 0, result, mission_name



def test():
    params_dict = json.loads(open("../Util/Json/param_ardu_new.json").read())
    envs_dict = json.loads(open("../Util/Json/param_envs.json").read())
    params_dict = random_fuzzer.update_params(params_dict, envs_dict)

if __name__ == '__main__':
    startTime=time.time()
    random_fuzzer = RandomFuzzer()

    # 是否添加飞行任务
    is_missions=True
    # 初始化覆盖率
    # random_fuzzer.gaManager.init_coverage()

    # 选择配置参数
    params_dict = json.loads(open("../Util/Json/param_ardu_new.json").read())
    # params_dict={}
    envs_dict = json.loads(open("../Util/Json/param_envs.json").read())
    params_dict=random_fuzzer.update_params(params_dict,envs_dict)
    print(params_dict)
    # random_fuzzer.random_generator_multiple(params_dict)


    # 写表头
    result_table=f"{project_dir}/Result/random_12_12.csv"
    if not os.path.exists(result_table):
        open(result_table,'w')
    table_head = list(params_dict.keys()) + ["Result", "Mission" , "Score"]
    with open(result_table, 'a+') as f:
        csv_file = csv.writer(f)
        csv_file.writerow(table_head)

    # 设置随机测试次数
    echos = 3000


    mission="mission.txt"

    for i in range(0, echos):
        logging.info(f"---------------{i}-----------------")
        if time.time()-start_time>=12*60*60:
            print("运行结束...")
            logging.info("------------------8h--------------------")
            break

        random_fuzzer.gaManager.killPort()
        if not is_missions:  # 不改变飞行任务的情况
            values, cov, result = random_fuzzer.run(params_dict)
            info = values + [result, mission, cov]
        else:  # 改变飞行任务的情况
            values, cov, result, mission = random_fuzzer.run_multiple_missions(params_dict)
            info = values + [result, mission, cov]
        # print("info:", info)
        logging.info(f"params:{params_dict.keys()}")
        logging.info(f"values:{values}")
        logging.info(f"mission:{mission}")
        with open(result_table, 'a+') as f:
            csv_file = csv.writer(f)
            csv_file.writerow(info)
        print("--------------------------------------------")
    print("程序运行时间:",time.time()-startTime)