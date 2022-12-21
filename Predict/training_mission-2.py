
import json
import os
import numpy as np
import torch
import torch.utils.data as Data
import pandas as pd
import matplotlib

cur_dir = os.path.dirname(os.path.realpath(__file__))
project_dir = os.path.realpath(os.path.join(cur_dir, '..'))
INPUT_CONF_NUM=len(json.loads(open(f"{project_dir}/Util/Json/param_ardu_new.json").read()))+len(json.loads(open(f"{project_dir}/Util/Json/param_envs.json").read()))
INPUT_FEATURES_NUM = 120+INPUT_CONF_NUM
OUTPUT_FEATURES_NUM = 8


def mission_extract(mission):
    lines=open(mission).read().strip().split('\n')
    mission_new=np.zeros((10 ,12))
    for i,line in enumerate(lines[1:]):
        line=line.split('\t')
        line = np.array(list(map(float, line)))
        mission_new[i]=np.absolute(line)+np.array([0.00001 for _ in range(12)])

    return mission_new.flatten()


def pre_trainset(train_data_path, mission_dir, result_path):
    data = np.genfromtxt(train_data_path, delimiter=',', skip_header=1)[:, 0:INPUT_CONF_NUM]
    data=np.absolute(data)
    missions = pd.read_csv(train_data_path)[['Mission']]
    print("mission_dir:",mission_dir)

    mission_list = []
    for index, row in missions.iterrows():
        # print("index:",index)
        mission_path = f"{mission_dir}/{row['Mission']}"

        mission_new = mission_extract(mission_path)
        mission_list.append(list(mission_new))

    mission_list = np.array(mission_list)

    result = np.append(data, mission_list, axis=1)
    print("result.shape:",len(result))
    np.set_printoptions(suppress=True)
    np.savetxt(result_path, result, delimiter=',',fmt='%.5f')

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

mission_dir = f"{project_dir}/Mission/random_missions"
# 训练集
train_data_path = f"{project_dir}/Result/predictor/predictor_train.csv"
# 训练集数据处理后的路径train_data_path_pro
train_data_path_pro= f"{project_dir}/Result/predictor/predictor_train_pro.csv"
pre_trainset(train_data_path, mission_dir, train_data_path_pro)

# 测试集
test_data_path = f"{project_dir}/Result/predictor/predictor_test.csv"
# 测试集数据处理后的路径test_data_path_pro
test_data_path_pro=f"{project_dir}/Result/predictor/predictor_test_pro.csv"
pre_trainset(test_data_path, mission_dir, test_data_path_pro)

# 加载训练集train_data
train_data = np.loadtxt(train_data_path_pro,delimiter=',')
# 数据归一化
mu = np.mean(train_data, axis=0)  # mean计算平均值 axis=0 对各列求平均值 axis=1对各行求平均值
std = np.std(train_data, axis=0)
train_data = (train_data - mu) / std
train_data_tensor = torch.from_numpy(train_data).to(dtype=torch.float32)

# 加载训练集标签
result_dict={"PreArm Failed":0, "Crash":1, "Thrust Loss":2, "Yaw Imbalance":3, "Vibration":4, "Deviation":5, "Stuck":6, "Pass":7}

train_label_str=pd.read_csv(train_data_path)[['Result']]
train_label=np.zeros(train_data.shape[0])
for index,row in train_label_str.iterrows():
    train_label[index]=result_dict[row["Result"]]
train_label_tensor = torch.from_numpy(train_label)
print(train_label_tensor.size())
train_set = Data.TensorDataset(train_data_tensor, train_label_tensor)
train_loader = Data.DataLoader(dataset=train_set, batch_size=32, shuffle=True, num_workers=4)

# 加载test_data
test_data = np.loadtxt(test_data_path_pro,delimiter=',')
test_data = (test_data - mu) / std
test_data_tensor = torch.from_numpy(test_data).to(dtype=torch.float32)
test_label_str=pd.read_csv(test_data_path)[['Result']]
test_label=np.zeros(test_data.shape[0])
for index,row in test_label_str.iterrows():
    test_label[index]=result_dict[row["Result"]]
test_label_tensor = torch.from_numpy(test_label)
print(test_label_tensor.size())
train_set = Data.TensorDataset(test_data_tensor, test_label_tensor)
test_loader = Data.DataLoader(dataset=train_set, batch_size=32, shuffle=False, num_workers=4)


#
class Net(torch.nn.Module):
    def __init__(self, n_feature=INPUT_FEATURES_NUM, n_hidden1=128, n_hidden2=64, n_output=OUTPUT_FEATURES_NUM):
        super(Net, self).__init__()
        self.hidden1 = torch.nn.Linear(n_feature, n_hidden1)  # n_feature 输入个数，n_hidden 隐藏层神经元个数，n_output 输出个数
        self.dropout = torch.nn.Dropout(p=0.5)
        self.hidden2 = torch.nn.Linear(n_hidden1, n_hidden2)
        self.predict = torch.nn.Linear(n_hidden2, n_output)

    def forward(self, x):
        x = torch.relu(self.hidden1(x))
        x = self.dropout(x)
        x = torch.relu(self.hidden2(x))
        # x = self.dropout(x)
        x = self.predict(x)
        return x
acc1_list=[]

def train(net, optimizer, loss_func):
    # 训练网络
    epochs=200

    best_acc = 0
    for epoch in range(epochs):
        if epoch in [50, 70, 90]:
            for param_group in optimizer.param_groups:
                param_group['lr'] /= 10
        net.train()

        running_loss = 0.0
        total = 0.0
        correct = 0.0
        for step, (data_x, data_y) in enumerate(train_loader):
            data_x = data_x.to(device)
            data_y = data_y.to(device)
            out = net(data_x)  
            loss = loss_func(out, data_y.long())
            optimizer.zero_grad() 
            loss.backward()  
            optimizer.step() 
            running_loss += loss.item()
            _, predicted = torch.max(out.data, 1)  
            total += data_y.shape[0]
            correct += (predicted == data_y).sum().item()
            if step % 100 == 99:  
                print('Epoch:%d / Step:%4d loss:%.3f' % (epoch + 1, step + 1, running_loss / step))
        print('Train Set Accuracy: %.4f%%' % (100 * correct / total))  

        acc1 = 0
        with torch.no_grad():
            correct = 0.0
            total = 0.0
            for data in test_loader:
                net.eval()
                datax, datay = data
                datax, datay = datax.to(device), datay.to(device)
                outputs = net(datax)
                _, predicted = torch.max(outputs.data, 1)
                total += datay.shape[0]
                correct += float((predicted == datay).sum())
            acc1 = (100 * correct / total)
            acc1_list.append(acc1)
            if acc1 > best_acc:
                best_acc = acc1
                torch.save(net.state_dict(),'mission_conf.pt')
        print('Test Set Accuracy: %.4f%%' % acc1)
    print("acc1_list:",acc1_list)
    print("bast acc:",best_acc)


# 对测试集测试
def test(net):
    acc1 = 0
    with torch.no_grad():
        correct = 0.0
        total = 0.0
        for data in test_loader:
            net.eval()
            datax, datay = data
            datax, datay = datax.to(device), datay.to(device)
            outputs = net(datax)
            _, predicted = torch.max(outputs.data, 1)
            total += datay.shape[0]
            correct += float((predicted == datay).sum())
        acc1 = (100 * correct / total)
    print('Test Set Accuracy: %.4f%%' % acc1)


if __name__=='__main__':

    net = Net(INPUT_FEATURES_NUM, 128, 64, OUTPUT_FEATURES_NUM)  # 输入是n*98的矩阵，一个中间层，神经元个数10个，输出是n*8的矩阵
    net = net.to(device)

    is_train=True
    if is_train:
        print("INPUT_FEATURES_NUM:",INPUT_FEATURES_NUM)

        # 定义损失函数和优化器
        optimizer = torch.optim.SGD(net.parameters(),lr=0.01, momentum=0.9, weight_decay=5e-4)  ## weight decay
        loss_func = torch.nn.CrossEntropyLoss()  # 损失函数

        # 开始训练
        train(net, optimizer, loss_func)
    else:
        # 对测试集测试
        net.load_state_dict(torch.load("mission_conf.pt"))
        test(net)