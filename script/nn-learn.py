# -*- coding: utf-8 -*-
"""
Created on Thu May 16 11:29:39 2019

@author: naoyamada
"""

import numpy as np
import matplotlib.pyplot as plt
import csv
import math

#--------------------------------------------------------------------
#---------------------------入力データの作成---------------------------
    
sit = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/4NN-learn/sit.csv'
stand = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/4NN-learn/stand.csv'
cross = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/4NN-learn/cross.csv'
#２次元のcsvを読んで要素をfloatにする
def str2float(path):
    with open(path) as f:
        reader = csv.reader(f)
        data = [row for row in reader]
    for i in range(np.shape(data)[0]):
        for j in range(np.shape(data)[1]):
            data[i][j] = float(data[i][j])
    return data

sit_data = str2float(sit)
stand_data = str2float(stand)
cross_data = str2float(cross)

#データを結合
sit_data.extend(stand_data)
sit_data.extend(cross_data)
input_data = np.array(sit_data)

#標準化する
#def standardization(array):
    #ave_input = np.average(array, axis=0)
    #std_input = np.std(array, axis=0)
    #standardized = (array - ave_input) / std_input 
    #return standardized




#----------------------正解データを作成----------------
clusta_Num = 3
c0 = [0]*(clusta_Num*100)
c1 = [1]*(clusta_Num*100)
c2 = [2]*(clusta_Num*100)
c0.extend(c1)
c0.extend(c2)
correct = c0

n_data = len(correct)


#正解データをone-hot表現にする
correct_data = np.zeros((n_data,clusta_Num)) #[0,0,0]をn_data個 (n_datax3の行列)
for i in range(n_data):
    correct_data[i, correct[i]] = 1.0
    #correct_dataのi番目の要素(i番目の行)の中の
    #corcorrectrect[i]番目(0番目,1番目,2番目)の要素を1に変換する
print('correct_data --> {}'.format(np.shape(correct_data)))
#------------------------------------------------------------


#訓練データとテストデータに分割    
index = np.arange(n_data)
index_train = index[index%2 == 0] # == [0 2 4 ... ]
index_test = index[index%2 != 0]  # == [1 3 5 ... ]

input_train = []
input_test = []
correct_train = []
correct_test = []


for i in range(n_data):
    if i%2==0:
        input_train.append(input_data[i])
        correct_train.append(correct_data[i])
for i in range(n_data):
    if i%2!=0:
        input_test.append(input_data[i])
        correct_test.append(correct_data[i])
input_train = np.array(input_train)
input_test = np.array(input_test)
correct_train = np.array(correct_train)
correct_test = np.array(correct_test)

n_train = input_train.shape[0] #訓練用データのサンプル数
n_test = input_test.shape[0]    #テスト用データのサンプル数

#NNの定義
n_in = 8
n_mid = 50
n_out = clusta_Num

wb_width = 0.1
eta = 0.01
epoch = 5000
batch_size = 8
interval = 200


# -- 各層の継承元 --
class BaseLayer:
    def __init__(self, n_upper, n):
        self.w = wb_width * np.random.randn(n_upper, n)  # 重み（行列）
        self.b = wb_width * np.random.randn(n)  # バイアス（ベクトル）

        self.h_w = np.zeros(( n_upper, n)) + 1e-8
        self.h_b = np.zeros(n) + 1e-8
        
    def update(self, eta):      
        self.h_w += self.grad_w * self.grad_w
        self.w -= eta / np.sqrt(self.h_w) * self.grad_w
        
        self.h_b += self.grad_b * self.grad_b
        self.b -= eta / np.sqrt(self.h_b) * self.grad_b

# -- 中間層 --
class MiddleLayer(BaseLayer):
    def forward(self, x):
        self.x = x
        self.u = np.dot(x, self.w) + self.b
        self.y = np.where(self.u <= 0, 0, self.u)  # ReLU
    
    def backward(self, grad_y):
        delta = grad_y * np.where(self.u <= 0, 0, 1)  # ReLUの微分

        self.grad_w = np.dot(self.x.T, delta)
        self.grad_b = np.sum(delta, axis=0)
        
        self.grad_x = np.dot(delta, self.w.T) 

# -- 出力層 --
class OutputLayer(BaseLayer):     
    def forward(self, x):
        self.x = x
        u = np.dot(x, self.w) + self.b
        self.y = np.exp(u)/np.sum(np.exp(u), axis=1, keepdims=True)  # ソフトマックス関数

    def backward(self, t):
        delta = self.y - t
        
        self.grad_w = np.dot(self.x.T, delta)
        self.grad_b = np.sum(delta, axis=0)
        
        self.grad_x = np.dot(delta, self.w.T) 
        
# -- ドロップアプト --
class Dropout:
    def __init__(self, dropout_ratio):
        self.dropout_ratio = dropout_ratio  # ニューロンを無効にする確率

    def forward(self, x, is_train):  # is_train: 学習時はTrue
        if is_train:
            rand = np.random.rand(*x.shape)  # 入力と同じ形状の乱数の行列
            self.dropout = np.where(rand > self.dropout_ratio, 1, 0)  # 1:有効 0:無効
            self.y = x * self.dropout  # ニューロンをランダムに無効化
        else:
            self.y = (1-self.dropout_ratio)*x  # テスト時は出力を下げる
        
    def backward(self, grad_y):
        self.grad_x = grad_y * self.dropout  # 無効なニューロンでは逆伝播しない


# -- 各層の初期化 --
ml_1 = MiddleLayer(n_in, n_mid)
dp_1 = Dropout(0.5)
ml_2 = MiddleLayer(n_mid, n_mid)
dp_2 = Dropout(0.5)
ol = OutputLayer(n_mid, n_out)

# -- 順伝播 --
def fp(x, is_train):
    ml_1.forward(x)
    dp_1.forward(ml_1.y, is_train)
    ml_2.forward(dp_1.y)
    dp_2.forward(ml_2.y, is_train)
    ol.forward(dp_2.y)

# -- 逆伝播 --
def bp(t):
    ol.backward(t)
    dp_2.backward(ol.grad_x)
    ml_2.backward(dp_2.grad_x)
    dp_1.backward(ml_2.grad_x)
    ml_1.backward(dp_1.grad_x)

# -- 重みとバイアスの更新 --
def uppdate_wb():
    ml_1.update(eta)
    ml_2.update(eta)
    ol.update(eta)

# -- 誤差を計算 --
def get_error(t, batch_size):
    return -np.sum(t * np.log(ol.y + 1e-7)) / batch_size  # 交差エントロピー誤差


# -- 誤差の記録用 --
train_error_x = []
train_error_y = []
test_error_x = []
test_error_y = []

# -- 学習と経過の記録 --
n_batch = n_train // batch_size  # 1エポックあたりのバッチ数
for i in range(epoch):

    # -- 誤差の計測 --  
    fp(input_train, False)
    error_train = get_error(correct_train, n_train)
    fp(input_test, False)
    error_test = get_error(correct_test, n_test)
    
    # -- 誤差の記録 -- 
    test_error_x.append(i)
    test_error_y.append(error_test) 
    train_error_x.append(i)
    train_error_y.append(error_train) 
    
    # -- 経過の表示 -- 
    if i%interval == 0:
        print("Epoch:" + str(i) + "/" + str(epoch),
              "Error_train:" + str(error_train),
              "Error_test:" + str(error_test))

    # -- 学習 -- 
    index_random = np.arange(n_train)
    np.random.shuffle(index_random)  # インデックスをシャッフルする
    for j in range(n_batch):
        
        # ミニバッチを取り出す
        mb_index = index_random[j*batch_size : (j+1)*batch_size]
        x = input_train[mb_index, :]
        t = correct_train[mb_index, :]
        
        # 順伝播と逆伝播
        fp(x, True)
        bp(t)
        
        # 重みとバイアスの更新
        uppdate_wb() 


#誤差の記録をグラフ表示
plt.plot(train_error_x, train_error_y, label="Train")
plt.plot(test_error_x, test_error_y, label="Test")
plt.legend()
plt.xlabel("Epochs")
plt.ylabel("Error")


#学習した重みを保存
w_ml_1 = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/NN-result/w_ml_1.csv'
b_ml_1 = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/NN-result/b_ml_1.csv'
w_ml_2 = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/NN-result/w_ml_2.csv'
b_ml_2 = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/NN-result/b_ml_2.csv'
w_ol = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/NN-result/w_ol.csv'
b_ol = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/NN-result/b_ol.csv'
        
with open(w_ml_1, 'w') as f:
    writer = csv.writer(f,lineterminator='\n')
    writer.writerows(ml_1.w)
with open(b_ml_1, 'w') as f:
    writer = csv.writer(f,lineterminator='\n')
    writer.writerow(ml_1.b)
with open(w_ml_2, 'w') as f:
    writer = csv.writer(f,lineterminator='\n')
    writer.writerows(ml_2.w)
with open(b_ml_2, 'w') as f:
    writer = csv.writer(f,lineterminator='\n')
    writer.writerow(ml_2.b)
with open(w_ol, 'w') as f:
    writer = csv.writer(f,lineterminator='\n')
    writer.writerows(ol.w)
with open(b_ol, 'w') as f:
    writer = csv.writer(f,lineterminator='\n')
    writer.writerow(ol.b)



#正解率の計算
fp(input_train,False)
count_train = np.sum(np.argmax(ol.y,axis=1) == np.argmax(correct_train, axis=1))

fp(input_test,False)
count_test = np.sum(np.argmax(ol.y,axis=1) == np.argmax(correct_test, axis=1))

print('学習データでの正解数---{}/{}'.format(count_train,n_train))
print('テストデータでの正解数---{}/{}'.format(count_test,n_test))

print("Accuracy Train:" + str(float(count_train)/n_train*100) + "%",
      "Accuracy Test:" + str(float(count_test)/n_test*100) + "%")
