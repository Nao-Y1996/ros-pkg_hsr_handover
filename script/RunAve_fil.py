#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
import math

#時刻tの値を t±Range の範囲で平均を取っていく移動平均フィルター
#入力は Length×3 の二次元配列　3はx,y,z
#ただし端点で平均を取る範囲にデータがないところは、必要分だけ端点におけるデータで水増しする
#よって入力と出力のデータサイズは変わらない
def running_average_filter(Range,data,Length):
    for i in range(Range):
        data.insert(0,data[0])
        length = np.shape(data)[0]
        data.append(data[Length-1])
    running_average = []
    X = 0
    Y = 1
    Z = 2
    Xelement = [0]*(Range*2+1)
    Yelement = [0]*(Range*2+1)
    Zelement = [0]*(Range*2+1)
    for j in range(Length):
        for i in range(Range*2+1):
	    Xelement[i] = data[i+j][X] 
            Yelement[i] = data[i+j][Y]
            Zelement[i] = data[i+j][Z]
        x_ave = sum(Xelement)/(Range*2+1)
        y_ave = sum(Yelement)/(Range*2+1)
        z_ave = sum(Zelement)/(Range*2+1)
        running_average.append([x_ave,y_ave,z_ave])
    return running_average
