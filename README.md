# RM实习生任务

[TOC]

## 任务一

### 1.装甲板识别

#### 使用的类：
1. Light: 灯条类，将识别到的灯条信息用于储存,以便使用后续装甲板匹配需要用的特征
2. Armor: 装甲板类，通过灯条匹配后将合适的灯条两两组合成装甲板，并且储存装甲板的特征信息。
3. Armor_Link: 装甲板单链表，识别每帧采集到的装甲板，当达到一定的出现帧数(例如两帧)时，将装甲板添加进链表中，当消失达到一定帧数时，将装甲板从链表中删除。以此达到防止相近装甲板出现时，目标一直跳动的情况。(待完善)
4. RMVideoCapture: 哨兵使用的相机接口
5. Serialort: 串口类，通过定义头帧和尾帧串口输出给哨兵。

#### 击打策略：
  击打策略采用了两种模式，分别定义成宏（FEATURE_SCREEN 和 SOLVEPNP）
1. 特征筛选：通过对装甲板的特征进行筛选，选取面积最大（离得最近）的装甲板进行打击，当面积相似（差值在15%以内时），选取灯条对中心距离最近的为优先打击目标。
2. solvePnP:根据装甲板的实际尺寸创建世界坐标，再将识别到的装甲板的角点创建成图像坐标，分别传入solvePnP函数中，得到平移向量和旋转向量，然后选取平移向量中Z值最小的装甲板即为最近装甲板，作为打击目标。

#### 预测算法：
  预测算法使用两种方法，分别定义成宏（PREDICT 和 KALMAN）
1. 一阶梯度预测: 直接对相邻两帧目标点坐标作差，求得一阶微分后作为预测量加到前一帧坐标上作为预测点。
2. 卡尔曼滤波器预测: 使用opencv中的卡尔曼滤波器，将两帧之间的x差值和y差值作为输入求微分，预测出下一帧的差值加给上一帧的目标点进行预测。

#### 效果
  见文件夹“帧率预测”

### 2.能量开关识别

#### 使用的类：

1. Energy_Switch: 用于储存识别的能量开关的信息

#### 识别方法

1. 仿射变换: 通过对识别到的轮廓取最小外接矩形，调整矩形角度后，进行仿射变换，变换成固定大小（120 * 60），再与同样大小的模板进行模板匹配
2. 轮廓层次以及角点: 通过轮廓提取之后，选择子轮廓数为1的轮廓，并且通过多边形逼近后角点大于&等于6，即为打击目标

## 任务二

  与许婉婷配合完成SSH通信，实现获取对方的终端，进行文件复制

## 任务三

  与许婉婷配合串口通信，完成发送和接收“HelloWorld!"

## 任务五

  创建一个Marker，识别出视频中的装甲板后，使用solvePnP解算出平移向量和旋转向量，再将旋转向量转化成四元数后，储存在Marker中，发送给rviz

## 任务六

  实现了VGG-16的网络结构，由于电脑不支持GPU加速，找同学借的游戏本，所以有些不方便。跑的最好的参数是当Batch_Size = 256, learning_rate = 0.005时，准确率达到90%以上。
  阅读论文之后的总结写在“论文总结”中

## 任务七

  完成了OpenVINO推理视频




