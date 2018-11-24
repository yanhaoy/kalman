# Kalman
卡尔曼滤波的理论网上有很多，可以自己了解。

只是用aurco做个demo，识别到装甲片后的处理是一样的。

不清楚上实际云台会不会有自激振荡。

视觉上做的话控制矩阵是置零处理的，如果电控通过串口上发控制也不太好处理。当然可以尝试把卡尔曼滤波放到单片机上处理，视觉下发六轴信息。但是就要考虑控制量和视觉信息之间的时间差问题了。

滤波器的状态参数必须是六个自由度信息，不然不能假设相对静止，需要考虑云台运动状态（考虑极限状态下完全跟踪，图像点二位坐标是完全不变的）。

考虑输入阶跃信号的处理逻辑（比如初始化后第一次看见装甲片）。

考虑多个装甲片同时出现的预测、处理逻辑。

考虑切换追踪装甲片时的预测、处理逻辑。

Environment, python 3.7.1, opencv-contrib-python 3.4.2.17, opencv-python 3.4.2.17, numpy 1.15.4

。。。

![image](https://github.com/yyh2503/kalman/blob/master/example/1.gif)
![image](https://github.com/yyh2503/kalman/blob/master/example/2.gif)
![image](https://github.com/yyh2503/kalman/blob/master/example/3.gif)
