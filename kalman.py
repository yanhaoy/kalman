# 卡尔曼滤波预测

import cv2 as cv
import numpy as np


class CustomKalmanFilter(cv.KalmanFilter):
    # 卡尔曼滤波器初始化
    # 状态变量，观测变量，测量协方差，预测协方差
    def __init__(self, _status_num, _measurements_num, _mnoise, _pnoise):
        super(CustomKalmanFilter, self).__init__(_status_num, _measurements_num)
        # 测量矩阵
        self.measurementMatrix = np.concatenate((np.eye(6, dtype=np.float32), np.zeros((6, 12), dtype=np.float32)),
                                                axis=-1)
        # 转移矩阵
        self.transitionMatrix = np.eye(18, k=0, dtype=np.float32) + np.eye(18, k=6, dtype=np.float32) + np.eye(18, k=12,
                                                                                                               dtype=np.float32)
        # 测量协方差，这个可以调的
        self.measurementNoiseCov = np.eye(6, dtype=np.float32) * _mnoise
        # 预测协方差，这个可以调的
        self.processNoiseCov = np.eye(18, dtype=np.float32) * _pnoise
        # 融合协方差，初始化时要全设为最大，不然一会用加速度会振荡
        self.errorCovPost = np.eye(18, dtype=np.float32)
        # 重置标志位，防止阶跃信号输入而预测跳变
        self.reset_flag = True

    def reset_error(self):
        kalman.errorCovPost = np.eye(18, dtype=np.float32)
        self.reset_flag = True

    def reset_state(self):
        kalman.statePost = np.zeros((18, 1), dtype=np.float32)
        kalman.statePost[0:6] = current_measurement
        self.reset_flag = False


# 画二维码并输出单个、两个图片的函数
def make_mark(_aruco_dict, _id, _pixel, _border):
    marker = cv.aruco.drawMarker(_aruco_dict, _id, _pixel)
    # 外面最好有白框不然识别不了，这是aruco识别函数自己的问题
    marker = cv.copyMakeBorder(marker, _border, _border, _border, _border, cv.BORDER_CONSTANT, value=255)
    cv.imwrite('marker.png', marker)
    muilti_marker = np.ones(shape=[marker.shape[0], marker.shape[1] * 2 + 50]) * 255
    muilti_marker[:, 0:marker.shape[1]] = marker
    muilti_marker[:, marker.shape[1] + 50:marker.shape[1] * 2 + 50] = marker
    cv.imwrite('muilti_marker.png', muilti_marker)


# 读取相机内参和畸变矩阵
def read_cam_para():
    fs = cv.FileStorage('camera_paraments.yaml', cv.FileStorage_READ)
    # 内参
    _mtx = np.array(fs.getNode('mtx').mat())
    # 畸变
    _dist = np.array(fs.getNode('dist').mat())
    fs.release()
    return _mtx, _dist


# 画目标的三个坐标轴
def draw(_img, _corners, _imgpts):
    _corners = tuple(_corners[0].ravel())
    _img = cv.line(_img, _corners, tuple(_imgpts[0].ravel()), (255, 0, 0), 5)
    _img = cv.line(_img, _corners, tuple(_imgpts[1].ravel()), (0, 255, 0), 5)
    _img = cv.line(_img, _corners, tuple(_imgpts[2].ravel()), (0, 0, 255), 5)
    return _img


def euclidean_distances(A, B):
    # https://blog.csdn.net/wanghai00/article/details/54021200
    BT = B.transpose()
    vecProd = np.dot(A, BT)

    SqA = A ** 2
    sumSqA = np.matrix(np.sum(SqA, axis=1))
    sumSqAEx = np.tile(sumSqA.transpose(), (1, vecProd.shape[1]))

    SqB = B ** 2
    sumSqB = np.sum(SqB, axis=1)
    sumSqBEx = np.tile(sumSqB, (vecProd.shape[0], 1))

    SqED = sumSqBEx + sumSqAEx - 2 * vecProd
    SqED[SqED < 0] = 0.0
    ED = np.sqrt(SqED)
    return ED


# 获取真实尺寸点
def get_objpoints(_num, _lenth):
    _objp = np.zeros((_num * _num, 3), np.float32)
    _objp[:, :2] = np.mgrid[0:2, 0:2].T.reshape(-1, 2) * _lenth

    _axisp = np.array([[0, 0, 0],
                       [_lenth, 0, 0],
                       [0, _lenth, 0],
                       [0, 0, _lenth]], dtype=np.float32)

    return _objp, _axisp


# 生成aruco二维码的字典
aruco_dict = cv.aruco.getPredefinedDictionary(1)

make_mark(aruco_dict, 5, 200, 50)
mtx, dist = read_cam_para()
objp, axisp = get_objpoints(2, 38)
kalman = CustomKalmanFilter(18, 6, 1e-10, 1e-5)

cap = cv.VideoCapture(2)
fourcc = cv.VideoWriter_fourcc(*'XVID')
out = cv.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

while True:
    # 读摄像头
    ret, frame = cap.read()
    h, w = frame.shape[:2]
    # 因为做了畸变纠正，以后要用新的相机内参矩阵
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0, (w, h))
    # 畸变纠正
    frame = cv.undistort(frame, mtx, dist, None, newcameramtx)
    # 灰度处理快
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # 检测aruco二维码
    corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, aruco_dict, cameraMatrix=newcameramtx)

    # 如果检测到
    if corners:
        # 画出来
        cv.aruco.drawDetectedMarkers(frame, corners, ids)
        # 按每个二维码分开
        corners = np.array(corners).reshape(-1, 4, 2)
        # 测量矩阵的存储容器
        measurement = np.zeros((corners.shape[0], 6), dtype=np.float32)
        for i, corner in enumerate(corners):
            corner = np.squeeze(np.array(corner))
            # 检测的点顺序是左上 右上 右下 左下 所以调换一下
            corner_pnp = np.array([corner[0], corner[1], corner[3], corner[2]])
            # solvePNP获取r,t矩阵
            retval, rvec, tvec = cv.solvePnP(objp, corner_pnp, newcameramtx, None)
            if retval:
                measurement[i] = np.concatenate((rvec, tvec), axis=0).reshape(6).astype(np.float32)
            else:
                # 如果不能求解就删掉
                measurement = np.delete(measurement, i, axis=0)

        # 如果有求解结果
        if measurement.size > 0:
            if not kalman.reset_flag:
                # 和上一帧预测结果距离判断
                dis = euclidean_distances(tvec_p.reshape(1, 3), measurement[:, 3:6].reshape(-1, 3))
                dis = np.array(dis).reshape(-1)
                min_arg = np.argmin(dis)

                # 若最近的检测点都太远 就直接reset 这个度量可以调整 大概和真实尺寸有关吧
                if dis[min_arg] > 38:
                    kalman.reset_error()

                current_measurement = measurement[min_arg].reshape(6, 1)
            else:
                # 若上一帧已经执行过reset 即没有预测点存储 就选一个最近的
                dis = measurement[:, 3:6].reshape(-1, 3) ** 2
                dis = np.sum(dis, axis=1).reshape(-1)
                min_arg = np.argmin(dis)

                current_measurement = measurement[min_arg].reshape(6, 1)

            # 用当前测量来校正卡尔曼滤波器
            correct_retval = kalman.correct(current_measurement)
            # 重置 将位置状态重置为测量 速度加速度重置为0 此时预测等于测量
            if kalman.reset_flag:
                kalman.reset_state()
            # 计算卡尔曼预测值，作为当前预测
            current_prediction = kalman.predict()
            rvec_p, tvec_p = current_prediction[0:3], current_prediction[3:6]
            # 重新映射回图像
            imagePoints_p, _ = cv.projectPoints(axisp, rvec_p, tvec_p, newcameramtx, None)
            # 画坐标轴
            frame = draw(frame, imagePoints_p[0], imagePoints_p[1:4])
        else:
            # 找不到可行解 重置
            kalman.reset_error()
    else:
        # 找不到二维码 重置
        kalman.reset_error()

    cv.imshow('img', frame)
    out.write(frame)

    key = cv.waitKey(1)
    if key & 0xFF == ord('q'):
        break

# 清理设备
cap.release()
out.release()
cv.destroyAllWindows()
