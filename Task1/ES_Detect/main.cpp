#include "Class/Energy_Switch/include/Energy_Switch.h"
#include <iostream>

using namespace cv;
using namespace std;

#define BLUE
#define DEBUG
#define CLAC_TIME
#define AFFINE

// 预测模式 -- 一阶梯度预测 -- 卡尔曼预测 --
#define PREDICT
// #define KALMAN


int main(int argc, char **argv)
{

    //define images
    Mat frame, src, bin_src, Temp1, Temp2, dst, result;
    Mat c_frame[3];

#ifdef AFFINE

    //能量开关模板匹配度
    double max1, max2;

    //load Template from image
    Temp1 = imread("/home/rosen/桌面/Rosen/RM实习任务/Task1/ES_Detect/Temp/p1.jpg", 0);
    Temp2 = imread("/home/rosen/桌面/Rosen/RM实习任务/Task1/ES_Detect/Temp/p2.jpg", 0);

#endif // AFFINE

    //导入视频
    VideoCapture capture(0);
    capture.open("/home/rosen/桌面/Rosen/RM实习任务/Task1/ES_Detect/Video/能量机关 (1).avi");
    if (!capture.isOpened())
    {
        printf("could not find video data file...\n");
        return -1;
    }

#ifdef DEBUG
    //创建窗口
    namedWindow("Origin", WINDOW_NORMAL);
    resizeWindow("Origin", 640, 480);

    // namedWindow("Video", WINDOW_NORMAL);
    // resizeWindow("Video", Size(640, 480));
    // moveWindow("Origin", 800, 90);
#endif // DEBUG

    //储存中心点和上一帧中心点信息
    Point2f center, pre_center, predict_center;

#ifdef PREDICT

    float dx, dy;

    //计算时间变量
    double start, end, dt;

    //定义相乘系数
    double factor = 500.0;

#endif // PREDICT

#ifdef CLAC_TIME
    //计算时间变量
    double start_time, end_time, time;
#endif // CLAC_TIME

#ifdef KALMAN
    const int stateNum = 4;   //状态值4×1向量(x,y,△x,△y)
    const int measureNum = 2; //测量值2×1向量(x,y)
    KalmanFilter KF(stateNum, measureNum, 0);
    KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1); //转移矩阵A
    setIdentity(KF.measurementMatrix);                                                           //测量矩阵H
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                                          //系统噪声方差矩阵Q
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                                      //测量噪声方差矩阵R
    setIdentity(KF.errorCovPost, Scalar::all(1));                                                //后验错误估计协方差矩阵P
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));
    Mat measurement = Mat::zeros(2, 1, CV_32F); //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义
#endif                                          // Kalman

    while (capture.read(frame))
    {

#ifdef CLAC_TIME
        //计算程序运行时间
        start_time = static_cast<double>(getTickCount()); //获取开始执行时间
#endif                                                    // CLAC_TIME

        //定义发现目标标志位
        bool flag = false;

#ifdef PREDICT
        //计算程序运行时间
        start = static_cast<double>(getTickCount()); //获取开始执行时间
#endif                                               // PREDICT

        //split the image RGB channels
        split(frame, c_frame);

        //sub channel ;

#ifdef BLUE
        //blue - red
        src = c_frame[0] - c_frame[2];
#else
        //red - blue
        src = c_frame[2] - c_frame[0];
#endif // BLUE

        GaussianBlur(src, src, Size(5, 5), 0);

        //二值化
        threshold(src, bin_src, 50, 255, THRESH_BINARY);

        //定义结构元素，进行闭操作
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
        dilate(bin_src, bin_src, element);
        morphologyEx(bin_src, bin_src, MORPH_CLOSE, element);

        // 查找轮廓
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(bin_src, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());

#ifdef AFFINE
        //定义矩形的4个顶点
        Point2f vertices[4];
        Moments mu;
        Point2f mc;

        Energy_Switch es;

        for (size_t i = 0; i < contours.size(); i++)
        {
            if (contourArea(contours[i]) > 800)
            {
                RotatedRect rect = minAreaRect(contours[i]);
                //获得旋转矩形的四个角点
                rect.points(vertices);

                //创建能量开关类，并获得仿射变换矩形角点
                es = Energy_Switch(vertices);

                //仿射变换
                dst = Affine(bin_src, es);

                //分别进行模板匹配
                matchTemplate(dst, Temp1, result, TM_CCOEFF_NORMED, Mat()); //模板匹配
                minMaxLoc(result, 0, &max1, 0, 0, Mat());                   //定位最匹配的位置
                matchTemplate(dst, Temp2, result, TM_CCOEFF_NORMED, Mat()); //模板匹配
                minMaxLoc(result, 0, &max2, 0, 0, Mat());                   //定位最匹配的位置

                if (max1 >= 0.65 || max2 >= 0.66)
                {
                    if (hierarchy[i][2] != -1)
                    {
                        flag = true;

                        //计算拓扑结构中子轮廓中心点
                        mu = moments(contours[hierarchy[i][2]], false);
                        mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

                        center = mc;

                        //标定目标点
                        circle(frame, center, 3, Scalar(255, 255, 255), -1, 8, 0);
                        circle(frame, center, 27, Scalar(255, 255, 255), 1, 8, 0);
                    }

                    // // draw retangle of the aim energy_switch
                    // for (int i = 0; i < 4; i++)
                    // line(frame, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 1);
                }
            }
        }

#ifdef PREDICT
        if (distance(center, pre_center) > es.get_width() / 2.0)
        {
            flag = false;
        }
#endif // PREDICT

#else

        Moments mu, moment;
        Point2f mc, mark_point;

        int min_index;

        vector<vector<Point>> contours_ploy(contours.size());

        double min_area;

        int inside_contour_num[contours.size()] = {0};

        for (size_t i = 0; i < contours.size(); i++)
        {
            if (hierarchy[i][3] != -1)
            {
                inside_contour_num[hierarchy[i][3]]++;
            }
        }

        for (size_t i = 0; i < contours.size(); i++)
        {
            approxPolyDP(Mat(contours[i]), contours_ploy[i], 6, true);

            if (inside_contour_num[i] == 1)
            {
                if (contours_ploy[i].size() >= 6 && hierarchy[i][2] != -1 && hierarchy[i][3] == -1)
                {
                    if (!flag)
                        min_area = contourArea(contours[i]);
                    double area = contourArea(contours[i]);

                    if (area <= min_area)
                    {
                        flag = true;
                        min_index = i;
                        min_area = area;
                    }
                }
            }
        }
        if (flag)
        {
            mu = moments(contours[hierarchy[min_index][2]], false);
            mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

            center = mc;

            circle(frame, mc, 3, Scalar(255, 255, 255), -1, 8, 0);
            circle(frame, mc, 25, Scalar(255, 255, 255), 1, 8, 0);
        }

#endif // AFFINE

#ifdef PREDICT

        if (flag)
        {
            //求导进行预测
            dx = center.x - pre_center.x;
            dy = center.y - pre_center.y;

            //计算运行时间
            end = static_cast<double>(getTickCount());
            dt = (end - start) / getTickFrequency();

            //计算出预测中心点坐标
            predict_center = Point2f(center.x + factor * dx * dt, center.y + factor * dy * dt);
            circle(frame, predict_center, 7, Scalar(0, 255, 0), -1, 8, 0);
        }

        pre_center = center;
#endif // PREDICT

#ifdef KALMAN
        if (flag)
        {
            //2.kalman prediction
            Mat prediction = KF.predict();
            // Point2f predict_pt = Point2f(prediction.at<float>(0), prediction.at<float>(1)); //预测值(x',y')

            //3.update measurement
            measurement.at<float>(0) = center.x - pre_center.x;
            measurement.at<float>(1) = center.y - pre_center.y;

            //4.update
            Mat cor = KF.correct(measurement);

            // cout << "mea:  " << measurement.at<float>(0) << '\t' << measurement.at<float>(1) << endl;
            // cout << "cor:  " << cor.at<float>(0) << '\t' << cor.at<float>(1) << endl;
            // cout << "prediction:  " << prediction.at<float>(0) << '\t' << prediction.at<float>(1) << endl;

            predict_center = Point2f(center.x + 2 * cor.at<float>(0), center.y + 2 * cor.at<float>(1));

            pre_center = center;

            circle(frame, predict_center, 7, Scalar(0, 255, 0), -1, 8, 0);
        }

#endif // KALMAN

#ifdef DEBUG
        //显示图像
        imshow("Origin", frame);
        if (waitKey(10) == 27)
        {
            if (waitKey(0) == 27)
            {
                break;
            }
        }
#endif // DEBUG

#ifdef CLAC_TIME
        //计算运行时间
        end_time = static_cast<double>(getTickCount());
        time = (end_time - start_time) / getTickFrequency() * 1000;
        cout << "每帧运行时间为：" << time << " ms" << endl;
#endif // CLAC_TIME
    }

    capture.release();

    return 0;
}
