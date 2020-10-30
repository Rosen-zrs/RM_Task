#include "Class/Energy_Switch/include/Energy_Switch.h"
#include <iostream>

using namespace cv;
using namespace std;

// #define RED
#define BLUE
#define DEBUG
// #define PREDICT

int main(int argc, char **argv)
{
    //define images
    Mat frame, src, bin_src, Temp1, Temp2, dst, result, img;
    Mat c_frame[3];
    double max1, max2;

    //load Template from image
    Temp1 = imread("/home/rosen/桌面/Rosen/RM/ES_Detect/p1.jpg", 0);
    Temp2 = imread("/home/rosen/桌面/Rosen/RM/ES_Detect/p2.jpg", 0);

    //opencv vedio
    VideoCapture capture(0);
    capture.open("/home/rosen/桌面/Rosen/RM/ES_Detect/Video1.avi");
    if (!capture.isOpened())
    {
        printf("could not find video data file...\n");
        return -1;
    }

#ifdef DEBUG
    //create windows
    namedWindow("Origin", WINDOW_AUTOSIZE);
    namedWindow("Video", WINDOW_AUTOSIZE);
    moveWindow("Origin", 800, 90);
#endif // DEBUG

    //计算时间变量
    double start, end, dt;

    //储存中心点和上一帧中心点信息
    Point2f center, pre_center, predict_center;
    float dx, dy, Derivative, modulus;

    //定义相乘系数
    double factor = 600.0;

    while (capture.read(frame))
    {
        //计算程序运行时间
        start = static_cast<double>(getTickCount()); //获取开始执行时间
#ifdef PREDICT
        //计算程序运行时间
        start = static_cast<double>(getTickCount()); //获取开始执行时间
#endif                                               // PREDICT

        //split the image RGB channels
        split(frame, c_frame);

//sub channel ;
#ifdef RED
        // //red - blue
        src = c_frame[2] - c_frame[0];
#endif // RED

#ifdef BLUE
        //blue - red
        src = c_frame[0] - c_frame[2];
#endif // BLUE

        GaussianBlur(src, src, Size(5, 5), 0);

        //二值化
        threshold(src, bin_src, 50, 255, THRESH_BINARY);

        //定义结构元素，进行闭操作
        Mat element = getStructuringElement(MORPH_RECT, Size(8, 8), Point(-1, -1));
        dilate(bin_src, bin_src, element);
        morphologyEx(bin_src, bin_src, MORPH_CLOSE, element);

        // 查找轮廓
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(bin_src, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());

        Moments mu, moment;
        Point2f mc, mark_point;

        int min_index;
        bool flag = false;

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
            circle(frame, mc, 3, Scalar(255, 255, 255), -1, 8, 0);
            circle(frame, mc, 25, Scalar(255, 255, 255), 1, 8, 0);
        }

        center = mc;
        if(distance(center, pre_center) < 1500)

#ifdef PREDICT
        
        //求导进行预测
        dx = center.x - pre_center.x;
        dy = center.y - pre_center.y;

        //计算运行时间
        end = static_cast<double>(getTickCount());
        dt = (end - start) / getTickFrequency();

        //计算出预测中心点坐标
        predict_center = Point2f(center.x + factor * dx * dt, center.y + factor * dy * dt);
        circle(frame, predict_center, 4, Scalar(0, 0, 255), -1, 8, 0);

#endif // PREDICT

        pre_center = center;

#ifdef DEBUG
        //显示图像
        imshow("Origin", frame);
        // imshow("Video", dst);
#endif // DEBUG

        if (waitKey(30) == 27)
        {
            if (waitKey(0) == 27)
            {
                break;
            }
        }

        //计算运行时间
        end = static_cast<double>(getTickCount());
        dt = (end - start) / getTickFrequency() * 1000;
        cout << dt << endl;
    }

    capture.release();

    return 0;
}
