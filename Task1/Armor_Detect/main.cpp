#include "Class/Armor/include/Armor.h"
#include "Class/Light/include/Light.h"
#include "Class/Light/include/Match_Condition.h"
#include "Class/SerialPort/include/SerialPort.h"
#include "Class/RMVideoCapture/include/RMVideoCapture.h"
#include "Class/Armor_Link/include/Armor_Link.h"
#include <iostream>
#include <math.h>
#include <time.h>

using namespace cv;
using namespace std;

#define DEBUG

#define BLUE
#define CLAC_TIME
#define ARMOR_DETECT
// #define Serial

//选择识别装甲板模式 --- solvePnP()姿态解算 --- 特征筛选 ---
// #define SOLVEPNP
#define FEATURE_SCREEN

// 预测模式 -- 一阶梯度预测 -- 卡尔曼预测 --
// #define PREDICT
#define KALMAN

#ifdef Serial
#pragma pack(1)
struct DataStruct
{
    uint8_t Flag; //决定策略
    short X;      //装甲板坐标与像素中心的坐标差值，相机分辨率：（640，480）
    short Y;      //
    short Z;      //置零
    uint8_t end;  //每次发送都要变，最大255
};
#pragma pack()
#endif // Serial

static inline Point calcPoint(Point2f center, double R, double angle)
{
    return center + Point2f((float)cos(angle), (float)-sin(angle)) * (float)R;
}

int main(int argc, char **argv)
{

    //导入视频
    VideoCapture capture(0);
    capture.open("/home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/Video/Armor1.mp4");

    if (!capture.isOpened())
    {
        printf("could not find video data file...\n");
        return -1;
    }

    // Ptr<VideoCapture> capture;
    // capture = (new RMVideoCapture());
    // capture->set(CAP_PROP_EXPOSURE, 400); //设置曝光//

#ifdef DEBUG
    //创建窗口
    namedWindow("Origin", WINDOW_NORMAL);
    resizeWindow("Origin", 640, 480);

    // namedWindow("Video", WINDOW_NORMAL);
    // resizeWindow("Video", Size(640, 480));
    // moveWindow("Origin", 800, 90);
#endif // DEBUG

    //定义Mat变量
    Mat frame, src, bin_src, Temp1, Temp2, dst, result;

    //分离RGB通道输出Mat数组
    Mat c_frame[3];

    //定义目标打击点
    Point2f center;

    //定义世界坐标和图像坐标
    vector<Point3d> World_Coor = {Point3f(0, 0, 0), Point3f(0, 26.5, 0), Point3f(67.5, 26.5, 0), Point3f(67.5, 0, 0)};

    //读取yml文件
    FileStorage fs2("/home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/cam/cam.yml", FileStorage::READ);
    Mat cameraMatrix2, distCoeffs2;
    fs2["camera_matrix"] >> cameraMatrix2;
    fs2["distortion_coefficients"] >> distCoeffs2;

#ifdef Serial
    //定义串口通信类
    DataStruct data;
    SerialPort port;
    data.end = 0x00;
#endif // Serial

    //储存中心点和上一帧中心点信息
    Point2f pre_center(0, 0), predict_center;

    //定义装甲板单链表
    Armor_Link armor_link;

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

#ifdef CLAC_TIME
    //计算时间变量
    double start_time, end_time, time;
#endif // CLAC_TIME

#ifdef PREDICT
    float dx, dy;

    //计算时间变量
    double start, end, dt;

    //定义相乘系数
    double factor = 500.0;
#endif // PREDICT

    while (capture.read(frame))
    {
        //定义输出目标点标志位
        bool flag = false;

#ifdef CLAC_TIME
        //计算程序运行时间
        start_time = static_cast<double>(getTickCount()); //获取开始执行时间
#endif                                                    // CLAC_TIME

#ifdef PREDICT
        //计算程序运行时间
        start = static_cast<double>(getTickCount()); //获取开始执行时间
#endif

        //分离图像RGB通道
        split(frame, c_frame);

#ifdef BLUE
        //蓝色通道 - 红色通道
        src = c_frame[0] - c_frame[2];
#else
        //红色通道 - 蓝色通道
        src = c_frame[2] - c_frame[0];
#endif // BLUE

#ifdef ARMOR_DETECT

        //设定阈值二值化
        threshold(src, src, 60, 255, THRESH_BINARY);

        //膨胀
        Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
        dilate(src, src, element);

        //提取轮廓
        vector<vector<Point>> contours;
        findContours(src, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());

        //定义外接拟合矩形
        RotatedRect rect;
        Point2f vertices[4]; //定义矩形的4个顶点

        //定义动态灯条数组
        vector<Light> lights;

        //定义匹配条件结构体
        Match_Condition MATCH_COND;

        for (size_t i = 0; i < contours.size(); i++)
        {
            //筛除小轮廓(斑点)
            if (contourArea(contours[i]) < 80)
            {
                continue;
            }

            if (contours[i].size() > 5)
            {
                //椭圆拟合
                rect = fitEllipse(contours[i]);

                //调整矩形角度和宽高
                tranform_rect(rect);

                rect.points(vertices);

                float contour_area = contourArea(contours[i]);

                if (rect.size.width / rect.size.height > MATCH_COND.MAX_WH_RATIO || contour_area / rect.size.area() < MATCH_COND.MIN_AREA_FULL)
                    continue;

                // //太远的装甲板不选择打击
                // if ( rect.size.height / rect.size.width < 1.8)
                //     continue;

                //绘制矩形
                for (int i = 0; i < 4; i++)
                {
                    line(frame, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 1);
                }

                //配置灯条成员信息
                Light light = Light(rect, contourArea(contours[i]), arcLength(contours[i], true));
                light.rect = rect;
                lights.push_back(light);
            }
        }

        vector<Armor> Matching_Armor;

        //定义公灯条矩形的4个顶点
        Point2f left_light_vertices[4];
        //定义母灯条的4个顶点
        Point2f right_light_vertices[4];

        //匹配灯条
        for (auto left_light : lights)
        {
            for (auto right_light : lights)
            {
                if (left_light.get_center().x < right_light.get_center().x)
                {
                    //计算灯条特征逐步筛选

                    //面积差值
                    float area_ratio_diff = abs(left_light.get_area() - right_light.get_area());
                    if (area_ratio_diff > left_light.get_area() * MATCH_COND.MAX_AREA_DIFF && area_ratio_diff > right_light.get_area() * MATCH_COND.MAX_AREA_DIFF)
                        continue;

                    //宽度和高度差值
                    float height_diff = abs(left_light.get_height() - right_light.get_height());
                    if (height_diff > left_light.get_height() * MATCH_COND.MAX_HEIGHT_DIFF && height_diff > right_light.get_height() * MATCH_COND.MAX_HEIGHT_DIFF)
                        continue;

                    //角度差值
                    float angle_diff = abs(left_light.get_angle() - right_light.get_angle());
                    if (angle_diff > MATCH_COND.MAX_ANGLE_DIFF)
                        continue;

                    //中心距离差值
                    float center_dis_diff = distance(left_light.get_center(), right_light.get_center());
                    float center_y_diff = abs(left_light.get_center().y - right_light.get_center().y);
                    float center_x_diff = abs(left_light.get_center().x - right_light.get_center().x);

                    //根据中心距筛除
                    if (center_dis_diff < 3.5 * left_light.get_height() && center_y_diff < left_light.get_height() && center_x_diff > left_light.get_width())
                    {
                        Matching_Armor.push_back(Armor(left_light, right_light, center_dis_diff));
                    }
                }
            }
        }

        int min_index = 0;
        float min_dis, max_area;

#ifdef SOLVEPNP
        if (Matching_Armor.size() >= 1)
        {
            for (size_t i = 0; i < Matching_Armor.size(); i++)
            {
                //改变标志位
                flag = true;

                Mat rvec, tvec, R;

                //传入图像坐标
                vector<Point2d> Img_Coor;
                Img_Coor.push_back(Matching_Armor[i].bl());
                Img_Coor.push_back(Matching_Armor[i].tl());
                Img_Coor.push_back(Matching_Armor[i].tr());
                Img_Coor.push_back(Matching_Armor[i].br());

                //slovepnp姿态解算
                solvePnP(World_Coor, Img_Coor, cameraMatrix2, distCoeffs2, rvec, tvec);
                // Rodrigues(rvec, R);

                //挑选计算出距离最近的装甲板
                double Z = tvec.at<double>(2);

                //记录最近装甲板下标
                if (i == 0)
                {
                    min_dis = Z;
                    min_index = i;
                }
                if (Z < min_dis)
                {
                    min_dis = Z;
                    min_index = i;
                }
            }
        }

#endif // SOLVEPNP

        Armor_node *p_cursor;

#ifdef FEATURE_SCREEN
        if (Matching_Armor.size() >= 1)
        {
            for (size_t i = 0; i < Matching_Armor.size(); i++)
            {
                //改变标志位
                flag = true;

                bool is_arised = false;

                Matching_Armor[i].arise_frame++;
                Matching_Armor[i].vanish_frame = 0;

                p_cursor = armor_link.get_head();
                Armor *p = &Armor(&Matching_Armor[i]);

                while (p_cursor->next != NULL)
                {
                    p_cursor->get_next_p_armor()->is_vanished = true;
                    if (distance(Matching_Armor[i].get_center(), p_cursor->get_next_p_armor()->get_center()) < Matching_Armor[i].get_width())
                    {
                        p_cursor->get_next_p_armor()->is_vanished = false;
                        p_cursor->next->p_armor = &Matching_Armor[i];
                    }
                    else
                    {
                        if (Matching_Armor[i].arise_frame >= 2)
                        {
                            armor_link.Add_Armor(p);
                        }
                    }

                    p_cursor = p_cursor->next;
                }

                // float mean_area = (Matching_Armor[i].get_left_light().get_area() + Matching_Armor[i].get_right_light().get_area()) / 2;

                // //记录灯条对最大面积
                // if (i == 0)
                // {
                //     max_area = mean_area;
                // }
                // else
                // {
                //     if (mean_area > max_area)
                //     {
                //         max_area = mean_area;
                //     }
                // }

                // //当面积与最大面积相近时，以中心矩离优先匹配
                // if (flag)
                // {
                //     for (size_t j = 0; j < Matching_Armor.size(); j++)
                //     {
                //         if (j == 0)
                //         {
                //             min_dis = Matching_Armor[j].get_center_dis();
                //         }
                //         //计算灯条平均面积
                //         float mean_area = (Matching_Armor[j].get_left_light().get_area() + Matching_Armor[j].get_right_light().get_area()) / 2;

                //         if (abs(mean_area - max_area) / max_area < 0.15)
                //         {
                //             //匹配最近灯条对
                //             if (Matching_Armor[j].get_center_dis() < min_dis)
                //             {
                //                 min_dis = Matching_Armor[j].get_center_dis();
                //                 min_index = j;
                //             }
                //         }
                //     }
                // }
            }
        }

        p_cursor = armor_link.get_head();
        while (p_cursor->next != NULL)
        {
            if (p_cursor->get_next_p_armor()->is_vanished)
            {
                p_cursor->get_next_p_armor()->vanish_frame++;
                if (p_cursor->get_next_p_armor()->vanish_frame >= 4)
                {
                    armor_link.Del_Armor(p_cursor->get_next_p_armor()->identifier);
                }
            }
        }

#endif // FEATURE_SCREEN

        if (flag)
        {
            //取出目标装甲板及其灯条
            Light aim_left_light, aim_right_light;
            aim_left_light = Matching_Armor[min_index].get_left_light();
            aim_right_light = Matching_Armor[min_index].get_right_light();

            aim_left_light.rect.points(left_light_vertices);
            aim_right_light.rect.points(right_light_vertices);

            //绘制矩形
            for (int i = 0; i < 4; i++)
                line(frame, left_light_vertices[i], left_light_vertices[(i + 1) % 4], Scalar(0, 0, 255), 2, 8, 0);
            for (int i = 0; i < 4; i++)
                line(frame, right_light_vertices[i], right_light_vertices[(i + 1) % 4], Scalar(0, 0, 255), 2, 8, 0);

            //标定目标矩形
            line(frame, aim_left_light.rect.center, aim_right_light.rect.center, Scalar(255, 255, 255), 2, 8, 0);
            center = Matching_Armor[min_index].get_center();
            circle(frame, center, 7, Scalar(0, 0, 255), -1, 8, 0);
        }

#endif // ARMOR_DETECT

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
            cout << "cor:  " << cor.at<float>(0) << '\t' << cor.at<float>(1) << endl;
            // cout << "prediction:  " << prediction.at<float>(0) << '\t' << prediction.at<float>(1) << endl;

            predict_center = Point2f(center.x + cor.at<float>(0), center.y + cor.at<float>(1));

            pre_center = center;

            circle(frame, predict_center, 7, Scalar(0, 255, 0), -1, 8, 0);
        }

#endif // Kalman

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

#ifdef Serial
        if (Matching_Armor.size() >= 1)
        {
            data.Flag = 137;
            data.X = predict_center.x - 320;
            data.Y = predict_center.y - 240;
            data.Z = 0;
        }
        else
        {
            data.Flag = 137;
            data.X = 0;
            data.Y = 0;
            data.Z = 0;
        }

        port.writeStruct<DataStruct>(data);
        //改变尾帧
        data.end += 1;
        if (data.end >= 255)
        {
            data.end = 0;
        }

#endif // Serial

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
}
