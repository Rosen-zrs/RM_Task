#include </usr/local/include/opencv4/opencv2/opencv.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "../Class/Armor/include/Armor.h"
#include "../Class/Light/include/Light.h"
#include "../Class/Light/include/Match_Condition.h"
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>
// #include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "math.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Armor_Shapes");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    //定义世界坐标和图像坐标
    vector<Point3d> World_Coor = {Point3f(0, 0, 0), Point3f(0, 26.5, 0), Point3f(67.5, 26.5, 0), Point3f(67.5, 0, 0)};

    //读取yml文件
    FileStorage fs2("/home/rosen/ROS/catkin_ws/src/Armor_Simulation/src/cam.yml", FileStorage::READ);
    Mat cameraMatrix2, distCoeffs2;
    fs2["camera_matrix"] >> cameraMatrix2;
    fs2["distortion_coefficients"] >> distCoeffs2;

    //定义Mat变量
    Mat frame, src, dst;

    //分离RGB通道输出Mat数组
    Mat c_frame[3];

    //定义目标打击点
    Point2f center;

    //导入视频
    VideoCapture capture(0);
    capture.open("/home/rosen/ROS/catkin_ws/src/Armor_Simulation/Video/water.avi");

    if (!capture.isOpened())
    {
        printf("could not find video data file...\n");
        return -1;
    }

    while (ros::ok() && capture.read(frame))
    {

        //分离图像RGB通道
        split(frame, c_frame);

        //红蓝通道相减
        src = c_frame[2] - c_frame[0];

        //设定阈值二值化
        threshold(src, src, 80, 255, THRESH_BINARY);

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

        bool flag = false;

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

                    Matching_Armor.push_back(Armor(left_light, right_light, center_dis_diff));
                }
            }
        }

        int min_index = 0;
        float min_dis, max_area;

        //定义旋转矩阵和平移矩阵
        Mat rvec, tvec, R;
        double q[4];

        if (Matching_Armor.size() >= 1)
        {
            for (int i = 0; i < Matching_Armor.size(); i++)
            {
                //改变标志位
                flag = true;

                //传入图像坐标
                vector<Point2d> Img_Coor;
                Img_Coor.push_back(Matching_Armor[i].bl());
                Img_Coor.push_back(Matching_Armor[i].tl());
                Img_Coor.push_back(Matching_Armor[i].tr());
                Img_Coor.push_back(Matching_Armor[i].br());

                //slovepnp姿态解算
                solvePnP(World_Coor, Img_Coor, cameraMatrix2, distCoeffs2, rvec, tvec);
                Rodrigues(rvec, R);

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

                float mean_area = (Matching_Armor[i].get_left_light().get_area() + Matching_Armor[i].get_right_light().get_area()) / 2;

                //记录灯条对最大面积
                if (i == 0)
                {
                    max_area = mean_area;
                }
                else
                {
                    if (mean_area > max_area)
                    {
                        max_area = mean_area;
                    }
                }
            }

            //当面积与最大面积相近时，以中心矩离优先匹配
            if (flag)
            {
                for (int i = 0; i < Matching_Armor.size(); i++)
                {
                    if (i == 0)
                    {
                        min_dis = Matching_Armor[i].get_center_dis();
                    }
                    //计算灯条平均面积
                    float mean_area = (Matching_Armor[i].get_left_light().get_area() + Matching_Armor[i].get_right_light().get_area()) / 2;

                    if (abs(mean_area - max_area) / max_area < 0.1)
                    {
                        //匹配最近灯条对
                        if (Matching_Armor[i].get_center_dis() < min_dis)
                        {
                            min_dis = Matching_Armor[i].get_center_dis();
                            min_index = i;
                        }
                    }
                }
            }
        }

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
            center = Point2f((aim_left_light.rect.center.x + aim_right_light.rect.center.x) / 2, (aim_left_light.rect.center.y + aim_right_light.rect.center.y) / 2);
            circle(frame, center, 3, Scalar(0, 0, 255), -1, 8, 0);
        }

        //创建Marker类
        visualization_msgs::Marker marker;

        // image_transport::ImageTransport it(n);
        // image_transport::Publisher pub = it.advertise("camera/image", 1);

        //设置frame_id
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "Armor_Shapes";
        marker.id = 0;

        //设置Marker形状
        marker.type = visualization_msgs::Marker::CUBE;

        marker.action = visualization_msgs::Marker::ADD;

        getQuaternion(R, q);

        //Marker的位置
        marker.pose.position.x = (tvec.at<_Float64>(0) - 10) / 100.0;
        marker.pose.position.y = (tvec.at<_Float64>(1) - 10) / 100.0;
        marker.pose.position.z = tvec.at<_Float64>(2) / 1000.0;

        //设置Marder的角度
        marker.pose.orientation.x = q[0];
        marker.pose.orientation.y = q[1];
        marker.pose.orientation.z = q[2];
        marker.pose.orientation.w = q[3];

        //Marker的颜色和透明度
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        //设置Marker的尺寸
        marker.scale.x = 0.675;
        marker.scale.y = 0.05;
        marker.scale.z = 0.265;

        //Marker被自动销毁之前的存活时间，rospy.Duration()意味着在程序结束之前一直存在
        marker.lifetime = ros::Duration();

        // sensor_msgs::ImagePtr msg;
        // msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

        while (marker_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }

        marker_pub.publish(marker);
        // pub.publish(msg);

        imshow("Origin", frame);

        if (waitKey(30) == 27)
        {
            if (waitKey(0) == 27)
            {
                break;
            }
        }
    }
}
