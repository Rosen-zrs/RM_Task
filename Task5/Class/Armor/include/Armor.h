#ifndef __ARMOR_
#define __ARMOR_

#include "../../Light/include/Light.h"
#include </usr/local/include/opencv4/opencv2/opencv.hpp>
#include "math.h"

class Armor
{
public:
    Armor(Light &male_light, Light &female_light, float center_dis);

    float get_height();
    float get_width();
    float get_center_dis();
    float get_area();

    Light get_left_light();
    Light get_right_light();

    Point2f bl();
    Point2f tl();
    Point2f tr();
    Point2f br();

private:
    //定义装甲板的长和宽
    float height, width;
    //定义灯条对
    Light lights[2];
    //灯条中心距离差
    float center_dis;
    //定义装甲板面积
    float Area;
    //装甲板矩形角点
    Point2f p_tl, p_tr, p_bl, p_br;
};

RotatedRect tranform_rect(RotatedRect &rect);

double distance(Point2f p1, Point2f p2);

void getQuaternion(Mat R, double Q[]);

#endif // ! __ARMOR_
