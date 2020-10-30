#ifndef __ARMOR_
#define __ARMOR_

#include "../../Light/include/Light.h"
#include </usr/local/include/opencv4/opencv2/opencv.hpp>
#include "math.h"

class Armor
{
public:
    Armor();
    Armor(Light &left_light, Light &right_light, float center_dis);
    Armor(Armor* p);

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
    Point2f get_center();

    //出现帧数
    int arise_frame;
    //消失帧数
    int vanish_frame;

    bool is_vanished;
    int identifier = 0;

    const Armor& operator=(const Armor& a1);

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
    Point2f tl_point, tr_point, bl_point, br_point, center_point;
};

typedef Armor* p_Armor;

RotatedRect tranform_rect(RotatedRect &rect);

double distance(Point2f p1, Point2f p2);



#endif // ! __ARMOR_
