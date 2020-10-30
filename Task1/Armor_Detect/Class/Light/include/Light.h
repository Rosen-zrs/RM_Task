#ifndef __LIGHT_
#define __LIGHT_

#include </usr/local/include/opencv4/opencv2/opencv.hpp>

using namespace cv;

class Light
{
public:
    Light();
    Light(RotatedRect rect, float area, float perimeter);

    RotatedRect rect;

    float get_height();
    float get_width();
    float get_angle();
    float get_area();

    //设置面积
    void set_Area(float area);
    //设置角度
    void set_angle(float angle);

    //获取中心点
    Point2f get_center();

    //获取中心点
    void set_center(float x, float y);

    //重载！= 符号
    friend bool operator!=(const Light &t1, const Light &t2);

private:
    //定义灯条的长和宽
    float height, width;
    
    //定义灯条的面积
    float area;

    //定义灯条的角度
    float angle;

    //定义中心点
    Point2f center;
};

#endif // ! __LIGHT_
