#ifndef __ENERGY_SWITCH__
#define __ENERGY_SWITCH__

#include <opencv2/opencv.hpp>
#include <math.h>

using namespace cv;

class Energy_Switch
{
public:
    Energy_Switch();
    Energy_Switch(Point2f *p_rect);

    //获得能量开关的高和宽
    double get_height();
    double get_width();

    //交换宽和高
    void swap_height_width();

    //获得能量开关面积
    double get_area();

    //定义仿射变换所需点集
    Point2f srcQuad[4], dstQuad[4];

private:
    //能量开关的高和宽
    double height, width;
    //能量开关面积
    double area;
};

Mat Affine(Mat src, Energy_Switch es);

double distance(Point2f p1, Point2f p2);

#endif // !__ENERGY_SWITCH__