#include "include/Energy_Switch.h"


Energy_Switch::Energy_Switch()
{
}

/**
 * @brief 能量开关构造函数，通过传入点集设置仿射变换矩形角点
 * 
 * @param p_rect 矩形点数组首元素指针
 * 
 */
Energy_Switch::Energy_Switch(Point2f *p_rect)
{
    this->height = distance(p_rect[0], p_rect[1]);
    this->width = distance(p_rect[1], p_rect[2]);
    if (this->height > this->width)
    {
        //如果高大于宽，则判断为纵向，此时交换高和宽，并调整仿射变换角点数组顺序
        swap_height_width();
        this->srcQuad[0] = p_rect[0];
        this->srcQuad[1] = p_rect[1];
        this->srcQuad[2] = p_rect[2];
        this->srcQuad[3] = p_rect[3];
    }
    else
    {
        this->srcQuad[0] = p_rect[1];
        this->srcQuad[1] = p_rect[2];
        this->srcQuad[2] = p_rect[3];
        this->srcQuad[3] = p_rect[0];
    }
    
    //设置仿射变换输出尺寸
    dstQuad[0] = Point2f(0, 0);
    dstQuad[1] = Point2f(120, 0);
    dstQuad[2] = Point2f(120, 60);
    dstQuad[3] = Point2f(0, 60);
}

/**
 * @brief 获得能量开关的高
 */
double Energy_Switch::get_height()
{
    return this->height;
}

/**
 * @brief 获得能量开关的宽
 */
double Energy_Switch::get_width()
{
    return this->width;
}

/**
 * @brief 交换能量开关的高宽(使其从纵向变为横向)
 * @return 无
 */
void Energy_Switch::swap_height_width()
{
    double n = this->height;
    this->height = this->width;
    this->width = n;
}

/**
 * @brief 获得能量开关面积
 */
double Energy_Switch::get_area()
{
    return this->area;
}

/**
 * @brief 计算两点之间的距离
 * 
 * @param p1 
 * @param p2
 * 
 * @return distance(double)
 */
double distance(Point2f p1, Point2f p2)
{
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}

/**
 * @brief 将传入的图片中识别到的轮廓旋转矩形进行仿射变换， 并输出为指定大小
 * 
 * @param src 二值化图像
 * @param es 识别出的疑似能量开关类
 * 
 * @return 返回指定大小的Mat图像作为后续模板匹配的模板
 */
Mat Affine(Mat src, Energy_Switch es)
{
    Mat dst, output;
    Mat warp_mat = getAffineTransform(es.srcQuad, es.dstQuad);
    warpAffine(src, dst, warp_mat, src.size());
    output = dst(Rect(0, 0, 120, 60));
    return output;
}

