#include "include/Light.h"

//灯条构造函数
Light::Light() {}

/**
 * @brief 灯条构造函数，初始化灯条成员变量
 * 
 * @param rect 灯条旋转矩形
 * @param area 灯条面积
 * @param perimeter
 */
Light::Light(RotatedRect rect, float area, float perimeter)
{
    this->width = rect.size.width;
    this->height = rect.size.height;
    this->angle = rect.angle;
    this->center = rect.center;
    this->area = area;
}

/**
 * @brief 获得灯条高度
 * 
 * @return height(float)
 */
float Light::get_height()
{
    return this->height;
}

/**
 * @brief 获得灯条宽度
 * 
 * @return width(float)
 */
float Light::get_width()
{
    return this->width;
}

/**
 * @brief 获得灯条角度
 * 
 * @return angle(float)
 */
float Light::get_angle()
{
    return this->angle;
}

/**
 * @brief 获得灯条面积
 * 
 * @return area(float)
 */
float Light::get_area()
{
    return this->area;
}


/**
 * @brief 设置灯条面积
 * 
 * @return 无
 */
void Light::set_Area(float area)
{
    this->area = area;
}

/**
 * @brief 设置灯条角度
 * 
 * @return 无
 */
void Light::set_angle(float angle)
{
    this->angle = angle;
}

/**
 * @brief 获取中心点
 * 
 * @return center(Point2f)
 */
Point2f Light::get_center()
{
    return this->center;
}


/**
 * @brief 设置中心点
 * 
 * @return 无
 */
void Light::set_center(float x, float y)
{
    this->center.x = x;
    this->center.y = y;
}

/**
 * @brief 重载！=符号， 方便后序对灯条进行筛选和排序
 * 
 * @return 无
 */
bool operator!=(const Light &t1, const Light &t2)
{
    if (t1.width != t2.width && t1.height != t2.height && t1.angle != t2.angle)
        return true;
    else
        return false;
}