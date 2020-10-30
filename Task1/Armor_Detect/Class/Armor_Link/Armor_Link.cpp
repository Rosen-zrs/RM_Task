#include "include/Armor_Link.h"

using namespace std;

/**
 * @brief 结构体Armor_node的构造函数
 */
Armor_node::Armor_node()
{
    this->p_armor = NULL;
    this->next = NULL;
}

Armor_node::Armor_node(p_Armor a, Armor_node *next)
{
    this->p_armor = a;
    this->next = next;
}

p_Armor Armor_node::get_next_p_armor()
{
    return this->next->p_armor;
}

/**************************************************************/

/**
 * @brief 初始化一个装甲板单链表
 * 
 * @param 无
 */
Armor_Link::Armor_Link()
{
    this->Head = new Armor_node;
    Head->p_armor = NULL;
    Head->next = NULL;
    this->length = 0;
}

/**
 * @brief 获得单链表的头节点
 * 
 * @return Head
 */
Armor_node *Armor_Link::get_head()
{
    return this->Head;
}

/**
 * @brief 获取装甲板链表的长度
 * 
 */
int Armor_Link::get_length()
{
    return this->length;
}

/**
 * @brief 在单链表中插入装甲板
 */
void Armor_Link::Insert_Armor(p_Armor a, int n)
{
    Armor_node *p = this->Head;
    while (--n)
        p = p->next;
    Armor_node *node = new Armor_node(a, p->next);
    p->next = node;
    this->length++;
}

/**
 * @brief 在Armor_Link末尾添加装甲板
 */
void Armor_Link::Add_Armor(p_Armor a)
{
    a->identifier = this->length + 1;
    Insert_Armor(a, a->identifier);
}

/**
 * @brief 在单链表中删除装甲板
 */
void Armor_Link::Del_Armor(int n)
{
    Armor_node *p = this->Head;
    while (--n)
        p = p->next;
    Armor_node *q = p->next;
    p->next = q->next;
    while (p->next != NULL)
    {
        p->next->p_armor->identifier--;
        p = p->next;
    }
    q->p_armor->identifier = 0;
    this->length--;
    free(q);
}

/**
 * @brief 寻找装甲板链表中的目标装甲板
 */
void Armor_Link::find_goal()
{
    Armor_node *p = this->get_head();
    float min_dis, max_area;

    while (p->next != NULL)
    {
        float mean_area = (p->get_next_p_armor()->get_left_light().get_area() + p->get_next_p_armor()->get_right_light().get_area()) / 2;

        //记录灯条对最大面积
        if (p == this->get_head())
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

    p = this->get_head();
    //当面积与最大面积相近时，以中心矩离优先匹配
    while (p->next != NULL)
    {
        if (p == this->get_head())
        {
            min_dis = p->get_next_p_armor()->get_center_dis();
        }

        //计算灯条平均面积
        float mean_area = (p->get_next_p_armor()->get_left_light().get_area() + p->get_next_p_armor()->get_right_light().get_area()) / 2;

        if (abs(mean_area - max_area) / max_area < 0.15)
        {
            //匹配最近灯条对
            if (p->get_next_p_armor()->get_center_dis() < min_dis)
            {
                min_dis = p->get_next_p_armor()->get_center_dis();
                this->goal = p->get_next_p_armor();
            }
        }
    }
}
