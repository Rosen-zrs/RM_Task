#ifndef __ARMOR_LINK_
#define __ARMOR_LINK_

#include "../../Armor/include/Armor.h"

typedef struct Armor_node{
    //定义节点储存的装甲板指针
    p_Armor p_armor;
    //指向下一节点指针
    Armor_node* next = NULL;
    Armor_node();
    Armor_node(p_Armor a, Armor_node* next);
    p_Armor get_next_p_armor();
}Armor_node;


class Armor_Link
{
    public:
        Armor_Link();
        Armor_node* get_head();
        int get_length();
        
        void Del_Armor(int n);
        void Add_Armor(p_Armor a);

        //寻找链表中的目标装甲板
        void find_goal();

        p_Armor goal = NULL;
    private:
        //装甲板节点
        Armor_node* Head;
        int length;

        void Insert_Armor(p_Armor node, int n);
};


#endif // !__ARMOR_LINK