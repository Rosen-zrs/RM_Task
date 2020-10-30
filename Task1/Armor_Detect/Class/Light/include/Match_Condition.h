#ifndef __MATCH_CONDITION__
#define __MATCH_CONDITION__

struct Match_Condition
{
    //筛除最大面积差值
    float MAX_AREA_DIFF = 0.55;
     //筛除最大宽高比
    float MAX_WH_RATIO = 1.2;
     //筛除最大角度差值
    float MAX_ANGLE_DIFF = 30;
     //筛除最大轮廓面积占比
    float MIN_AREA_FULL = 0.5;
     //筛除最大宽度差值
    float MAX_WIDTH_DIFF = 0.4;
     //筛除最大高度差值
    float MAX_HEIGHT_DIFF = 0.4;
};

#endif // !__MATCH_CONDITION__