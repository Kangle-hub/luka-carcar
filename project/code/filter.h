#ifndef CODE_FILTER_H_
#define CODE_FILTER_H_

#include "zf_common_headfile.h"

int Ave_Filter_L(int new_Spe,float *speed_Record);
int Ave_Filter_R(int new_Spe,float *speed_Record);

#ifndef M_PI
#define M_PI (3.141592f)
#endif
typedef struct
{
    float ts;       //采样周期 s
    float fc;       //截至频率 hz
    float lastYn;   //上一次滤波值
    float alpha;    //滤波系数
} low_pass_filter_t;

void Init_lowPass_alpha(low_pass_filter_t* const filter,const float ts, const float fc);  //初始化滤波系数
float Low_pass_filter(low_pass_filter_t* const filter, const float data);  //低通滤波
void kalmanFilter(float z);
#endif /* CODE_FILTER_H_ */
