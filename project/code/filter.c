#include "Filter.h"

#define SPEED_RECORD_NUM 2

int Ave_Filter_L(int new_Spe,float *speed_Record_L)   //编码器值的均值滤波
{
    float sum = 0.0f;
    int test_Speed = new_Spe;
    for(uint8_t i=SPEED_RECORD_NUM-1; i>0; i--)   //将现有数据后移一位
    {
        speed_Record_L[i] = speed_Record_L[i-1];
        sum += speed_Record_L[i-1];
    }
    speed_Record_L[0] = new_Spe;  //第一位是新的数据
    sum += new_Spe;
    test_Speed = sum/SPEED_RECORD_NUM;

    return test_Speed;  //返回均值
}

int Ave_Filter_R(int new_Spe,float *speed_Record_R)
{
    float sum = 0.0f;
    int test_Speed = new_Spe;
    for(uint8_t i=SPEED_RECORD_NUM-1; i>0; i--)
    {
        speed_Record_R[i] = speed_Record_R[i-1];
        sum += speed_Record_R[i-1];
    }
    speed_Record_R[0] = new_Spe;
    sum += new_Spe;
    test_Speed = sum/SPEED_RECORD_NUM;

    return test_Speed;
}

/*******************************************************************************
 * @fn Init_lowPass_alpha
 * @brief 初始化低通滤波器滤波系数
 * @param filter 滤波器
 * @param ts 采用周期 单位s
 * @return fc 截至频率 单位hz
 ******************************************************************************/
void Init_lowPass_alpha(low_pass_filter_t* const filter,const float ts, const float fc)
{
  float b=2*M_PI*fc*ts;
  filter->ts=ts;             //采样周期
  filter->fc=fc;             //截止频率
  filter->lastYn=0;          //上一次滤波值
  filter->alpha=b/(b+1);     //滤波系数
}

/*******************************************************************************
 * @fn Low_pass_filter
 * @brief 低通滤波函数
 * @param data 采样数据
 * @return 滤波结果
 ******************************************************************************/
float Low_pass_filter(low_pass_filter_t* const filter, const float data)
{
  float tem=filter->lastYn+(filter->alpha*(data-filter->lastYn));
  filter->lastYn=tem;
  return tem;
}

int groy_z;
void imuGetUse(){
/*3.12目前这样勉强能用 后面计算角度是加入低通滤波后在进行计算*/
  imu_gyro_z = imu_get_raw(imu_file_path[GYRO_Z_RAW]);

//        printf("%d\r\n",groy_z);

}



//低阶卡尔曼滤波  能用
// 定义卡尔曼滤波器的状态变量和参数
// extern float err_all ;  // 状态向量
//float P = 1.0;  // 状态协方差矩阵
//float Q = 0.01; // 过程噪声协方差矩阵
//float R = 0.1;  // 测量噪声协方差矩阵
//
//// 卡尔曼滤波算法
//void kalmanFilter(float z) {
//    // 预测步骤
//    float x_pred = err_all;
//    float P_pred = P + Q;
//
//    // 更新步骤
//    float K = P_pred / (P_pred + R);
//
//    err_all = x_pred + K * (z - x_pred);
//    P = (1 - K) * P_pred;
//
//}









