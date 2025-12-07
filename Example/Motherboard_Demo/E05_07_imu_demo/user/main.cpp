/*********************************************************************************************************************
* LS2K0300 Opensourec Library 即（LS2K0300 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是LS2K0300 开源库的一部分
*
* LS2K0300 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K0300
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-02-27        大W            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

// *************************** 例程硬件连接说明 ***************************
//  久久派与主板使用54pin排线连接，再将久久派插到主板上面，确保插到底核心板与主板插座间没有缝隙即可
//  久久派与主板使用54pin排线连接，再将久久派插到主板上面，确保插到底核心板与主板插座间没有缝隙即可
//  久久派与主板使用54pin排线连接，再将久久派插到主板上面，确保插到底核心板与主板插座间没有缝隙即可
//  使用本历程，就需要使用我们逐飞科技提供的内核。
// 
// 接入 IMU963RA或IMU660RA或IMU660RB
// 接入 IMU963RA或IMU660RA或IMU660RB
// 接入 IMU963RA或IMU660RA或IMU660RB
// 改例程序自动判断是哪一个姿态传感器
//      模块管脚             单片机管脚
//      SCL/SPC             查看 seekfree_smart_cat_pai_99 文件中 spi3 节点定义 默认 GPIO22 
//      SDA/DSI             查看 seekfree_smart_cat_pai_99 文件中 spi3 节点定义 默认 GPIO23 
//      SA0/SDO             查看 seekfree_smart_cat_pai_99 文件中 spi3 节点定义 默认 GPIO24 
//      CS                  查看 seekfree_smart_cat_pai_99 文件中 spi3 节点定义 默认 GPIO25 
//      GND                 电源地 GND
//      3V3                 电源 3V3

// *************************** 例程测试说明 ***************************
// 1.先将姿态传感器插入主板，然后重启久久派，等待重启完成。
//      姿态传感器的初始化在久久派开机的时候完成的。
//      姿态传感器的初始化在久久派开机的时候完成的。
//      姿态传感器的初始化在久久派开机的时候完成的。
// 
// 2.打开终端，使用115200波特率
//
// 4.终端中看到
//      imu_acc_x  = xxx.
//      ...
// 5.移动旋转 姿态传感器 就会看到数值变化
// 
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

timer_fd *pit_timer;

void pit_callback()
{
    if(DEV_IMU660RA == imu_type || DEV_IMU660RB == imu_type)
    {
        imu_acc_x = imu_get_raw(imu_file_path[ACC_X_RAW]);
        imu_acc_y = imu_get_raw(imu_file_path[ACC_Y_RAW]);
        imu_acc_z = imu_get_raw(imu_file_path[ACC_Z_RAW]);
    
        imu_gyro_x = imu_get_raw(imu_file_path[GYRO_X_RAW]);
        imu_gyro_y = imu_get_raw(imu_file_path[GYRO_Y_RAW]);
        imu_gyro_z = imu_get_raw(imu_file_path[GYRO_Z_RAW]);
    }
    else if(DEV_IMU963RA == imu_type)
    {
        imu_acc_x = imu_get_raw(imu_file_path[ACC_X_RAW]);
        imu_acc_y = imu_get_raw(imu_file_path[ACC_Y_RAW]);
        imu_acc_z = imu_get_raw(imu_file_path[ACC_Z_RAW]);
    
        imu_gyro_x = imu_get_raw(imu_file_path[GYRO_X_RAW]);
        imu_gyro_y = imu_get_raw(imu_file_path[GYRO_Y_RAW]);
        imu_gyro_z = imu_get_raw(imu_file_path[GYRO_Z_RAW]);

        imu_mag_x = imu_get_raw(imu_file_path[MAG_X_RAW]);
        imu_mag_y = imu_get_raw(imu_file_path[MAG_Y_RAW]);
        imu_mag_z = imu_get_raw(imu_file_path[MAG_Z_RAW]);
    }
}


int main(int, char**) 
{

    imu_get_dev_info();
    
    if(DEV_IMU660RA == imu_type)
    {
        printf("IMU DEV IS IMU660RA\r\n");
    }
    else if(DEV_IMU660RB == imu_type)
    {
        printf("IMU DEV IS IMU660RB\r\n");
    }
    else if(DEV_IMU963RA == imu_type)
    {
        printf("IMU DEV IS IMU963RA\r\n");
    }
    else
    {
        printf("NO FIND IMU DEV\r\n");
        return -1;
    }
    
    // // 创建一个定时器10ms周期，回调函数为pit_callback
    // pit_ms_init(10, pit_callback);

    // 创建一个定时器10ms周期，回调函数为pit_callback
    pit_timer = new timer_fd(10, pit_callback);
    pit_timer->start();

    while(1)
    {

        if(DEV_IMU660RA == imu_type || DEV_IMU660RB == imu_type)
        {
            printf("imu_acc_x  = %d\r\n", imu_acc_x);
            printf("imu_acc_y  = %d\r\n", imu_acc_y);
            printf("imu_acc_z  = %d\r\n", imu_acc_z);
    
            printf("imu_gyro_x = %d\r\n", imu_gyro_x);
            printf("imu_gyro_y = %d\r\n", imu_gyro_y);
            printf("imu_gyro_z = %d\r\n", imu_gyro_z);
        }
        else if(DEV_IMU963RA == imu_type)
        {
            printf("imu_acc_x  = %d\r\n", imu_acc_x);
            printf("imu_acc_y  = %d\r\n", imu_acc_y);
            printf("imu_acc_z  = %d\r\n", imu_acc_z);
    
            printf("imu_gyro_x = %d\r\n", imu_gyro_x);
            printf("imu_gyro_y = %d\r\n", imu_gyro_y);
            printf("imu_gyro_z = %d\r\n", imu_gyro_z);
    
            printf("imu_mag_x = %d\r\n", imu_mag_x);
            printf("imu_mag_y = %d\r\n", imu_mag_y);
            printf("imu_mag_z = %d\r\n", imu_mag_z);
        }


        system_delay_ms(100);
    }
}

// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 
// 问题1：终端提示未找到xxx文件
//      使用本历程，就需要使用我们逐飞科技提供的内核，否则提示xxx文件找不到
//      使用本历程，就需要使用我们逐飞科技提供的内核，否则提示xxx文件找不到
//      使用本历程，就需要使用我们逐飞科技提供的内核，否则提示xxx文件找不到
// 
// 问题2：终端输出NO FIND IMU DEV
//      检查 姿态传感器 的接线是否正确
//      检查 姿态传感器 的模块是不是坏了
// 
// 问题3：姿态传感器 数值异常
//      看看是不是线松了 或者信号线被短路了
//      可能模块部分受损
