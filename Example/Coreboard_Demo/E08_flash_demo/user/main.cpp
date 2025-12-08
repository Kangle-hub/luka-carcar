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


// *************************** 例程测试说明 ***************************
// 1.久久派与主板使用54pin排线连接 再将久久派插到主板上面 主板使用电池供电 下载本例程
// 
// 2.打开终端可以看到:
// read_u16_dat = 1234567
// read_float_dat = 1.234560
// 
// 3.按下按键即可看到不同的输出状态
//
// **************************** 代码区域 ****************************

// 对于linux来说，一切皆文件，所以存储参数，也是通过文件的。
// 这里我们只是简单的演示下，如何使用。

#define FILENAME_U16    "parameter_u16.txt"
#define FILENAME_FLOAT  "parameter_float.txt"

uint32 write_u16_dat = 1234567;
float write_float_dat = 1.23456;

uint32 read_u16_dat = 0;
float read_float_dat = 0;


int main(int, char**) 
{
    // ---------- 写入并读取整数 ---------- //
    // 设置临时buffer
    char buffer[100] = {0};

    // 将需要写入的数组，以字符串的方式存放到数组中
    sprintf(buffer, "%d", write_u16_dat);

    // 文件写
    file_write_string(FILENAME_U16, buffer);

    // 清空buffer
    memset(buffer, 0, sizeof(buffer));

    // 读取字符串
    file_read_string(FILENAME_U16, buffer);

    // 这里的atoi是将字符转化为整数。
    // 如果是浮点数需要使用atof
    read_u16_dat = atoi(buffer);
    
    printf("read_u16_dat = %d\r\n", read_u16_dat);
    // ---------- 写入并读取整数 ---------- //

    // ---------- 写入并读取浮点数 ---------- //
    // 将需要写入的数组，以字符串的方式存放到数组中
    sprintf(buffer, "%f", write_float_dat);

    // 文件写
    file_write_string(FILENAME_FLOAT, buffer);

    // 清空buffer
    memset(buffer, 0, sizeof(buffer));

    // 读取字符串
    file_read_string(FILENAME_FLOAT, buffer);

    // 这里的atof是将字符转化为浮点数。
    // 如果是整数需要使用atoi
    read_float_dat = atof(buffer);
    
    printf("read_float_dat = %f\r\n", read_float_dat);
    // ---------- 写入并读取浮点数 ---------- //
}    