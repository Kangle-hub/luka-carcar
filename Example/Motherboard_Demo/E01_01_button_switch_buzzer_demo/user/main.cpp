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
//  主板按键       单片机管脚
//  KEY_0         查看 seekfree_smart_cat_pai_99 文件中 zf_driver_gpio_key_0 节点定义 默认 GPIO13
//  KEY_1         查看 seekfree_smart_cat_pai_99 文件中 zf_driver_gpio_key_1 节点定义 默认 GPIO14
//  KEY_2         查看 seekfree_smart_cat_pai_99 文件中 zf_driver_gpio_key_2 节点定义 默认 GPIO15
//  KEY_3         查看 seekfree_smart_cat_pai_99 文件中 zf_driver_gpio_key_3 节点定义 默认 GPIO16
//  SWITCH_0      查看 seekfree_smart_cat_pai_99 文件中 zf_driver_gpio_switch_0 节点定义 默认 GPIO20
//  SWITCH_1      查看 seekfree_smart_cat_pai_99 文件中 zf_driver_gpio_switch_1 节点定义 默认 GPIO21

// *************************** 例程测试说明 ***************************
// 1.久久派与主板使用54pin排线连接 再将久久派插到主板上面 主板使用电池供电 下载本例程
// 
// 2.打开终端可以看到:
// key_0 = x.
// key_1 = x.
// key_2 = x.
// key_3 = x.
// switch_0 = x.
// switch_1 = x.
//
// 3.按下按键即可看到不同的输出状态
//
// **************************** 代码区域 ****************************

#define KEY_0       "/dev/zf_driver_gpio_key_0"
#define KEY_1       "/dev/zf_driver_gpio_key_1"
#define KEY_2       "/dev/zf_driver_gpio_key_2"
#define KEY_3       "/dev/zf_driver_gpio_key_3"
#define SWITCH_0    "/dev/zf_driver_gpio_switch_0"
#define SWITCH_1    "/dev/zf_driver_gpio_switch_1"

int main(int, char**) 
{

    while(1)
    {
        printf("key_0 = %d\r\n", gpio_get_level(KEY_0));
        printf("key_1 = %d\r\n", gpio_get_level(KEY_1));
        printf("key_2 = %d\r\n", gpio_get_level(KEY_2));
        printf("key_3 = %d\r\n", gpio_get_level(KEY_3));

        printf("switch_0 = %d\r\n", gpio_get_level(SWITCH_0));
        printf("switch_1 = %d\r\n", gpio_get_level(SWITCH_1));

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
