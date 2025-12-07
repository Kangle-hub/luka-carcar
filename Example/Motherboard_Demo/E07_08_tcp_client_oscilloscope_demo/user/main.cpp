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
// *************************** 例程使用步骤说明 ***************************
// 1.根据硬件连接说明连接好模块，使用电源供电(下载器供电会导致模块电压不足)
//
// 2.查看电脑所连接的wifi，记录IP地址
//
// 3.在下方的代码区域中修改宏定义，SERVER_IP为电脑的IP地址，PORT为端口号
//
// 5.下载例程到久久派中
//
// 6.打开逐飞助手，设置为TCP，设置端口，选择合适的本机地址后点击连接


// *************************** 例程测试说明 ***************************
// 1.当本机设备成功连接到目标后（例如电脑使用逐飞助手上位机进入TCP模式 然后本机连接到电脑的 IP 与端口）
//  
// 2.打开逐飞助手软件的波形显示，就可以看到波形。
// 
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************
// 另外一端的IP地址
#define SERVER_IP "192.168.2.32"
// 端口号
#define PORT 8888

int main(int, char**) 
{

    // 初始化TCP客户端,需要先打开TCP服务器,这才不会卡主。
    // 初始化TCP客户端,需要先打开TCP服务器,这才不会卡主。
    // 初始化TCP客户端,需要先打开TCP服务器,这才不会卡主。
    if(tcp_client_init(SERVER_IP, PORT) == 0)
    {
        printf("tcp_client ok\r\n");
    }
    else
    {
        printf("tcp_client error\r\n");
        return -1;
    }

    
    // 逐飞助手初始化 设置回调函数
    seekfree_assistant_interface_init(tcp_client_send_data, tcp_client_read_data);


    while(1)
    {
  // 写入需要发送的数据，有几个通道就写多少个数据
        // 这里仅写入4个通道数据
        seekfree_assistant_oscilloscope_data.data[0] += 0.1;
        seekfree_assistant_oscilloscope_data.data[1] += 0.5;
        seekfree_assistant_oscilloscope_data.data[2] += 1;
        seekfree_assistant_oscilloscope_data.data[3] += 2;
//        detector_oscilloscope_data.data[4] = 10;
//        detector_oscilloscope_data.data[5] = 100;
//        detector_oscilloscope_data.data[6] = 1000;
//        detector_oscilloscope_data.data[7] = 10000;
        
        // 设置本次需要发送几个通道的数据
        seekfree_assistant_oscilloscope_data.channel_num = 4;
        
        // 这里进发送了4个通道的数据，最大支持8通道
        seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
        
        system_delay_ms(20);
        // 有可能会在逐飞助手软件上看到波形更新不够连续，这是因为使用WIFI有不确定的延迟导致的
        
        // 解析上位机发送过来的参数，解析后数据会存放在seekfree_assistant_oscilloscope_data数组中，可以通过在线调试的方式查看数据
        // 例程为了方便因此写在了主循环，实际使用中推荐放到周期中断等位置，需要确保函数能够及时的被调用，调用周期不超过20ms
        seekfree_assistant_data_analysis();
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
// 问题2：逐飞助手的UDP或TCP一直连接不上
//      需要将久久派连接上网络
//      查看IP地址和端口号是否正确