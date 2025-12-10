#include "zf_common_headfile.h"
#include "dirs.h"
#include "my_uvc.h"

void sigint_handler(int signum)
{
    printf("收到Ctrl+C，程序即将退出\n");
    // 关闭电机
    pwm_set_duty(MOTOR1_PWM, 0);
    pwm_set_duty(MOTOR2_PWM, 0);
    gpio_set_level(BEEP, 0x0);
    // maunualExit = true; // 设置手动退出标志
    pwm_set_duty(PWM_1, 0);
    pwm_set_duty(PWM_2, 0);
    system("systemctl restart systemd-networkd"); // 重启网络服务

    // pthread.join();
    exit(0);
}

int main(int, char**) 
{


    while(1)
    {
    }
    
}