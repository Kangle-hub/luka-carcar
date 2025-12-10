#include "zf_common_headfile.h"

#define MOTOR1_DIR "/dev/zf_driver_gpio_motor_1"
#define MOTOR1_PWM "/dev/zf_device_pwm_motor_1"

#define MOTOR2_DIR "/dev/zf_driver_gpio_motor_2"
#define MOTOR2_PWM "/dev/zf_device_pwm_motor_2"

// 在设备树中，设置的10000。如果要修改，需要与设备树对应。
#define MOTOR1_PWM_DUTY_MAX (motor_1_pwm_info.duty_max)
// 在设备树中，设置的10000。如果要修改，需要与设备树对应。
#define MOTOR2_PWM_DUTY_MAX (motor_2_pwm_info.duty_max)

// 电机PWM信息结构体声明
extern struct pwm_info motor_1_pwm_info;
extern struct pwm_info motor_2_pwm_info;
extern struct pwm_info servo_pwm_info;

// 电调PWM设备路径定义
extern struct pwm_info pwm_1_info;
extern struct pwm_info pwm_2_info;

#define PWM_1           "/dev/zf_device_pwm_esc_1"
#define PWM_2           "/dev/zf_device_pwm_esc_2"

// 按键与拨码开关设备路径定义
#define KEY_0       "/dev/zf_driver_gpio_key_0"
#define KEY_1       "/dev/zf_driver_gpio_key_1"
#define KEY_2       "/dev/zf_driver_gpio_key_2"
#define KEY_3       "/dev/zf_driver_gpio_key_3"
#define SWITCH_0    "/dev/zf_driver_gpio_switch_0"
#define SWITCH_1    "/dev/zf_driver_gpio_switch_1"

// 在设备树中，默认设置的10000。如果要修改，需要直接修改设备树。
#define PWM_DUTY_MAX (servo_pwm_info.duty_max)

// 方向编码器设备路径定义
#define ENCODER_1 "/dev/zf_encoder_1"
#define ENCODER_2 "/dev/zf_encoder_2"

// 蜂鸣器设备路径定义
#define BEEP "/dev/zf_driver_gpio_beep"