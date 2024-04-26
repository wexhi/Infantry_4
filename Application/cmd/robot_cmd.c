/**
 * @file robot_cmd.c
 * @author your name (you@domain.com)
 * @brief 机器人核心控制任务
 * @attention 因为底盘接的是遥控器，但是云台需要进行视觉传输，因此在预编译时不应该屏蔽掉RobotCMDInit，
 *             否则会出现内存访问错误，应在该文件中修改预编译指令。
 *             由于底盘板和云台板的任务都包含有云台电机的任务，因此应该在此处进行双板之间的通信。
 * @version 0.1
 * @date 2024-01-15
 *
 * @copyright Copyright (c) 2024
 *
 */
// application layer for robot command
#include "robot_cmd.h"
#include "robot_def.h"

// module layer
#include "remote.h"
#include "miniPC_process.h"
#include "VideoTransmitter.h"
#include "message_center.h"
#include "user_lib.h"
#include "miniPC_process.h"
#include "referee_protocol.h"
#include "scara_kinematics.h"
#include "arm_math.h"
#include "UARTComm.h"

#ifdef GIMBAL_BOARD
static Vision_Recv_s *vision_ctrl; // 视觉控制信息
#endif

#ifdef CHASSIS_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static void RemoteControlSet(void); // 遥控器控制量设置
static void VideoControlSet(void);  // 图传链路控制量设置
static void EmergencyHandler(void);
static RC_ctrl_t *rc_data;       // 遥控器数据指针,初始化时返回
static Video_ctrl_t *video_data; // 视觉数据指针,初始化时返回

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 *
 * @attention 工程机器人使用两块板子，它们之间的数据使用UART 1进行通信
 *
 */
void RobotCMDInit(void)
{
#ifdef GIMBAL_BOARD
    video_data  = VideoTransmitterControlInit(&huart6); // 初始化图传链路
    vision_ctrl = VisionInit(&huart3);                  // 初始化视觉控制
#endif                                                  // GIMBAL_BOARD

#ifdef CHASSIS_BOARD
    // 初始化遥控器,使用串口3
    rc_data          = RemoteControlInit(&huart3); // 初始化遥控器,C板上使用USART3
    chassis_cmd_pub  = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // CHASSIS_BOARD

    // 此处初始化与视觉的通信
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask(void)
{
    // 获取各个模块的数据
#ifdef CHASSIS_BOARD
    // 获取底盘反馈信息
    SubGetMessage(chassis_feed_sub, &chassis_fetch_data);
#endif
#ifdef GIMBAL_BOARD
#endif
    if (!rc_data[TEMP].rc.switch_right ||
        switch_is_down(rc_data[TEMP].rc.switch_right)) // 当收不到遥控器信号时，使用图传链路
        VideoControlSet();
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 当收到遥控器信号时,且右拨杆为中，使用遥控器
        RemoteControlSet();
    else if (switch_is_up(rc_data[TEMP].rc.switch_right))
        EmergencyHandler();

        // 发送控制信息

#ifdef GIMBAL_BOARD
        // 发送给云台
#endif
#ifdef CHASSIS_BOARD
    // 发送给底盘
    PubPushMessage(chassis_cmd_pub, &chassis_cmd_send);
#endif
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet(void)
{
#ifdef CHASSIS_BOARD
    chassis_cmd_send.chassis_mode = CHASSIS_SLOW; // 底盘模式
    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    chassis_cmd_send.vx = 20.0f * (float)rc_data[TEMP].rc.rocker_l_; // _水平方向
    chassis_cmd_send.vy = 20.0f * (float)rc_data[TEMP].rc.rocker_l1; // 1竖直方向
    chassis_cmd_send.wz = -10.0f * (float)rc_data[TEMP].rc.dial;     // _水平方向
#endif
}

/**
 * @brief 图传链路以及自定义控制器的模式和控制量设置
 *
 */

static void VideoControlSet(void)
{
    // 直接测试，稍后会添加到函数中
#ifdef GIMBAL_BOARD
#endif

    if (video_data[TEMP].key[KEY_PRESS_WITH_CTRL].v) {
        chassis_cmd_send.ui_mode = UI_REFRESH;
    } else {
        chassis_cmd_send.ui_mode = UI_KEEP;
    }

    chassis_cmd_send.vx = (video_data[TEMP].key[KEY_PRESS].a - video_data[TEMP].key[KEY_PRESS].d) * 30000 * chassis_cmd_send.chassis_speed_buff; // 系数待测
    chassis_cmd_send.vy = (video_data[TEMP].key[KEY_PRESS].w - video_data[TEMP].key[KEY_PRESS].s) * 30000 * chassis_cmd_send.chassis_speed_buff; // 系数待测                                                                                                         // test
    chassis_cmd_send.wz = (float)video_data[TEMP].key_data.mouse_x * 10 +
                          (-video_data[TEMP].key[KEY_PRESS].q + video_data[TEMP].key[KEY_PRESS].e) * 26000 * chassis_cmd_send.chassis_speed_buff;
    chassis_cmd_send.chassis_speed_buff = 1; // test
}

/**
 * @brief  紧急停止,包括遥控器右侧上侧拨杆打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler(void)
{
// 底盘急停
#ifdef CHASSIS_BOARD
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
#endif
    // 机械臂急停
#ifdef ARM_BOARD
    arm_cmd_send.arm_mode = ARM_ZERO_FORCE;
#endif
}
