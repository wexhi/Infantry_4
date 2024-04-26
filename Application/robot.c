#include "robot.h"
#include "roboTask.h"
#include "robot_def.h"
#include "robot_cmd.h"
#include "test.h"
#include "chassis.h"

#include "bsp_init.h"

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

/**
 * @brief 机器人初始化
 *
 */
void RobotInit(void)
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    // BSP初始化
    BSPInit();
    // 应用层初始化
    RobotCMDInit();
// 机械臂初始化
#ifdef ARM_BOARD
    ArmInit();
#endif // ARM_BOARD
    // 底盘初始化
#ifdef CHASSIS_BOARD
    ChassisInit();
#endif // CHASSIS_BOARD

    // 测试代码
    // TESTInit();

    // rtos创建任务
    OSTaskInit();
    // 初始化完成,开启中断
    __enable_irq();
}

/**
 * @brief 机器人任务入口
 *
 */
void RobotTask()
{
    // 应用层任务
    RobotCMDTask();
    // 机械臂任务
#ifdef ARM_BOARD
    ARMTask();
#endif // ARM_BOARD
    // 底盘任务
#ifdef CHASSIS_BOARD
    ChassisTask();
#endif // CHASSIS_BOARD
}

/*  下面为测试代码,可忽略    */
