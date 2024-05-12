#include "chassis.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "referee_task.h"
#include "general_def.h"
#include "user_lib.h"
#include "referee_UI.h"
#include "super_cap.h"

#include "bsp_dwt.h"
#include "arm_math.h"

#ifdef CHASSIS_BOARD
#include "ins_task.h"
#include "C_comm.h"
static CAN_Comm_Instance *chasiss_can_comm; // 用于底盘的CAN通信
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;  // 用于发布底盘的数据
static Subscriber_t *chassis_sub; // 用于订阅底盘的控制命令
#endif
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;                  // 底盘接收到的控制命令
__unused static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
static referee_info_t *referee_data;                         // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data;                   // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCap_Instance *super_cap;                                 // 超级电容实例
static DJIMotor_Instance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
// 功率限制算法的变量定义
// static float K_limit = 1.0f, P_limit = 0;                    // 功率限制系数
// static float chassis_power;                                  // 底盘功率
// static uint16_t chassis_power_buffer;                        // 底盘功率缓冲区
// static float chassis_speed_err;                              // 底盘速度误差
// static float scaling_lf, scaling_rf, scaling_lb, scaling_rb; // 电机输出缩放系数
#define CHASSIS_MAX_POWER 240000.f // 底盘最大功率,15384 * 4，取了4个3508电机最大电流的一个保守值
#define CHASSIS_MAX_SPEED 240000.f // 底盘最大速度,单位mm/s
#ifdef CHASSIS_MCNAMEE_WHEEL
#define CHASSIS_WHEEL_OFFSET 1.0f // 机器人底盘轮子修正偏移量
#elif defined(CHASSIS_OMNI_WHEEL)
#define CHASSIS_WHEEL_OFFSET 0.7071f // 机器人底盘轮子修正偏移量，根号2/2，即45度，用于修正全向轮的安装位置
#endif                               // CHASSIS_OMNI_WHEEL

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 1, // 4.5
                .Ki            = 0, // 0
                .Kd            = 0, // 0
                .IntegralLimit = 5000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 12000,
            },
            .current_PID = {
                .Kp            = 1., // 0.4
                .Ki            = 0,  // 0
                .Kd            = 0,
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id                             = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb                                                               = DJIMotorInit(&chassis_motor_config);

    referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI

    SuperCap_Init_Config_s super_cap_config = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id      = 0x302,
            .rx_id      = 0x301,
        },
    };
    super_cap = SuperCapInit(&super_cap_config); // 超级电容初始化

#ifdef CHASSIS_BOARD
    Chassis_IMU_data                 = INS_Init(); // 底盘IMU初始化
    CAN_Comm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id      = 0x311,
            .rx_id      = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}

/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    vt_lf = (-chassis_vx + chassis_vy) * CHASSIS_WHEEL_OFFSET + chassis_cmd_recv.wz; // 1
    vt_rf = (chassis_vx + chassis_vy) * CHASSIS_WHEEL_OFFSET + chassis_cmd_recv.wz;  // 2
    vt_rb = (-chassis_vx - chassis_vy) * CHASSIS_WHEEL_OFFSET + chassis_cmd_recv.wz; // 3
    vt_lb = (chassis_vx - chassis_vy) * CHASSIS_WHEEL_OFFSET + chassis_cmd_recv.wz;  // 4
}

/**
 * @brief 电机速度限制
 *
 */
// static void Motor_Speed_limiting()
// {
//     VAL_LIMIT(vt_lf, -CHASSIS_MAX_SPEED, CHASSIS_MAX_SPEED);
//     VAL_LIMIT(vt_rf, -CHASSIS_MAX_SPEED, CHASSIS_MAX_SPEED);
//     VAL_LIMIT(vt_lb, -CHASSIS_MAX_SPEED, CHASSIS_MAX_SPEED);
//     VAL_LIMIT(vt_rb, -CHASSIS_MAX_SPEED, CHASSIS_MAX_SPEED);
// }

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    // 功率限制待添加
    uint16_t chassis_power_buffer = 0; // 底盘功率缓冲区
    uint16_t chassis_power        = 0;
    uint16_t chassis_power_limit  = 0;
    float P_limit                 = 1; // 功率限制系数

    chassis_power_buffer = referee_data->PowerHeatData.buffer_energy;
    chassis_power        = referee_data->PowerHeatData.chassis_power;
    chassis_power_limit  = referee_data->GameRobotState.chassis_power_limit;

    switch (chassis_cmd_recv.super_cap_mode) {
        case SUPER_CAP_OFF:
            SuperCapSet(referee_data->PowerHeatData.buffer_energy, referee_data->GameRobotState.chassis_power_limit, 2); // 设置超级电容数据
            break;
        case SUPER_CAP_ON:
            SuperCapSet(referee_data->PowerHeatData.buffer_energy, referee_data->GameRobotState.chassis_power_limit, 1); // 设置超级电容数据
            break;
        default:
            break;
    }

    if (super_cap->cap_data.status == 2) {
        /*缓冲能量占比环，总体约束*/
        if (chassis_power_buffer >= 50)
            P_limit = 1;
        else if (chassis_power_buffer < 50 && chassis_power_buffer >= 40)
            P_limit = 0.9; // 近似于以一个线性来约束比例（为了保守可以调低Plimit，但会影响响应速度）
        else if (chassis_power_buffer < 40 && chassis_power_buffer >= 35)
            P_limit = 0.75;
        else if (chassis_power_buffer < 35 && chassis_power_buffer >= 30)
            P_limit = 0.5;
        else if (chassis_power_buffer < 30 && chassis_power_buffer >= 20)
            P_limit = 0.25;
        else if (chassis_power > chassis_power_limit && chassis_power_buffer < 20 && chassis_power_buffer >= 10)
            P_limit = 0.125;
        else if (chassis_power > chassis_power_limit && chassis_power_buffer < 10 && chassis_power_buffer > 0)
            P_limit = 0.05;
        else
            P_limit = 0.125;
    } else {
        P_limit = 1;
    }
    ui_data.Chassis_Power_Data.chassis_power_mx = super_cap->cap_data.voltage;
    SuperCapSend(); // 发送超级电容数据
    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf * P_limit);
    DJIMotorSetRef(motor_rf, vt_rf * P_limit);
    DJIMotorSetRef(motor_lb, vt_lb * P_limit);
    DJIMotorSetRef(motor_rb, vt_rb * P_limit);
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed()
{
    // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
    // chassis_feedback_data.vx vy wz =
    //  ...
    // max 48000
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif                                                         // CHASSIS_BOARD                                                    // CHASSIS_BOARD
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    } else { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    // 根据控制模式设定旋转速度
    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode) {
        case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
            if (chassis_cmd_recv.offset_angle > 0.1f || chassis_cmd_recv.offset_angle < -0.1f)
                chassis_cmd_recv.wz = -6.f * chassis_cmd_recv.offset_angle * abs(chassis_cmd_recv.offset_angle);
            else
                chassis_cmd_recv.wz = 0;
            break;
        default:
            break;
    }

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    // cos_theta = 0, sin_theta = 1;
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();

    chassis_feedback_data.shoot_heat  = referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
    chassis_feedback_data.shoot_limit = referee_data->GameRobotState.shooter_barrel_heat_limit;

    ui_data.ui_mode        = chassis_cmd_recv.ui_mode;
    ui_data.chassis_mode   = chassis_cmd_recv.chassis_mode;
    ui_data.friction_mode  = chassis_cmd_recv.friction_mode;
    ui_data.vision_mode    = chassis_cmd_recv.vision_mode;
    ui_data.level          = referee_data->GameRobotState.robot_level;
    ui_data.lid_mode       = chassis_cmd_recv.lid_mode;
    ui_data.super_cap_mode = chassis_cmd_recv.super_cap_mode;
    ui_data.loader_mode    = chassis_cmd_recv.loader_mode;
    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}