#include "chassis.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "referee_task.h"
#include "general_def.h"
#include "referee_UI.h"

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

static DJIMotor_Instance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

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
                .Kp            = 11,   // 4.5
                .Ki            = 0.08,    // 0
                .Kd            = 0.02, // 0
                .IntegralLimit = 5000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 12000,
            },
            .current_PID = {
                .Kp            = 0.7, // 0.4
                .Ki            = 0,   // 0
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
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb                                                               = DJIMotorInit(&chassis_motor_config);

    referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI
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
    vt_lf = (chassis_vx - chassis_vy) * CHASSIS_WHEEL_OFFSET - chassis_cmd_recv.wz; // 1
    vt_rf = (chassis_vx + chassis_vy) * CHASSIS_WHEEL_OFFSET + chassis_cmd_recv.wz; // 2
    vt_rb = (chassis_vx + chassis_vy) * CHASSIS_WHEEL_OFFSET - chassis_cmd_recv.wz; // 3
    vt_lb = (chassis_vx - chassis_vy) * CHASSIS_WHEEL_OFFSET + chassis_cmd_recv.wz; // 4
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    // 功率限制待添加

    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
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

    chassis_feedback_data.shoot_heat  = referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
    chassis_feedback_data.shoot_limit = referee_data->GameRobotState.shooter_barrel_heat_limit;

    ui_data.ui_mode       = chassis_cmd_recv.ui_mode;
    ui_data.chassis_mode  = chassis_cmd_recv.chassis_mode;
    ui_data.friction_mode = chassis_cmd_recv.friction_mode;
    ui_data.vision_mode   = chassis_cmd_recv.vision_mode;

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}