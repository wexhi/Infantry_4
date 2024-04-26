#include "miniPC_process.h"
#include "string.h"
#include "robot_def.h"
#include "crc_ref.h"
#include "daemon.h"

static Vision_Instance *vision_instance; // 用于和视觉通信的串口实例
static uint8_t *vis_recv_buff __attribute__((unused));
static Daemon_Instance *vision_daemon_instance;
// 全局变量区
extern uint16_t CRC_INIT;
/**
 * @brief 处理视觉传入的数据
 *
 * @param recv
 * @param rx_buff
 */
static void RecvProcess(Vision_Recv_s *recv, uint8_t *rx_buff)
{
    /* 使用memcpy接收浮点型小数 */
    // memcpy(&recv->maximal_arm, rx_buff + 1, 4);
    // memcpy(&recv->minimal_arm, rx_buff + 5, 4);
    // memcpy(&recv->z_height, rx_buff + 9, 4);
    // memcpy(&recv->finesse, rx_buff + 13, 4);
    // memcpy(&recv->pitch_arm, rx_buff + 17, 4);
    // memcpy(&recv->yaw, rx_buff + 21, 4);

    memcpy(&recv->is_tracking, rx_buff + 1, sizeof(Vision_Recv_s) - 1); /* 从第二个字节开始拷贝 */
}

/**
 * @brief 回调函数，确认帧头后用于解析视觉数据
 *
 */
static void DecodeVision(uint16_t var)
{
    UNUSED(var);                          // 仅为了消除警告
    DaemonReload(vision_daemon_instance); // 喂狗
#ifdef VISION_USE_VCP
    if (vis_recv_buff[0] == vision_instance->recv_data->header) {
        // 读取视觉数据
        RecvProcess(vision_instance->recv_data, vis_recv_buff);
    }
#endif
#ifdef VISION_USE_UART
    if (vision_instance->usart->recv_buff[0] == vision_instance->recv_data->header) {
        // 读取视觉数据
        RecvProcess(vision_instance->recv_data, vision_instance->usart->recv_buff);
    }
#endif
}

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_instance->usart);
#endif // !VISION_USE_UART
}

/**
 * @brief 设置发送给视觉的IMU数据
 *
 * @param yaw
 * @param pitch
 * @param roll
 */
void VisionSetAltitude(float yaw, float pitch, float roll)
{
    // vision_instance->send_data->yaw   = yaw;
    // vision_instance->send_data->pitch = pitch;
    // vision_instance->send_data->roll  = roll;
}

/**
 * @brief 发送数据处理函数
 *
 * @param send 待发送数据
 * @param tx_buff 发送缓冲区
 *
 */
__unused static void SendProcess(Vision_Send_s *send, uint8_t *tx_buff)
{
    /* 发送帧头，目标颜色，是否重置等数据 */
}

/**
 * @brief 用于注册一个视觉接收数据结构体,返回一个视觉接收数据结构体指针
 *
 * @param recv_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionRecvRegister(Vision_Recv_Init_Config_s *recv_config)
{
    Vision_Recv_s *recv_data = (Vision_Recv_s *)malloc(sizeof(Vision_Recv_s));
    memset(recv_data, 0, sizeof(Vision_Recv_s));

    recv_data->header = recv_config->header;

    return recv_data;
}

/**
 * @brief 用于注册一个视觉发送数据结构体,返回一个视觉发送数据结构体指针
 *
 * @param send_config
 * @return Vision_Send_s*
 */
Vision_Send_s *VisionSendRegister(Vision_Send_Init_Config_s *send_config)
{
    Vision_Send_s *send_data = (Vision_Send_s *)malloc(sizeof(Vision_Send_s));
    memset(send_data, 0, sizeof(Vision_Send_s));

    send_data->header = send_config->header;
    // send_data->detect_color  = send_config->detect_color;
    // send_data->reset_tracker = send_config->reset_tracker;
    // send_data->is_shoot      = send_config->is_shoot;
    // send_data->tail          = send_config->tail;
    return send_data;
}

#ifdef VISION_USE_UART

/**
 * @brief 用于注册一个视觉通信模块实例,返回一个视觉接收数据结构体指针
 *
 * @param init_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *video_usart_handle)
{
    vision_instance = (Vision_Instance *)malloc(sizeof(Vision_Instance));
    memset(vision_instance, 0, sizeof(Vision_Instance));

    init_config->usart_config.module_callback = DecodeVision;
    init_config->recv_config.header           = VISION_RECV_HEADER;
    vision_instance->usart                    = USARTRegister(&init_config->usart_config);
    vision_instance->recv_data                = VisionRecvRegister(&init_config->recv_config);
    vision_instance->send_data                = VisionSendRegister(&init_config->send_config);
    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback     = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id     = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return vision_instance->recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    SendProcess(vision_instance->send_data, send_buff);
    USARTSend(vision_instance->usart, send_buff, VISION_SEND_SIZE, USART_TRANSFER_BLOCKING);
}
#endif

#ifdef VISION_USE_VCP

#include "bsp_usb.h"

/**
 * @brief 用于注册一个视觉通信模块实例,返回一个视觉接收数据结构体指针
 *
 * @param init_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *video_usart_handle)
{
    UNUSED(video_usart_handle); // 仅为了消除警告
    vision_instance = (Vision_Instance *)malloc(sizeof(Vision_Instance));
    memset(vision_instance, 0, sizeof(Vision_Instance));
    Vision_Recv_Init_Config_s recv_config = {
        .header = VISION_RECV_HEADER,
    };

    USB_Init_Config_s conf     = {.rx_cbk = DecodeVision};
    vis_recv_buff              = USBInit(conf);
    recv_config.header         = VISION_RECV_HEADER;
    vision_instance->recv_data = VisionRecvRegister(&recv_config);

    Vision_Send_Init_Config_s send_config = {
        .header = VISION_SEND_HEADER,
    };
    vision_instance->send_data = VisionSendRegister(&send_config);
    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback     = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id     = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);
    return vision_instance->recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend(uint8_t is_start)
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    if (is_start == 1) {
        send_buff[0] = vision_instance->send_data->header;
    } else {
        send_buff[0] = 0;
    }
    USBTransmit(send_buff, VISION_SEND_SIZE);
}

#endif