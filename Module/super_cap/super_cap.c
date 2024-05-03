/**
 * @file super_cap.h
 * @author Bi KaiXiang (wexhi@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-05-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "super_cap.h"
#include "memory.h"
#include "stdlib.h"

static SuperCap_Instance *super_cap_instance = NULL; // 可以由app保存此指针

static void SuperCapRxCallback(CAN_Instance *_instance)
{
    uint8_t *rxbuff        = _instance->rx_buff;
    SuperCap_Instance *ins = (SuperCap_Instance *)_instance->id;
    SuperCapData_t *data   = &ins->cap_data; // 获取实例的数据指针
    data->voltage          = (((uint16_t)rxbuff[0] << 8) | rxbuff[1]) / 1000;
    data->power            = (((uint16_t)rxbuff[2] << 8) | rxbuff[3]) / 1000;
    data->status           = rxbuff[4];
}

static void SuperCapLostCallback(void *cap_ptr)
{
    SuperCap_Instance *cap = (SuperCap_Instance *)cap_ptr;
    // 显示丢失的信息
    uint16_t can_bus __attribute__((unused)) = cap->can_ins->can_handle == &hcan1 ? 1 : 2;
}

SuperCap_Instance *SuperCapInit(SuperCap_Init_Config_s *config)
{
    super_cap_instance = (SuperCap_Instance *)malloc(sizeof(SuperCap_Instance));
    memset(super_cap_instance, 0, sizeof(SuperCap_Instance));

    config->can_config.can_module_callback = SuperCapRxCallback;
    config->can_config.id                  = super_cap_instance;
    super_cap_instance->can_ins            = CANRegister(&config->can_config);

    Daemon_Init_Config_s daemon_config = {
        .callback     = SuperCapLostCallback,
        .owner_id     = (void *)super_cap_instance,
        .reload_count = 6,
    };
    super_cap_instance->daemon = DaemonRegister(&daemon_config);
    return super_cap_instance;
}

void SuperCapSet(uint16_t buffer, uint16_t power, uint8_t state)
{
    super_cap_instance->send_data.buffer = buffer;
    super_cap_instance->send_data.power  = power;
    super_cap_instance->send_data.state  = state;
}

void SuperCapSend(void)
{
    static uint8_t send_data[8] = {0};
    send_data[0]                = (super_cap_instance->send_data.buffer >> 8) & 0xff;
    send_data[1]                = super_cap_instance->send_data.buffer & 0xff;
    send_data[2]                = (super_cap_instance->send_data.power >> 8) & 0xff;
    send_data[3]                = super_cap_instance->send_data.power & 0xff;
    send_data[4]                = super_cap_instance->send_data.state;
    memcpy(super_cap_instance->can_ins->tx_buff, send_data, 8);
    CANTransmit(super_cap_instance->can_ins, 1);
}