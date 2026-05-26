#ifndef _zf_device_tof050f_h_
#define _zf_device_tof050f_h_

#include "zf_common_typedef.h"
#include "zf_driver_uart.h"

// 使用 GPS 串口直接接入 TOF050F 模块
#define TOF050F_UART        (UART_2)
#define TOF050F_RX          (UART2_TX_P10_1)    // TOF050F RX 线连接 MCU TX
#define TOF050F_TX          (UART2_RX_P10_0)    // TOF050F TX 线连接 MCU RX

extern volatile uint16 tof050f_distance_mm;
extern volatile uint32 tof050f_update_count;
extern volatile uint32 tof050f_update_period_ms; // Period in milliseconds between the last two valid TOF updates

void tof050f_init(void);
void tof050f_rx_callback(void);
void tof050f_request_distance(void);

#endif // _zf_device_tof050f_h_
