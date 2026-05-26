#include "zf_device_tof050f.h"
#include "zf_driver_uart.h"
#include "zf_common_typedef.h"
#include "zf_driver_delay.h"
#include "string.h"

#if defined(CY_CORE_CM7_0) || defined(CY_CORE_CM7_1)
#include "core_cm7.h"
#elif defined(CY_CORE_CM4)
#include "core_cm4.h"
#elif defined(CY_CORE_CM0P)
#include "core_cm0plus.h"
#endif

volatile uint16 tof050f_distance_mm = 0xFFFF;
volatile uint32 tof050f_update_count = 0;
volatile uint32 tof050f_update_period_ms = 0;

static uint8 tof050f_rx_buffer[16];
static uint8 tof050f_rx_length = 0;
static uint32 tof050f_last_timestamp_ticks = 0;
static bool tof050f_timestamp_initialized = false;

static uint16 tof050f_crc16_modbus(const uint8 *buff, uint16 len)
{
    uint16 crc = 0xFFFF;
    for (uint16 i = 0; i < len; i++)
    {
        crc ^= buff[i];
        for (uint8 bit = 0; bit < 8; bit++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static void tof050f_timestamp_init(void)
{
#if defined(CY_CORE_CM7_0) || defined(CY_CORE_CM7_1) || defined(CY_CORE_CM4) || defined(CY_CORE_CM0P)
    if (!tof050f_timestamp_initialized)
    {
        if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
        {
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        }
        DWT->CYCCNT = 0u;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        tof050f_timestamp_initialized = true;
    }
#endif
}

static uint32 tof050f_get_timestamp_ticks(void)
{
#if defined(CY_CORE_CM7_0) || defined(CY_CORE_CM7_1) || defined(CY_CORE_CM4) || defined(CY_CORE_CM0P)
    if (!tof050f_timestamp_initialized)
    {
        tof050f_timestamp_init();
    }
    return DWT->CYCCNT;
#else
    return 0;
#endif
}

static void tof050f_record_distance(uint16 distance)
{
    uint32 current_ticks = tof050f_get_timestamp_ticks();

    if (tof050f_last_timestamp_ticks != 0u)
    {
        uint32 delta_ticks = current_ticks - tof050f_last_timestamp_ticks;
        tof050f_update_period_ms = (uint32)((((uint64_t)delta_ticks * 1000ull) + ((uint64_t)SystemCoreClock / 2ull)) / (uint64_t)SystemCoreClock);
    }

    tof050f_last_timestamp_ticks = current_ticks;
    tof050f_distance_mm = distance;
    tof050f_update_count++;
}

static void tof050f_parse_frame(void)
{
    if (tof050f_rx_length < 5)
    {
        return;
    }

    while (tof050f_rx_length >= 5)
    {
        if (tof050f_rx_buffer[1] != 0x03 || tof050f_rx_buffer[2] != 0x02)
        {
            size_t shift = 1;
            while (shift + 2 < tof050f_rx_length)
            {
                if (tof050f_rx_buffer[shift] == 0x03 && tof050f_rx_buffer[shift + 1] == 0x02)
                {
                    break;
                }
                shift++;
            }

            if (shift + 2 >= tof050f_rx_length)
            {
                if (tof050f_rx_length > 2)
                {
                    tof050f_rx_buffer[0] = tof050f_rx_buffer[tof050f_rx_length - 2];
                    tof050f_rx_buffer[1] = tof050f_rx_buffer[tof050f_rx_length - 1];
                    tof050f_rx_length = 2;
                }
                break;
            }

            memmove(tof050f_rx_buffer, tof050f_rx_buffer + shift, tof050f_rx_length - shift);
            tof050f_rx_length -= shift;
            continue;
        }

        if (tof050f_rx_length >= 7)
        {
            uint16 recv_crc = (uint16)tof050f_rx_buffer[5] | ((uint16)tof050f_rx_buffer[6] << 8);
            uint16 calc_crc = tof050f_crc16_modbus(tof050f_rx_buffer, 5);
            if (recv_crc == calc_crc)
            {
                uint16 distance = ((uint16)tof050f_rx_buffer[3] << 8) | tof050f_rx_buffer[4];
                tof050f_record_distance(distance);
                if (tof050f_rx_length > 7)
                {
                    memmove(tof050f_rx_buffer, tof050f_rx_buffer + 7, tof050f_rx_length - 7);
                }
                tof050f_rx_length -= 7;
                tof050f_request_distance();
                continue;
            }
        }

        if (tof050f_rx_length >= 5)
        {
            uint16 distance = ((uint16)tof050f_rx_buffer[3] << 8) | tof050f_rx_buffer[4];
            tof050f_record_distance(distance);
            if (tof050f_rx_length > 5)
            {
                memmove(tof050f_rx_buffer, tof050f_rx_buffer + 5, tof050f_rx_length - 5);
            }
            tof050f_rx_length -= 5;
            tof050f_request_distance();
            continue;
        }

        break;
    }
}

void tof050f_request_distance(void)
{
    uint8 command[8] = {0x00, 0x03, 0x00, 0x10, 0x00, 0x01, 0x00, 0x00};
    uint16 crc = tof050f_crc16_modbus(command, 6);
    command[6] = (uint8)(crc & 0xFF);
    command[7] = (uint8)(crc >> 8);
    uart_write_buffer(TOF050F_UART, command, sizeof(command));
}

void tof050f_init(void)
{
    tof050f_distance_mm = 0xFFFF;
    tof050f_update_count = 0;
    tof050f_update_period_ms = 0;
    tof050f_rx_length = 0;
    tof050f_last_timestamp_ticks = 0;
    tof050f_timestamp_initialized = false;

    uart_init(TOF050F_UART, 115200, TOF050F_RX, TOF050F_TX);
    uart_rx_interrupt(TOF050F_UART, 1);
    tof050f_timestamp_init();
    system_delay_ms(20);
    tof050f_request_distance();
}

void tof050f_rx_callback(void)
{
    uint8 dat;

    while (uart_query_byte(TOF050F_UART, &dat))
    {
        if (tof050f_rx_length < sizeof(tof050f_rx_buffer))
        {
            tof050f_rx_buffer[tof050f_rx_length++] = dat;
        }

        if (tof050f_rx_length >= 5)
        {
            tof050f_parse_frame();
        }
    }
}
