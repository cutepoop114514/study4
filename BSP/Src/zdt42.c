#include "zdt42.h"
#include "usart.h"
#include <limits.h>
#include <stdint.h>

extern UART_HandleTypeDef huart3;

#define ZDT42_UART_TIMEOUT_MS       100U
#define ZDT42_RX_TIMEOUT_MS         50U

void ZDT42_Init(void)
{
}

static HAL_StatusTypeDef ZDT42_SendCmd(const uint8_t *cmd, uint16_t len)
{
    return HAL_UART_Transmit(&huart3, (uint8_t *)cmd, len,
                             ZDT42_UART_TIMEOUT_MS);
}

void ZDT42_Enable(uint8_t id, uint8_t enable)
{
    const uint8_t cmd[6] = {
        id, 0xF3, 0xAB, (enable != 0U) ? 0x01U : 0x00U, 0x00, 0x6B
    };
    (void)ZDT42_SendCmd(cmd, sizeof(cmd));
}

void ZDT42_Stop(uint8_t id)
{
    const uint8_t cmd[5] = {id, 0xFE, 0x98, 0x00, 0x6B};
    (void)ZDT42_SendCmd(cmd, sizeof(cmd));
}

void ZDT42_SetSpeed(uint8_t id, int16_t rpm, uint16_t accel)
{
    const uint8_t direction = (rpm < 0) ? 0x01U : 0x00U;
    const uint32_t rpm_magnitude = (rpm < 0)
        ? (uint32_t)(-(int32_t)rpm)
        : (uint32_t)rpm;
    const uint32_t speed_0_1_rpm = rpm_magnitude * 10U;
    uint8_t cmd[9];

    if (speed_0_1_rpm > UINT16_MAX) {
        return;
    }

    cmd[0] = id;
    cmd[1] = 0xF6;
    cmd[2] = direction;
    cmd[3] = (uint8_t)(accel >> 8);
    cmd[4] = (uint8_t)accel;
    cmd[5] = (uint8_t)(speed_0_1_rpm >> 8);
    cmd[6] = (uint8_t)speed_0_1_rpm;
    cmd[7] = 0x00;
    cmd[8] = 0x6B;

    (void)ZDT42_SendCmd(cmd, sizeof(cmd));
}

static void ZDT42_MovePos(uint8_t id, float angle_deg, uint16_t rpm,
                          uint8_t accel, uint8_t mode)
{
    const uint8_t direction = (angle_deg < 0.0f) ? 0x01U : 0x00U;
    const double magnitude = (angle_deg < 0.0f)
        ? -(double)angle_deg
        : (double)angle_deg;
    const double angle_0_1_deg = magnitude * 10.0;
    const uint32_t speed_0_1_rpm = (uint32_t)rpm * 10U;
    uint32_t angle_value;
    uint8_t cmd[12];

    if (angle_deg != angle_deg ||
        angle_0_1_deg > (double)UINT32_MAX ||
        speed_0_1_rpm > UINT16_MAX) {
        return;
    }

    (void)accel;
    angle_value = (uint32_t)angle_0_1_deg;
    cmd[0] = id;
    cmd[1] = 0xFB;
    cmd[2] = direction;
    cmd[3] = (uint8_t)(speed_0_1_rpm >> 8);
    cmd[4] = (uint8_t)speed_0_1_rpm;
    cmd[5] = (uint8_t)(angle_value >> 24);
    cmd[6] = (uint8_t)(angle_value >> 16);
    cmd[7] = (uint8_t)(angle_value >> 8);
    cmd[8] = (uint8_t)angle_value;
    cmd[9] = mode;
    cmd[10] = 0x00;
    cmd[11] = 0x6B;

    (void)ZDT42_SendCmd(cmd, sizeof(cmd));
}

void ZDT42_MoveRelative (uint8_t id, float angle_deg, uint16_t rpm, uint8_t accel)
{
    ZDT42_MovePos(id, angle_deg, rpm, accel, 0x02);
}

void ZDT42_MoveAbsolute(uint8_t id, float angle_deg, uint16_t rpm, uint8_t accel)
{
    ZDT42_MovePos(id, angle_deg, rpm, accel, 0x01);
}

int16_t ZDT42_ReadSpeed(uint8_t id)
{
    const uint8_t cmd[3] = {id, 0x35, 0x6B};
    uint8_t rx_buf[6];
    int32_t speed = 0;
    volatile uint32_t discarded_data;

    __HAL_UART_CLEAR_OREFLAG(&huart3);
    discarded_data = huart3.Instance->DR;
    (void)discarded_data;

    if (ZDT42_SendCmd(cmd, sizeof(cmd)) != HAL_OK ||
        HAL_UART_Receive(&huart3, rx_buf, sizeof(rx_buf),
                         ZDT42_RX_TIMEOUT_MS) != HAL_OK ||
        rx_buf[0] != id || rx_buf[1] != 0x35 || rx_buf[5] != 0x6B) {
        return 0;
    }

    speed = (((int32_t)rx_buf[3] << 8) | rx_buf[4]) / 10;
    if (rx_buf[2] == 0x01U) {
        speed = -speed;
    }

    if (speed > INT16_MAX) {
        speed = INT16_MAX;
    } else if (speed < INT16_MIN) {
        speed = INT16_MIN;
    }
    return (int16_t)speed;
}

float ZDT42_ReadPosition(uint8_t id)
{
    const uint8_t cmd[3] = {id, 0x36, 0x6B};
    uint8_t rx_buf[8];
    uint32_t magnitude;
    double position_deg;
    volatile uint32_t discarded_data;

    __HAL_UART_CLEAR_OREFLAG(&huart3);
    discarded_data = huart3.Instance->DR;
    (void)discarded_data;

    if (ZDT42_SendCmd(cmd, sizeof(cmd)) != HAL_OK ||
        HAL_UART_Receive(&huart3, rx_buf, sizeof(rx_buf),
                         ZDT42_RX_TIMEOUT_MS) != HAL_OK ||
        rx_buf[0] != id || rx_buf[1] != 0x36 || rx_buf[7] != 0x6B) {
        return 0.0f;
    }

    magnitude = ((uint32_t)rx_buf[3] << 24) |
                ((uint32_t)rx_buf[4] << 16) |
                ((uint32_t)rx_buf[5] << 8) |
                (uint32_t)rx_buf[6];
    position_deg = (double)magnitude / 10.0;
    if (rx_buf[2] == 0x01U) {
        position_deg = -position_deg;
    }
    return (float)position_deg;
}