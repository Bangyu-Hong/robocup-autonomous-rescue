#include "imu.h"

volatile IMU_Data_t robot_imu = {0};

static uint8_t rx_byte;
static uint8_t rx_buffer[11];
static uint8_t rx_index = 0;
static uint8_t rx_state = 0;

static float yaw_offset = 0.0f;
static uint8_t yaw_calibrated = 0;

#define YAW_CALIBRATION_FRAMES    (3U)
#define IMU_FRAME_LENGTH          (11U)

volatile uint32_t imu_rx_count = 0;
volatile uint32_t imu_frame_count = 0;
volatile uint32_t imu_angle_count = 0;
volatile uint32_t imu_checksum_fail = 0;

volatile uint8_t last_frame[11] = {0};
volatile uint8_t last_calc_sum = 0;
volatile uint8_t last_recv_sum = 0;

static const uint8_t CMD_UNLOCK[]     = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
static const uint8_t CMD_ACC_CALIB[]  = {0xFF, 0xAA, 0x01, 0x01, 0x00};
static const uint8_t CMD_ANGLE_ZERO[] = {0xFF, 0xAA, 0x01, 0x04, 0x00};
static const uint8_t CMD_SAVE[]       = {0xFF, 0xAA, 0x00, 0x00, 0x00};
static const uint8_t CMD_SET_OUTPUT[] = {0xFF, 0xAA, 0x02, 0x0E, 0x00};
static const uint8_t CMD_SET_RATE[]   = {0xFF, 0xAA, 0x03, 0x09, 0x00};

static void IMU_SendCommand(const uint8_t * cmd, uint8_t len)
{
    R_SCI_UART_Write(&g_uart_imu_ctrl, cmd, len);
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
}

static void IMU_ClearData(void)
{
    robot_imu.roll = 0.0f;
    robot_imu.pitch = 0.0f;
    robot_imu.yaw = 0.0f;
    robot_imu.acc_x = 0.0f;
    robot_imu.acc_y = 0.0f;
    robot_imu.acc_z = 0.0f;
    robot_imu.gyro_x = 0.0f;
    robot_imu.gyro_y = 0.0f;
    robot_imu.gyro_z = 0.0f;
}

static void IMU_ResetRxState(void)
{
    rx_state = 0;
    rx_index = 0;
}

static void IMU_RecordFrame(const uint8_t * frame, uint8_t calc_sum)
{
    for (uint8_t i = 0; i < IMU_FRAME_LENGTH; i++)
    {
        last_frame[i] = frame[i];
    }

    last_calc_sum = calc_sum;
    last_recv_sum = frame[10];
}

static void IMU_ParseFrame(const uint8_t * frame)
{
    uint8_t sum = 0;

    for (uint8_t i = 0; i < IMU_FRAME_LENGTH - 1U; i++)
    {
        sum = (uint8_t) (sum + frame[i]);
    }

    IMU_RecordFrame(frame, sum);

    if (sum != frame[10])
    {
        imu_checksum_fail++;
        return;
    }

    imu_frame_count++;

    switch (frame[1])
    {
        case 0x51:
        {
            int16_t raw_x = (int16_t) ((frame[3] << 8) | frame[2]);
            int16_t raw_y = (int16_t) ((frame[5] << 8) | frame[4]);
            int16_t raw_z = (int16_t) ((frame[7] << 8) | frame[6]);

            robot_imu.acc_x = raw_x / 32768.0f * 16.0f;
            robot_imu.acc_y = raw_y / 32768.0f * 16.0f;
            robot_imu.acc_z = raw_z / 32768.0f * 16.0f;
            break;
        }

        case 0x52:
        {
            int16_t raw_x = (int16_t) ((frame[3] << 8) | frame[2]);
            int16_t raw_y = (int16_t) ((frame[5] << 8) | frame[4]);
            int16_t raw_z = (int16_t) ((frame[7] << 8) | frame[6]);

            robot_imu.gyro_x = raw_x / 32768.0f * 2000.0f;
            robot_imu.gyro_y = raw_y / 32768.0f * 2000.0f;
            robot_imu.gyro_z = raw_z / 32768.0f * 2000.0f;
            break;
        }

        case 0x53:
        {
            int16_t raw_roll = (int16_t) ((frame[3] << 8) | frame[2]);
            int16_t raw_pitch = (int16_t) ((frame[5] << 8) | frame[4]);
            int16_t raw_yaw = (int16_t) ((frame[7] << 8) | frame[6]);
            float yaw_deg = raw_yaw / 32768.0f * 180.0f;

            robot_imu.roll = raw_roll / 32768.0f * 180.0f;
            robot_imu.pitch = raw_pitch / 32768.0f * 180.0f;

            if ((!yaw_calibrated) && (imu_angle_count >= YAW_CALIBRATION_FRAMES))
            {
                yaw_offset = yaw_deg;
                yaw_calibrated = 1U;
            }

            robot_imu.yaw = yaw_deg - yaw_offset;

            while (robot_imu.yaw > 180.0f)
            {
                robot_imu.yaw -= 360.0f;
            }

            while (robot_imu.yaw < -180.0f)
            {
                robot_imu.yaw += 360.0f;
            }

            imu_angle_count++;
            break;
        }

        default:
            break;
    }
}

void IMU_Calibrate(void)
{
    IMU_SendCommand(CMD_UNLOCK, sizeof(CMD_UNLOCK));
    IMU_SendCommand(CMD_ACC_CALIB, sizeof(CMD_ACC_CALIB));
    R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);

    IMU_SendCommand(CMD_UNLOCK, sizeof(CMD_UNLOCK));
    IMU_SendCommand(CMD_ANGLE_ZERO, sizeof(CMD_ANGLE_ZERO));
    R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);

    yaw_calibrated = 0U;
    yaw_offset = 0.0f;
    imu_angle_count = 0U;
    IMU_ClearData();
}

void IMU_ResetYaw(void)
{
    IMU_SendCommand(CMD_UNLOCK, sizeof(CMD_UNLOCK));
    IMU_SendCommand(CMD_ANGLE_ZERO, sizeof(CMD_ANGLE_ZERO));
    R_BSP_SoftwareDelay(50, BSP_DELAY_UNITS_MILLISECONDS);

    yaw_calibrated = 0U;
    yaw_offset = 0.0f;
    imu_angle_count = 0U;
    robot_imu.yaw = 0.0f;
}

void IMU_SoftResetYaw(void)
{
    yaw_calibrated = 0U;
    yaw_offset = 0.0f;
    imu_angle_count = 0U;
    robot_imu.yaw = 0.0f;
}

void IMU_Init(void)
{
    R_SCI_UART_Open(&g_uart_imu_ctrl, &g_uart_imu_cfg);
    R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS);

    IMU_SendCommand(CMD_UNLOCK, sizeof(CMD_UNLOCK));
    R_BSP_SoftwareDelay(50, BSP_DELAY_UNITS_MILLISECONDS);

    IMU_SendCommand(CMD_SET_OUTPUT, sizeof(CMD_SET_OUTPUT));
    R_BSP_SoftwareDelay(50, BSP_DELAY_UNITS_MILLISECONDS);

    IMU_SendCommand(CMD_UNLOCK, sizeof(CMD_UNLOCK));
    IMU_SendCommand(CMD_SET_RATE, sizeof(CMD_SET_RATE));
    R_BSP_SoftwareDelay(50, BSP_DELAY_UNITS_MILLISECONDS);

    IMU_SendCommand(CMD_UNLOCK, sizeof(CMD_UNLOCK));
    IMU_SendCommand(CMD_SAVE, sizeof(CMD_SAVE));
    R_BSP_SoftwareDelay(200, BSP_DELAY_UNITS_MILLISECONDS);

    yaw_calibrated = 0U;
    yaw_offset = 0.0f;
    imu_angle_count = 0U;
    IMU_ClearData();
    IMU_ResetRxState();

    R_SCI_UART_Read(&g_uart_imu_ctrl, &rx_byte, 1);
}

void imu_uart_callback(uart_callback_args_t * p_args)
{
    uint8_t data = 0U;

    if ((p_args->event != UART_EVENT_RX_CHAR) && (p_args->event != UART_EVENT_RX_COMPLETE))
    {
        return;
    }

    if (p_args->event == UART_EVENT_RX_COMPLETE)
    {
        data = rx_byte;
    }
    else
    {
        data = (uint8_t) p_args->data;
    }

    imu_rx_count++;

    if ((rx_state != 0U) && (data == 0x55U) && (rx_index < IMU_FRAME_LENGTH))
    {
        rx_buffer[0] = 0x55U;
        rx_index = 1U;
        rx_state = 1U;
    }
    else
    {
        switch (rx_state)
        {
            case 0:
                if (data == 0x55U)
                {
                    rx_buffer[0] = 0x55U;
                    rx_index = 1U;
                    rx_state = 1U;
                }
                break;

            case 1:
                rx_buffer[rx_index++] = data;

                if ((rx_index == 2U) && ((data < 0x50U) || (data > 0x5FU)))
                {
                    IMU_ResetRxState();

                    if (data == 0x55U)
                    {
                        rx_buffer[0] = 0x55U;
                        rx_index = 1U;
                        rx_state = 1U;
                    }

                    break;
                }

                if (rx_index >= IMU_FRAME_LENGTH)
                {
                    IMU_ParseFrame(rx_buffer);
                    IMU_ResetRxState();
                }
                break;

            default:
                IMU_ResetRxState();
                break;
        }
    }

    R_SCI_UART_Read(&g_uart_imu_ctrl, &rx_byte, 1);
}
