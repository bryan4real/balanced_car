/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h> // 提供 abs() 絕對值運算
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CONTROL_DT          0.005f

#define MPU_ADDR_68         (0x68 << 1)
#define MPU_ADDR_69         (0x69 << 1)

/* 前倒輪子卻往後轉，就把 1 改成 -1 */
#define MOTOR_DIR           1

/* 馬達保護參數：TIM15 Period = 999，所以 PWM 建議控制在 0~999 內 */
#define PWM_RUN_LIMIT       320
#define MOTOR_MIN_PWM       70
#define PWM_DEAD_ZONE       2
#define PWM_STEP_LIMIT      40
#define MOTOR_OUT_MAX       PWM_RUN_LIMIT

/* MPU6050：Accel ±2g = 16384 LSB/g；Gyro ±250 dps = 131 LSB/(deg/s) */
#define ACC_SCALE           16384.0f
#define GYRO_SCALE          131.0f
#define GYRO_SPIKE_LIMIT    400.0f
#define ACC_LPF_ALPHA       0.12f
#define GYRO_LPF_ALPHA      0.35f

/* 接近平衡點時停止微小輸出，避免細抖 */
#define NEAR_BALANCE_DEG    0.08f
#define NEAR_GYRO_DPS       0.6f

/* 你的車最多 ±10 度，測試時先給 20 度保護 */
#define ANGLE_STOP_LIMIT    90.0f

#define BUTTON_PRESSED_LEVEL GPIO_PIN_RESET

/* 0 = 模仿 GitHub：連續輸出；1 = 短脈衝 + 煞車 */
#define USE_MOTOR_PULSE     0
#define MOTOR_ON_MS         3
#define MOTOR_OFF_MS        12

/* GitHub control.c 的速度環、方向環週期概念；目前未接編碼器/循跡，預設輸出為 0 */
#define VC_PERIOD           4
#define DC_PERIOD           2
#define VC_OUT_MAX          120
#define DC_OUT_MAX          120

/* 速度環 PID：沒有編碼器前先設 0，等於保留功能但不介入 */
#define VC_PID_P            0.0f
#define VC_PID_I            0.0f
#define VC_PID_D            0.0f

/* 方向環 PID：沒有方向誤差來源前先設 0，等於保留功能但不介入 */
#define DC_PID_P            0.0f
#define DC_PID_D            0.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

volatile uint8_t control_flag = 0;
uint8_t motor_enable = 0;

uint8_t mpu_addr = 0;

int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

float Ax, Ay, Az;
float Gx, Gy, Gz;
float gyro_balance;

float gyro_offset = 0.0f;
float angle = 0.0f;
float balance_offset = 0.0f;

float Kp = 25.0f;
float Ki = 140.0f;
float Kd = 1.2f;

float integral = 0.0f;

float gyro_lpf = 0.0f;
float acc_angle_lpf = 0.0f;
uint8_t angle_filter_ready = 0;

uint32_t mpu_read_ok_count = 0;
uint32_t mpu_read_fail_count = 0;
uint8_t mpu_last_status = 0;

uint32_t debug_count = 0;

/* GitHub control.c 同功能架構：ac=角度環、vc=速度環、dc=方向環 */
int32_t speed = 0;
int16_t state = 0;
int32_t ac_pwm = 0;
int32_t vc_pwm = 0;
int32_t dc_pwm = 0;
int32_t left_pwm = 0;
int32_t right_pwm = 0;

/* 輸出平滑用：避免一有傾角 PWM 瞬間跳太大造成過衝 */
int32_t last_left_pwm = 0;
int32_t last_right_pwm = 0;

/* 速度目標：沒有編碼器前保持 0 */
int32_t velocity_set = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */
void Motor_Stop(void);
void Motor_Brake(void);
void Motor_Set(int left_pwm, int right_pwm);
void Motor_Set_Gated(int left_pwm, int right_pwm);
void Motor_Proc(int32_t LEFT_MOTOR_OUT, int32_t RIGHT_MOTOR_OUT);

int32_t Get_PWM(void);
int32_t Smooth_PWM(int32_t target, int32_t *last);
int32_t Get_Speed(void);
int16_t Get_Direction_Error(void);
int32_t Angle_Proc(void);
int32_t Velocity_Proc(int32_t now_speed);
int32_t Direction_Proc(int32_t now_speed);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t Button_IsPressed(void)
{
    return HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == BUTTON_PRESSED_LEVEL;
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

int PWM_Compensate(int pwm)
{
    int sign = 1;
    int value;

    if (pwm < 0)
    {
        sign = -1;
        pwm = -pwm;
    }

    if (pwm < PWM_DEAD_ZONE)
    {
        return 0;
    }

    value = MOTOR_MIN_PWM + pwm / 2;

    if (value > PWM_RUN_LIMIT)
    {
        value = PWM_RUN_LIMIT;
    }

    return sign * value;
}

void Motor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);

    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
}

void Motor_Brake(void)
{
    /* TB6612 short brake：STBY 保持啟用，AIN1/AIN2 與 BIN1/BIN2 同時為 High */
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 999);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 999);

    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
}

void Motor_Set(int left_pwm, int right_pwm)
{
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

    left_pwm = PWM_Compensate(left_pwm);
    right_pwm = PWM_Compensate(right_pwm);

    if (left_pwm >= 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, left_pwm);
    }
    else
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, -left_pwm);
    }

    if (right_pwm >= 0)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, right_pwm);
    }
    else
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, -right_pwm);
    }
}

void MPU_Scan(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, MPU_ADDR_68, 3, 100) == HAL_OK)
    {
        mpu_addr = MPU_ADDR_68;
        printf("MPU found at 0x68\r\n");
    }
    else if (HAL_I2C_IsDeviceReady(&hi2c1, MPU_ADDR_69, 3, 100) == HAL_OK)
    {
        mpu_addr = MPU_ADDR_69;
        printf("MPU found at 0x69\r\n");
    }
    else
    {
        mpu_addr = 0;
        printf("MPU not found\r\n");
    }
}

void MPU_Write(uint8_t reg, uint8_t data)
{
    if (mpu_addr == 0) return;
    HAL_I2C_Mem_Write(&hi2c1, mpu_addr, reg, 1, &data, 1, 100);
}

uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t data = 0xFF;

    if (mpu_addr == 0) return 0xFF;

    HAL_I2C_Mem_Read(&hi2c1, mpu_addr, reg, 1, &data, 1, 5);

    return data;
}

void MPU_Init(void)
{
    MPU_Scan();

    if (mpu_addr == 0)
    {
        return;
    }

    uint8_t who = MPU_Read_Byte(0x75);
    printf("WHO_AM_I = 0x%02X\r\n", who);

    /* 先軟體重置 MPU6050，再喚醒，避免上次狀態卡住導致讀值全為 0 */
    MPU_Write(0x6B, 0x80);
    HAL_Delay(100);

    MPU_Write(0x6B, 0x00);   // Wake up
    HAL_Delay(100);

    MPU_Write(0x19, 0x04);   // Sample rate = 1kHz/(1+4)=200Hz，配合 5ms 控制
    MPU_Write(0x1A, 0x03);   // DLPF 約 44Hz，降低馬達震動雜訊
    MPU_Write(0x1B, 0x00);   // Gyro ±250 dps，先用最穩定比例 131 LSB/(deg/s)
    MPU_Write(0x1C, 0x00);   // Accel ±2g，比例 16384 LSB/g

    HAL_Delay(100);

    printf("MPU init done\r\n");
}

uint8_t MPU_Read_All(void)
{
    uint8_t data[14];
    HAL_StatusTypeDef status;

    if (mpu_addr == 0)
    {
        mpu_last_status = 99;
        mpu_read_fail_count++;
        return 0;
    }

    status = HAL_I2C_Mem_Read(&hi2c1, mpu_addr, 0x3B, I2C_MEMADD_SIZE_8BIT, data, 14, 20);

    if (status != HAL_OK)
    {
        mpu_last_status = (uint8_t)status;
        mpu_read_fail_count++;
        return 0;
    }

    mpu_last_status = 0;
    mpu_read_ok_count++;

    Accel_X_RAW = (int16_t)((data[0] << 8) | data[1]);
    Accel_Y_RAW = (int16_t)((data[2] << 8) | data[3]);
    Accel_Z_RAW = (int16_t)((data[4] << 8) | data[5]);

    Gyro_X_RAW = (int16_t)((data[8] << 8) | data[9]);
    Gyro_Y_RAW = (int16_t)((data[10] << 8) | data[11]);
    Gyro_Z_RAW = (int16_t)((data[12] << 8) | data[13]);

    Ax = Accel_X_RAW / ACC_SCALE;
    Ay = Accel_Y_RAW / ACC_SCALE;
    Az = Accel_Z_RAW / ACC_SCALE;

    Gx = Gyro_X_RAW / GYRO_SCALE;
    Gy = Gyro_Y_RAW / GYRO_SCALE;
    Gz = Gyro_Z_RAW / GYRO_SCALE;

    return 1;
}

void MPU_Calibrate_Gyro(void)
{
    float sum = 0.0f;

    printf("Gyro calibrating...\r\n");

    for (int i = 0; i < 500; i++)
    {
        if (MPU_Read_All())
        {
            /*
              預設使用 Gy 當平衡角速度。
              如果後面你改用 Gx，這裡要改成 sum += Gx;
            */
            sum += Gy;
        }

        HAL_Delay(2);
    }

    gyro_offset = sum / 500.0f;

    printf("Gyro offset x100 = %ld\r\n", (long)(gyro_offset * 100));
}

void MPU_Update_Angle(void)
{
    if (!MPU_Read_All())
    {
        return;
    }

    /*
      預設：
      angle 使用 Ax / Az
      gyro_balance 使用 Gy

      如果你前後傾斜時 angle 幾乎不變，改成：
      acc_angle = atan2f(Ay, Az) * 57.2958f;

      如果你前後傾斜時 Gx 變化最大，改成：
      gyro_balance = Gx - gyro_offset;
    */
    float acc_angle_raw = atan2f(Ax, Az) * 57.2958f;

    float gyro_raw = Gy - gyro_offset;

    if (gyro_raw > GYRO_SPIKE_LIMIT) gyro_raw = GYRO_SPIKE_LIMIT;
    if (gyro_raw < -GYRO_SPIKE_LIMIT) gyro_raw = -GYRO_SPIKE_LIMIT;

    if (angle_filter_ready == 0)
    {
        acc_angle_lpf = acc_angle_raw;
        gyro_lpf = gyro_raw;
        angle = acc_angle_raw;
        angle_filter_ready = 1;
    }

    acc_angle_lpf = (1.0f - ACC_LPF_ALPHA) * acc_angle_lpf + ACC_LPF_ALPHA * acc_angle_raw;
    gyro_lpf = (1.0f - GYRO_LPF_ALPHA) * gyro_lpf + GYRO_LPF_ALPHA * gyro_raw;

    gyro_balance = gyro_lpf;

    angle = 0.98f * (angle + gyro_balance * CONTROL_DT) + 0.02f * acc_angle_lpf;
}

int32_t Limit_Int32(int32_t value, int32_t limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

int32_t Smooth_PWM(int32_t target, int32_t *last)
{
    if (target > *last + PWM_STEP_LIMIT)
    {
        *last += PWM_STEP_LIMIT;
    }
    else if (target < *last - PWM_STEP_LIMIT)
    {
        *last -= PWM_STEP_LIMIT;
    }
    else
    {
        *last = target;
    }

    return *last;
}

int32_t Get_Speed(void)
{
    /* GitHub 原本會讀 TIM2/TIM4 編碼器。
       你目前硬體沒有編碼器，所以先回傳 0，速度環功能保留但不介入。 */
    return 0;
}

int16_t Get_Direction_Error(void)
{
    /* GitHub 原本會讀循跡/方向誤差。
       你目前尚未接入 TCRT5000 方向誤差，所以先回傳 0，方向環功能保留但不介入。 */
    return 0;
}

static int32_t Angle_PID(float set, float nextPoint)
{
    /* 回到較常用的 D 項方向：Angle_Proc 之後等效為 Kp*error + Kd*gyro */
    float error = set - nextPoint;
    float P = Kp * error;
    float D = Kd * gyro_balance;

    return (int32_t)(P - D);
}

int32_t Angle_Proc(void)
{
    /* 對應 GitHub：return -angle_PID(AC_Set, pitch);
       這裡 set 使用按鈕記錄的 balance_offset，nextPoint 使用目前 angle。 */
    return (int32_t)(-MOTOR_DIR * Angle_PID(balance_offset, angle));
}

static int32_t Velocity_PID(int32_t set, int32_t nextPoint)
{
    /* 對應 GitHub 的增量式速度環 PID。
       目前 VC_PID_* 都是 0，所以 vc_pwm 預設為 0。 */
    float error;
    static float lastError = 0.0f, prevError = 0.0f;
    float P, I, D;
    static int32_t incpid = 0;

    error = (float)(set - nextPoint);

    P = VC_PID_P * (error - lastError);
    I = VC_PID_I * error;
    D = VC_PID_D * (error - 2.0f * lastError + prevError);

    if (I > 10.0f || I < -10.0f) I = 0.0f;

    prevError = lastError;
    lastError = error;

    incpid += (int32_t)(P + I + D);
    incpid = Limit_Int32(incpid, VC_OUT_MAX);

    return incpid;
}

int32_t Velocity_Proc(int32_t now_speed)
{
    /* 對應 GitHub：速度環每 VC_PERIOD 次更新一次，並做線性過渡 */
    static uint8_t count = 0;
    static int32_t VC_Out_Old = 0, VC_Out_New = 0;
    int32_t VC_Out;

    if (count >= VC_PERIOD)
    {
        count = 0;
        VC_Out_Old = VC_Out_New;
        VC_Out_New = Velocity_PID(velocity_set, now_speed);
        VC_Out_New = Limit_Int32(VC_Out_New, VC_OUT_MAX);
    }

    count++;
    VC_Out = VC_Out_Old + (VC_Out_New - VC_Out_Old) * count / VC_PERIOD;

    return VC_Out;
}

static int32_t Direction_PID(float dc_p, float dc_d)
{
    /* 對應 GitHub：P 使用方向誤差，D 使用 Z 軸角速度。
       目前方向誤差為 0 且 DC_PID_* 為 0，所以 dc_pwm 預設為 0。 */
    int32_t P, D;

    state = Get_Direction_Error();

    P = (int32_t)(dc_p * state / 25.0f);
    D = (int32_t)(dc_d * Gz / 100.0f);

    return P + D;
}

int32_t Direction_Proc(int32_t now_speed)
{
    /* 對應 GitHub：方向環每 DC_PERIOD 次更新一次 */
    static uint8_t count = 0;
    static int32_t DC_Out_Old = 0, DC_Out_New = 0;
    int32_t DC_Out;

    if (count >= DC_PERIOD)
    {
        count = 0;
        DC_Out_Old = DC_Out_New;
        DC_Out_New = Direction_PID(DC_PID_P, DC_PID_D);
        DC_Out_New = Limit_Int32(DC_Out_New, DC_OUT_MAX);
    }

    count++;
    DC_Out = DC_Out_Old + (DC_Out_New - DC_Out_Old) * count / DC_PERIOD;

    return DC_Out;
}

int32_t Get_PWM(void)
{
    /* 對應 GitHub get_pwm()：
       get_mpu(); speed=get_speed(); ac=angle; vc=velocity; dc=direction;
       left = ac - vc + dc; right = ac - vc - dc; */
    MPU_Update_Angle();

    speed = Get_Speed();
    ac_pwm = Angle_Proc();
    vc_pwm = Velocity_Proc(speed);
    dc_pwm = Direction_Proc(speed);

    left_pwm  = ac_pwm - vc_pwm + dc_pwm;
    right_pwm = ac_pwm - vc_pwm - dc_pwm;

    float error = angle - balance_offset;

    if (fabsf(error) < NEAR_BALANCE_DEG && fabsf(gyro_balance) < NEAR_GYRO_DPS)
    {
        left_pwm = 0;
        right_pwm = 0;
    }

    left_pwm = Limit_Int32(left_pwm, MOTOR_OUT_MAX);
    right_pwm = Limit_Int32(right_pwm, MOTOR_OUT_MAX);

    left_pwm = Smooth_PWM(left_pwm, &last_left_pwm);
    right_pwm = Smooth_PWM(right_pwm, &last_right_pwm);

    return 0;
}

void Motor_Proc(int32_t LEFT_MOTOR_OUT, int32_t RIGHT_MOTOR_OUT)
{
    /* 對應 GitHub motor_proc()：限制輸出，再依正負號控制左右馬達。 */
    LEFT_MOTOR_OUT = Limit_Int32(LEFT_MOTOR_OUT, MOTOR_OUT_MAX);
    RIGHT_MOTOR_OUT = Limit_Int32(RIGHT_MOTOR_OUT, MOTOR_OUT_MAX);

    if (LEFT_MOTOR_OUT == 0 && RIGHT_MOTOR_OUT == 0)
    {
#if USE_MOTOR_PULSE
        Motor_Brake();
#else
        Motor_Stop();
#endif
        return;
    }

#if USE_MOTOR_PULSE
    Motor_Set_Gated((int)LEFT_MOTOR_OUT, (int)RIGHT_MOTOR_OUT);
#else
    Motor_Set((int)LEFT_MOTOR_OUT, (int)RIGHT_MOTOR_OUT);
#endif
}

void Balance_Control(void)
{
    if (motor_enable == 0)
    {
        MPU_Update_Angle();
        Motor_Stop();

        if (Button_IsPressed())
        {
            balance_offset = angle;
            integral = 0.0f;
            gyro_lpf = 0.0f;
            acc_angle_lpf = angle;
            angle_filter_ready = 1;
            motor_enable = 1;

            /* 重置 GitHub-style 控制輸出 */
            speed = 0;
            ac_pwm = 0;
            vc_pwm = 0;
            dc_pwm = 0;
            left_pwm = 0;
            right_pwm = 0;
            last_left_pwm = 0;
            last_right_pwm = 0;

            printf("MOTOR ARMED, offset_x100=%ld\r\n",
                   (long)(balance_offset * 100));

            HAL_Delay(300);   // 簡單防彈跳
        }

        return;
    }

    Get_PWM();

    float error = angle - balance_offset;

    if (error > ANGLE_STOP_LIMIT || error < -ANGLE_STOP_LIMIT)
    {
        Motor_Stop();
        motor_enable = 0;
        integral = 0.0f;
        last_left_pwm = 0;
        last_right_pwm = 0;

        printf("ANGLE STOP, error_x100=%ld\r\n", (long)(error * 100));
        return;
    }

    Motor_Proc(left_pwm, right_pwm);

    static uint32_t last_debug_time = 0;

    if (HAL_GetTick() - last_debug_time >= 200)
    {
        last_debug_time = HAL_GetTick();

        printf("angle_x100=%ld offset_x100=%ld error_x100=%ld gyro_x100=%ld ac=%ld vc=%ld dc=%ld L=%ld R=%ld AX=%d AY=%d AZ=%d GX=%d GY=%d GZ=%d ok=%lu fail=%lu st=%u\r\n",
               (long)(angle * 100),
               (long)(balance_offset * 100),
               (long)(error * 100),
               (long)(gyro_balance * 100),
               (long)ac_pwm,
               (long)vc_pwm,
               (long)dc_pwm,
               (long)left_pwm,
               (long)right_pwm,
               Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW,
               Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW,
               (unsigned long)mpu_read_ok_count,
               (unsigned long)mpu_read_fail_count,
               mpu_last_status);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        control_flag = 1;
    }
}

void Motor_Set_Gated(int left_pwm, int right_pwm)
{
    static uint8_t motor_on = 0;
    static uint32_t last_switch_time = 0;

    uint32_t now = HAL_GetTick();

    if (left_pwm == 0 && right_pwm == 0)
    {
        Motor_Brake();
        motor_on = 0;
        last_switch_time = now;
        return;
    }

    if (motor_on)
    {
        Motor_Set(left_pwm, right_pwm);

        if (now - last_switch_time >= MOTOR_ON_MS)
        {
            Motor_Brake();
            motor_on = 0;
            last_switch_time = now;
        }
    }
    else
    {
        Motor_Brake();

        if (now - last_switch_time >= MOTOR_OFF_MS)
        {
            motor_on = 1;
            last_switch_time = now;
            Motor_Set(left_pwm, right_pwm);
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);

  Motor_Stop();

  MPU_Init();
  MPU_Calibrate_Gyro();


  //HAL_TIM_Base_Start_IT(&htim6);

  printf("System Start\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
      while (1)
      {
    	  static uint32_t last_control_time = 0;

    	     if (HAL_GetTick() - last_control_time >= 5)
    	     {
    	         last_control_time = HAL_GetTick();
    	         Balance_Control();
    	     }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00F12981;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 3;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN1_Pin|AIN2_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN1_Pin AIN2_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN1_Pin|AIN2_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : STBY_Pin */
  GPIO_InitStruct.Pin = STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STBY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

