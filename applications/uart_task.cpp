#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"

// C板
sp::DBus remote(&huart3);
sp::RM_Motor motor6020_1(1, sp::RM_Motors::GM6020);  // 一个电机ID为1, 电流控制模式的6020
sp::RM_Motor motor6020_2(2, sp::RM_Motors::GM6020);  // 一个电机ID为2, 电流控制模式的6020

void motor6020_1_SetTorque(float torque);
void motor6020_2_SetTorque(float torque);
void motor6020_1_SetTargetAngle(float angle);
void motor6020_2_SetTargetAngle(float angle);
float motor6020_1_GetAngle(void);
float motor6020_2_GetAngle(void);

const float RESET_ANGLE = 0.0f; // 复位基准角度：C板箭头基准

extern "C" void motor_control_task(void *arg)
{
  sp::DBusSwitchMode sw_r;

  while (1)
  {
    // 安全读取DBus遥控器数据
    __disable_irq();
    sw_r = remote.sw_r;
    __enable_irq();

    // ========== 只做失能判断 ==========
    if (sw_r == sp::DBusSwitchMode::DOWN) // 右拨杆下档
    {
      // 失能：力矩置0，电机自由转动无力
      motor6020_1_SetTorque(0.0f);
      motor6020_2_SetTorque(0.0f);
    }
    else if (sw_r == sp::DBusSwitchMode::UP)
    {
      // =========复位模式：右拨上档，对齐C板基准箭头=========
      // 设置两台电机目标角度为基准角，电机位置闭环转到该角度
      motor6020_1_SetTargetAngle(RESET_ANGLE);
      motor6020_2_SetTargetAngle(RESET_ANGLE);
    }
    else if (sw_r == sp::DBusSwitchMode::MID)
    {
      // 中档：预留姿态联动，暂时先置0力矩，后面再写
      motor6020_1_SetTorque(0.0f);
      motor6020_2_SetTorque(0.0f);
    }

    osDelay(2); // 控制周期
  }
}

extern "C" void uart_task()
{
  remote.request();

  while (true)
  {
    // 使用调试(f5)查看remote内部变量的变化
    osDelay(10);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3)
  {
    remote.update(Size, stamp_ms);
    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    remote.request();
  }
}