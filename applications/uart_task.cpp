#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"

// C板
sp::DBus remote(&huart3);

void motorA_SetTorque(float torque);
void motorB_SetTorque(float torque);
void motorA_SetTargetAngle(float angle);
void motorB_SetTargetAngle(float angle);
float motorA_GetAngle(void);
float motorB_GetAngle(void);

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
      motorA_SetTorque(0.0f);
      motorB_SetTorque(0.0f);
    }
    else if (sw_r == sp::DBusSwitchMode::UP)
    {
      // =========复位模式：右拨上档，对齐C板基准箭头=========
      // 设置两台电机目标角度为基准角，电机位置闭环转到该角度
      motorA_SetTargetAngle(RESET_ANGLE);
      motorB_SetTargetAngle(RESET_ANGLE);
    }
    else if (sw_r == sp::DBusSwitchMode::MID)
    {
      // 中档：预留姿态联动，暂时先置0力矩，后面再写
      motorA_SetTorque(0.0f);
      motorB_SetTorque(0.0f);
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