#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"

// C板
sp::DBus remote(&huart3);

void motorA_SetTorque(float torque);
void motorB_SetTorque(float torque);

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
    else
    {
      // 右拨不是下档，先保持力矩0（后面再加联动逻辑）
      motorA_SetTorque(0.0f);
      motorB_SetTorque(0.0f);
    }

    osDelay(2); // 控制周期，2ms足够
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