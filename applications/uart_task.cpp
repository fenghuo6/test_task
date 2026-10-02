#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"
#include "io/can/can.hpp"
#include "cstring"
// C板
sp::DBus remote(&huart3);
sp::RM_Motor motor6020_1(1, sp::RM_Motors::GM6020); // 一个电机ID为1, 电流控制模式的6020
sp::RM_Motor motor6020_2(2, sp::RM_Motors::GM6020); // 一个电机ID为2, 电流控制模式的6020

sp::PID pid6020_1(0.002f, 8.0f, 0.0f, 0.3f, sp::GM6020_MAX_TORQUE, 0.5f, 1.0f, true, false);
sp::PID pid6020_2(0.002f, 8.0f, 0.0f, 0.3f, sp::GM6020_MAX_TORQUE, 0.5f, 1.0f, true, false);

sp::CAN can1(&hcan1);
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
      motor6020_1.cmd(0.0f);
      motor6020_2.cmd(0.0f);
      pid6020_1.clear();
      pid6020_2.clear();
    }
    else if (sw_r == sp::DBusSwitchMode::UP)
    {
      // =========复位模式：右拨上档，对齐C板基准箭头=========

      float ang1 = motor6020_1.angle;
      float ang2 = motor6020_2.angle;

      pid6020_1.calc(RESET_ANGLE, ang1);
      pid6020_2.calc(RESET_ANGLE, ang2);

      motor6020_1.cmd(pid6020_1.out);
      motor6020_2.cmd(pid6020_2.out);
    }

    else if (sw_r == sp::DBusSwitchMode::MID)
    {
      // 中档：预留姿态联动，暂时先置0力矩，后面再写
      motor6020_1.cmd(0.0f);
      motor6020_2.cmd(0.0f);
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

extern "C" void can_send_task(void *arg)
{
  while(1)
  {
    memset(can1.tx_data, 0, sizeof(can1.tx_data));
    motor6020_1.write(can1.tx_data);
    motor6020_2.write(can1.tx_data);

    can1.send(motor6020_1.tx_id);
    osDelay(5);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

  can1.recv(CAN_RX_FIFO0);
  uint32_t tick = osKernelSysTick();

  if (motor6020_1.rx_id == can1.rx_id)
  {
    motor6020_1.read(can1.rx_data, tick);
  }
  if (motor6020_2.rx_id == can1.rx_id)
  {
    motor6020_2.read(can1.rx_data, tick);
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