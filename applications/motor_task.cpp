#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"

#include <cstring>

extern sp::DBus remote;

namespace
{
sp::CAN can1(&hcan1);
sp::RM_Motor motor6020_1(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor6020_2(2, sp::RM_Motors::GM6020);
}

extern "C" void motor_task(void const *argument)
{
  (void)argument;

  osDelay(500);
  can1.config();
  can1.start();

  while (true) {
    sp::DBusSwitchMode right_switch;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t now_ms = osKernelSysTick();
    const bool remote_alive = remote.is_alive(now_ms);
    right_switch = remote.sw_r;
    __set_PRIMASK(primask);

    float command = 0.0f;
    if (remote_alive) {
      if (right_switch == sp::DBusSwitchMode::UP) {
        command = 0.1f;
      } else if (right_switch == sp::DBusSwitchMode::DOWN) {
        command = 0.05f;
      }
    }
    motor6020_1.cmd(command);
    motor6020_2.cmd(command);

    memset(can1.tx_data, 0, sizeof(can1.tx_data));
    motor6020_1.write(can1.tx_data);
    motor6020_2.write(can1.tx_data);
    can1.send(motor6020_1.tx_id);

    osDelay(5);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan != &hcan1) {
    return;
  }

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
    can1.recv(CAN_RX_FIFO0);
    const uint32_t stamp_ms = osKernelSysTick();
    if (can1.rx_id == motor6020_1.rx_id) {
      motor6020_1.read(can1.rx_data, stamp_ms);
    }
    if (can1.rx_id == motor6020_2.rx_id) {
      motor6020_2.read(can1.rx_data, stamp_ms);
    }
  }
}
