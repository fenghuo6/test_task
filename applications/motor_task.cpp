#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

#include <cstring>

extern sp::DBus remote;

extern "C" {
volatile float live_motor1_angle = 0.0f;
volatile float live_motor1_speed = 0.0f;
volatile float live_motor1_position_out = 0.0f;
volatile float live_motor1_torque_out = 0.0f;
volatile float live_motor1_target_angle = 0.0f;
volatile float live_motor1_pid_error = 0.0f;
volatile float live_motor1_pid_pout = 0.0f;
volatile float live_motor1_pid_out = 0.0f;

volatile float live_motor2_angle = 0.0f;
volatile float live_motor2_speed = 0.0f;
volatile float live_motor2_position_out = 0.0f;
volatile float live_motor2_torque_out = 0.0f;
volatile float live_motor2_target_angle = 0.0f;
}

namespace
{
sp::CAN can1(&hcan1);
sp::RM_Motor motor6020_1(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor6020_2(2, sp::RM_Motors::GM6020);

constexpr float target_speed = 120.0f * 2.0f * 3.14159265f / 60.0f;
constexpr float max_torque = 0.1f;
constexpr float control_period = 0.005f;
sp::PID speed_pid_1(control_period, 0.3f, 0.0f, 0.0f, max_torque, 0.0f);
sp::PID speed_pid_2(control_period, 0.3f, 0.0f, 0.0f, max_torque, 0.0f);
}

extern "C" void motor_task(void const *argument)
{
  (void)argument;

  osDelay(500);
  can1.config();
  can1.start();

  while (true) {
    sp::DBusSwitchMode right_switch = sp::DBusSwitchMode::DOWN;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t now_ms = osKernelSysTick();
    const bool remote_alive = remote.is_alive(now_ms);
    if (remote_alive) {
      right_switch = remote.sw_r;
    }
    __set_PRIMASK(primask);

    const bool motors_alive =
      motor6020_1.is_alive(now_ms) && motor6020_2.is_alive(now_ms);
    float target_speed_1 = 0.0f;
    float target_speed_2 = 0.0f;
    float torque_command_1 = 0.0f;
    float torque_command_2 = 0.0f;

    if (remote_alive && right_switch == sp::DBusSwitchMode::UP) {
      speed_pid_1.clear();
      speed_pid_2.clear();
      torque_command_1 = 0.1f;
    } else if (remote_alive && motors_alive && right_switch == sp::DBusSwitchMode::MID) {
      target_speed_1 = target_speed;
      target_speed_2 = target_speed;
      speed_pid_1.calc(target_speed_1, motor6020_1.speed);
      speed_pid_2.calc(target_speed_2, motor6020_2.speed);
      torque_command_1 = speed_pid_1.out;
      torque_command_2 = speed_pid_2.out;
    } else {
      speed_pid_1.clear();
      speed_pid_2.clear();
    }

    motor6020_1.cmd(torque_command_1);
    motor6020_2.cmd(torque_command_2);

    live_motor1_angle = motor6020_1.angle;
    live_motor1_speed = motor6020_1.speed;
    live_motor1_position_out = target_speed_1;
    live_motor1_torque_out = torque_command_1;
    live_motor1_target_angle = motor6020_1.angle;
    live_motor1_pid_error = speed_pid_1.data.err[0];
    live_motor1_pid_pout = speed_pid_1.data.pout;
    live_motor1_pid_out = speed_pid_1.out;
    live_motor2_angle = motor6020_2.angle;
    live_motor2_speed = motor6020_2.speed;
    live_motor2_position_out = target_speed_2;
    live_motor2_torque_out = torque_command_2;
    live_motor2_target_angle = motor6020_2.angle;

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
