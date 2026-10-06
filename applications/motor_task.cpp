#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor_plot_data.hpp"
#include "tools/pid/pid.hpp"

#include <cstring>

extern sp::DBus remote;

MotorPlotData motor_plot_data;

namespace
{
sp::CAN can1(&hcan1);
sp::RM_Motor motor6020_1(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor6020_2(2, sp::RM_Motors::GM6020);

constexpr float control_period = 0.005f;
constexpr float motor1_turn_angle_rad = 2.0f * 3.14159265f;
constexpr float motor1_position_kp = 2.0f;
constexpr float motor1_max_speed_rpm = 120.0f;
constexpr float motor1_max_speed_rad_s =
  motor1_max_speed_rpm * 2.0f * 3.14159265f / 60.0f;
constexpr float motor1_speed_kp = 0.05f;
constexpr float motor1_max_torque_nm = 0.1f;
constexpr float motor2_target_speed_rpm = 120.0f;
constexpr float motor2_target_speed_rad_s =
  motor2_target_speed_rpm * 2.0f * 3.14159265f / 60.0f;
constexpr float motor2_speed_kp = 0.3f;
constexpr float motor2_max_torque_nm = 0.1f;

sp::PID position_pid_1(
  control_period, motor1_position_kp, 0.0f, 0.0f, motor1_max_speed_rad_s, 0.0f);
sp::PID position_pid_2(
  control_period, motor1_position_kp, 0.0f, 0.0f, motor1_max_speed_rad_s, 0.0f);
sp::PID speed_pid_1(
  control_period, motor1_speed_kp, 0.2f, 0.0f, motor1_max_torque_nm, 0.03f);
sp::PID speed_pid_2(
  control_period, motor2_speed_kp, 0.0f, 0.0f, motor2_max_torque_nm, 0.0f);
sp::PID reset_speed_pid_2(
  control_period, motor1_speed_kp, 0.2f, 0.0f, motor2_max_torque_nm, 0.03f);
bool mid_active = false;
bool reset_active = false;
bool home_reference_ready = false;
float motor1_home_angle = 0.0f;
float motor2_home_angle = 0.0f;
float motor1_goal_angle = 0.0f;
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
    if (!home_reference_ready && motors_alive) {
      motor1_home_angle = motor6020_1.angle;
      motor2_home_angle = motor6020_2.angle;
      home_reference_ready = true;
    }

    float target_speed_1 = 0.0f;
    float target_speed_2 = 0.0f;
    float torque_command_1 = 0.0f;
    float torque_command_2 = 0.0f;

    if (
      remote_alive && motors_alive && home_reference_ready &&
      right_switch == sp::DBusSwitchMode::UP) {
      mid_active = false;
      if (!reset_active) {
        position_pid_1.clear();
        position_pid_2.clear();
        speed_pid_1.clear();
        speed_pid_2.clear();
        reset_speed_pid_2.clear();
        reset_active = true;
      }

      motor1_goal_angle = motor1_home_angle;
      position_pid_1.calc(motor1_home_angle, motor6020_1.angle);
      position_pid_2.calc(motor2_home_angle, motor6020_2.angle);
      target_speed_1 = position_pid_1.out;
      target_speed_2 = position_pid_2.out;
      speed_pid_1.calc(target_speed_1, motor6020_1.speed);
      reset_speed_pid_2.calc(target_speed_2, motor6020_2.speed);
      torque_command_1 = speed_pid_1.out;
      torque_command_2 = reset_speed_pid_2.out;
    } else if (
      remote_alive && motors_alive && right_switch == sp::DBusSwitchMode::MID) {
      reset_active = false;
      if (!mid_active) {
        motor1_goal_angle = motor6020_1.angle + motor1_turn_angle_rad;
        position_pid_1.clear();
        position_pid_2.clear();
        speed_pid_1.clear();
        speed_pid_2.clear();
        reset_speed_pid_2.clear();
        mid_active = true;
      }

      position_pid_1.calc(motor1_goal_angle, motor6020_1.angle);
      target_speed_1 = position_pid_1.out;
      target_speed_2 = motor2_target_speed_rad_s;
      speed_pid_1.calc(target_speed_1, motor6020_1.speed);
      speed_pid_2.calc(target_speed_2, motor6020_2.speed);
      torque_command_1 = speed_pid_1.out;
      torque_command_2 = speed_pid_2.out;
    } else {
      mid_active = false;
      reset_active = false;
      position_pid_1.clear();
      position_pid_2.clear();
      speed_pid_1.clear();
      speed_pid_2.clear();
      reset_speed_pid_2.clear();
    }

    motor6020_1.cmd(torque_command_1);
    motor6020_2.cmd(torque_command_2);

    motor_plot_data.motor1_angle = motor6020_1.angle;
    motor_plot_data.motor1_target_angle = motor1_goal_angle;
    motor_plot_data.motor1_position_error = position_pid_1.data.err[0];
    motor_plot_data.motor1_target_speed = target_speed_1;
    motor_plot_data.motor1_speed = motor6020_1.speed;
    motor_plot_data.motor1_torque_x100 = torque_command_1 * 100.0f;
    motor_plot_data.motor2_target_speed = target_speed_2;
    motor_plot_data.motor2_speed = motor6020_2.speed;
    motor_plot_data.motor2_torque_x100 = torque_command_2 * 100.0f;

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
