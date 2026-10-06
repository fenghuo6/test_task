#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor_plot_data.hpp"
#include "tools/math_tools/math_tools.hpp"
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
constexpr float motor1_position_kp = 0.2f;
constexpr float motor2_position_kp = 0.9f;
constexpr float reset_position_kp = 2.0f;
constexpr float motor1_max_speed_rpm = 120.0f;
constexpr float motor1_max_speed_rad_s =
  motor1_max_speed_rpm * 2.0f * 3.14159265f / 60.0f;
constexpr float motor1_speed_kp = 0.05f;
constexpr float motor1_max_torque_nm = 0.1f;
constexpr float motor2_max_torque_nm = 0.1f;
constexpr float linkage_max_torque_nm = 0.1f;
constexpr float reset_max_torque_nm = 0.15f;
constexpr float linkage_manual_speed_threshold_rad_s = 0.05f;
constexpr float linkage_manual_error_threshold_rad = 0.003f;

sp::PID position_pid_1(
  control_period, motor1_position_kp, 0.0f, 0.0f, motor1_max_speed_rad_s, 0.0f);
sp::PID position_pid_2(
  control_period, motor2_position_kp, 0.0f, 0.0f, motor1_max_speed_rad_s, 0.0f);
sp::PID reset_position_pid_1(
  control_period, reset_position_kp, 0.0f, 0.0f, motor1_max_speed_rad_s, 0.0f);
sp::PID reset_position_pid_2(
  control_period, reset_position_kp, 0.0f, 0.0f, motor1_max_speed_rad_s, 0.0f);
sp::PID speed_pid_1(
  control_period, 0.04f, 0.0f, 0.0f, linkage_max_torque_nm, 0.0f);
sp::PID speed_pid_2(
  control_period, 0.04f, 0.0f, 0.0f, linkage_max_torque_nm, 0.0f);
sp::PID reverse_linkage_speed_pid_1(
  control_period, 0.03f, 0.0f, 0.0f, linkage_max_torque_nm, 0.0f);
sp::PID reverse_linkage_speed_pid_2(
  control_period, 0.03f, 0.0f, 0.0f, linkage_max_torque_nm, 0.0f);
sp::PID reset_speed_pid_1(
  control_period, 0.03f, 0.2f, 0.0f, reset_max_torque_nm, 0.08f);
sp::PID reset_speed_pid_2(
  control_period, 0.03f, 0.15f, 0.0f, reset_max_torque_nm, 0.1f);
sp::AngleUnwrapper yaw_unwrapper;
bool yaw_initialized = false;
float previous_yaw = 0.0f;
float filtered_yaw_rate = 0.0f;
bool linkage_active = false;
bool reset_active = false;
bool home_reference_ready = false;
uint8_t manual_motor_id = 0;
float motor1_home_angle = 0.0f;
float motor2_home_angle = 0.0f;
float motor1_goal_angle = 0.0f;
float linkage_motor1_reference = 0.0f;
float linkage_motor2_reference = 0.0f;
float linkage_yaw_reference = 0.0f;
float linkage_motor2_ratio = 0.5f;
sp::DBusSwitchMode linkage_left_switch = sp::DBusSwitchMode::DOWN;
bool linkage_manual_adjustment_active = false;

float get_motor2_linkage_ratio(sp::DBusSwitchMode left_switch)
{
  switch (left_switch) {
    case sp::DBusSwitchMode::DOWN:
      return 0.5f;
    case sp::DBusSwitchMode::MID:
      return -1.0f;
    case sp::DBusSwitchMode::UP:
      return 3.0f;
  }

  return 0.5f;
}
}

extern "C" void motor_task(void const *argument)
{
  (void)argument;

  osDelay(500);
  can1.config();
  can1.start();

  while (true) {
    sp::DBusSwitchMode right_switch = sp::DBusSwitchMode::DOWN;
    sp::DBusSwitchMode left_switch = sp::DBusSwitchMode::DOWN;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t now_ms = osKernelSysTick();
    const bool remote_alive = remote.is_alive(now_ms);
    if (remote_alive) {
      right_switch = remote.sw_r;
      left_switch = remote.sw_l;
    }
    __set_PRIMASK(primask);

    const float yaw = yaw_unwrapper.update(motor_plot_data.yaw);
    const float yaw_delta = yaw_initialized ? yaw - previous_yaw : 0.0f;
    const float yaw_rate = yaw_delta / control_period;
    filtered_yaw_rate += 0.25f * (yaw_rate - filtered_yaw_rate);
    previous_yaw = yaw;
    yaw_initialized = true;

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
      linkage_active = false;
      if (!reset_active) {
        reset_position_pid_1.clear();
        reset_position_pid_2.clear();
        reset_speed_pid_1.clear();
        reset_speed_pid_2.clear();
        reset_active = true;
      }

      motor1_goal_angle = motor1_home_angle;
      reset_position_pid_1.calc(motor1_home_angle, motor6020_1.angle);
      reset_position_pid_2.calc(motor2_home_angle, motor6020_2.angle);
      target_speed_1 = reset_position_pid_1.out;
      target_speed_2 = reset_position_pid_2.out;
      reset_speed_pid_1.calc(target_speed_1, motor6020_1.speed);
      reset_speed_pid_2.calc(target_speed_2, motor6020_2.speed);
      torque_command_1 = reset_speed_pid_1.out;
      torque_command_2 = reset_speed_pid_2.out;
    } else if (
      remote_alive && motors_alive && right_switch == sp::DBusSwitchMode::MID) {
      reset_active = false;
      if (!linkage_active) {
        linkage_motor1_reference = motor6020_1.angle;
        linkage_motor2_reference = motor6020_2.angle;
        linkage_yaw_reference = yaw;
        linkage_left_switch = left_switch;
        linkage_motor2_ratio = get_motor2_linkage_ratio(left_switch);
        position_pid_1.clear();
        position_pid_2.clear();
        speed_pid_1.clear();
        speed_pid_2.clear();
        reverse_linkage_speed_pid_1.clear();
        reverse_linkage_speed_pid_2.clear();
        linkage_manual_adjustment_active = false;
        manual_motor_id = 0;
        linkage_active = true;
      } else if (left_switch != linkage_left_switch) {
        linkage_motor1_reference = motor6020_1.angle;
        linkage_motor2_reference = motor6020_2.angle;
        linkage_yaw_reference = yaw;
        linkage_left_switch = left_switch;
        linkage_motor2_ratio = get_motor2_linkage_ratio(left_switch);
        position_pid_1.clear();
        position_pid_2.clear();
        speed_pid_1.clear();
        speed_pid_2.clear();
        reverse_linkage_speed_pid_1.clear();
        reverse_linkage_speed_pid_2.clear();
        linkage_manual_adjustment_active = false;
        manual_motor_id = 0;
      }
      sp::PID & linkage_pid_1 =
        linkage_motor2_ratio == -1.0f ? reverse_linkage_speed_pid_1 : speed_pid_1;
      sp::PID & linkage_pid_2 =
        linkage_motor2_ratio == -1.0f ? reverse_linkage_speed_pid_2 : speed_pid_2;

      float target_angle_1 = linkage_motor1_reference + (yaw - linkage_yaw_reference);
      float target_angle_2 =
        linkage_motor2_reference + linkage_motor2_ratio * (yaw - linkage_yaw_reference);
      const float motor1_error = target_angle_1 - motor6020_1.angle;
      const float motor2_error = target_angle_2 - motor6020_2.angle;
      const float motor1_manual_speed = motor6020_1.speed - yaw_rate;
      const float motor2_manual_speed = motor6020_2.speed - linkage_motor2_ratio * yaw_rate;
      const bool yaw_is_stationary = std::abs(filtered_yaw_rate) < 0.05f;
      const bool motor1_manually_moved =
        yaw_is_stationary &&
        std::abs(motor1_manual_speed) > linkage_manual_speed_threshold_rad_s &&
        std::abs(motor1_error) > linkage_manual_error_threshold_rad &&
        motor6020_1.speed * motor1_error < 0.0f;
      const bool motor2_manually_moved =
        yaw_is_stationary &&
        std::abs(motor2_manual_speed) > linkage_manual_speed_threshold_rad_s &&
        std::abs(motor2_error) > linkage_manual_error_threshold_rad &&
        motor6020_2.speed * motor2_error < 0.0f;

      if (
        manual_motor_id != 0 &&
        std::abs(yaw_rate) < linkage_manual_speed_threshold_rad_s &&
        std::abs(motor6020_1.speed) <= linkage_manual_speed_threshold_rad_s &&
        std::abs(motor6020_2.speed) <= linkage_manual_speed_threshold_rad_s) {
        linkage_motor1_reference = motor6020_1.angle;
        linkage_motor2_reference = motor6020_2.angle;
        linkage_yaw_reference = yaw;
        position_pid_1.clear();
        position_pid_2.clear();
        speed_pid_1.clear();
        speed_pid_2.clear();
        reverse_linkage_speed_pid_1.clear();
        reverse_linkage_speed_pid_2.clear();
        linkage_manual_adjustment_active = false;
        manual_motor_id = 0;
      }

      if (manual_motor_id == 0 && motor1_manually_moved) {
        manual_motor_id = 1;
      } else if (manual_motor_id == 0 && motor2_manually_moved) {
        manual_motor_id = 2;
      }

      if (manual_motor_id == 1) {
        linkage_yaw_reference = yaw - (motor6020_1.angle - linkage_motor1_reference);
        target_angle_1 = motor6020_1.angle;
        target_angle_2 =
          linkage_motor2_reference + linkage_motor2_ratio * (yaw - linkage_yaw_reference);
        target_speed_2 = linkage_motor2_ratio * motor1_manual_speed;
        position_pid_2.clear();
        linkage_pid_1.clear();
        linkage_manual_adjustment_active = true;
        torque_command_1 = 0.0f;
        position_pid_1.clear();
      } else if (manual_motor_id == 2) {
        linkage_yaw_reference =
          yaw - (motor6020_2.angle - linkage_motor2_reference) / linkage_motor2_ratio;
        target_angle_1 = linkage_motor1_reference + (yaw - linkage_yaw_reference);
        target_angle_2 = motor6020_2.angle;
        target_speed_1 = motor2_manual_speed / linkage_motor2_ratio;
        position_pid_2.clear();
        linkage_pid_2.clear();
        linkage_manual_adjustment_active = true;
        torque_command_2 = 0.0f;
      }

      const float linkage_yaw_delta = yaw - linkage_yaw_reference;
      if (manual_motor_id == 0) {
        target_angle_1 = linkage_motor1_reference + linkage_yaw_delta;
        target_angle_2 = linkage_motor2_reference + linkage_motor2_ratio * linkage_yaw_delta;
      }
      motor1_goal_angle = target_angle_1;
      if (manual_motor_id == 0) {
        position_pid_1.calc(target_angle_1, motor6020_1.angle);
        position_pid_2.calc(target_angle_2, motor6020_2.angle);
        target_speed_1 = position_pid_1.out;
        target_speed_2 = position_pid_2.out;
        target_speed_1 =
          sp::limit_max(target_speed_1 + yaw_rate, motor1_max_speed_rad_s);
        target_speed_2 = sp::limit_max(
          target_speed_2 + linkage_motor2_ratio * yaw_rate, motor1_max_speed_rad_s);
      } else if (manual_motor_id == 1) {
        position_pid_2.calc(target_angle_2, motor6020_2.angle);
      } else {
        position_pid_1.calc(target_angle_1, motor6020_1.angle);
      }
      if (manual_motor_id != 1) {
        linkage_pid_1.calc(target_speed_1, motor6020_1.speed);
        torque_command_1 = linkage_pid_1.out;
      }
      if (manual_motor_id != 2) {
        linkage_pid_2.calc(target_speed_2, motor6020_2.speed);
        torque_command_2 = linkage_pid_2.out;
      }
    } else {
      linkage_active = false;
      reset_active = false;
      position_pid_1.clear();
      position_pid_2.clear();
      speed_pid_1.clear();
      speed_pid_2.clear();
      reverse_linkage_speed_pid_1.clear();
      reverse_linkage_speed_pid_2.clear();
      linkage_manual_adjustment_active = false;
      manual_motor_id = 0;
    }

    motor6020_1.cmd(torque_command_1);
    motor6020_2.cmd(torque_command_2);

    motor_plot_data.motor1_angle = motor6020_1.angle;
    motor_plot_data.motor1_target_angle = motor1_goal_angle;
    motor_plot_data.motor1_position_error = motor1_goal_angle - motor6020_1.angle;
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
