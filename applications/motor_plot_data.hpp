#ifndef APPLICATIONS_MOTOR_PLOT_DATA_HPP
#define APPLICATIONS_MOTOR_PLOT_DATA_HPP

struct MotorPlotData
{
  float motor1_angle = 0.0f;
  float motor1_target_angle = 0.0f;
  float motor1_position_error = 0.0f;
  float motor1_target_speed = 0.0f;
  float motor1_speed = 0.0f;
  float motor1_torque_x100 = 0.0f;
  float motor2_target_speed = 0.0f;
  float motor2_speed = 0.0f;
  float motor2_torque_x100 = 0.0f;
};

extern MotorPlotData motor_plot_data;

#endif
