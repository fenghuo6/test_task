#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"
#include "io/plotter/plotter.hpp"
#include "motor_plot_data.hpp"

const float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

// C板
sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);

// 达妙
// sp::BMI088 bmi088(&hspi2, GPIOC, GPIO_PIN_0, GPIOC, GPIO_PIN_3, r_ab);

sp::Mahony imu(1e-3f);
sp::Plotter plotter(&huart1, true);
extern "C" void imu_task()
{
    bmi088.init();
    static uint16_t print_cnt = 0;
    while (true)
    {
        bmi088.update();
        imu.update(bmi088.acc, bmi088.gyro);
        motor_plot_data.yaw = imu.yaw;

        print_cnt++;
        if (print_cnt >= 50) // 50*1ms = 50ms，20Hz输出波形
        {
            print_cnt = 0;
            // 顺序：9路IMU；电机1角度/目标角/位置误差/目标速度/实际速度/力矩×100；
            // 电机2目标速度/实际速度/力矩×100，共18路
            plotter.plot(
                imu.roll, imu.pitch, imu.yaw,
                bmi088.acc[0], bmi088.acc[1], bmi088.acc[2],
                bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2],
                motor_plot_data.motor1_angle,
                motor_plot_data.motor1_target_angle,
                motor_plot_data.motor1_position_error,
                motor_plot_data.motor1_target_speed,
                motor_plot_data.motor1_speed,
                motor_plot_data.motor1_torque_x100,
                motor_plot_data.motor2_target_speed,
                motor_plot_data.motor2_speed,
                motor_plot_data.motor2_torque_x100);
        }
        osDelay(1);
    }
}