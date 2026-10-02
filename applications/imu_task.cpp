#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"
#include "io/plotter/plotter.hpp"
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

        // 使用调试(f5)查看bmi088和imu内部变量的变化
        print_cnt++;
        if (print_cnt >= 50) // 50*1ms = 50ms，20Hz输出波形，避免串口压力过大
        {
            print_cnt = 0;
            // 顺序：roll,pitch,yaw, ax,ay,az, gx,gy,gz 共9路
            plotter.plot(
                imu.roll, imu.pitch, imu.yaw,
                bmi088.acc[0], bmi088.acc[1], bmi088.acc[2],
                bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2]);
        }
        osDelay(1);
    }
}