#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);

extern "C" void led_task(void const *argument)
{
  (void)argument;
  led.start();

  while (true)
  {
    // 红灯亮
    led.set(0.1f, 0, 0);
    osDelay(200);
    led.set(0, 0, 0);
    osDelay(100);

    // 绿灯亮
    led.set(0, 0.1f, 0);
    osDelay(200);
    led.set(0, 0, 0);
    osDelay(100);

    // 蓝灯亮
    led.set(0, 0, 0.1f);
    osDelay(200);
    led.set(0, 0, 0);
    osDelay(100);
  }
}
