#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"
#include "io/can/can.hpp"
#include <cstring>
#include "tools/mahony/mahony.hpp"

extern sp::Mahony imu;

// C板
sp::DBus remote(&huart3);
sp::RM_Motor motor6020_1(1, sp::RM_Motors::GM6020); // 一个电机ID为1, 电流控制模式的6020
sp::RM_Motor motor6020_2(2, sp::RM_Motors::GM6020); // 一个电机ID为2, 电流控制模式的6020

sp::PID pid6020_1(0.002f, 8.0f, 0.0f, 0.3f, sp::GM6020_MAX_TORQUE, 0.5f, 1.0f, true, false);
sp::PID pid6020_2(0.002f, 8.0f, 0.0f, 0.3f, sp::GM6020_MAX_TORQUE, 0.5f, 1.0f, true, false);

sp::CAN can1(&hcan1);
const float RESET_ANGLE = 0.0f; // 复位基准角度：C板箭头基准

static float yaw_raw_last = 0.0f; // 上一帧原始imu.yaw [-π,π]
static float yaw_unwrap = 0.0f;   // 解卷绕后连续yaw(rad)，无±π跳变

static float yaw_zero = 0.0f;             // 联动参考：解卷绕yaw零点
static float encA_zero = 0.0f;            // 联动参考：A电机零点角度
static float encB_zero = 0.0f;            // 联动参考：B电机零点角度
const float ENC_NOISE_THRESHOLD = 0.002f; // 弧度，A电机变化阈值，过滤编码器噪声

/**
 * @brief yaw角度解卷绕函数，把[-π,π]跳变转为连续角度
 * @param raw_yaw imu.yaw原始值，范围[-M_PI,M_PI]
 * @return float 连续无跳变yaw(rad)
 */
static float unwrap_yaw(float raw_yaw)
{
  const float PI = 3.1415926535f;
  float delta = raw_yaw - yaw_raw_last;

  // 检测正负π跳变，补偿2π
  if (delta > PI)
    delta -= 2.0f * PI;
  else if (delta < -PI)
    delta += 2.0f * PI;

  yaw_unwrap += delta;
  yaw_raw_last = raw_yaw;
  return yaw_unwrap;
}

static bool link_ready = false;
static uint32_t boot_wait_tick = 0U;

extern "C" void motor_control_task(void *arg)
{
  sp::DBusSwitchMode sw_r; // 右拨杆
  sp::DBusSwitchMode sw_l; // 左拨杆

  float yaw_now;
  float encA_now, encB_now;
  float delta_yaw;
  float ratio_B;

  boot_wait_tick = osKernelSysTick();

  while (1)
  {
    // 安全读取DBus遥控器数据
    __disable_irq();
    sw_r = remote.sw_r;
    sw_l = remote.sw_l;
    __enable_irq();

    if (!link_ready && (osKernelSysTick() - boot_wait_tick > 800U))
    {
      if (motor6020_1.is_open() && motor6020_2.is_open())
      {
        yaw_raw_last = imu.yaw;
        yaw_unwrap = imu.yaw;
        yaw_zero = unwrap_yaw(imu.yaw);
        encA_zero = motor6020_1.angle;
        encB_zero = motor6020_2.angle;
        link_ready = true;
      }
    }
    // 失能判断
    if (sw_r == sp::DBusSwitchMode::DOWN) // 右拨杆下档
    {
      // 失能：力矩置0，电机自由转动无力
      motor6020_1.cmd(0.0f);
      motor6020_2.cmd(0.0f);
      pid6020_1.clear();
      pid6020_2.clear();

      link_ready = false;
      yaw_zero = 0.0f;
      encA_zero = 0.0f;
      encB_zero = 0.0f;
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

      // 采集此刻状态，初始化联动三组零点
      yaw_raw_last = imu.yaw;
      yaw_unwrap = imu.yaw;
      yaw_zero = unwrap_yaw(imu.yaw);
      encA_zero = motor6020_1.angle;
      encB_zero = motor6020_2.angle;
      link_ready = true;
    }

    else if (sw_r == sp::DBusSwitchMode::MID)
    {
      if (!link_ready)
      {
        motor6020_1.cmd(0.0f);
        motor6020_2.cmd(0.0f);
        pid6020_1.clear();
        pid6020_2.clear();
      }
      else
      {
        // 1.左拨杆选择B电机联动比例 C板:B电机
        if (sw_l == sp::DBusSwitchMode::DOWN)
        {
          ratio_B = 0.5f; // 1 : 0.5
        }
        else if (sw_l == sp::DBusSwitchMode::MID)
        {
          ratio_B = -1.0f; // 1 : -1 反向
        }
        else // sw_l == UP
        {
          ratio_B = 3.0f; // 1 : 3
        }
        // 2.读取IMU yaw，执行解卷绕，获得连续yaw角度
        yaw_now = unwrap_yaw(imu.yaw);

        // 3.读取两台电机当前输出轴角度(rad，RM_Motor多圈连续)
        encA_now = motor6020_1.angle;
        encB_now = motor6020_2.angle;

        // 4.【核心逻辑】手动转动A电机：刷新整套联动零点
        float delta_encA = encA_now - encA_zero;
        if (fabsf(delta_encA) > ENC_NOISE_THRESHOLD)
        {
          // A电机被手动掰动，更新全部联动参考零点
          yaw_zero = yaw_now;
          encA_zero = encA_now;
          encB_zero = encB_now;
          // 清空PID积分，防止手动掰电机时积分累积
          pid6020_1.clear();
          pid6020_2.clear();
        }
        // 5.计算C板yaw相对零点的偏移量
        delta_yaw = yaw_now - yaw_zero;

        // 6.计算两台电机目标角度
        float targetA = encA_zero + delta_yaw;           // A电机1:1跟随C板yaw偏移
        float targetB = encB_zero + ratio_B * delta_yaw; // B电机按左拨杆比例跟随

        // 7.位置PID闭环计算输出力矩
        pid6020_1.calc(targetA, encA_now);
        pid6020_2.calc(targetB, encB_now);

        // 8.下发力矩指令
        motor6020_1.cmd(pid6020_1.out);
        motor6020_2.cmd(pid6020_2.out);
      }
      osDelay(2); // 控制周期
    }
  }
}
extern "C" void uart_task(void *arg)
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
  while (1)
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