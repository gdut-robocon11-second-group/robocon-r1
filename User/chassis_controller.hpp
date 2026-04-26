#ifndef CHASSIS_CONTROLLER_HPP
#define CHASSIS_CONTROLLER_HPP

#include "bsp_atk_ms901m.hpp"
#include "bsp_motor.hpp"
#include "bsp_timer.hpp"
#include "chassis_kinematics.hpp"
#include "clock.hpp"
#include "mutex.hpp"
#include "pid_controller.hpp"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_uart.h"
#include "thread.hpp"
#include "uncopyable.hpp"
#include <mutex>

// extern "C" int __io_putchar(int ch);

// void print_float(float value) {
//   if (value >= 0) {
//     std::printf("%d.%d", static_cast<int>(value), static_cast<int>((value -
//     static_cast<int>(value)) * 1000));
//   } else {
//     std::printf("-%d.%d", static_cast<int>(-value), static_cast<int>((-value
//     - static_cast<int>(-value)) * 1000));
//   }
// }

namespace gdut {

class chassis_controller : private uncopyable {
public:
  chassis_controller() = default;

  chassis_controller(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2,
                     TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4,
                     TIM_HandleTypeDef *htim5, TIM_HandleTypeDef *htim9)
      : m_htim1(htim1), m_htim2(htim2), m_htim3(htim3), m_htim4(htim4),
        m_htim5(htim5), m_htim9(htim9) {}

  ~chassis_controller() = default;

  void set_parameters(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2,
                      TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4,
                      TIM_HandleTypeDef *htim5, TIM_HandleTypeDef *htim9,
                      UART_HandleTypeDef *huart2) {
    m_htim1 = htim1;
    m_htim2 = htim2;
    m_htim3 = htim3;
    m_htim4 = htim4;
    m_htim5 = htim5;
    m_htim9 = htim9;

    m_tim1 = timer{m_htim1};
    m_tim2 = timer{m_htim2};
    m_tim3 = timer{m_htim3};
    m_tim4 = timer{m_htim4};
    m_tim5 = timer{m_htim5};
    m_tim9 = timer{m_htim9};

    m_huart2 = huart2;

    m_motor1 = motor{&m_tim5, TIM_CHANNEL_1, GPIOF, GPIO_PIN_15, &m_tim1, 13};
    m_motor2 = motor{&m_tim5, TIM_CHANNEL_2, GPIOG, GPIO_PIN_0, &m_tim2, 13};
    m_motor3 = motor{&m_tim5, TIM_CHANNEL_4, GPIOG, GPIO_PIN_1, &m_tim3, 13};
    m_motor4 = motor{&m_tim9, TIM_CHANNEL_2, GPIOE, GPIO_PIN_7, &m_tim4, 13};

    m_imu.set_uart(m_huart2);
    m_imu.set_send_func([this](const uint8_t *data, uint16_t size) {
      if (m_huart2) {
        HAL_UART_Transmit(m_huart2, const_cast<uint8_t *>(data), size, 10);
      }
    });
    m_imu.set_euler_callback([this](float roll, float pitch, float yaw) {
      std::lock_guard<mutex> lock(m_imu_mutex);
      if (!m_imu_initialized) {
        m_last_euler_update_time = steady_clock::now();
        m_imu_initialized = true;
        // 更新当前欧拉角数据
        m_current_euler_angles.roll = roll;
        m_current_euler_angles.pitch = pitch;
        m_current_euler_angles.yaw = yaw;
      } else {
        // 计算欧拉角数据的更新间隔，单位为毫秒
        auto now = steady_clock::now();
        m_euler_update_interval =
            std::chrono::duration<float>(now - m_last_euler_update_time)
                .count() *
            1000.0f;
        m_last_euler_update_time = now;

        // 根据欧拉角数据计算当前速度和角速度，更新原子变量
        m_current_w.store((yaw - m_current_euler_angles.yaw) /
                              (m_euler_update_interval + 1e-6f),
                          std::memory_order_release);

        // 更新当前欧拉角数据
        m_current_euler_angles.roll = roll;
        m_current_euler_angles.pitch = pitch;
        m_current_euler_angles.yaw = yaw;
      }
    });
    m_imu.set_gyro_and_acc_callback([this](float gyro_x, float gyro_y,
                                           float gyro_z, float acc_x,
                                           float acc_y, float acc_z) {
      std::lock_guard<mutex> lock(m_imu_mutex);
      if (!m_imu_initialized) {
        return; // 在 IMU 初始化之前忽略陀螺仪和加速度计数据
      }
      // 计算陀螺仪和加速度计数据的更新间隔，单位为毫秒
      auto now = steady_clock::now();
      m_gyro_and_acc_update_interval =
          std::chrono::duration<float>(now - m_last_gyro_and_acc_update_time)
              .count() *
          1000.0f;

      // 根据陀螺仪和加速度计数据计算当前速度，更新原子变量
      std::atomic_thread_fence(std::memory_order_acquire);
      const float last_vx = m_current_vx.load(std::memory_order_relaxed);
      const float last_vy = m_current_vy.load(std::memory_order_relaxed);
      m_current_vx.store(last_vx + acc_x * m_gyro_and_acc_update_interval,
                         std::memory_order_relaxed);
      m_current_vy.store(last_vy + acc_y * m_gyro_and_acc_update_interval,
                         std::memory_order_relaxed);
      std::atomic_thread_fence(std::memory_order_release);

      // 更新最后一次接收陀螺仪和加速度计数据的时间戳
      m_last_gyro_and_acc_update_time = now;
      m_current_gyro_and_acc.gyro_x = gyro_x;
      m_current_gyro_and_acc.gyro_y = gyro_y;
      m_current_gyro_and_acc.gyro_z = gyro_z;
      m_current_gyro_and_acc.acc_x = acc_x;
      m_current_gyro_and_acc.acc_y = acc_y;
      m_current_gyro_and_acc.acc_z = acc_z;
    });
  }

  void start() {
    if (m_thread.joinable()) {
      return; // 已经在运行了
    }
    m_last_imu_control_time = steady_clock::now();
    m_imu.start();
    m_thread = thread<2048, osPriorityRealtime>{
        "encoder_thread", [&]() {
          steady_clock::time_point last_time = steady_clock::now();
          // Ki减小到0.05，积分增长速度更合理
          pid_controller pid1{1.0f, 0.7f, 0.3f, 0.03f, 1.0f, -1.0f, 1.0f};
          pid_controller pid2{1.0f, 0.7f, 0.3f, 0.03f, 1.0f, -1.0f, 1.0f};
          pid_controller pid3{1.0f, 0.7f, 0.3f, 0.03f, 1.0f, -1.0f, 1.0f};
          pid_controller pid4{1.0f, 0.7f, 0.3f, 0.03f, 1.0f, -1.0f, 1.0f};
          for (;;) {
            auto now = steady_clock::now();
            float delta_time =
                std::chrono::duration<float>(now - last_time).count() * 1000.0f;
            m_motor1.refresh_encoder_state(delta_time);
            m_motor2.refresh_encoder_state(delta_time);
            m_motor3.refresh_encoder_state(delta_time);
            m_motor4.refresh_encoder_state(delta_time);

            // motor.get_current_speed() 已经是按 ppr 和采样周期换算后的速度
            // 控制器的输入，使其更敏感，避免死区过大导致响应迟钝
            float speed1 =
                -m_motor1.get_current_speed() / 3584.61562f * 10000.0f / 1.5f;
            float speed2 =
                -m_motor2.get_current_speed() / 3584.61562f * 10000.0f / 1.5f;
            float speed3 =
                -m_motor3.get_current_speed() / 3584.61562f * 10000.0f / 1.5f;
            float speed4 =
                m_motor4.get_current_speed() / 3584.61562f * 10000.0f / 1.5f;

            // 获取目标速度，更新 PID 控制器的目标值
            std::atomic_thread_fence(std::memory_order_acquire);
            const float target1 =
                m_target_speed1.load(std::memory_order_relaxed);
            const float target2 =
                m_target_speed2.load(std::memory_order_relaxed);
            const float target3 =
                m_target_speed3.load(std::memory_order_relaxed);
            const float target4 =
                m_target_speed4.load(std::memory_order_relaxed);

            pid1.set_target(target1);
            pid2.set_target(target2);
            pid3.set_target(target3);
            pid4.set_target(target4);

            // 更新 PID 控制器，获取控制输出，并设置 PWM 占空比
            float control1 = pid1.update(speed1, delta_time);
            float control2 = pid2.update(speed2, delta_time);
            float control3 = pid3.update(speed3, delta_time);
            float control4 = pid4.update(speed4, delta_time);
            m_motor1.set_pwm_duty(control1);
            m_motor2.set_pwm_duty(control2);
            m_motor3.set_pwm_duty(control3);
            m_motor4.set_pwm_duty(control4);
            last_time = now;
            osDelay(2);
          }
        }};
    // m_imu_thread = thread<2048, osPriorityHigh>{
    //     "imu_thread", [&]() {
    //       pid_controller imu_vx{1.0f, 0.5f, 0.0f, 0.01f, 1.0f, -1.0f, 1.0f};
    //       pid_controller imu_vy{1.0f, 0.5f, 0.0f, 0.01f, 1.0f, -1.0f, 1.0f};
    //       pid_controller imu_w{1.0f, 0.5f, 0.0f, 0.01f, 1.0f, -1.0f, 1.0f};
    //       while (true) {
    //         {
    //           std::lock_guard<mutex> lock(m_imu_mutex);
    //           if (!m_imu_initialized) {
    //             osDelay(5);
    //             continue; // 在 IMU 初始化之前不进行控制
    //           }
    //           // 获取当前速度和角速度，更新 PID 控制器的目标值
    //           std::atomic_thread_fence(std::memory_order_acquire);
    //           const float target_vx =
    //               m_target_vx.load(std::memory_order_relaxed);
    //           const float target_vy =
    //               m_target_vy.load(std::memory_order_relaxed);
    //           const float target_w = m_target_w.load(std::memory_order_relaxed);
    //           imu_vx.set_target(target_vx);
    //           imu_vy.set_target(target_vy);
    //           imu_w.set_target(target_w);

    //           // 更新积分项，积分项的增长速度与陀螺仪和加速度计数据的更新频率成正比
    //           auto now = steady_clock::now();
    //           float delta_time =
    //               std::chrono::duration<float>(now - m_last_imu_control_time)
    //                   .count() *
    //               1000.0f;
    //           m_last_imu_control_time = now;

    //           // 更新 PID 控制器，获取控制输出，并设置 PWM 占空比
    //           // 目前直接使用加速度计的 x 和 y
    //           // 轴数据来控制前后和左右速度，陀螺仪的 z 轴数据来控制旋转速度
    //           // 后面可能会根据实际情况进行调整，比如使用融合算法来综合考虑加速度计和陀螺仪的数据，或者使用更复杂的控制策略
    //           std::atomic_thread_fence(std::memory_order_acquire);
    //           float control_vx = imu_vx.update(
    //               m_current_vx.load(std::memory_order_relaxed), delta_time);
    //           float control_vy = imu_vy.update(
    //               m_current_vy.load(std::memory_order_relaxed), delta_time);
    //           float control_w = imu_w.update(
    //               m_current_w.load(std::memory_order_relaxed), delta_time);
    //           // 将 IMU 控制输出与编码器控制输出进行融合，得到最终的控制信号
    //           // 这里简单地将两者进行加权平均，权重可以根据实际情况进行调整
    //           set_encoder_target_speed({control_vx, control_vy, control_w});
    //         }
    //         osDelay(5);
    //       }
    //     }};
  }

  // vx 范围为 -1.0f 到 1.0f，表示前后速度
  // vy 范围为 -1.0f 到 1.0f，表示左右速度
  // w 范围为 -1.0f 到 1.0f，表示旋转速度
  void set_speed(vector<float, 3> velocity) {
    // m_target_vx.store(velocity[0], std::memory_order_relaxed);
    // m_target_vy.store(velocity[1], std::memory_order_relaxed);
    // m_target_w.store(velocity[2], std::memory_order_relaxed);
    // std::atomic_thread_fence(std::memory_order_release);
    set_encoder_target_speed(velocity);
  }

  vector<float, 3> get_speed() const {
    // std::atomic_thread_fence(std::memory_order_acquire);
    // return {m_target_vx.load(std::memory_order_relaxed),
    //         m_target_vy.load(std::memory_order_relaxed),
    //         m_target_w.load(std::memory_order_relaxed)};
    return get_encoder_target_speed();
  }

  atk_ms901m &get_imu() { return m_imu; }
  const atk_ms901m &get_imu() const { return m_imu; }

protected:
  struct euler_angles {
    float roll;
    float pitch;
    float yaw;
  };

  struct gyro_and_acc {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
  };

  // vx 范围为 -1.0f 到 1.0f，表示前后速度
  // vy 范围为 -1.0f 到 1.0f，表示左右速度
  // w 范围为 -1.0f 到 1.0f，表示旋转速度
  void set_encoder_target_speed(vector<float, 3> velocity) {
    chassis_kinematics<1.0f> kinematics;
    auto wheel_speed = kinematics.forward_kinematics(velocity);
    // 因为电机安装方向的关系，轮速需要进行符号调整
    m_target_speed1.store(-wheel_speed[0], std::memory_order_relaxed);
    m_target_speed2.store(wheel_speed[1], std::memory_order_relaxed);
    m_target_speed3.store(wheel_speed[2], std::memory_order_relaxed);
    m_target_speed4.store(wheel_speed[3], std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
  }

  vector<float, 3> get_encoder_target_speed() const {
    chassis_kinematics<1.0f> kinematics;
    float speed1 = m_motor1.get_current_speed() / 3584.61562f * 10000.0f / 1.5f;
    float speed2 =
        -m_motor2.get_current_speed() / 3584.61562f * 10000.0f / 1.5f;
    float speed3 =
        -m_motor3.get_current_speed() / 3584.61562f * 10000.0f / 1.5f;
    float speed4 =
        -m_motor4.get_current_speed() / 3584.61562f * 10000.0f / 1.5f;
    // 因为电机安装方向的关系，轮速需要进行符号调整
    return kinematics.inverse_kinematics({speed1, -speed2, -speed3, -speed4});
  }

private:
  TIM_HandleTypeDef *m_htim1{nullptr};
  TIM_HandleTypeDef *m_htim2{nullptr};
  TIM_HandleTypeDef *m_htim3{nullptr};
  TIM_HandleTypeDef *m_htim4{nullptr};
  TIM_HandleTypeDef *m_htim5{nullptr};
  TIM_HandleTypeDef *m_htim9{nullptr};
  UART_HandleTypeDef *m_huart2{nullptr};

  // 异步线程，用于处理编码器数据和控制电机
  thread<2048, osPriorityRealtime> m_thread{empty_thread};
  thread<2048, osPriorityHigh> m_imu_thread{empty_thread};
  std::atomic<float> m_target_speed1{0.0f};
  std::atomic<float> m_target_speed2{0.0f};
  std::atomic<float> m_target_speed3{0.0f};
  std::atomic<float> m_target_speed4{0.0f};

  timer m_tim1;
  timer m_tim2;
  timer m_tim3;
  timer m_tim4;
  timer m_tim5;
  timer m_tim9;

  motor m_motor1;
  motor m_motor2;
  motor m_motor3;
  motor m_motor4;

  // IMU对象，用于获取姿态信息
  atk_ms901m m_imu;
  bool m_imu_initialized{false};
  mutable mutex m_imu_mutex;
  euler_angles m_current_euler_angles{0.0f, 0.0f, 0.0f};
  steady_clock::time_point m_last_euler_update_time;
  float m_euler_update_interval{0.0f};
  gyro_and_acc m_current_gyro_and_acc{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  steady_clock::time_point m_last_gyro_and_acc_update_time;
  float m_gyro_and_acc_update_interval{0.0f};
  steady_clock::time_point m_last_imu_control_time;

  std::atomic<float> m_target_vx{0.0f};
  std::atomic<float> m_target_vy{0.0f};
  std::atomic<float> m_target_w{0.0f};
  std::atomic<float> m_current_vx{0.0f};
  std::atomic<float> m_current_vy{0.0f};
  std::atomic<float> m_current_w{0.0f};
};

} // namespace gdut

#endif // CHASSIS_CONTROLLER_HPP
