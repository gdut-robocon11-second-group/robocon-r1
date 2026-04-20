#ifndef CHASSIS_CONTROLLER_HPP
#define CHASSIS_CONTROLLER_HPP

#include "bsp_motor.hpp"
#include "bsp_ps2.hpp"
#include "bsp_spi.hpp"
#include "bsp_timer.hpp"
#include "chassis_kinematics.hpp"
#include "clock.hpp"
#include "pid_controller.hpp"
#include "stm32f4xx_hal_gpio.h"
#include "thread.hpp"
#include "uncopyable.hpp"
#include <cstdio>

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
                      TIM_HandleTypeDef *htim5, TIM_HandleTypeDef *htim9) {
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

    m_motor1 = motor{&m_tim5, TIM_CHANNEL_1, GPIOF, GPIO_PIN_15, &m_tim1, 13};
    m_motor2 = motor{&m_tim5, TIM_CHANNEL_2, GPIOG, GPIO_PIN_0, &m_tim2, 13};
    m_motor3 = motor{&m_tim5, TIM_CHANNEL_4, GPIOG, GPIO_PIN_1, &m_tim3, 13};
    m_motor4 = motor{&m_tim9, TIM_CHANNEL_2, GPIOE, GPIO_PIN_7, &m_tim4, 13};
  }

  void start() {
    if (m_thread.joinable()) {
      return; // 已经在运行了
    }
    m_thread = thread<2048, osPriorityAboveNormal>{"encoder_thread", [&]() {
      steady_clock::time_point last_time = steady_clock::now();
      // Ki减小到0.05，积分增长速度更合理
      pid_controller pid1{1.0f, 0.7f, 0.0f, 0.015f, 1.0f, -1.0f, 1.0f};
      pid_controller pid2{1.0f, 0.7f, 0.0f, 0.015f, 1.0f, -1.0f, 1.0f};
      pid_controller pid3{1.0f, 0.7f, 0.0f, 0.015f, 1.0f, -1.0f, 1.0f};
      pid_controller pid4{1.0f, 0.7f, 0.0f, 0.015f, 1.0f, -1.0f, 1.0f};
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
            m_motor1.get_current_speed() / 3584.61562f * 10000.0f / 1.5f;
        float speed2 =
            -m_motor2.get_current_speed() / 3584.61562f * 10000.0f / 1.5f;
        float speed3 =
            -m_motor3.get_current_speed() / 3584.61562f * 10000.0f / 1.5f;
        float speed4 =
            -m_motor4.get_current_speed() / 3584.61562f * 10000.0f / 1.5f;

        std::atomic_thread_fence(std::memory_order_acquire);
        const float target1 = m_target_speed1.load(std::memory_order_relaxed);
        const float target2 = m_target_speed2.load(std::memory_order_relaxed);
        const float target3 = m_target_speed3.load(std::memory_order_relaxed);
        const float target4 = m_target_speed4.load(std::memory_order_relaxed);

        pid1.set_target(target1);
        pid2.set_target(target2);
        pid3.set_target(target3);
        pid4.set_target(target4);

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
  }

  // 提供一个接口，允许外部线程设置目标速度
  // vx 范围为 -1.0f 到 1.0f，表示前后速度
  // vy 范围为 -1.0f 到 1.0f，表示左右速度
  // w 范围为 -1.0f 到 1.0f，表示旋转速度
  void set_speed(float vx, float vy, float w) {
    chassis_kinematics<1.0f> kinematics;
    auto wheel_speed = kinematics.forward_kinematics({vx, vy, w});
    // 因为电机安装方向的关系，轮速需要进行符号调整
    m_target_speed1.store(wheel_speed[0], std::memory_order_relaxed);
    m_target_speed2.store(wheel_speed[1], std::memory_order_relaxed);
    m_target_speed3.store(wheel_speed[2], std::memory_order_relaxed);
    m_target_speed4.store(-wheel_speed[3], std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
  }

private:
  TIM_HandleTypeDef *m_htim1{nullptr};
  TIM_HandleTypeDef *m_htim2{nullptr};
  TIM_HandleTypeDef *m_htim3{nullptr};
  TIM_HandleTypeDef *m_htim4{nullptr};
  TIM_HandleTypeDef *m_htim5{nullptr};
  TIM_HandleTypeDef *m_htim9{nullptr};

  thread<2048, osPriorityAboveNormal> m_thread{empty_thread};
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
};

} // namespace gdut

#endif // CHASSIS_CONTROLLER_HPP
