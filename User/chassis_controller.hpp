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

  chassis_controller(SPI_HandleTypeDef *hspi, TIM_HandleTypeDef *htim1,
                     TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3,
                     TIM_HandleTypeDef *htim4, TIM_HandleTypeDef *htim5,
                     TIM_HandleTypeDef *htim9)
      : m_hspi(hspi), m_htim1(htim1), m_htim2(htim2), m_htim3(htim3),
        m_htim4(htim4), m_htim5(htim5), m_htim9(htim9) {}

  ~chassis_controller() = default;

  void set_parameters(SPI_HandleTypeDef *hspi, TIM_HandleTypeDef *htim1,
                      TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3,
                      TIM_HandleTypeDef *htim4, TIM_HandleTypeDef *htim5,
                      TIM_HandleTypeDef *htim9) {
    m_hspi = hspi;
    m_htim1 = htim1;
    m_htim2 = htim2;
    m_htim3 = htim3;
    m_htim4 = htim4;
    m_htim5 = htim5;
    m_htim9 = htim9;
  }

  void start() {
    if (m_thread.joinable()) {
      return; // 已经在运行了
    }
    m_thread = thread<8192>{[this]() { run_in_thread(); }};
  }

protected:
  // 占用的资源较多，建议在独立线程中运行
  // 需要保证足够的线程栈空间，否则可能会崩溃
  // 建议至少4096字节（默认线程栈大小为512字节，可能不足以容纳所有对象）
  void run_in_thread() {
    spi_proxy spi_proxy{m_hspi};
    ps2_controller ps2_controller{
        {[](bool state) {
           if (state)
             HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
           else
             HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
         },
         [](std::uint32_t ms) { osDelay(ms); }},
        &spi_proxy};
    ps2_controller.init();

    timer tim1{m_htim1};
    timer tim2{m_htim2};
    timer tim3{m_htim3};
    timer tim4{m_htim4};
    timer tim5{m_htim5};
    timer tim9{m_htim9};

    motor motor1{&tim5, TIM_CHANNEL_1, GPIOF, GPIO_PIN_15, &tim1, 13};
    motor motor2{&tim5, TIM_CHANNEL_2, GPIOG, GPIO_PIN_0, &tim2, 13};
    motor motor3{&tim5, TIM_CHANNEL_4, GPIOG, GPIO_PIN_1, &tim3, 13};
    motor motor4{&tim9, TIM_CHANNEL_2, GPIOE, GPIO_PIN_7, &tim4, 13};

    std::atomic<float> target_speed1{0.0f}, target_speed2{0.0f},
        target_speed3{0.0f}, target_speed4{0.0f};

    thread<1024, osPriorityAboveNormal> encoder_thread{[&]() {
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
        motor1.refresh_encoder_state(delta_time);
        motor2.refresh_encoder_state(delta_time);
        motor3.refresh_encoder_state(delta_time);
        motor4.refresh_encoder_state(delta_time);

        // motor.get_current_speed() 已经是按 ppr 和采样周期换算后的速度
        // 控制器的输入，使其更敏感，避免死区过大导致响应迟钝
        float speed1 =
            motor1.get_current_speed() / 3584.61562f * 10000.0f / 1.5f;
        float speed2 =
            -motor2.get_current_speed() / 3584.61562f * 10000.0f / 1.5f;
        float speed3 =
            -motor3.get_current_speed() / 3584.61562f * 10000.0f / 1.5f;
        float speed4 =
            -motor4.get_current_speed() / 3584.61562f * 10000.0f / 1.5f;

        std::atomic_thread_fence(std::memory_order_acquire);
        const float target1 = target_speed1.load(std::memory_order_relaxed);
        const float target2 = target_speed2.load(std::memory_order_relaxed);
        const float target3 = target_speed3.load(std::memory_order_relaxed);
        const float target4 = target_speed4.load(std::memory_order_relaxed);

        pid1.set_target(target1);
        pid2.set_target(target2);
        pid3.set_target(target3);
        pid4.set_target(target4);

        float control1 = pid1.update(speed1, delta_time);
        float control2 = pid2.update(speed2, delta_time);
        float control3 = pid3.update(speed3, delta_time);
        float control4 = pid4.update(speed4, delta_time);
        motor1.set_pwm_duty(control1);
        motor2.set_pwm_duty(control2);
        motor3.set_pwm_duty(control3);
        motor4.set_pwm_duty(control4);
        last_time = now;
        osDelay(2);
      }
    }};
    chassis_kinematics<1.0f> kinematics;
    bool ps2_correct = false;
    int counter = 0;
    float x_avg = 0.0f, y_avg = 0.0f, w_avg = 0.0f;

    /* Infinite loop */
    for (;;) {
      if (ps2_controller.poll()) {
        const auto state = ps2_controller.read_state();
        if (state.select_is_pressed()) {
          ps2_correct = true;
        }
        float vy =
            (state.left_y / 127.0f) - 1.0f - (ps2_correct ? 0.0f : y_avg);
        float vx =
            -((state.left_x / 127.0f) - 1.0f - (ps2_correct ? 0.0f : x_avg));
        float w =
            ((state.right_x / 127.0f) - 1.0f - (ps2_correct ? 0.0f : w_avg));

        constexpr float max_total_speed = 0.1f;
        float total_speed = std::sqrt(vx * vx + vy * vy + w * w);
        if (total_speed > max_total_speed && total_speed > 1e-6f) {
          const float scale = max_total_speed / total_speed;
          vx *= scale;
          vy *= scale;
          w *= scale;
        }

        if (ps2_correct && counter++ < 100) {
          x_avg += vx;
          y_avg += vy;
          w_avg += w;
        } else if (counter == 100) {
          ps2_correct = false;
          counter = 0;
          x_avg /= 100.0f;
          y_avg /= 100.0f;
          w_avg /= 100.0f;
        }
        auto wheel_speed = kinematics.forward_kinematics({vx, vy, w});
        // 因为电机安装方向的关系，轮速需要进行符号调整
        target_speed1.store(wheel_speed[0], std::memory_order_relaxed);
        target_speed2.store(wheel_speed[1], std::memory_order_relaxed);
        target_speed3.store(wheel_speed[2], std::memory_order_relaxed);
        target_speed4.store(-wheel_speed[3], std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
      } else {
        target_speed1.store(0.0f, std::memory_order_relaxed);
        target_speed2.store(0.0f, std::memory_order_relaxed);
        target_speed3.store(0.0f, std::memory_order_relaxed);
        target_speed4.store(0.0f, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
      }
      osDelay(10);
    }
  }

private:
  SPI_HandleTypeDef *m_hspi{nullptr};
  TIM_HandleTypeDef *m_htim1{nullptr};
  TIM_HandleTypeDef *m_htim2{nullptr};
  TIM_HandleTypeDef *m_htim3{nullptr};
  TIM_HandleTypeDef *m_htim4{nullptr};
  TIM_HandleTypeDef *m_htim5{nullptr};
  TIM_HandleTypeDef *m_htim9{nullptr};

  thread<8192> m_thread{empty_thread};
};

} // namespace gdut

#endif // CHASSIS_CONTROLLER_HPP
