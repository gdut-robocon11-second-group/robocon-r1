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

    thread<256, osPriorityAboveNormal> encoder_thread{[&]() {
      steady_clock::time_point last_time = steady_clock::now();
      for (;;) {
        auto now = steady_clock::now();
        float delta_time = static_cast<float>((now - last_time).count()) /
                           static_cast<float>(steady_clock::period::den);
        motor1.refresh_encoder_state(delta_time);
        motor2.refresh_encoder_state(delta_time);
        motor3.refresh_encoder_state(delta_time);
        motor4.refresh_encoder_state(delta_time);
        osDelay(1);
      }
    }};
    chassis_kinematics<1.0f> kinematics;
    pid_controller pid1{3.0f, 0.1f, 0.1f, 0.0f, 0.3f, -1.0f, 1.0f};
    pid_controller pid2{3.0f, 0.1f, 0.1f, 0.0f, 0.3f, -1.0f, 1.0f};
    pid_controller pid3{3.0f, 0.1f, 0.1f, 0.0f, 0.3f, -1.0f, 1.0f};
    pid_controller pid4{3.0f, 0.1f, 0.1f, 0.0f, 0.3f, -1.0f, 1.0f};
    float target_speed1 = 0.0f, target_speed2 = 0.0f, target_speed3 = 0.0f,
          target_speed4 = 0.0f;
    gdut::steady_clock::time_point last_control_time =
        gdut::steady_clock::now();

    /* Infinite loop */
    for (;;) {
      if (ps2_controller.poll()) {
        const auto state = ps2_controller.read_state();
        float vy = (state.left_y / 127.0f) - 1.0f;
        float vx = -((state.left_x / 127.0f) - 1.0f);
        float w = (state.right_x / 127.0f) - 1.0f;
        auto wheel_speed = kinematics.forward_kinematics({vx, vy, w});
        // 因为电机安装方向的关系，轮速需要进行符号调整
        target_speed1 = -wheel_speed[0];
        target_speed2 = -wheel_speed[1];
        target_speed3 = -wheel_speed[2];
        target_speed4 = wheel_speed[3];
        // motor1.set_pwm_duty(-wheel_speed[0]); // A
        // motor2.set_pwm_duty(-wheel_speed[1]); // B
        // motor3.set_pwm_duty(-wheel_speed[2]); // C
        // motor4.set_pwm_duty(wheel_speed[3]);  // D
      }
      float speed1 = motor1.get_current_speed();
      float speed2 = motor2.get_current_speed();
      float speed3 = motor3.get_current_speed();
      float speed4 = motor4.get_current_speed();
      auto now = gdut::steady_clock::now();
      float control1 = pid1.update(
          target_speed1 - speed1,
          std::chrono::duration<float>{now - last_control_time}.count());
      float control2 = pid2.update(
          target_speed2 - speed2,
          std::chrono::duration<float>{now - last_control_time}.count());
      float control3 = pid3.update(
          target_speed3 - speed3,
          std::chrono::duration<float>{now - last_control_time}.count());
      float control4 = pid4.update(
          target_speed4 - speed4,
          std::chrono::duration<float>{now - last_control_time}.count());
      motor1.set_pwm_duty(control1);
      motor2.set_pwm_duty(control2);
      motor3.set_pwm_duty(control3);
      motor4.set_pwm_duty(control4);
      // motor1.set_pwm_duty(target_speed1);
      // motor2.set_pwm_duty(target_speed2);
      // motor3.set_pwm_duty(target_speed3);
      // motor4.set_pwm_duty(target_speed4);
      last_control_time = now;
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
