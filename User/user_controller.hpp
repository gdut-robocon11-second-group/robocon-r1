#ifndef USER_CONTROLLER_HPP
#define USER_CONTROLLER_HPP

#include "auto_controller.hpp"
#include "bsp_ps2.hpp"
#include "bsp_spi.hpp"
#include "chassis_controller.hpp"
#include "stm32f4xx_hal.h"
#include "thread.hpp"
#include "transfer_controller.hpp"
#include "uncopyable.hpp"

namespace gdut {

class user_controller : private uncopyable {
public:
  user_controller() = default;
  ~user_controller() = default;

  void set_parameters(SPI_HandleTypeDef *m_hspi1, TIM_HandleTypeDef *htim1,
                      TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3,
                      TIM_HandleTypeDef *htim4, TIM_HandleTypeDef *htim5,
                      TIM_HandleTypeDef *htim9, UART_HandleTypeDef *huart2,
                      UART_HandleTypeDef *huart4) {
    m_chassis_controller.set_parameters(htim1, htim2, htim3, htim4, htim5,
                                        htim9, huart2);
    m_hspi = m_hspi1;
    m_transfer_controller.set_uart(huart4);
    m_transfer_controller.set_send_function(
        [huart4](const std::uint8_t *data, const std::uint8_t *end) {
          size_t size = static_cast<size_t>(end - data);
          HAL_UART_Transmit(huart4, const_cast<uint8_t *>(data), size,
                            HAL_MAX_DELAY);
        });
    m_transfer_controller.set_yolo_result_callback(
        [this](transfer_protocol::yolo_detection_result result) {
          auto number = result.number_detected;
          // 这里可以根据需要处理YOLO检测结果，例如更新底盘控制器的状态
          // 或者将结果发送到其他模块
        });
    m_transfer_controller.set_qr_code_result_callback(
        [this](transfer_protocol::qr_code_detection_result result) {
          auto qr_code = result.qr_code_data;
          // 这里可以根据需要处理二维码检测结果，例如更新底盘控制器的状态
          // 或者将结果发送到其他模块
        });
  }

  void start() {
    if (m_thread.joinable()) {
      return; // 已经在运行了
    }
    m_chassis_controller.start();
    m_transfer_controller.start();
    m_thread =
        thread<2048>{"user_controller_thread", [this]() { run_in_thread(); }};
  }

  chassis_controller &chassis() { return m_chassis_controller; }
  const chassis_controller &chassis() const { return m_chassis_controller; }
  transfer_controller &transfer() { return m_transfer_controller; }
  const transfer_controller &transfer() const { return m_transfer_controller; }
  atk_ms901m &imu() { return m_chassis_controller.get_imu(); }
  const atk_ms901m &imu() const { return m_chassis_controller.get_imu(); }

protected:
  void run_in_thread() {
    spi_proxy spi_proxy{m_hspi};
    ps2_controller ps2_controller{
        {[](bool state) {
           if (state)
             HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
           else
             HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
         },
         [](::std::uint32_t ms) { osDelay(ms); }},
        &spi_proxy};
    ps2_controller.init();

    bool ps2_correct = false;
    int counter = 0;
    float x_avg = 0.0f, y_avg = 0.0f, w_avg = 0.0f;

    /* Infinite loop */
    while (true) {
      if (ps2_controller.poll()) {
        // 读取状态
        const ps2_state state = ps2_controller.read_state();

        // 如果按下了方块键，发送数字请求
        if (state.square_is_pressed()) {
          send_number_request();
        }

        // 如果按下了圆圈键，发送二维码请求
        if (state.circle_is_pressed()) {
          send_qr_code_request();
        }

        if (state.select_is_pressed()) {
          ps2_correct = true; // 按下 select 键，进入校正模式
          counter = 0;
        }
        float vy =
            (state.left_y / 127.0f) - 1.0f - (ps2_correct ? 0.0f : y_avg);
        float vx =
            -((state.left_x / 127.0f) - 1.0f - (ps2_correct ? 0.0f : x_avg));
        float w =
            -((state.right_x / 127.0f) - 1.0f - (ps2_correct ? 0.0f : w_avg));

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

        constexpr float max_total_speed = 0.1f;
        float total_speed = std::sqrt(vx * vx + vy * vy + w * w);
        if (total_speed > max_total_speed && total_speed > 1e-6f) {
          const float scale = max_total_speed / total_speed;
          vx *= scale;
          vy *= scale;
          w *= scale * 1.5f; // 适当增加旋转速度的权重，使其在总速限制下更有响应
        }

        m_chassis_controller.set_speed({vx, vy, w});
      }
      osDelay(10);
    }
  }

  void send_number_request() { m_transfer_controller.send_number_request(); }

  void send_qr_code_request() { m_transfer_controller.send_qr_code_request(); }

private:
  chassis_controller m_chassis_controller;
  transfer_controller m_transfer_controller;

  thread<2048> m_thread{empty_thread};

  SPI_HandleTypeDef *m_hspi{nullptr};
};

} // namespace gdut

#endif // USER_CONTROLLER_HPP
