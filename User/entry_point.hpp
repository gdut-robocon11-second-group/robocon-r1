#ifndef ENTRY_POINT_HPP
#define ENTRY_POINT_HPP

#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "transfer_controller.hpp"
#include "uncopyable.hpp"
#include "user_controller.hpp"

namespace gdut {

class entry_point : private uncopyable {
public:
  void init(I2C_HandleTypeDef *hi2c2, I2C_HandleTypeDef *hi2c3,
            SPI_HandleTypeDef *hspi1, TIM_HandleTypeDef *htim1,
            TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3,
            TIM_HandleTypeDef *htim4, TIM_HandleTypeDef *htim5,
            TIM_HandleTypeDef *htim8, TIM_HandleTypeDef *htim9,
            TIM_HandleTypeDef *htim10, TIM_HandleTypeDef *htim11,
            TIM_HandleTypeDef *htim12, TIM_HandleTypeDef *htim13,
            UART_HandleTypeDef *huart4, UART_HandleTypeDef *huart5,
            UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2,
            UART_HandleTypeDef *huart3) {
    m_hi2c2 = hi2c2;
    m_hi2c3 = hi2c3;
    m_hspi1 = hspi1;
    m_htim1 = htim1;
    m_htim2 = htim2;
    m_htim3 = htim3;
    m_htim4 = htim4;
    m_htim5 = htim5;
    m_htim8 = htim8;
    m_htim9 = htim9;
    m_htim10 = htim10;
    m_htim11 = htim11;
    m_htim12 = htim12;
    m_htim13 = htim13;
    m_huart4 = huart4;
    m_huart5 = huart5;
    m_huart1 = huart1;
    m_huart2 = huart2;
    m_huart3 = huart3;
    m_initialized = true;
  }

  void start() {
    if (!m_initialized) {
      // Handle error: not initialized
      return;
    }
    m_user_controller.set_parameters(m_hi2c3, m_hspi1, m_htim1, m_htim2, m_htim3,
                                     m_htim4, m_htim5, m_htim8, m_htim9, m_huart2,
                                     m_huart4,m_htim10);
    m_user_controller.start();
  }

  static entry_point &instance() {
    static entry_point instance;
    return instance;
  }

  void uart4_rx_callback(uint16_t size) {
    m_user_controller.transfer().uart_rx_callback_it(size);
  }

  void uart2_rx_callback(uint16_t size) {
    m_user_controller.imu().handle_uart_rx(size);
  }

protected:
  entry_point() = default;
  ~entry_point() = default;

private:
  bool m_initialized = false;
  user_controller m_user_controller;

  I2C_HandleTypeDef *m_hi2c2;
  I2C_HandleTypeDef *m_hi2c3;
  SPI_HandleTypeDef *m_hspi1;
  TIM_HandleTypeDef *m_htim1;
  TIM_HandleTypeDef *m_htim2;
  TIM_HandleTypeDef *m_htim3;
  TIM_HandleTypeDef *m_htim4;
  TIM_HandleTypeDef *m_htim5;
  TIM_HandleTypeDef *m_htim8;
  TIM_HandleTypeDef *m_htim9;
  TIM_HandleTypeDef *m_htim10;
  TIM_HandleTypeDef *m_htim11;
  TIM_HandleTypeDef *m_htim12;
  TIM_HandleTypeDef *m_htim13;
  UART_HandleTypeDef *m_huart4;
  UART_HandleTypeDef *m_huart5;
  UART_HandleTypeDef *m_huart1;
  UART_HandleTypeDef *m_huart2;
  UART_HandleTypeDef *m_huart3;
};

} // namespace gdut

#endif // ENTRY_POINT_HPP
