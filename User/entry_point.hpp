#ifndef ENTRY_POINT_HPP
#define ENTRY_POINT_HPP

#include "chassis_controller.hpp"
#include "cmsis_os2.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "thread.hpp"

namespace gdut {

class entry_point {
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
    hi2c2_ = hi2c2;
    hi2c3_ = hi2c3;
    hspi1_ = hspi1;
    htim1_ = htim1;
    htim2_ = htim2;
    htim3_ = htim3;
    htim4_ = htim4;
    htim5_ = htim5;
    htim8_ = htim8;
    htim9_ = htim9;
    htim10_ = htim10;
    htim11_ = htim11;
    htim12_ = htim12;
    htim13_ = htim13;
    huart4_ = huart4;
    huart5_ = huart5;
    huart1_ = huart1;
    huart2_ = huart2;
    huart3_ = huart3;
    initialized_ = true;
  }

  void start() {
    if (!initialized_) {
      // Handle error: not initialized
      return;
    }
    chassis_controller_.set_parameters(hspi1_, htim1_, htim2_, htim3_, htim4_,
                                       htim5_, htim9_);
    chassis_controller_.start();
  }

  static entry_point &instance() {
    static entry_point instance;
    return instance;
  }

protected:
  entry_point() = default;
  ~entry_point() = default;
  entry_point(const entry_point &) = delete;
  entry_point &operator=(const entry_point &) = delete;

private:
  bool initialized_ = false;
  gdut::chassis_controller chassis_controller_{};

  I2C_HandleTypeDef *hi2c2_;
  I2C_HandleTypeDef *hi2c3_;
  SPI_HandleTypeDef *hspi1_;
  TIM_HandleTypeDef *htim1_;
  TIM_HandleTypeDef *htim2_;
  TIM_HandleTypeDef *htim3_;
  TIM_HandleTypeDef *htim4_;
  TIM_HandleTypeDef *htim5_;
  TIM_HandleTypeDef *htim8_;
  TIM_HandleTypeDef *htim9_;
  TIM_HandleTypeDef *htim10_;
  TIM_HandleTypeDef *htim11_;
  TIM_HandleTypeDef *htim12_;
  TIM_HandleTypeDef *htim13_;
  UART_HandleTypeDef *huart4_;
  UART_HandleTypeDef *huart5_;
  UART_HandleTypeDef *huart1_;
  UART_HandleTypeDef *huart2_;
  UART_HandleTypeDef *huart3_;
};

} // namespace gdut

#endif // ENTRY_POINT_HPP
