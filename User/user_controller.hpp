#ifndef USER_CONTROLLER_HPP
#define USER_CONTROLLER_HPP

#include "auto_controller.hpp"
#include "bsp_pca9685.hpp"
#include "bsp_ps2.hpp"
#include "bsp_servo.hpp"
#include "bsp_spi.hpp"
#include "bsp_stepper.hpp"
#include "chassis_controller.hpp"
#include "pca9685_controller.hpp"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "thread.hpp"
#include "transfer_controller.hpp"
#include "uncopyable.hpp"
#include <atomic>

namespace gdut {

class user_controller : private uncopyable {
public:
  enum class mode { auto_mode, manual_mode };

  user_controller() = default;
  ~user_controller() = default;

  // 传入所有需要的外设句柄，进行必要的初始化
  void set_parameters(I2C_HandleTypeDef *hi2c3, SPI_HandleTypeDef *m_hspi1,
                      TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2,
                      TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4,
                      TIM_HandleTypeDef *htim5, TIM_HandleTypeDef *htim8,
                      TIM_HandleTypeDef *htim9, UART_HandleTypeDef *huart2,
                      UART_HandleTypeDef *huart4, TIM_HandleTypeDef *htim10) {
    // 初始化底盘控制器
    m_chassis_controller.set_parameters(htim1, htim2, htim3, htim4, htim5,
                                        htim9, huart2);
    // 设置通信控制器参数
    m_hspi = m_hspi1;
    m_hi2c3 = hi2c3;
    // 设置步进电机控制器参数
    m_tim8_timer = timer{htim8};
    // 设置通信控制器的UART和回调函数
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
          // 0.0f是占位的mid_x，后续可以根据需要修改为实际值
          m_auto_controller.put_yolo_result(number, 0.0f);
        });
    m_transfer_controller.set_qr_code_result_callback(
        [this](transfer_protocol::qr_code_detection_result result) {
          const char *qr_code = result.qr_code_data;
          std::size_t length = 0;
          for (; length < sizeof(result.qr_code_data); ++length) {
            if (qr_code[length] == '\0') {
              break;
            }
          }
          // 提交二维码结果给自动控制器进行处理
          m_auto_controller.put_qr_code_result(
              std::span<const char>(qr_code, length));
        });
    // 设置PCA9685控制器参数
    m_pca9685_controller.set_parameters(m_hi2c3);
    m_pca9685_controller.init();
    // 设置步进电机控制器参数
    m_motor.set_parameters(&m_gpio_proxy, &m_tim8_timer, TIM_CHANNEL_1);
    m_auto_controller.set_parameters(&m_transfer_controller,
                                     &m_chassis_controller,
                                     &m_pca9685_controller, &m_motor);
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
    float lx_avg = 0.0f, ly_avg = 0.0f, rx_avg = 0.0f, ry_avg = 0.0f;

    /* Infinite loop */
    while (true) {
      if (ps2_controller.poll()) {
        // 读取状态
        const ps2_state state = ps2_controller.read_state();

        // 手动模式才响应手柄输入
        if (m_mode == mode::manual_mode) {

          // 按下三角键，步进电机抬升
          press_triangle(state);
          // 按下cross键，步进电机下降
          press_cross(state);

          // 如果按下了左肩键1，爪子夹取物品
          press_l1(state);

          // 如果按下了左肩键2，爪子松开物品
          press_l2(state);

          // 如果按下了右肩键1，爪子往前伸
          press_r1(state);
          // 如果按下了右肩键2，爪子往后缩
          press_r2(state);

          // 如果按下left，云台左转
          press_left(state);
          // 如果按下right，云台右转
          press_right(state);

          // 根据读取的状态计算底盘速度，进行简单的校正和限制
          float ly =
              (state.left_y / 127.0f) - 1.0f - (ps2_correct ? 0.0f : ly_avg);
          float lx =
              -((state.left_x / 127.0f) - 1.0f - (ps2_correct ? 0.0f : lx_avg));
          float ry = -((state.right_y / 127.0f) - 1.0f -
                       (ps2_correct ? 0.0f : ry_avg));
          float rx = -((state.right_x / 127.0f) - 1.0f -
                       (ps2_correct ? 0.0f : rx_avg));
          if (ps2_correct && counter++ < 100) {
            lx_avg += lx;
            ly_avg += ly;
            rx_avg += rx;
            ry_avg += ry;
          } else if (counter == 100) {
            ps2_correct = false;
            counter = 0;
            lx_avg /= 100.0f;
            ly_avg /= 100.0f;
            rx_avg /= 100.0f;
            ry_avg /= 100.0f;
          }

          // 将左摇杆的Y轴控制前后，左摇杆的X轴控制左右，右摇杆的X轴控制旋转速度
          set_chassis_speed(ly, lx, rx);
        } else if (m_mode == mode::auto_mode) {
          // 自动模式下可以根据需要处理PS2输入，例如按键控制某些自动行为

          if (state.up_is_pressed()) {
            // 如果按下了up键
            m_auto_controller.set_auto_submode(
                auto_controller::auto_submode::left_mode);
          
            }

            if (state.down_is_pressed()) {
              // 如果按下了down键
              m_auto_controller.set_auto_submode(
                  auto_controller::auto_submode::right_mode);
            }
          }
        }
        osDelay(10);
      }
    }

    void send_number_request() { m_transfer_controller.send_number_request(); }

    void send_qr_code_request() {
      m_auto_controller.clear_qr_code_received_success_flag();
      m_transfer_controller.send_qr_code_request();
    }

    bool is_qr_code_received_success() const {
      return m_auto_controller.is_qr_code_received_success();
    }

  protected:
    void set_chassis_speed(float vx, float vy, float omega) {
      constexpr float max_total_speed = 0.1f;
      float total_speed = std::sqrt(vx * vx + vy * vy + omega * omega);
      if (total_speed > max_total_speed && total_speed > 1e-6f) {
        const float scale = max_total_speed / total_speed;
        vx *= scale;
        vy *= scale;
        omega *=
            scale * 1.5f; // 适当增加旋转速度的权重，使其在总速限制下更有响应
      }
      m_chassis_controller.set_speed({vx, vy, omega});
    }

    void press_select(const ps2_state &state) {
      // 如果按下了select键，切换自动/手动模式
      if (state.select_is_pressed()) {
        if (m_mode == mode::manual_mode) {
          m_mode = mode::auto_mode;
        } else {
          m_mode = mode::manual_mode;
        }
      }
    }

    void press_triangle(const ps2_state &state) {
      // 如果按下了三角键，步进电机抬升
      if (state.triangle_is_pressed()) {
        m_motor.set_direction(true); // 设置方向，例如true表示抬升?
        m_motor.set_speed(30);       // 设置速度，例如100表示全速
      } else if (!state.triangle_is_pressed()) {
        m_motor.set_speed(0); // 停止PWM
      }
    }

    void press_cross(const ps2_state &state) {
      // 如果按下了cross键，步进电机下降
      if (state.cross_is_pressed()) {
        m_motor.set_direction(false); // 设置方向，例如false表示下降?
        m_motor.set_speed(30);        // 设置速度，例如100表示全速
      } else if (!state.cross_is_pressed()) {
        m_motor.set_speed(0); // 停止PWM
      }
    }

    void press_l1(const ps2_state &state) {
      // 如果按下了左肩键1，爪子夹取物品
      if (state.l1_is_pressed()) {
        m_pca9685_controller.set_claw_servo_angle(90);
      }
    }

    void press_l2(const ps2_state &state) {
      // 如果按下了左肩键2，爪子释放物品
      if (state.l2_is_pressed()) {
        m_pca9685_controller.set_claw_servo_angle(0);
      }
    }

    void press_r1(const ps2_state &state) {
      // 如果按下了右肩键1，爪子往前伸
      if (state.r1_is_pressed()) {
        m_pca9685_controller.belt_move_forward(1.0f);
      }
    }

    void press_r2(const ps2_state &state) {
      // 如果按下了右肩键2，爪子往后缩
      if (state.r2_is_pressed()) {
        m_pca9685_controller.belt_move_backward(1.0f);
      }
    }

    void press_left(const ps2_state &state) {
      // 如果按下left，云台左转
      if (state.left_is_pressed()) {
        m_pca9685_controller.turret_turn_left(1.0f);
      }
    }

    void press_right(const ps2_state &state) {
      // 如果按下right，云台右转
      if (state.right_is_pressed()) {
        m_pca9685_controller.turret_turn_right(1.0f);
      }
    }

  private:
    chassis_controller m_chassis_controller;
    transfer_controller m_transfer_controller;
    pca9685_controller m_pca9685_controller;
    auto_controller m_auto_controller;
    stepper_motor m_motor;
    mode m_mode{mode::manual_mode};

    thread<2048> m_thread{empty_thread};

    GPIO_InitTypeDef m_gpio_init_structure{GPIO_PIN_1, GPIO_MODE_OUTPUT_PP,
                                           GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0};
    gpio_proxy m_gpio_proxy{GPIOF, &m_gpio_init_structure};
    timer m_tim8_timer;

    I2C_HandleTypeDef *m_hi2c3{nullptr};
    SPI_HandleTypeDef *m_hspi{nullptr};
  };

} // namespace gdut

#endif // USER_CONTROLLER_HPP
