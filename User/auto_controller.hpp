#ifndef AUTO_CONTROLLER_HPP
#define AUTO_CONTROLLER_HPP

#include "bsp_pca9685.hpp"
#include "bsp_servo.hpp"
#include "bsp_spi.hpp"
#include "bsp_stepper.hpp"
#include "chassis_controller.hpp"
#include "pca9685_controller.hpp"
#include "semaphore.hpp"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "thread.hpp"
#include "transfer_controller.hpp"
#include "bsp_line8_follow_ch8.hpp"
#include "uncopyable.hpp"
#include <atomic>

namespace gdut {

class auto_controller {
public:
  enum class auto_submode { left_mode, right_mode };

  auto_controller() = default;
  ~auto_controller() = default;

  void set_parameters(transfer_controller *transfer_ctrl,
                      chassis_controller *chassis_ctrl,
                      pca9685_controller *pca9685_ctrl,
                      stepper_motor *motor_ctrl) {
    m_transfer_controller = transfer_ctrl;
    m_chassis_controller = chassis_ctrl;
    m_pca9685_controller = pca9685_ctrl;
    m_motor = motor_ctrl;
  }

  void start() {
    if (m_thread.joinable()) {
      return; // Thread already running
    }
    m_yolo_semaphore = binary_semaphore(0);
    m_running = true;
    m_thread = thread<2048>("auto_control_thread", [this] { run_in_thread(); });
  }

  void stop() {
    m_running = false;
    if (m_thread.joinable()) {
      m_thread.join();
    }
  }

  // 提交二维码结果和YOLO检测结果的接口
  void put_qr_code_result(std::span<const char> qr_code_data) {
    // 处理二维码结果
    m_current_qr_code_length =
        std::min(qr_code_data.size(), m_current_qr_code.size() - 1);
    std::copy(qr_code_data.begin(),
              qr_code_data.begin() + m_current_qr_code_length,
              m_current_qr_code.begin());
    m_current_qr_code[m_current_qr_code_length] = '\0';
    m_qr_code_ready = true;
  }

  // 提交YOLO检测结果的接口
  void put_yolo_result(uint16_t number_detected, float mid_x) {
    // 处理YOLO检测结果
    m_current_number_detected = number_detected;
    m_current_mid_x = mid_x;
    m_yolo_semaphore.release();
  }

  void set_auto_submode(auto_submode submode) { m_auto_submode = submode; }
  auto_submode get_auto_submode() const { return m_auto_submode; }

protected:
  void run_in_thread() {
    while (m_running) {
      // 等待二维码结果
      while (m_running && !m_qr_code_ready) {
        osDelay(10);
      }
      if (!m_running) {
        return;
      }

      // 准备巡线控制器
    }
  }

private:
  transfer_controller *m_transfer_controller{nullptr};
  chassis_controller *m_chassis_controller{nullptr};
  pca9685_controller *m_pca9685_controller{nullptr};
  stepper_motor *m_motor{nullptr};

  thread<2048> m_thread{empty_thread};
  std::atomic<bool> m_running{false};

  std::atomic<auto_submode> m_auto_submode{auto_submode::left_mode};

  // 存储当前二维码结果的缓冲区，长度为32字节（包括结尾的'\0'）
  std::array<char, 32> m_current_qr_code{};
  std::size_t m_current_qr_code_length{0};
  std::atomic<bool> m_qr_code_ready{false};

  // 存储当前YOLO检测结果
  std::atomic<uint16_t> m_current_number_detected{0};
  std::atomic<float> m_current_mid_x{0.0f};
  binary_semaphore m_yolo_semaphore{empty_semaphore};
};

} // namespace gdut

#endif // AUTO_CONTROLLER_HPP
