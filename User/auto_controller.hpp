#ifndef AUTO_CONTROLLER_HPP
#define AUTO_CONTROLLER_HPP

#include "bsp_line8_follow_ch8.hpp"
#include "bsp_stepper.hpp"
#include "chassis_controller.hpp"
#include "pca9685_controller.hpp"
#include "semaphore.hpp"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "thread.hpp"
#include "transfer_controller.hpp"
#include "uncopyable.hpp"
#include <algorithm>
#include <atomic>

namespace gdut {

class auto_controller : private uncopyable {
public:
  enum class auto_submode : std::uint8_t {
    left_mode, // 左边模式
    right_mode // 右边模式
  };

  enum class number_detect_mode : std::uint8_t {
    black_mode, // 黑底白字
    white_mode  // 白底黑字
  };

  enum class state : std::uint8_t {
    idle = 0,                   // 空闲状态，等待开始
    wait_qr_code,               // 等待二维码结果
    enter_line,                 // 进入线路
    detect_object,              // 检测物体
    grab_object,                // 抓取物体
    follow_line_to_destination, // 沿线路前往目的地
    release_object,             // 释放物体
    finished,                   // 完成
    error,                      // 错误状态
    // ...
  };

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
    m_running = true;
    m_state.store(state::idle);
    m_yolo_result_semaphore = binary_semaphore{0};
    m_qr_code_semaphore = binary_semaphore{0};
    m_yolo_history_mutex = mutex{};
    m_thread = thread<2048>("auto_control_thread", [this] { run_in_thread(); });
  }

  void stop() {
    m_running = false;
    if (m_thread.joinable()) {
      m_thread.join();
    }
  }

  void start_all() { m_state.store(state::wait_qr_code); }

  // 提交二维码结果和YOLO检测结果的接口
  void put_qr_code_result(std::span<const char> qr_code_data) {
    if (m_current_qr_code_length != 0) {
      return; // 已经收到过二维码结果了，忽略后续结果
    }
    // 处理二维码结果
    m_current_qr_code_length =
        std::min(qr_code_data.size(), m_current_qr_code.size() - 1);
    std::copy(qr_code_data.begin(),
              qr_code_data.begin() + m_current_qr_code_length,
              m_current_qr_code.begin());
    m_current_qr_code[m_current_qr_code_length] = '\0';
    m_qr_code_semaphore.release();
  }

  // 提交YOLO检测结果的接口
  void put_yolo_result(uint16_t number_detected, float mid_x) {
    // 处理YOLO检测结果
    m_current_number_detected = number_detected;
    m_current_mid_x = mid_x;
    m_yolo_result_semaphore.release();
  }

  // 设置和获取自动控制的子模式
  void set_auto_submode(auto_submode submode) { m_auto_submode = submode; }
  auto_submode get_auto_submode() const { return m_auto_submode; }

  // 设置和获取数字识别模式
  void set_number_detect_mode(number_detect_mode mode) {
    m_number_detect_mode = mode;
  }
  number_detect_mode get_number_detect_mode() const {
    return m_number_detect_mode;
  }

protected:
  class ir_controller {
    line_follower_8ch m_forward_line_follower;
    line_follower_8ch m_left_line_follower;
    line_follower_8ch m_right_line_follower;

  public:
    ir_controller() = default;
    ~ir_controller() = default;

    void set_parameters(gdut::i2c &i2c_bus) {
      m_forward_line_follower.set_parameters(i2c_bus);
      m_left_line_follower.set_parameters(i2c_bus);
      m_right_line_follower.set_parameters(i2c_bus);
    }

    void set_parameters(const line_follower_8ch::gpio_bus &gpio_bus) {
      m_forward_line_follower.set_parameters(gpio_bus);
      m_left_line_follower.set_parameters(gpio_bus);
      m_right_line_follower.set_parameters(gpio_bus);
    }

    struct original_line_state {
      uint8_t forward_mask;
      uint8_t left_mask;
      uint8_t right_mask;
    };

    // 获取原始的掩码状态，bit=1表示未检测到目标线，bit=0表示检测到目标线
    [[nodiscard]] original_line_state get_original_line_state() {
      original_line_state state{};
      m_forward_line_follower.read_line_mask(state.forward_mask);
      m_left_line_follower.read_line_mask(state.left_mask);
      m_right_line_follower.read_line_mask(state.right_mask);
      return state;
    }

    struct line_state {
      float forward_position;
      float left_position;
      float right_position;
    };

    // 获取处理后的线位置状态，范围为-1.0到1.0，0表示在中间，负数表示偏左，正数表示偏右
    [[nodiscard]] line_state get_line_state() {
      line_state state{};
      if (m_forward_line_follower.is_gpio_mode()) {
        std::uint8_t mask;
        if (m_forward_line_follower.read_line_mask(mask) != HAL_OK) {
          state.forward_position = 0.0f; // 读取失败，认为在中间
        } else {
          state.forward_position = calculate_position(mask);
        }
        if (m_left_line_follower.read_line_mask(mask) != HAL_OK) {
          state.left_position = 0.0f; // 读取失败，认为在中间
        } else {
          state.left_position = calculate_position(mask);
        }
        if (m_right_line_follower.read_line_mask(mask) != HAL_OK) {
          state.right_position = 0.0f; // 读取失败，认为在中间
        } else {
          state.right_position = calculate_position(mask);
        }
      }
      return state;
    }

    // 把掩码转换成位置，范围为-1.0到1.0，0表示在中间，负数表示偏左，正数表示偏右
    [[nodiscard]] float calculate_position(std::uint8_t mask) {
      // 计算位置，假设每个通道的权重为其索引加1（即从1到8），检测到线时对应权重相加
      float position_sum = 0.0f;
      for (uint8_t i = 0; i < line_follower_8ch::channel_count; ++i) {
        // bit=1 表示未检测到目标线，bit=0 表示检测到目标线
        if (mask & (1 << i)) {
          float weight =
              static_cast<float>(i) -
              static_cast<float>(line_follower_8ch::channel_count - 1) / 2.0f;
          position_sum += weight;
        }
      }
      return position_sum /
             static_cast<float>(line_follower_8ch::channel_count - 1) / 2.0f;
    }
  };

  void run_in_thread() {
    while (m_running) {
      osDelay(10); // 避免忙等待占用过多CPU资源
      state current_state = m_state.load(std::memory_order_acquire);
      while ((current_state = m_state.load(std::memory_order_acquire)) ==
             state::idle) {
        // 等待开始信号
        osDelay(10);
      }

      if (current_state == state::wait_qr_code) {
        wait_for_qr_code_result();
        // 收到二维码结果后进入巡线状态
        m_state.store(state::enter_line, std::memory_order_release);
      }

      if (current_state == state::enter_line) {
        enter_line();
        // 进入巡线状态
        m_state.store(state::detect_object, std::memory_order_release);
      }

      // 巡线时检测物体
      if (current_state == state::detect_object) {
        detect_object();
        m_state.store(state::grab_object, std::memory_order_release);
      }

      if (current_state == state::grab_object) {
        grab_object();
        // sub_grab_object();
        // 抓取物体后进入返回线路状态
        m_state.store(state::follow_line_to_destination,
                      std::memory_order_release);
      }

      if (current_state == state::follow_line_to_destination) {
        follow_line_to_destination();
        // 沿线路前往目的地后进入释放物体状态
        m_state.store(state::release_object, std::memory_order_release);
      }

      if (current_state == state::release_object) {
        release_object();
        // 释放物体后进入完成状态
        m_state.store(state::finished, std::memory_order_release);
      }
    }
  }

protected:
  void wait_for_qr_code_result() {
    // 停止底盘运动
    m_chassis_controller->set_speed({0.0f, 0.0f, 0.0f});
    // 等待二维码结果
    while (m_running) {
      m_transfer_controller->send_qr_code_request();
      // 等待二维码结果，超时后重新请求
      m_qr_code_semaphore.acquire(std::chrono::milliseconds(500));
      if (m_current_qr_code_length > 0) {
        break; // 收到二维码结果了，退出循环进入下一状态
      }
      osDelay(100);
    }
    if (!m_running) {
      return;
    }
  }

  void wait_for_yolo_result() {
    // 等待YOLO检测结果
    while (m_running) {
      m_transfer_controller->send_number_request();
      // 等待YOLO检测结果，超时后重新请求
      m_yolo_result_semaphore.acquire(std::chrono::milliseconds(500));
      if (m_current_number_detected > 0) {
        break; // 收到YOLO检测结果了，退出循环进入下一状态
      }
      osDelay(100);
    }
  }

  void move_wait_for_line(auto_submode submode) {
    if (submode == auto_submode::left_mode) {
      m_chassis_controller->set_speed({-0.5f, 0.0f, 0.0f});
      // 先向左移动一段时间，增加与线的偏移，利于后续调整底盘位置
      osDelay(500);

      // 等待前方有线
      auto original_line_state = m_ir_controller.get_original_line_state();
      while (m_running && ~original_line_state.forward_mask == 0xFF) {
        osDelay(10);
        original_line_state = m_ir_controller.get_original_line_state();
      }
      // 使前方线在底盘中间
      auto line_state = m_ir_controller.get_line_state();
      while (m_running && std::abs(line_state.forward_position) > 0.05f) {
        m_chassis_controller->set_speed(
            {std::clamp(line_state.forward_position, -0.8f, 0.8f), 0.0f, 0.0f});
        osDelay(10);
        line_state = m_ir_controller.get_line_state();
      }

    } else if (submode == auto_submode::right_mode) {
      m_chassis_controller->set_speed({0.5f, 0.0f, 0.0f});
      // 先向右移动一段时间，增加与线的偏移，利于后续调整底盘位置
      osDelay(500);

      // 等待前方有线
      auto original_line_state = m_ir_controller.get_original_line_state();
      while (m_running && ~original_line_state.forward_mask == 0xFF) {
        osDelay(10);
        original_line_state = m_ir_controller.get_original_line_state();
      }
      // 使前方线在底盘中间
      auto line_state = m_ir_controller.get_line_state();
      while (m_running && std::abs(line_state.forward_position) > 0.05f) {
        m_chassis_controller->set_speed(
            {std::clamp(line_state.forward_position, -0.8f, 0.8f), 0.0f, 0.0f});
        osDelay(10);
        line_state = m_ir_controller.get_line_state();
      }
    }
  }

  void enter_line() {
    const auto submode = m_auto_submode.load(std::memory_order_acquire);
    // 进入线路，等待一段时间让底盘进入线路
    m_chassis_controller->set_speed({0.0f, 0.8f, 0.0f});
    osDelay(1000);
    while (m_running) {
      auto original_line_state = m_ir_controller.get_original_line_state();
      if (submode == auto_submode::left_mode) {
        if (original_line_state.left_mask) {
          break; // 左边模式检测到线了，退出循环进入巡线状态
        }
      } else if (submode == auto_submode::right_mode) {
        if (original_line_state.right_mask) {
          break; // 右边模式检测到线了，退出循环进入巡线状态
        }
      }
    }
    // 利用红外传感器微调底盘位置
    if (submode == auto_submode::left_mode) {
      // 调整底盘使其左边的线位置在0.71附近，认为是进入线路了
      auto line_state = m_ir_controller.get_line_state();
      while (m_running && std::abs(line_state.left_position - 0.71f) > 0.05f) {
        m_chassis_controller->set_speed(
            {0.0f, std::clamp(line_state.left_position - 0.71f, -0.8f, 0.8f),
             0.0f});
        osDelay(10);
        line_state = m_ir_controller.get_line_state();
      }
    } else if (submode == auto_submode::right_mode) {
      // 调整底盘使其右边的线位置在-0.71附近，认为是进入线路了
      auto line_state = m_ir_controller.get_line_state();
      while (m_running && std::abs(line_state.right_position + 0.71f) > 0.05f) {
        m_chassis_controller->set_speed(
            {0.0f, std::clamp(line_state.right_position + 0.71f, -0.8f, 0.8f),
             0.0f});
        osDelay(10);
        line_state = m_ir_controller.get_line_state();
      }
    }
  }

  void detect_object() {
    // 边巡线边检测物体，等待YOLO检测结果
    const auto submode = m_auto_submode.load(std::memory_order_acquire);
    // 检测四次YOLO结果，利用多数表决法确定最终的检测结果，过滤掉偶尔的错误检测
    for (size_t j = 0; j < 4; ++j) {
      move_wait_for_line(submode);

      // 等待YOLO检测结果
      while (m_running) {
        std::array<std::uint8_t, 8> count_number{0};
        for (size_t i = 0; i < count_number.size(); ++i) {
          wait_for_yolo_result();
          // 更新检测历史和计数
          const auto number =
              m_current_number_detected.load(std::memory_order_acquire);
          if (number == 0 || number > count_number.size()) {
            continue; // 过滤掉无效的检测结果
          }
          count_number[number - 1] += 1;
        }
        auto iter = std::max_element(count_number.begin(), count_number.end());
        auto index = std::distance(count_number.begin(), iter);
        if (iter == count_number.end()) {
          continue; // 没有有效的检测结果，继续等待
        }
        if (index < 0 || index >= count_number.size()) {
          continue; // 结果索引无效，继续等待
        }
        const number_detect_mode number_mode =
            m_number_detect_mode.load(std::memory_order_acquire);
        if (index < 4 && number_mode == number_detect_mode::white_mode) {
          continue; // 白底黑字模式下检测到的数字无效，继续等待
        }
        if (index >= 4 && number_mode == number_detect_mode::black_mode) {
          continue; // 黑底白字模式下检测到的数字无效，继续等待
        }

        // 保存最终的检测结果
        std::unique_lock lock(m_yolo_history_mutex);
        if (number_mode == number_detect_mode::black_mode) {
          m_yolo_detection_history[m_yolo_history_size++] =
              index + 1; // 白底黑字模式下数字是1-4
        } else {
          m_yolo_detection_history[m_yolo_history_size++] =
              index - 3; // 黑底白字模式下数字是5-8，转换成1-4
        }
      }
    }
  }

  void grab_object() {
    // 根据YOLO检测结果调整底盘位置，靠近物体
    std::array<std::uint8_t, 4> detection_history{};
    {
      std::unique_lock lock(m_yolo_history_mutex);
      detection_history = m_yolo_detection_history;
    }

    const auto submode = m_auto_submode.load(std::memory_order_acquire);

    int current_positon_index = detection_history.size() - 1;
    for (size_t j = 0; j < 4; ++j) {
      if (submode == auto_submode::left_mode) {
        // 左边模式，根据检测历史调整底盘位置
        // 现在在最后一个数字的位置，需要根据检测历史判断物体是在左边还是右边，从而调整底盘位置
        int index = 0;
        for (; index < detection_history.size(); ++index) {
          if (detection_history[index] == 1) {
            break;
          }
        }

        // 根据当前位置与要移动的位置的差值判断物体是在左边还是右边，调整底盘位置
        const int need_to_move_count = current_positon_index - index;
        if (need_to_move_count > 0) {
          // 向右移动
          move_wait_for_line(auto_submode::right_mode);
        } else if (need_to_move_count < 0) {
          // 向左移动
          move_wait_for_line(auto_submode::left_mode);
        } else {
          // 已经在正确的位置了，不需要调整底盘位置了
        }

        // 调整底盘位置后控制机械臂抓取物体
        sub_grab_object(j);

      } else if (submode == auto_submode::right_mode) {
        // 右边模式，根据检测历史调整底盘位置
        // 现在在最后一个数字的位置，需要根据检测历史判断物体是在左边还是右边，从而调整底盘位置
        int index = 0;
        for (; index < detection_history.size(); ++index) {
          if (detection_history[index] == 1) {
            break;
          }
        }

        // 根据当前位置与要移动的位置的差值判断物体是在左边还是右边，调整底盘位置
        const int need_to_move_count = current_positon_index - index;
        if (need_to_move_count > 0) {
          // 向左移动
          move_wait_for_line(auto_submode::left_mode);
        } else if (need_to_move_count < 0) {
          // 向右移动
          move_wait_for_line(auto_submode::right_mode);
        } else {
          // 已经在正确的位置了，不需要调整底盘位置了
        }

        // 调整底盘位置后控制机械臂抓取物体
        sub_grab_object(j);
      }
    }
  }

  // 根据实际再改
  void sub_grab_object(size_t count = 0) {
    // 根据调整底盘位置后的位置，控制机械臂抓取物体
    // 这里可以根据需要实现具体的抓取逻辑，例如控制舵机和步进电机的动作序列

    // 步进电机下降
    m_motor->set_direction(true);    // 根据实际情况调整方向
    m_motor->move_steps(2000, 1000); // 根据实际情况调整步数和速度
    osDelay(2000);                   // 等待步进电机完成动作

    // 爪子前伸
    m_pca9685_controller->set_belt_servo_angle(
        90);       // 根据实际情况调整舵机编号和角度
    osDelay(1000); // 等待舵机完成动作

    // 爪子闭合
    m_pca9685_controller->set_claw_servo_angle(
        90);       // 根据实际情况调整舵机编号和角度
    osDelay(1000); // 等待舵机完成动作

    // 步进电机上升
    m_motor->set_direction(false);   // 根据实际情况调整方向
    m_motor->move_steps(2000, 1000); // 根据实际情况调整步数和速度
    osDelay(2000);                   // 等待步进电机完成动作

    // 云台旋转
    size_t turn_count = 0;
    if (count == 0) {
      turn_count = 170;
    } else if (count == 1) {
      turn_count = 135;
    } else if (count == 2) {
      turn_count = 85;
    } else {
      turn_count = 0;
    }
    for (size_t i = 0; i < turn_count; ++i) {
      m_pca9685_controller->turret_turn_right(
          i);      // 根据实际情况调整舵机编号和角度
      osDelay(50); // 等待舵机完成动作
    }

    // 同步带后缩
    m_pca9685_controller->belt_move_backward(
        1.0f); // 根据实际情况调整舵机编号和角度

    // 如果是前面三个位置，抓取后需要把物体放在对应的位置上，如果是最后一个位置，抓取后直接抓在爪子上
    if (count != 3) {
      // 爪子张开
      m_pca9685_controller->set_claw_servo_angle(
          0);        // 根据实际情况调整舵机编号和角度
      osDelay(1000); // 等待舵机完成动作
      // 复位
      m_pca9685_controller->set_belt_servo_angle(
          180);                        // 根据实际情况调整舵机编号和角度
      m_motor->set_direction(true);    // 根据实际情况调整方向
      m_motor->move_steps(2000, 1000); // 根据实际情况调整步数和速度
      m_pca9685_controller->turret_turn_left(
          1.0f); // 根据实际情况调整舵机编号和角度
      m_pca9685_controller->set_claw_servo_angle(
          180); // 根据实际情况调整舵机编号和角度
    }
  }

  // 返回线路
  void back_to_line() {
    // 根据调整底盘位置后的位置，控制底盘返回线路
    // 这里可以根据需要实现具体的返回线路逻辑，例如控制底盘的运动方向和速度
    // 下面是一个简单的示例，实际情况可能需要更复杂的逻辑来确保能够正确返回线路
    if (m_auto_submode.load(std::memory_order_acquire) ==
        auto_submode::left_mode) {
      // 左边模式，向左移动一段时间增加与线的偏移
      m_chassis_controller->set_speed({0.5f, 0.0f, 0.0f});
      osDelay(500);
    } else if (m_auto_submode.load(std::memory_order_acquire) ==
               auto_submode::right_mode) {
      // 右边模式，向右移动一段时间增加与线的偏移
      m_chassis_controller->set_speed({-0.5f, 0.0f, 0.0f});
      osDelay(500);
    }
  }

  void follow_line_to_destination() {
    // 根据调整底盘位置后的位置，控制底盘沿线路前往目的地
    const auto current_submode = m_auto_submode.load(std::memory_order_acquire);
    while (m_running) {
      auto original_line_state = m_ir_controller.get_original_line_state();
      if (current_submode == auto_submode::left_mode) {
        if (~original_line_state.left_mask == 0xFF) {
          break; // 左边模式检测不到线了，进入改变方向状态
        }
      } else if (current_submode == auto_submode::right_mode) {
        if (~original_line_state.right_mask == 0xFF) {
          break; // 右边模式检测不到线了，进入改变方向状态
        }
      }
      osDelay(10);
    }

    if (m_running && current_submode == auto_submode::right_mode) {
      // 改变方向
      // 右边模式，向左移动一段时间增加与线的偏移
      m_chassis_controller->set_speed({0.0f, 0.0f, 1.0f});
      osDelay(500);

      // 使前方线在底盘中间
      auto line_state = m_ir_controller.get_line_state();
      while (m_running && std::abs(line_state.forward_position) > 0.05f) {
        m_chassis_controller->set_speed(
            {std::clamp(line_state.forward_position, -0.8f, 0.8f), 0.0f, 0.0f});
        osDelay(10);
        line_state = m_ir_controller.get_line_state();
      }

      m_chassis_controller->set_speed({0.0f, 0.8f, 0.0f});
      while (m_running) {
        auto original_line_state = m_ir_controller.get_original_line_state();
        if (~original_line_state.forward_mask == 0xFF) {
          break; // 前方检测不到线了，进入下一个状态
        }
        osDelay(10);
      }

      // 使线在底盘中间（右边模式）
      line_state = m_ir_controller.get_line_state();
      while (m_running && std::abs(line_state.right_position + 0.71f) > 0.05f) {
        m_chassis_controller->set_speed(
            {0.0f, std::clamp(line_state.right_position + 0.71f, -0.8f, 0.8f),
             0.0f});
        osDelay(10);
        line_state = m_ir_controller.get_line_state();
      }

      // 侧边前进直到前方检测不到线了，认为到达目的地了（右边模式）
      m_chassis_controller->set_speed({0.5f, 0.0f, 0.0f});
      while (m_running) {
        auto original_line_state = m_ir_controller.get_original_line_state();
        if (~original_line_state.right_mask == 0xFF) {
          break; // 右边模式前方检测不到线了，认为到达目的地了，进入下一个状态
        }
        osDelay(10);
      }
    } else if (m_running && current_submode == auto_submode::left_mode) {
      // 后退
      m_chassis_controller->set_speed({0.0f, -0.5f, 0.0f});
      osDelay(500);
      // 使左边线在底盘中间
      auto original_line_state = m_ir_controller.get_original_line_state();
      while (m_running && ~original_line_state.left_mask == 0xFF) {
        osDelay(10);
        original_line_state = m_ir_controller.get_original_line_state();
      }
      auto line_state = m_ir_controller.get_line_state();
      while (m_running && std::abs(line_state.left_position - 0.71f) > 0.05f) {
        m_chassis_controller->set_speed(
            {0.0f, std::clamp(line_state.left_position - 0.71f, -0.8f, 0.8f),
             0.0f});
        osDelay(10);
        line_state = m_ir_controller.get_line_state();
      }

      // 左边前进直到前方检测不到线了，认为到达目的地了（左边模式）
      m_chassis_controller->set_speed({-0.5f, 0.0f, 0.0f});
      while (m_running) {
        auto original_line_state = m_ir_controller.get_original_line_state();
        if (~original_line_state.left_mask == 0xFF) {
          break; // 左边模式前方检测不到线了，认为到达目的地了，进入下一个状态
        }
        osDelay(10);
      }

      // 停止底盘运动
      m_chassis_controller->set_speed({0.0f, 0.0f, 0.0f});
    }
  }

  // 根据实际再改
  void release_object() {
    const auto current_submode = m_auto_submode.load(std::memory_order_acquire);
    if (current_submode == auto_submode::left_mode) {

      // 先放置夹爪上的方块,所有动作均需要根据实际情况调参

      // 底盘运动到指定位置
      m_chassis_controller->set_speed({-0.5f, 0.0f, 0.0f});
      osDelay(1000); // 等待底盘移动到放置位置了
      m_chassis_controller->set_speed({0.0f, -0.5f, 0.0f});

      // 云台旋转到指定位置
      m_pca9685_controller->set_turret_servo_angle(0);
      osDelay(1000); // 等待舵机完成动作

      // 步进电机下降
      m_motor->set_direction(true);
      m_motor->move_steps(2000, 1000);
      osDelay(2000); // 等待步进电机完成动作

      // 爪子打开
      m_pca9685_controller->set_claw_servo_angle(180);
      osDelay(1000); // 等待舵机完成动作

      // 复位
      reset_motor_status();

      // 再依次序放置之前抓取的方块
      // 底盘运动到指定位置
      m_chassis_controller->set_speed({0.0f, 0.5f, 0.0f});

      // 步进下降
      m_motor->set_direction(true);
      m_motor->move_steps(2000, 1000);
      osDelay(2000); // 等待步进电机完成动作

      // 旋转云台
      m_pca9685_controller->set_turret_servo_angle(90);
      osDelay(1000); // 等待舵机完成动作

      // 底盘运动到指定位置
      m_chassis_controller->set_speed({0.5f, 0.0f, 0.0f});

      // 旋转云台
      m_pca9685_controller->set_turret_servo_angle(90);
      osDelay(1000); // 等待舵机完成动作

      // 底盘运动到指定位置
      m_chassis_controller->set_speed({0.0f, -0.5f, 0.0f});

      // 旋转云台
      m_pca9685_controller->set_turret_servo_angle(90);
      osDelay(1000); // 等待舵机完成动作

    } else if (current_submode == auto_submode::right_mode) {
      // 先使底盘旋转至与左边相同
      m_chassis_controller->set_speed({0.0f, 0.0f, 1.0f});
      osDelay(500);

      // 与左边模式相同的放置流程
      //  底盘运动到指定位置
      m_chassis_controller->set_speed({-0.5f, 0.0f, 0.0f});
      osDelay(1000); // 等待底盘移动到放置位置了
      m_chassis_controller->set_speed({0.0f, -0.5f, 0.0f});

      // 云台旋转到指定位置
      m_pca9685_controller->set_turret_servo_angle(0);
      osDelay(1000); // 等待舵机完成动作

      // 步进电机下降
      m_motor->set_direction(true);
      m_motor->move_steps(2000, 1000);
      osDelay(2000); // 等待步进电机完成动作

      // 爪子打开
      m_pca9685_controller->set_claw_servo_angle(180);
      osDelay(1000); // 等待舵机完成动作

      // 复位
      reset_motor_status();

      // 再依次序放置之前抓取的方块
      // 底盘运动到指定位置
      m_chassis_controller->set_speed({0.0f, 0.5f, 0.0f});

      // 步进下降
      m_motor->set_direction(true);
      m_motor->move_steps(2000, 1000);
      osDelay(2000); // 等待步进电机完成动作

      // 旋转云台
      m_pca9685_controller->set_turret_servo_angle(90);
      osDelay(1000); // 等待舵机完成动作

      // 底盘运动到指定位置
      m_chassis_controller->set_speed({0.5f, 0.0f, 0.0f});

      // 旋转云台
      m_pca9685_controller->set_turret_servo_angle(90);
      osDelay(1000); // 等待舵机完成动作

      // 底盘运动到指定位置
      m_chassis_controller->set_speed({0.0f, -0.5f, 0.0f});

      // 旋转云台
      m_pca9685_controller->set_turret_servo_angle(90);
      osDelay(1000); // 等待舵机完成动作
    }
  }

  // 所有机构复位
  void reset_motor_status() {
    // 根据实际情况实现复位逻辑，例如控制底盘、机械臂和传感器回到初始状态
    // 步进复位
    m_motor->set_direction(true);
    m_motor->move_steps(2000, 1000);

    // 云台复位
    m_pca9685_controller->set_turret_servo_angle(0);
    osDelay(1000); // 等待舵机完成动作
    // 爪子复位
    m_pca9685_controller->set_claw_servo_angle(180);
    osDelay(1000); // 等待舵机完成动作
    // belt复位
    m_pca9685_controller->set_belt_servo_angle(180);
    osDelay(1000); // 等待舵机完成动作
  }

private:
  transfer_controller *m_transfer_controller{nullptr};
  chassis_controller *m_chassis_controller{nullptr};
  pca9685_controller *m_pca9685_controller{nullptr};
  stepper_motor *m_motor{nullptr};

  thread<2048> m_thread{empty_thread};
  std::atomic<bool> m_running{false};

  std::atomic<auto_submode> m_auto_submode{auto_submode::left_mode};
  std::atomic<number_detect_mode> m_number_detect_mode{
      number_detect_mode::black_mode};
  std::atomic<state> m_state{state::idle};

  // 存储当前二维码结果的缓冲区，长度为32字节（包括结尾的'\0'）
  std::array<char, 32> m_current_qr_code{};
  std::size_t m_current_qr_code_length{0};
  binary_semaphore m_qr_code_semaphore{empty_semaphore};

  // 存储当前YOLO检测结果
  std::atomic<uint8_t> m_current_number_detected{0};
  std::atomic<float> m_current_mid_x{0.0f};
  binary_semaphore m_yolo_result_semaphore{empty_semaphore};

  // 历史YOLO检测结果，用于判断检测顺序
  mutex m_yolo_history_mutex{empty_mutex};
  std::array<std::uint8_t, 4> m_yolo_detection_history{};
  std::size_t m_yolo_history_size{0};

  // 三路红外循迹传感器
  ir_controller m_ir_controller;
};

} // namespace gdut

#endif // AUTO_CONTROLLER_HPP
