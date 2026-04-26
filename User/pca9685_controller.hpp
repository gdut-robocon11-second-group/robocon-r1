#ifndef PCA9685_CONTROLLER_HPP
#define PCA9685_CONTROLLER_HPP

#include "bsp_iic.hpp"
#include "bsp_pca9685.hpp"
#include "cmsis_os2.h"
#include "memory_resource.hpp"
#include "mutex.hpp"
#include "stm32f4xx_hal.h"
#include <memory>
#include <mutex>

namespace gdut {

class pca9685_controller {
public:
  pca9685_controller() = default;
  ~pca9685_controller() = default;

  void set_parameters(gdut::pca9685 *pca9685_ctrl) {
    m_mutex = mutex{};
    std::lock_guard<mutex> lock(m_mutex);
    if (!pca9685_ctrl) {
      return;
    }
    m_pca9685 = pca9685_ctrl;
  }

  bool init() {
    std::lock_guard<mutex> lock(m_mutex);
    if (!m_pca9685) {
      return false;
    }
    m_turret_current_angle = 0.0f;
    m_belt_current_angle = 0.0f;
    m_claw_current_angle = 90.0f;
    m_door_current_angle = 0.0f;
    return m_pca9685->init(50.0f) == HAL_OK; // 设置PWM频率为50Hz，适合舵机控制
  }

  // 云台控制接口
  bool set_turret_servo_angle(float angle_deg) {
    std::lock_guard<mutex> lock(m_mutex);
    if (!m_pca9685) {
      return false;
    }
    if (angle_deg < 0.0f || angle_deg > 270.0f) {
      return false; // 无效角度
    }
    if (m_pca9685->set_servo_angle(m_turret_servo_channel, angle_deg) != HAL_OK) {
      return false;
    }
    m_turret_current_angle = angle_deg;
    return true;
  }

  // 同步带上的平移舵机控制接口
  bool set_belt_servo_angle(float angle_deg) {
    std::lock_guard<mutex> lock(m_mutex);
    if (!m_pca9685) {
      return false;
    }
    if (angle_deg < 0.0f || angle_deg > 270.0f) {
      return false; // 无效角度
    }
    if (m_pca9685->set_servo_angle(m_belt_servo_channel, angle_deg) != HAL_OK) {
      return false;
    }
    m_belt_current_angle = angle_deg;
    return true;
  }

  // 夹爪舵机控制接口
  bool set_claw_servo_angle(float angle_deg) {
    std::lock_guard<mutex> lock(m_mutex);
    if (!m_pca9685) {
      return false;
    }
    if (angle_deg < 45.0f || angle_deg > 135.0f) {
      return false; // 无效角度
    }
    if (m_pca9685->set_servo_angle(m_claw_servo_channel, angle_deg) != HAL_OK) {
      return false;
    }
    m_claw_current_angle = angle_deg;
    return true;
  }

  // 门舵机控制接口
  bool set_door_servo_angle(float angle_deg) {
    std::lock_guard<mutex> lock(m_mutex);
    if (!m_pca9685) {
      return false;
    }
    if (angle_deg < 0.0f || angle_deg > 270.0f) {
      return false; // 无效角度
    }
    if (m_pca9685->set_servo_angle(m_door_servo_channel, angle_deg) != HAL_OK) {
      return false;
    }
    m_door_current_angle = angle_deg;
    return true;
  }

  bool turret_turn_left(float angle_deg) {
    if (m_turret_current_angle + angle_deg > 270.0f) {
      return false; // 无效角度
    }
    return set_turret_servo_angle(m_turret_current_angle + angle_deg);
  }

  bool turret_turn_right(float angle_deg) {
    if (m_turret_current_angle - angle_deg < 0.0f) {
      return false; // 无效角度
    }
    return set_turret_servo_angle(m_turret_current_angle - angle_deg);
  }

  bool belt_move_forward(float angle_deg) {
    // 假设同步带上舵机的前进是通过减少角度实现的，即从初始位置向0度方向前进
    // 否则需要根据实际机械结构调整逻辑
    if (m_belt_current_angle + angle_deg > 270.0f) {
      return false; // 无效角度
    }
    return set_belt_servo_angle(m_belt_current_angle + angle_deg);
  }

  bool belt_move_backward(float angle_deg) {
    if (m_belt_current_angle - angle_deg < 0.0f) {
      return false; // 无效角度
    }
    return set_belt_servo_angle(m_belt_current_angle - angle_deg);
  }

  bool claw_open(float angle_deg) {
    // 假设夹爪的打开是通过减少角度实现的，即从初始位置向0度方向打开
    // 否则需要根据实际机械结构调整逻辑
    if (m_claw_current_angle - angle_deg < 45.0f) {
      return false; // 无效角度
    }
    return set_claw_servo_angle(m_claw_current_angle - angle_deg);
  }

  bool claw_close(float angle_deg) {
    if (m_claw_current_angle + angle_deg > 135.0f) {
      return false; // 无效角度
    }
    return set_claw_servo_angle(m_claw_current_angle + angle_deg);
  }

  bool door_open(float angle_deg) {
    // 假设门的打开是通过减少角度实现的，即从初始位置向0度方向打开
    // 否则需要根据实际机械结构调整逻辑
    if (m_door_current_angle - angle_deg < 0.0f) {
      return false; // 无效角度
    }
    return set_door_servo_angle(m_door_current_angle - angle_deg);
  }

  bool door_close(float angle_deg) {
    if (m_door_current_angle + angle_deg > 270.0f) {
      return false; // 无效角度
    }
    return set_door_servo_angle(m_door_current_angle + angle_deg);
  }

  pca9685& get_pca9685() { return *m_pca9685; }
  const pca9685& get_pca9685() const { return *m_pca9685; }

protected:
  struct i2c_deleter {
    void operator()(gdut::i2c *ptr) const {
      if (ptr) {
        ptr->deinit();        // 先反初始化i2c对象
        std::destroy_at(ptr); // 显式调用析构函数
      }
    }
  };

  struct pca9685_deleter {
    void operator()(gdut::pca9685 *ptr) const {
      if (ptr) {
        std::destroy_at(ptr); // 显式调用析构函数
      }
    }
  };

private:
  mutable mutex m_mutex{empty_mutex};
  gdut::pca9685 *m_pca9685{nullptr};

  std::uint8_t m_turret_servo_channel{4}; // 假设通道0用于云台
  std::uint8_t m_belt_servo_channel{5};   // 假设通道1用于同步带上的平移舵机
  std::uint8_t m_claw_servo_channel{6};   // 假设通道2用于夹爪舵机
  std::uint8_t m_door_servo_channel{7};   // 假设通道3用于门的舵机

  float m_turret_current_angle{0}; // 当前云台角度
  float m_belt_current_angle{0};   // 当前同步带角度
  float m_claw_current_angle{0};   // 当前夹爪角度
  float m_door_current_angle{0};   // 当前门角度
};

} // namespace gdut

#endif // PCA9685_CONTROLLER_HPP
