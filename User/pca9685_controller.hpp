#ifndef PCA9685_CONTROLLER_HPP
#define PCA9685_CONTROLLER_HPP

#include "bsp_iic.hpp"
#include "bsp_pca9685.hpp"
#include "memory_resource.hpp"
#include <memory>
#include <new>

namespace gdut {

class pca9685_controller {
public:
  pca9685_controller() = default;
  ~pca9685_controller() = default;

  void set_parameters(I2C_HandleTypeDef *hi2c) {
    if (!hi2c) {
      return;
    }
    auto *resource = pmr::portable_resource::get_instance();
    void *storage = resource->allocate(sizeof(gdut::i2c), alignof(gdut::i2c));
    auto *i2c_bus = new (storage) gdut::i2c(hi2c);
    m_i2c = std::unique_ptr<gdut::i2c, i2c_deleter>(i2c_bus);
    m_i2c->init();
    m_pca9685.set_parameters(*m_i2c);
  }  

  void init() {
    if (m_pca9685.is_ready() == HAL_OK) {
      m_turret_current_angle = 135.0f;
      m_belt_current_angle = 135.0f;
      m_claw_current_angle = 90.0f;
      m_door_current_angle = 90.0f;
      m_pca9685.init(50.0f); // 设置PWM频率为50Hz，适合舵机控制
    }
  }

  // 云台控制接口
  bool set_turret_servo_angle(float angle_deg) {
    if (angle_deg < 0.0f || angle_deg > 270.0f) {
      return false; // 无效角度
    }
    if (m_pca9685.set_servo_angle(m_turret_servo_channel, angle_deg) !=
        HAL_OK) {
      return false;
    }
    m_turret_current_angle = angle_deg;
    return true;
  }

  // 同步带上的平移舵机控制接口
  bool set_belt_servo_angle(float angle_deg) {
    if (angle_deg < 0.0f || angle_deg > 270.0f) {
      return false; // 无效角度
    }
    if (m_pca9685.set_servo_angle(m_belt_servo_channel, angle_deg) != HAL_OK) {
      return false;
    }
    m_belt_current_angle = angle_deg;
    return true;
  }

  // 夹爪舵机控制接口
  bool set_claw_servo_angle(float angle_deg) {
    if (angle_deg < 0.0f || angle_deg > 270.0f) {
      return false; // 无效角度
    }
    if (m_pca9685.set_servo_angle(m_claw_servo_channel, angle_deg) != HAL_OK) {
      return false;
    }
    m_claw_current_angle = angle_deg;
    return true;
  }

  // 门舵机控制接口
  bool set_door_servo_angle(float angle_deg) {
    if (angle_deg < 0.0f || angle_deg > 270.0f) {
      return false; // 无效角度
    }
    if (m_pca9685.set_servo_angle(m_door_servo_channel, angle_deg) != HAL_OK) {
      return false;
    }
    m_door_current_angle = angle_deg;
    return true;
  }

  bool turret_turn_left(float angle_deg) {
    if (m_turret_current_angle - angle_deg < 0.0f) {
      return false; // 无效角度
    }
    return set_turret_servo_angle(m_turret_current_angle - angle_deg);
  }

  bool turret_turn_right(float angle_deg) {
    if (m_turret_current_angle + angle_deg > 270.0f) {
      return false; // 无效角度
    }
    return set_turret_servo_angle(m_turret_current_angle + angle_deg);
  }

  bool belt_move_forward(float angle_deg) {
    // 假设同步带上舵机的前进是通过减少角度实现的，即从初始位置向0度方向前进
    // 否则需要根据实际机械结构调整逻辑
    if (m_belt_current_angle - angle_deg < 0.0f) {
      return false; // 无效角度
    }
    return set_belt_servo_angle(m_belt_current_angle - angle_deg);
  }

  bool belt_move_backward(float angle_deg) {
    if (m_belt_current_angle + angle_deg > 270.0f) {
      return false; // 无效角度
    }
    return set_belt_servo_angle(m_belt_current_angle + angle_deg);
  }

  bool claw_open(float angle_deg) {
    // 假设夹爪的打开是通过减少角度实现的，即从初始位置向0度方向打开
    // 否则需要根据实际机械结构调整逻辑
    if (m_claw_current_angle - angle_deg < 0.0f) {
      return false; // 无效角度
    }
    return set_claw_servo_angle(m_claw_current_angle - angle_deg);
  }

  bool claw_close(float angle_deg) {
    if (m_claw_current_angle + angle_deg > 270.0f) {
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

protected:
  struct i2c_deleter {
    void operator()(gdut::i2c *ptr) const {
      if (ptr) {
        ptr->deinit(); // 先反初始化i2c对象
        ptr->~i2c();   // 显式调用析构函数
        pmr::portable_resource::get_instance()->deallocate(
            ptr, sizeof(gdut::i2c), alignof(gdut::i2c));
      }
    }
  };

private:
  std::unique_ptr<gdut::i2c, i2c_deleter> m_i2c;
  pca9685 m_pca9685;

  std::uint8_t m_turret_servo_channel{0}; // 假设通道0用于云台
  std::uint8_t m_belt_servo_channel{1};   // 假设通道1用于同步带上的平移舵机
  std::uint8_t m_claw_servo_channel{2};   // 假设通道2用于夹爪舵机
  std::uint8_t m_door_servo_channel{3};   // 假设通道3用于门的舵机

  float m_turret_current_angle{0}; // 当前云台角度
  float m_belt_current_angle{0};   // 当前同步带角度
  float m_claw_current_angle{0};   // 当前夹爪角度
  float m_door_current_angle{0};   // 当前门角度
};

} // namespace gdut

#endif // PCA9685_CONTROLLER_HPP
