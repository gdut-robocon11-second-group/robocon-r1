#ifndef TRANSFER_CONTROLLER_HPP
#define TRANSFER_CONTROLLER_HPP

#include "function.hpp"
#include "message_queue.hpp"
#include "stm32f4xx_hal.h"
#include "thread.hpp"
#include "transfer_protocol.hpp"
#include "uncopyable.hpp"

#include <iterator>
#include <utility>

namespace gdut {

// transfer_controller负责连接底层的UART接口和上层的transfer_protocol，
// 并在接收数据时通过消息队列将数据传递给一个独立的线程进行处理，
// 避免在中断上下文中进行复杂的协议解析。
class transfer_protocol : private uncopyable {
public:
  using packet_manager_t = packet_manager<crc16_algorithm>;
  using packet_t = typename packet_manager_t::packet_t;

  enum class command_code : uint16_t {
    get_number_detected = 0x0001,
    get_qr_code_detected = 0x0002,
  };

  struct yolo_detection_result {
    uint16_t number_detected;
  } __attribute__((packed));

  struct qr_code_detection_result {
    char qr_code_data[32];
  } __attribute__((packed));

  transfer_protocol() {
    m_package_manager.set_receive_function([this](packet_t packet) {
      this->handle_received_packet(std::move(packet));
    });
  }
  ~transfer_protocol() = default;

  void set_send_function(
      gdut::function<void(const std::uint8_t *, const std::uint8_t *)> func) {
    m_package_manager.set_send_function(std::move(func));
  }

  void set_yolo_result_callback(
      gdut::function<void(yolo_detection_result)> callback) {
    m_yolo_result_callback = std::move(callback);
  }

  void set_qr_code_result_callback(
      gdut::function<void(qr_code_detection_result)> callback) {
    m_qr_code_result_callback = std::move(callback);
  }

  template <std::input_iterator It> void original_receive(It data, It end) {
    m_package_manager.receive(data, end);
  }

  void send_number_request() {
    char body[] = {0};
    packet_t packet(std::to_underlying(command_code::get_number_detected), body,
                    body, build_packet);
    m_package_manager.send(packet);
  }

  void send_qr_code_request() {
    char body[] = {0};
    packet_t packet(std::to_underlying(command_code::get_qr_code_detected),
                    body, body, build_packet);
    m_package_manager.send(packet);
  }

protected:
  void handle_received_packet(packet_t packet) {
    const uint16_t code = packet.code();
    const uint8_t *body = packet.body_data();
    const size_t body_size = packet.body_size();

    switch (static_cast<command_code>(code)) {
    case command_code::get_number_detected: {
      yolo_detection_result result;
      result.number_detected = body[0] - '0';
      if (m_yolo_result_callback) {
        std::invoke(m_yolo_result_callback, result);
      }
    } break;
    case command_code::get_qr_code_detected: {
      qr_code_detection_result result;
      std::copy(body, body + body_size, reinterpret_cast<uint8_t *>(&result));
      if (m_qr_code_result_callback) {
        std::invoke(m_qr_code_result_callback, result);
      }
    } break;
    default:
      // 忽略未知的命令码
      break;
    }
  }

private:
  packet_manager_t m_package_manager;
  gdut::function<void(yolo_detection_result)> m_yolo_result_callback;
  gdut::function<void(qr_code_detection_result)> m_qr_code_result_callback;
};

// transfer_controller负责连接底层的UART接口和上层的transfer_protocol，
// 并在接收数据时通过消息队列将数据传递给一个独立的线程进行处理，
// 避免在中断上下文中进行复杂的协议解析。
class transfer_controller {
public:
  transfer_controller() = default;
  explicit transfer_controller(UART_HandleTypeDef *uart_instance)
      : m_uart(uart_instance) {}
  ~transfer_controller() = default;

  transfer_protocol &protocol() { return m_protocol; }

  void set_uart(UART_HandleTypeDef *uart_instance) { m_uart = uart_instance; }

  void set_send_function(
      gdut::function<void(const std::uint8_t *, const std::uint8_t *)> func) {
    m_protocol.set_send_function(std::move(func));
  }

  void set_yolo_result_callback(
      gdut::function<void(transfer_protocol::yolo_detection_result)> callback) {
    m_protocol.set_yolo_result_callback(std::move(callback));
  }

  void set_qr_code_result_callback(
      gdut::function<void(transfer_protocol::qr_code_detection_result)>
          callback) {
    m_protocol.set_qr_code_result_callback(std::move(callback));
  }

  void uart_rx_callback_it(size_t size) {
    queue_data msg;
    while (size > 0) {
      size_t chunk_size = std::min(size, sizeof(msg.data));
      std::copy(m_receive_buffer, m_receive_buffer + chunk_size, msg.data);
      msg.size = static_cast<uint8_t>(chunk_size);
      m_message_queue.send_from_isr(msg);
      size -= chunk_size;
    }
    HAL_UARTEx_ReceiveToIdle_IT(m_uart, m_receive_buffer,
                                sizeof(m_receive_buffer));
  }

  void send_number_request() { m_protocol.send_number_request(); }
  void send_qr_code_request() { m_protocol.send_qr_code_request(); }

  void start() {
    if (m_processing_thread.joinable()) {
      return; // Processing thread already running
    }
    m_message_queue = message_queue<queue_data>(10);
    m_processing_thread =
        thread<2048>("raspberry_uart_processing_thread", [this] {
          queue_data msg;
          while (true) {
            if (m_message_queue.receive(msg)) {
              this->original_receive(msg.data, msg.data + msg.size);
            }
          }
        });
    HAL_UARTEx_ReceiveToIdle_IT(m_uart, m_receive_buffer,
                                sizeof(m_receive_buffer));
  }

protected:
  struct queue_data {
    uint8_t size;
    uint8_t data[32];
  };

  template <std::input_iterator It> void original_receive(It data, It end) {
    m_protocol.original_receive(data, end);
  }

private:
  UART_HandleTypeDef *m_uart = nullptr;
  transfer_protocol m_protocol;
  message_queue<queue_data> m_message_queue{empty_message_queue};
  uint8_t m_receive_buffer[256];
  thread<2048> m_processing_thread{empty_thread};
};

} // namespace gdut

#endif // TRANSFER_CONTROLLER_HPP
