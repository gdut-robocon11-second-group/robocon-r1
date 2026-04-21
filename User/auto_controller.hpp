#ifndef AUTO_CONTROLLER_HPP
#define AUTO_CONTROLLER_HPP

#include "transfer_controller.hpp"
#include "chassis_controller.hpp"

namespace gdut {

class auto_controller {
public:
  auto_controller() = default;
  ~auto_controller() = default;

  void start() {}

protected:
  void run_in_thread() {}

private:
  transfer_controller *m_transfer_controller{nullptr};
  chassis_controller *m_chassis_controller{nullptr};
};

} // namespace gdut

#endif // AUTO_CONTROLLER_HPP
