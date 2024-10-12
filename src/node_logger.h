#ifndef __NODE_LOGGER_H__
#define __NODE_LOGGER_H__

#include "rclcpp/rclcpp.hpp"

class NodeLogger : public Logger {
  private:
    rclcpp::Node *node;

  public:
    NodeLogger(rclcpp::Node *n) : node(n) {}

    void info(const std::string& msg)
    {
      RCLCPP_INFO(node->get_logger(), msg.c_str());
    }

    void warning(const std::string& msg)
    {
      RCLCPP_WARN(node->get_logger(), msg.c_str());
    }

    void error(const std::string& msg)
    {
      RCLCPP_ERROR(node->get_logger(), msg.c_str());
    }
};

#endif // __NODE_LOGGER_H__