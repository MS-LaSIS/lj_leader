#ifndef LJ_LEADER__LJ_LEADER_HPP_
#define LJ_LEADER__LJ_LEADER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <LabJackM.h>
#include <string>

class LJLeaderNode : public rclcpp::Node
{
public:
  LJLeaderNode();
  ~LJLeaderNode();

private:
  void read_and_publish();
  double calculate_steering_ratio();
  double calculate_throttle_ratio();
  int read_all_voltages(double voltages[12]);
  bool is_valid(double master1, double master2, double slave1, double slave2,
                double nominal_master, double nominal_slave,
                bool opposition, const std::string& axis_name);

  // LabJack handle
  int handle_;
  
  // Pin names for reading analog inputs (AIN0-AIN11)
  // Steering signals
  std::string steering_master1_pin_;   // AIN0
  std::string steering_slave1_pin_;    // AIN1
  std::string steering_master2_pin_;   // AIN2
  std::string steering_slave2_pin_;    // AIN3
  
  // Throttle/Brake signals
  std::string throttle_master1_pin_;   // AIN4
  std::string throttle_slave1_pin_;    // AIN5
  std::string throttle_master2_pin_;   // AIN6
  std::string throttle_slave2_pin_;    // AIN7
  
  // Nominal voltages
  std::string nominal_vs_steer_master_pin_;    // AIN8
  std::string nominal_vs_steer_slave_pin_;     // AIN9
  std::string nominal_vs_throttle_master_pin_; // AIN10
  std::string nominal_vs_throttle_slave_pin_;  // AIN11
  
  // Percentage limits for mapping
  double steering_min_perc_;
  double steering_max_perc_;
  double throttle_min_perc_;
  double throttle_max_perc_;
  
  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pedal_pub_;
  
  // Timer for periodic reading
  rclcpp::TimerBase::SharedPtr read_timer_;
  double publish_rate_;  // Hz
  
  static constexpr int INITIAL_ERR_ADDRESS = -1;
};

#endif // LJ_LEADER__LJ_LEADER_HPP_
