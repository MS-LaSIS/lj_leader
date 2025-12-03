#include "lj_leader/lj_leader.hpp"
#include <cmath>

LJLeaderNode::LJLeaderNode() : Node("lj_leader")
{
  // Declare parameters for pin assignments - Steering signals
  this->declare_parameter<std::string>("steering_master1_pin", "AIN0");
  this->declare_parameter<std::string>("steering_slave1_pin", "AIN1");
  this->declare_parameter<std::string>("steering_master2_pin", "AIN2");
  this->declare_parameter<std::string>("steering_slave2_pin", "AIN3");
  
  // Throttle/Brake signals
  this->declare_parameter<std::string>("throttle_master1_pin", "AIN4");
  this->declare_parameter<std::string>("throttle_slave1_pin", "AIN5");
  this->declare_parameter<std::string>("throttle_master2_pin", "AIN6");
  this->declare_parameter<std::string>("throttle_slave2_pin", "AIN7");
  
  // Nominal voltages
  this->declare_parameter<std::string>("nominal_vs_steer_master_pin", "AIN8");
  this->declare_parameter<std::string>("nominal_vs_steer_slave_pin", "AIN9");
  this->declare_parameter<std::string>("nominal_vs_throttle_master_pin", "AIN10");
  this->declare_parameter<std::string>("nominal_vs_throttle_slave_pin", "AIN11");
  
  // Declare percentage limits for steering
  this->declare_parameter<double>("steering_min_perc", 0.15);
  this->declare_parameter<double>("steering_max_perc", 0.85);
  
  // Declare percentage limits for throttle/brake
  this->declare_parameter<double>("throttle_min_perc", 0.23);
  this->declare_parameter<double>("throttle_max_perc", 0.77);
  
  // Declare publish rate
  this->declare_parameter<double>("publish_rate", 60.0); // Hz
  
  // Declare topic parameters
  this->declare_parameter<std::string>("steering_topic", "leader/steering_cmd");
  this->declare_parameter<std::string>("pedal_topic", "leader/pedal_cmd");
  
  // Get parameters
  steering_master1_pin_ = this->get_parameter("steering_master1_pin").as_string();
  steering_slave1_pin_ = this->get_parameter("steering_slave1_pin").as_string();
  steering_master2_pin_ = this->get_parameter("steering_master2_pin").as_string();
  steering_slave2_pin_ = this->get_parameter("steering_slave2_pin").as_string();
  
  throttle_master1_pin_ = this->get_parameter("throttle_master1_pin").as_string();
  throttle_slave1_pin_ = this->get_parameter("throttle_slave1_pin").as_string();
  throttle_master2_pin_ = this->get_parameter("throttle_master2_pin").as_string();
  throttle_slave2_pin_ = this->get_parameter("throttle_slave2_pin").as_string();
  
  nominal_vs_steer_master_pin_ = this->get_parameter("nominal_vs_steer_master_pin").as_string();
  nominal_vs_steer_slave_pin_ = this->get_parameter("nominal_vs_steer_slave_pin").as_string();
  nominal_vs_throttle_master_pin_ = this->get_parameter("nominal_vs_throttle_master_pin").as_string();
  nominal_vs_throttle_slave_pin_ = this->get_parameter("nominal_vs_throttle_slave_pin").as_string();
  
  steering_min_perc_ = this->get_parameter("steering_min_perc").as_double();
  steering_max_perc_ = this->get_parameter("steering_max_perc").as_double();
  throttle_min_perc_ = this->get_parameter("throttle_min_perc").as_double();
  throttle_max_perc_ = this->get_parameter("throttle_max_perc").as_double();
  
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  
  std::string steering_topic = this->get_parameter("steering_topic").as_string();
  std::string pedal_topic = this->get_parameter("pedal_topic").as_string();
  
  // Open LabJack T7
  int err = LJM_Open(LJM_dtT7, LJM_ctUSB, "ANY", &handle_);
  if (err != LJME_NOERROR) {
    char errName[LJM_MAX_NAME_SIZE];
    LJM_ErrorToString(err, errName);
    RCLCPP_ERROR(this->get_logger(), "Failed to open LabJack T7: %s", errName);
    rclcpp::shutdown();
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "LabJack T7 opened successfully");
  
  // Create publishers
  steering_pub_ = this->create_publisher<std_msgs::msg::Float32>(steering_topic, 10);
  pedal_pub_ = this->create_publisher<std_msgs::msg::Float32>(pedal_topic, 10);
  
  // Create timer for periodic reading
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  read_timer_ = this->create_wall_timer(
    period,
    std::bind(&LJLeaderNode::read_and_publish, this));
  
  RCLCPP_INFO(this->get_logger(), "LJ Leader Node initialized");
  RCLCPP_INFO(this->get_logger(), "Publishing steering to: '%s'", steering_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing pedal to: '%s'", pedal_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Steering limits: %.1f%% - %.1f%%", 
              steering_min_perc_ * 100, steering_max_perc_ * 100);
  RCLCPP_INFO(this->get_logger(), "Throttle limits: %.1f%% - %.1f%%", 
              throttle_min_perc_ * 100, throttle_max_perc_ * 100);
  RCLCPP_INFO(this->get_logger(), "Publish rate: %.1f Hz", publish_rate_);
}

LJLeaderNode::~LJLeaderNode()
{
  // Close LabJack handle
  if (handle_ > 0) {
    LJM_Close(handle_);
    RCLCPP_INFO(this->get_logger(), "LabJack connection closed");
  }
}

void LJLeaderNode::read_and_publish()
{
  // Read all voltages from LabJack
  double voltages[12];
  int err = read_all_voltages(voltages);
  
  if (err != LJME_NOERROR) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Failed to read voltages from LabJack");
    // Publish zero values on error
    auto steering_msg = std_msgs::msg::Float32();
    steering_msg.data = 0.0f;
    steering_pub_->publish(steering_msg);
    
    auto pedal_msg = std_msgs::msg::Float32();
    pedal_msg.data = 0.0f;
    pedal_pub_->publish(pedal_msg);
    return;
  }
  
  // Calculate and publish steering ratio
  double steering_ratio = calculate_steering_ratio(voltages);
  auto steering_msg = std_msgs::msg::Float32();
  steering_msg.data = static_cast<float>(steering_ratio);
  steering_pub_->publish(steering_msg);
  
  // Calculate and publish throttle ratio
  double throttle_ratio = calculate_throttle_ratio(voltages);
  auto pedal_msg = std_msgs::msg::Float32();
  pedal_msg.data = static_cast<float>(throttle_ratio);
  pedal_pub_->publish(pedal_msg);
  
  RCLCPP_DEBUG(this->get_logger(), "Steering ratio: %.3f, Throttle ratio: %.3f", 
               steering_ratio, throttle_ratio);
}

bool LJLeaderNode::is_valid(double master1, double master2, double slave1, double slave2,
                            double nominal_master, double nominal_slave,
                            bool opposition, const std::string& axis_name)
{
  // Check for invalid nominal voltages
  if (nominal_master < 0.1 || nominal_slave < 0.1) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "%s: Invalid nominal voltages - Master: %.2fV, Slave: %.2fV",
                        axis_name.c_str(), nominal_master, nominal_slave);
    return false;
  }
  
  // If opposition mode is enabled, check that Master1 + Master2 ≈ nominal_master
  if (opposition) {
    double master_sum = master1 + master2;
    double master_tolerance = nominal_master * 0.15; // 15% tolerance
    
    if (std::abs(master_sum - nominal_master) > master_tolerance) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "%s: Master opposition check failed - M1+M2=%.2fV, Nominal=%.2fV",
                          axis_name.c_str(), master_sum, nominal_master);
      return false;
    }
    
    // Check that Slave1 + Slave2 ≈ nominal_slave
    double slave_sum = slave1 + slave2;
    double slave_tolerance = nominal_slave * 0.15; // 15% tolerance
    
    if (std::abs(slave_sum - nominal_slave) > slave_tolerance) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "%s: Slave opposition check failed - S1+S2=%.2fV, Nominal=%.2fV",
                          axis_name.c_str(), slave_sum, nominal_slave);
      return false;
    }
  }
  
  return true;
}

double LJLeaderNode::calculate_steering_ratio(const double voltages[12])
{
  
  // Extract steering voltages
  double steer_master1 = voltages[0];  // AIN0
  double steer_slave1 = voltages[1];   // AIN1
  double steer_master2 = voltages[2];  // AIN2
  double steer_slave2 = voltages[3];   // AIN3
  double nom_vs_steer_master = voltages[8];  // AIN8
  double nom_vs_steer_slave = voltages[9];   // AIN9
  
  // Validate voltages with opposition mode check
  if (!is_valid(steer_master1, steer_master2, steer_slave1, steer_slave2,
                nom_vs_steer_master, nom_vs_steer_slave, true, "Steering")) {
    return 0.0;
  }
  
  // Calculate expected voltage range based on percentage limits
  double master_min = nom_vs_steer_master * steering_min_perc_;
  double master_max = nom_vs_steer_master * steering_max_perc_;
  
  // Map the master1 voltage to a ratio [0, 1]
  // This reverses the mapping done in lj_handler
  double ratio = (steer_master1 - master_min) / (master_max - master_min);
  ratio = std::max(0.0, std::min(1.0, ratio));
  
  // Convert to steering ratio [-1, 1]
  // ratio 0.0 -> -1.0 (right), ratio 0.5 -> 0.0 (center), ratio 1.0 -> 1.0 (left)
  double steering_ratio = (ratio * 2.0) - 1.0;
  
  return steering_ratio;
}

double LJLeaderNode::calculate_throttle_ratio(const double voltages[12])
{
  
  // Extract throttle voltages
  double throttle_master1 = voltages[4];  // AIN4
  double throttle_slave1 = voltages[5];   // AIN5
  double throttle_master2 = voltages[6];  // AIN6
  double throttle_slave2 = voltages[7];   // AIN7
  double nom_vs_throttle_master = voltages[10];  // AIN10
  double nom_vs_throttle_slave = voltages[11];   // AIN11
  
  // Validate voltages with opposition mode check
  if (!is_valid(throttle_master1, throttle_master2, throttle_slave1, throttle_slave2,
                nom_vs_throttle_master, nom_vs_throttle_slave, true, "Throttle")) {
    return 0.0;
  }
  
  // Calculate expected voltage range based on percentage limits
  double master_min = nom_vs_throttle_master * throttle_min_perc_;
  double master_max = nom_vs_throttle_master * throttle_max_perc_;
  
  // Map the master1 voltage to a ratio [0, 1]
  // This reverses the mapping done in lj_handler
  double ratio = (throttle_master1 - master_min) / (master_max - master_min);
  ratio = std::max(0.0, std::min(1.0, ratio));
  
  // Convert to throttle ratio [-1, 1]
  // ratio 0.0 -> -1.0 (brake), ratio 0.5 -> 0.0 (neutral), ratio 1.0 -> 1.0 (accelerate)
  double throttle_ratio = (ratio * 2.0) - 1.0;
  
  return throttle_ratio;
}

int LJLeaderNode::read_all_voltages(double voltages[12])
{
  // Prepare array of all 12 pin names
  const char* names[12] = {
    steering_master1_pin_.c_str(),   // AIN0
    steering_slave1_pin_.c_str(),    // AIN1
    steering_master2_pin_.c_str(),   // AIN2
    steering_slave2_pin_.c_str(),    // AIN3
    throttle_master1_pin_.c_str(),   // AIN4
    throttle_slave1_pin_.c_str(),    // AIN5
    throttle_master2_pin_.c_str(),   // AIN6
    throttle_slave2_pin_.c_str(),    // AIN7
    nominal_vs_steer_master_pin_.c_str(),    // AIN8
    nominal_vs_steer_slave_pin_.c_str(),     // AIN9
    nominal_vs_throttle_master_pin_.c_str(), // AIN10
    nominal_vs_throttle_slave_pin_.c_str()   // AIN11
  };
  
  int errorAddress = INITIAL_ERR_ADDRESS;
  
  int err = LJM_eReadNames(handle_, 12, names, voltages, &errorAddress);
  
  if (err != LJME_NOERROR) {
    char errName[LJM_MAX_NAME_SIZE];
    LJM_ErrorToString(err, errName);
    RCLCPP_DEBUG(this->get_logger(), "Error reading voltages: %s (address: %d)", 
                errName, errorAddress);
  }
  
  return err;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LJLeaderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
